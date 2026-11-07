// SPDX-License-Identifier: GPL-2.0-only
/*
 * VC8000E VCMD core plus the in-kernel client interface.
 *
 * The vendored eswin source stays unmodified for re-syncing; it is built as
 * part of this translation unit so the port can adapt its mapping call and
 * the client wrappers below can reach its static scheduler entry points.
 * See ax630c_vcmd_client.h.
 */
#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/workqueue.h>
#include "framebuf_alloc.h"

#define remap_pfn_range vcmd_fb_remap_pfn_range
#include "eswin/vc8000_vcmd_driver.c"
#undef remap_pfn_range

#include "ax630c_vcmd_client.h"

struct device *ax630c_vcmd_device(void)
{
	return venc_pdev && hantrovcmd_data ? &venc_pdev->dev : NULL;
}
EXPORT_SYMBOL_GPL(ax630c_vcmd_device);

static int vcmd_client_fop_release(struct inode *inode, struct file *filp)
{
	if (filp->private_data)
		hantrovcmd_release(inode, filp);
	return 0;
}

static const struct file_operations vcmd_client_fops = {
	.owner = THIS_MODULE,
	.release = vcmd_client_fop_release,
};

struct file *ax630c_vcmd_client_open(void)
{
	struct file *filp;
	int ret;

	if (!ax630c_vcmd_device())
		return ERR_PTR(-ENODEV);
	filp = anon_inode_getfile("ax630c-vcmd-client", &vcmd_client_fops, NULL, O_RDWR);
	if (IS_ERR(filp))
		return filp;
	ret = hantrovcmd_open(file_inode(filp), filp);
	if (ret) {
		/* A failed open may leave freed private data behind. */
		filp->private_data = NULL;
		fput(filp);
		return ERR_PTR(ret < 0 && ret != -1 ? ret : -ENOMEM);
	}
	return filp;
}
EXPORT_SYMBOL_GPL(ax630c_vcmd_client_open);

struct vcmd_client_close {
	struct work_struct work;
	struct file *filp;
};

static void vcmd_client_close_work(struct work_struct *work)
{
	struct vcmd_client_close *c = container_of(work, struct vcmd_client_close, work);

	__fput_sync(c->filp);
}

/*
 * Release aborts and delinks buffers still queued for this client. It takes
 * the scheduler semaphores interruptibly, so run it on a kernel thread where
 * a pending signal (an exiting task) cannot skip the cleanup.
 */
void ax630c_vcmd_client_close(struct file *client)
{
	struct vcmd_client_close c = { .filp = client };

	if (!client)
		return;
	INIT_WORK_ONSTACK(&c.work, vcmd_client_close_work);
	queue_work(system_unbound_wq, &c.work);
	flush_work(&c.work);
	destroy_work_on_stack(&c.work);
}
EXPORT_SYMBOL_GPL(ax630c_vcmd_client_close);

static struct cmdbuf_obj *vcmd_client_cmdbuf_obj(struct file *client, u16 id)
{
	bi_list_node *node;
	struct cmdbuf_obj *obj;

	if (id >= TOTAL_DISCRETE_CMDBUF_NUM)
		return NULL;
	node = global_cmdbuf_node[id];
	if (!node)
		return NULL;
	obj = node->data;
	return obj->filp == client ? obj : NULL;
}

int ax630c_vcmd_reserve(struct file *client, u16 *id)
{
	struct exchange_parameter p = { .module_type = VCMD_TYPE_ENCODER };
	long ret = reserve_cmdbuf(client, &p);

	if (ret)
		return ret < -1 ? ret : -EBUSY;
	*id = p.cmdbuf_id;
	return 0;
}
EXPORT_SYMBOL_GPL(ax630c_vcmd_reserve);

u32 *ax630c_vcmd_cmdbuf(struct file *client, u16 id)
{
	struct cmdbuf_obj *obj = vcmd_client_cmdbuf_obj(client, id);

	return obj ? obj->cmdbuf_virtualAddress : NULL;
}
EXPORT_SYMBOL_GPL(ax630c_vcmd_cmdbuf);

u32 *ax630c_vcmd_status(struct file *client, u16 id)
{
	struct cmdbuf_obj *obj = vcmd_client_cmdbuf_obj(client, id);

	return obj ? obj->status_virtualAddress : NULL;
}
EXPORT_SYMBOL_GPL(ax630c_vcmd_status);

u64 ax630c_vcmd_status_bus(u16 id)
{
	/* The address user space receives from GET_CMDBUF_PARAMETER. */
	return vcmd_status_buf_mem_pool.busAddress - base_ddr_addr +
	       (u64)id * CMDBUF_MAX_SIZE;
}
EXPORT_SYMBOL_GPL(ax630c_vcmd_status_bus);

int ax630c_vcmd_link_run(struct file *client, u16 id, u16 size)
{
	struct exchange_parameter p = {
		.module_type = VCMD_TYPE_ENCODER,
		.cmdbuf_id = id,
		.cmdbuf_size = size,
	};
	long ret;

	if (!size || size > CMDBUF_MAX_SIZE || (size & 7))
		return -EINVAL;
	ret = link_and_run_cmdbuf(client, &p);
	return ret ? (ret < -1 ? ret : -EIO) : 0;
}
EXPORT_SYMBOL_GPL(ax630c_vcmd_link_run);

int ax630c_vcmd_wait(struct file *client, u16 id, unsigned long timeout,
		     u32 *status)
{
	struct cmdbuf_obj *obj = vcmd_client_cmdbuf_obj(client, id);
	struct hantrovcmd_dev *dev;
	long ret;

	if (!obj)
		return -EINVAL;
	dev = &hantrovcmd_data[obj->core_id];
	/* The ISR wakes the queue with wake_up_interruptible. */
	ret = wait_event_interruptible_timeout(*dev->wait_queue,
					       check_cmdbuf_irq(dev, obj, status), timeout);
	if (ret < 0)
		return ret;
	if (!ret)
		return -ETIMEDOUT;
	/* Same accounting as HANTRO_IOCH_WAIT_CMDBUF. */
	atomic64_add(obj->status_virtualAddress[dev->vcmd_core_cfg.submodule_main_addr / 2 / 4 + 82],
		     &dev->core_tot_cycles);
	atomic64_add(obj->executing_time, &dev->core_tot_exetime);
	return 0;
}
EXPORT_SYMBOL_GPL(ax630c_vcmd_wait);

int ax630c_vcmd_release(struct file *client, u16 id)
{
	if (!vcmd_client_cmdbuf_obj(client, id))
		return -EINVAL;
	return release_cmdbuf(client, id) ? -EIO : 0;
}
EXPORT_SYMBOL_GPL(ax630c_vcmd_release);
