// SPDX-License-Identifier: GPL-2.0-only
/*
 * Open driver for the Axera AX630C (AX620E family) NPU.
 *
 * Written from observed hardware behavior; no vendor source. Single NPU
 * mode (the hardware's two virtual NPUs merged): the program is fetched
 * from DDR, two command queues live in the on-chip memory, and the sync
 * manager raises one interrupt when the program ends and another on error.
 *
 * Register blocks inside the NPU window:
 *   EU0..EU3 +0x00000..+0x30000, DISP +0x40000, SM +0x50000, GLB +0x90000.
 * Addresses given to the hardware are physical addresses >> 5.
 */
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/xarray.h>

#include "../../include/uapi/onekvm/ax630c_npu.h"

#define EU(n)		(0x10000 * (n))
#define DISP		0x40000
#define SM		0x50000
#define GLB		0x90000

#define EU_CTRL		0x00
#define EU_DATA_CRC	0x08
#define EU_CMD_CRC	0x0c
#define EU_CFG_CC	0xcc
#define EU_ERR		0xd0
#define EU_ERR_CLR	0xd4
#define EU_IR_DROP	0xdc

#define DISP_OCM	0x000
#define DISP_PROG	0x004
#define DISP_Q(n)	(0x100 + 8 * (n))	/* +0 start >> 5, +4 (size >> 5) - 1 */
#define DISP_CFG	0x120
#define DISP_THROTTLE	0x124
#define DISP_TRIGGER	0x168
#define DISP_HALT	0x178
#define DISP_HALT_CLR	0x188
#define DISP_IDLE	0x190
#define DISP_ERR	0x20c
#define DISP_ERR_CLR	0x210

#define SM_ISP0		0x14
#define SM_ISP1		0x18
#define SM_ISP2		0x34
#define SM_ISP3		0x38
#define SM_DONE_CLR	0x5c
#define SM_INT_EN0	0x64
#define SM_INT_EN1	0x68
#define SM_DONE		0x74
#define SM_ERR_CLR	0x7c
#define SM_ERR		0x84
#define SM_SW_MASK	0x8c
#define SM_CONV_DELAY	0x1a4
#define SM_BYPASS	0x1ac

#define GLB_MUX		0x00
#define GLB_AUTO_GATE	0x10
#define GLB_IDLE	0x48
#define GLB_EB0_SET	0x64
#define GLB_EB0_CLR	0x68
#define GLB_EB1_SET	0x6c
#define GLB_EB1_CLR	0x70
#define GLB_RST_SET	0x74
#define GLB_RST_CLR	0x78

#define PMU_STATUS	0x0c
#define PMU_UP_SET	0x74
#define PMU_UP_CLR	0x78
#define PMU_NPU_OFF	(BIT(20) | BIT(22))

/* Hardware queues 0 and 2 belong to the first (here: only) virtual NPU. */
#define QUEUE_IO	0
#define QUEUE_COMPUTE	2
#define IDLE_MASK	(BIT(QUEUE_IO) | BIT(QUEUE_COMPUTE))

#define OCM_SIZE	0x280000

/* Cached buffers with explicit SYNC (default) or uncached coherent ones. */
static bool cached = true;
module_param(cached, bool, 0444);
MODULE_PARM_DESC(cached, "Map NPU buffers cached and sync them explicitly");

struct axnpu {
	struct device *dev;
	void __iomem *regs;
	void __iomem *pmu;
	phys_addr_t ocm_phys;
	struct miscdevice misc;
	struct mutex lock;	/* one program at a time */
	struct completion done;
	u32 done_value;
	u32 err_value;
	bool failed;		/* re-initialize before the next run */
};

struct axnpu_buf {
	void *cpu;		/* coherent buffers */
	struct page *pages;	/* cached buffers */
	dma_addr_t dma;
	size_t size;
};

static int axnpu_buf_alloc(struct device *dev, struct axnpu_buf *b)
{
	if (cached) {
		b->pages = dma_alloc_pages(dev, b->size, &b->dma, DMA_BIDIRECTIONAL, GFP_KERNEL);
		if (!b->pages)
			return -ENOMEM;
		memset(page_address(b->pages), 0, b->size);
		dma_sync_single_for_device(dev, b->dma, b->size, DMA_BIDIRECTIONAL);
		return 0;
	}
	b->cpu = dma_alloc_coherent(dev, b->size, &b->dma, GFP_KERNEL);
	return b->cpu ? 0 : -ENOMEM;
}

static void axnpu_buf_free(struct device *dev, struct axnpu_buf *b)
{
	if (b->pages)
		dma_free_pages(dev, b->size, b->pages, b->dma, DMA_BIDIRECTIONAL);
	else
		dma_free_coherent(dev, b->size, b->cpu, b->dma);
	kfree(b);
}

struct axnpu_file {
	struct axnpu *npu;
	struct mutex lock;
	struct xarray bufs;	/* handle -> buf; handle 0 is the OCM */
};

static inline void w(struct axnpu *npu, u32 off, u32 v)
{
	writel(v, npu->regs + off);
}

static inline u32 r(struct axnpu *npu, u32 off)
{
	return readl(npu->regs + off);
}

static int axnpu_power_on(struct axnpu *npu)
{
	u32 v;

	if (!npu->pmu || (readl(npu->pmu + PMU_STATUS) & PMU_NPU_OFF) != PMU_NPU_OFF)
		return 0;
	writel(BIT(5), npu->pmu + PMU_UP_SET);
	if (readl_poll_timeout(npu->pmu + PMU_STATUS, v, !(v & PMU_NPU_OFF), 2000, 1000000)) {
		dev_err(npu->dev, "power domain did not come up\n");
		return -ETIMEDOUT;
	}
	writel(BIT(5), npu->pmu + PMU_UP_CLR);
	return 0;
}

/* Clocks, resets and the default configuration of every block. */
static void axnpu_hw_init(struct axnpu *npu)
{
	u32 mux;
	int i;

	w(npu, GLB + GLB_RST_CLR, 0xfff);
	/* EU 800 MHz, GLB source 3, timer source 5 (switched with its clock off). */
	w(npu, GLB + GLB_EB0_CLR, 0x2);
	mux = r(npu, GLB + GLB_MUX) & ~0xffu;
	w(npu, GLB + GLB_MUX, mux | 4 | (3 << 3) | (5 << 5));
	w(npu, GLB + GLB_EB0_SET, 0x2);
	w(npu, GLB + GLB_EB0_CLR, 0x4);
	w(npu, GLB + GLB_EB0_SET, 0xfffffffb);
	w(npu, GLB + GLB_EB1_CLR, 0x311);
	w(npu, GLB + GLB_EB1_SET, 0xfffffcee);
	w(npu, GLB + GLB_AUTO_GATE, r(npu, GLB + GLB_AUTO_GATE) | 0x3);

	w(npu, SM + SM_INT_EN0, 0xfffff000);
	w(npu, SM + SM_INT_EN1, 0xfffff000);
	for (i = 0; i < 4; i++) {
		w(npu, EU(i) + EU_CFG_CC, 0);
		w(npu, EU(i) + EU_DATA_CRC, 0);
		w(npu, EU(i) + EU_CMD_CRC, 0);
		w(npu, EU(i) + EU_CTRL, 0x3f);
	}
	w(npu, DISP + DISP_CFG, 0x2000);
	w(npu, EU(0) + EU_IR_DROP, 0xc80);
	w(npu, EU(1) + EU_IR_DROP, 0xc80);
	w(npu, SM + SM_CONV_DELAY, 0x50);
	w(npu, DISP + DISP_THROTTLE, 0xf00);
	for (i = 0x128; i <= 0x154; i += 4)
		w(npu, DISP + i, 0);
	w(npu, DISP + 0x218, 0);
	w(npu, DISP + DISP_THROTTLE, 0);
	w(npu, SM + SM_ISP0, 0xffffffff);
	w(npu, SM + SM_ISP1, 0x2108);
	w(npu, SM + SM_ISP2, 0xffffffff);
	w(npu, SM + SM_ISP3, 0x2108);
	npu->failed = false;
}

/* After an error or timeout: stop the dispatcher and reset every engine. */
static void axnpu_recover(struct axnpu *npu)
{
	u32 v;

	w(npu, DISP + DISP_HALT, 1);
	w(npu, SM + SM_BYPASS, r(npu, SM + SM_BYPASS) | 0x101);
	readl_poll_timeout(npu->regs + GLB + GLB_IDLE, v, v & 1, 1000, 300000);
	w(npu, DISP + DISP_HALT_CLR, 1);
	w(npu, DISP + DISP_HALT, 0);
	w(npu, SM + SM_BYPASS, r(npu, SM + SM_BYPASS) & ~0x101);
	w(npu, SM + SM_SW_MASK, 0xffff);
	/* EU, sync manager and dispatcher soft resets. */
	w(npu, GLB + GLB_EB1_CLR, 0xcc);
	w(npu, GLB + GLB_RST_SET, 0x50c | 0x200 | 0x10);
	w(npu, GLB + GLB_RST_CLR, 0x50c | 0x200 | 0x10);
	w(npu, GLB + GLB_EB1_SET, 0xcc);
	axnpu_hw_init(npu);
}

static irqreturn_t axnpu_done_irq(int irq, void *data)
{
	struct axnpu *npu = data;
	u32 v = r(npu, SM + SM_DONE);

	w(npu, SM + SM_DONE_CLR, v);
	npu->done_value = v;
	complete(&npu->done);
	return IRQ_HANDLED;
}

static irqreturn_t axnpu_err_irq(int irq, void *data)
{
	struct axnpu *npu = data;
	u32 v = r(npu, SM + SM_ERR);
	int i;

	w(npu, SM + SM_ERR_CLR, v);
	for (i = 0; i < 4; i++)
		if (v & BIT(i)) {
			dev_warn(npu->dev, "EU%d error %#x\n", i, r(npu, EU(i) + EU_ERR) & 0xffff);
			w(npu, EU(i) + EU_ERR_CLR, 1);
		}
	if (v & BIT(4)) {
		dev_warn(npu->dev, "dispatch error %#x\n", r(npu, DISP + DISP_ERR));
		w(npu, DISP + DISP_ERR_CLR, 1);
	}
	if (v == 0x20)		/* handshake event, not fatal */
		return IRQ_HANDLED;
	npu->err_value = v;
	complete(&npu->done);
	return IRQ_HANDLED;
}

static struct axnpu_buf *axnpu_find(struct axnpu_file *f, u64 addr, u64 len)
{
	struct axnpu_buf *b;
	unsigned long id;

	xa_for_each(&f->bufs, id, b)
		if (addr >= b->dma && len <= b->size && addr - b->dma <= b->size - len)
			return b;
	return NULL;
}

static bool queue_ok(const struct onekvm_axnpu_queue *q)
{
	return !q->size || (!(q->ocm_offset & 31) && !(q->size & 31) &&
			    q->ocm_offset < OCM_SIZE && q->size <= OCM_SIZE - q->ocm_offset);
}

static long axnpu_run(struct axnpu_file *f, struct onekvm_axnpu_run *run)
{
	struct axnpu *npu = f->npu;
	unsigned long timeout = msecs_to_jiffies(run->timeout_ms ? run->timeout_ms : 30000);
	ktime_t start;
	u32 v, mask = 0;
	long left;
	int i, rc = 0;

	if ((run->program & 31) || !queue_ok(&run->queue[0]) || !queue_ok(&run->queue[1]) ||
	    (!run->queue[0].size && !run->queue[1].size))
		return -EINVAL;
	mutex_lock(&f->lock);
	if (!axnpu_find(f, run->program, 32)) {
		mutex_unlock(&f->lock);
		return -EINVAL;
	}
	mutex_unlock(&f->lock);

	mutex_lock(&npu->lock);
	if (npu->failed)
		axnpu_recover(npu);
	if (readl_poll_timeout(npu->regs + DISP + DISP_IDLE, v,
			       (v & IDLE_MASK) == IDLE_MASK, 10, 100000)) {
		axnpu_recover(npu);
		rc = -EBUSY;
		goto out;
	}
	reinit_completion(&npu->done);
	npu->done_value = 0;
	npu->err_value = 0;
	w(npu, SM + SM_SW_MASK, 0xffff);
	w(npu, DISP + DISP_OCM, 0);
	w(npu, DISP + DISP_PROG, lower_32_bits(run->program >> 5));
	for (i = 0; i < 2; i++) {
		const struct onekvm_axnpu_queue *q = &run->queue[i];
		int hw = i ? QUEUE_COMPUTE : QUEUE_IO;

		if (!q->size)
			continue;
		w(npu, DISP + DISP_Q(hw), q->ocm_offset >> 5);
		w(npu, DISP + DISP_Q(hw) + 4, (q->size >> 5) - 1);
		mask |= BIT(hw);
	}
	start = ktime_get();
	wmb();
	w(npu, DISP + DISP_TRIGGER, mask);
	left = wait_for_completion_interruptible_timeout(&npu->done, timeout);
	run->elapsed_us = ktime_us_delta(ktime_get(), start);
	run->result = npu->done_value;
	run->error = npu->err_value;
	if (left <= 0 || npu->err_value || npu->done_value == 0xffffffff) {
		rc = left < 0 ? -EINTR : left == 0 ? -ETIMEDOUT : -EIO;
		dev_warn(npu->dev, "run failed (%d): done %#x error %#x idle %#x\n", rc,
			 npu->done_value, npu->err_value, r(npu, DISP + DISP_IDLE));
		axnpu_recover(npu);
	}
out:
	mutex_unlock(&npu->lock);
	return rc;
}

static long axnpu_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct axnpu_file *f = file->private_data;
	void __user *uarg = (void __user *)arg;

	switch (cmd) {
	case ONEKVM_AXNPU_IOC_INFO: {
		struct onekvm_axnpu_info info = {
			.struct_size = sizeof(info), .ocm_size = OCM_SIZE, .eu_clock_mhz = 800,
		};
		return copy_to_user(uarg, &info, sizeof(info)) ? -EFAULT : 0;
	}
	case ONEKVM_AXNPU_IOC_ALLOC: {
		struct onekvm_axnpu_alloc a;
		struct axnpu_buf *b;
		u32 id;
		int rc;

		if (copy_from_user(&a, uarg, sizeof(a)))
			return -EFAULT;
		if (!a.size || a.size > SZ_256M)
			return -EINVAL;
		b = kzalloc(sizeof(*b), GFP_KERNEL);
		if (!b)
			return -ENOMEM;
		b->size = PAGE_ALIGN(a.size);
		rc = axnpu_buf_alloc(f->npu->dev, b);
		if (rc) {
			kfree(b);
			return rc;
		}
		rc = xa_alloc(&f->bufs, &id, b, XA_LIMIT(1, 0xffff), GFP_KERNEL);
		if (rc) {
			axnpu_buf_free(f->npu->dev, b);
			return rc;
		}
		a.phys = b->dma;
		a.handle = id;
		a.mmap_offset = (u64)id << 28;
		if (copy_to_user(uarg, &a, sizeof(a))) {
			xa_erase(&f->bufs, id);
			axnpu_buf_free(f->npu->dev, b);
			return -EFAULT;
		}
		return 0;
	}
	case ONEKVM_AXNPU_IOC_FREE: {
		struct axnpu_buf *b;
		u32 id;

		if (get_user(id, (u32 __user *)uarg))
			return -EFAULT;
		/* Not while a program of any client may still read it. */
		mutex_lock(&f->npu->lock);
		b = xa_erase(&f->bufs, id);
		mutex_unlock(&f->npu->lock);
		if (!b)
			return -ENOENT;
		axnpu_buf_free(f->npu->dev, b);
		return 0;
	}
	case ONEKVM_AXNPU_IOC_SYNC: {
		struct onekvm_axnpu_sync y;
		struct axnpu_buf *b;
		u64 size;

		if (copy_from_user(&y, uarg, sizeof(y)))
			return -EFAULT;
		mutex_lock(&f->lock);
		b = xa_load(&f->bufs, y.handle);
		size = y.size ? y.size : (b && y.offset < b->size ? b->size - y.offset : 0);
		if (!b || y.offset >= b->size || size > b->size - y.offset ||
		    (y.direction != ONEKVM_AXNPU_SYNC_TO_DEVICE &&
		     y.direction != ONEKVM_AXNPU_SYNC_FROM_DEVICE)) {
			mutex_unlock(&f->lock);
			return -EINVAL;
		}
		if (b->pages) {
			if (y.direction == ONEKVM_AXNPU_SYNC_TO_DEVICE)
				dma_sync_single_for_device(f->npu->dev, b->dma + y.offset, size,
							   DMA_BIDIRECTIONAL);
			else
				dma_sync_single_for_cpu(f->npu->dev, b->dma + y.offset, size,
							DMA_BIDIRECTIONAL);
		}
		mutex_unlock(&f->lock);
		return 0;
	}
	case ONEKVM_AXNPU_IOC_RUN: {
		struct onekvm_axnpu_run run;
		long rc;

		if (copy_from_user(&run, uarg, sizeof(run)))
			return -EFAULT;
		rc = axnpu_run(f, &run);
		if (copy_to_user(uarg, &run, sizeof(run)))
			return -EFAULT;
		return rc;
	}
	}
	return -ENOTTY;
}

static int axnpu_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct axnpu_file *f = file->private_data;
	unsigned long len = vma->vm_end - vma->vm_start;
	u64 off = (u64)vma->vm_pgoff << PAGE_SHIFT;
	struct axnpu_buf *b;
	u32 id = off >> 28;

	if (off & ((1ull << 28) - 1))
		return -EINVAL;
	if (!id) {
		if (len > OCM_SIZE)
			return -EINVAL;
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		vm_flags_set(vma, VM_IO | VM_DONTEXPAND | VM_DONTDUMP);
		return io_remap_pfn_range(vma, vma->vm_start, f->npu->ocm_phys >> PAGE_SHIFT, len,
					  vma->vm_page_prot);
	}
	b = xa_load(&f->bufs, id);
	if (!b || len > b->size)
		return -EINVAL;
	vma->vm_pgoff = 0;
	if (b->pages)
		return dma_mmap_pages(f->npu->dev, vma, len, b->pages);
	return dma_mmap_coherent(f->npu->dev, vma, b->cpu, b->dma, len);
}

static int axnpu_open(struct inode *inode, struct file *file)
{
	struct axnpu *npu = container_of(file->private_data, struct axnpu, misc);
	struct axnpu_file *f = kzalloc(sizeof(*f), GFP_KERNEL);

	if (!f)
		return -ENOMEM;
	f->npu = npu;
	mutex_init(&f->lock);
	xa_init_flags(&f->bufs, XA_FLAGS_ALLOC1);
	file->private_data = f;
	return 0;
}

static int axnpu_release(struct inode *inode, struct file *file)
{
	struct axnpu_file *f = file->private_data;
	struct axnpu_buf *b;
	unsigned long id;

	mutex_lock(&f->npu->lock);	/* no program of this client runs */
	mutex_unlock(&f->npu->lock);
	xa_for_each(&f->bufs, id, b)
		axnpu_buf_free(f->npu->dev, b);
	xa_destroy(&f->bufs);
	kfree(f);
	return 0;
}

static const struct file_operations axnpu_fops = {
	.owner = THIS_MODULE,
	.open = axnpu_open,
	.release = axnpu_release,
	.unlocked_ioctl = axnpu_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.mmap = axnpu_mmap,
};

static int axnpu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct axnpu *npu;
	int irq, rc;

	npu = devm_kzalloc(dev, sizeof(*npu), GFP_KERNEL);
	if (!npu)
		return -ENOMEM;
	npu->dev = dev;
	mutex_init(&npu->lock);
	init_completion(&npu->done);

	npu->regs = devm_platform_ioremap_resource_byname(pdev, "regs");
	if (IS_ERR(npu->regs))
		return PTR_ERR(npu->regs);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ocm");
	if (!res || resource_size(res) < OCM_SIZE)
		return dev_err_probe(dev, -EINVAL, "missing OCM\n");
	npu->ocm_phys = res->start;
	/* The power controller is shared: map it without claiming it. */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmu");
	if (res) {
		npu->pmu = devm_ioremap(dev, res->start, resource_size(res));
		if (!npu->pmu)
			return -ENOMEM;
	}
	rc = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (rc)
		return rc;

	irq = platform_get_irq_byname(pdev, "done");
	if (irq < 0)
		return irq;
	rc = devm_request_irq(dev, irq, axnpu_done_irq, 0, "onekvm-ax-npu", npu);
	if (rc)
		return rc;
	irq = platform_get_irq_byname(pdev, "error");
	if (irq < 0)
		return irq;
	rc = devm_request_irq(dev, irq, axnpu_err_irq, 0, "onekvm-ax-npu-err", npu);
	if (rc)
		return rc;

	rc = axnpu_power_on(npu);
	if (rc)
		return rc;
	axnpu_hw_init(npu);

	npu->misc.minor = MISC_DYNAMIC_MINOR;
	npu->misc.name = "onekvm-ax-npu";
	npu->misc.fops = &axnpu_fops;
	npu->misc.mode = 0600;
	npu->misc.parent = dev;
	rc = misc_register(&npu->misc);
	if (rc)
		return rc;
	platform_set_drvdata(pdev, npu);
	dev_info(dev, "NPU ready (glb mux %#x, idle %#x)\n", r(npu, GLB + GLB_MUX),
		 r(npu, DISP + DISP_IDLE));
	return 0;
}

static void axnpu_remove(struct platform_device *pdev)
{
	struct axnpu *npu = platform_get_drvdata(pdev);

	misc_deregister(&npu->misc);
	w(npu, SM + SM_INT_EN0, 0);
	w(npu, SM + SM_INT_EN1, 0);
}

static const struct of_device_id axnpu_of_match[] = {
	{ .compatible = "onekvm,ax630c-npu" },
	{ }
};
MODULE_DEVICE_TABLE(of, axnpu_of_match);

static struct platform_driver axnpu_driver = {
	.probe = axnpu_probe,
	.remove = axnpu_remove,
	.driver = {
		.name = "onekvm-ax630c-npu",
		.of_match_table = axnpu_of_match,
	},
};
module_platform_driver(axnpu_driver);

MODULE_DESCRIPTION("Open AX630C NPU driver");
MODULE_LICENSE("GPL");
