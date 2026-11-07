/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * In-kernel clients of the AX630C VC8000E command engine.
 *
 * A client is an anonymous file: VCMD keys its scheduling, runtime PM and
 * abort-on-close by struct file, exactly as for /dev/es_venc users, so kernel
 * and user-space submitters share the core. reserve, link_run, wait and
 * release use interruptible waits inside the vendor scheduler; call them from
 * a kernel thread (workqueue), where no signal can cut them short. open and
 * close are safe from any process context.
 */
#ifndef AX630C_VCMD_CLIENT_H
#define AX630C_VCMD_CLIENT_H

#include <linux/types.h>

struct device;
struct file;

/* The VCMD platform device, or NULL before ax630c_venc_vcmd has probed. */
struct device *ax630c_vcmd_device(void);

struct file *ax630c_vcmd_client_open(void);
void ax630c_vcmd_client_close(struct file *client);

int ax630c_vcmd_reserve(struct file *client, u16 *id);
/* Command slot (CPU address) and status slot (CPU and bus addresses) of a
 * reserved command buffer. Both pools are coherent. */
u32 *ax630c_vcmd_cmdbuf(struct file *client, u16 id);
u32 *ax630c_vcmd_status(struct file *client, u16 id);
u64 ax630c_vcmd_status_bus(u16 id);
int ax630c_vcmd_link_run(struct file *client, u16 id, u16 size);
/* Returns -ETIMEDOUT if the buffer has not completed; *status is the VCMD
 * execution status (0 = OK) otherwise. */
int ax630c_vcmd_wait(struct file *client, u16 id, unsigned long timeout,
		     u32 *status);
int ax630c_vcmd_release(struct file *client, u16 id);

#endif
