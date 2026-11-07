/* SPDX-License-Identifier: (GPL-2.0-only WITH Linux-syscall-note) OR BSD-2-Clause */
/*
 * Open AX630C (AX620E family) NPU driver interface: /dev/onekvm-ax-npu.
 *
 * The kernel only powers the NPU, hands out physically contiguous buffers,
 * maps the on-chip memory (OCM) and starts a program: user space lays out the
 * model (program bytes, OCM address table, queue descriptors, parameters).
 *
 * mmap offsets: ONEKVM_AXNPU_OCM_OFFSET maps the OCM (uncached); a buffer is
 * mapped at the offset returned by ALLOC. Buffers are mapped cached: SYNC
 * them to the device after the CPU writes and from the device before the
 * CPU reads what the NPU wrote.
 */
#ifndef ONEKVM_AX630C_NPU_H
#define ONEKVM_AX630C_NPU_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define ONEKVM_AXNPU_DEVICE "/dev/onekvm-ax-npu"
#define ONEKVM_AXNPU_OCM_OFFSET 0ull

struct onekvm_axnpu_info {
	__u32 struct_size;
	__u32 ocm_size;        /* bytes of OCM user space may use */
	__u32 eu_clock_mhz;
	__u32 reserved;
};

struct onekvm_axnpu_alloc {
	__u64 size;            /* in: bytes; rounded up to pages */
	__u64 phys;            /* out: physical address seen by the NPU */
	__u64 mmap_offset;     /* out: offset for mmap() on this fd */
	__u32 handle;          /* out */
	__u32 reserved;
};

/* A hardware queue: a command list inside the OCM. */
struct onekvm_axnpu_queue {
	__u32 ocm_offset;      /* 32-byte aligned */
	__u32 size;            /* bytes, multiple of 32, > 0 */
};

struct onekvm_axnpu_run {
	__u64 program;         /* physical address of the program, inside a buffer */
	struct onekvm_axnpu_queue queue[2]; /* io, compute; size 0 = unused */
	__u32 timeout_ms;      /* 0 = 30 s */
	__u32 result;          /* out: completion value (low bits = result code) */
	__u32 error;           /* out: error interrupt value, 0 if none */
	__u32 elapsed_us;      /* out */
};

#define ONEKVM_AXNPU_SYNC_TO_DEVICE   1u
#define ONEKVM_AXNPU_SYNC_FROM_DEVICE 2u

struct onekvm_axnpu_sync {
	__u32 handle;
	__u32 direction;       /* ONEKVM_AXNPU_SYNC_* */
	__u64 offset;
	__u64 size;            /* 0 = to the end of the buffer */
};

#define ONEKVM_AXNPU_IOC_INFO  _IOR('X', 0x40, struct onekvm_axnpu_info)
#define ONEKVM_AXNPU_IOC_ALLOC _IOWR('X', 0x41, struct onekvm_axnpu_alloc)
#define ONEKVM_AXNPU_IOC_FREE  _IOW('X', 0x42, __u32)
#define ONEKVM_AXNPU_IOC_RUN   _IOWR('X', 0x43, struct onekvm_axnpu_run)
#define ONEKVM_AXNPU_IOC_SYNC  _IOW('X', 0x44, struct onekvm_axnpu_sync)

#endif
