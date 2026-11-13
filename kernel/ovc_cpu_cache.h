/* SPDX-License-Identifier: GPL-2.0 */
/*
 * CPU cache ownership of one capture buffer that user space maps cacheable.
 *
 * The WDMA writes DDR behind the CPU caches, and the kernel linear map lets
 * the CPU speculate lines of the buffer in at any time. Ownership therefore
 * moves with the buffer:
 *
 *   QBUF    user PTEs go (ZAP) and, if user space could write since the
 *           last QBUF, dirty lines are written back (CLEAN), so nothing it
 *           wrote is evicted onto the frame the WDMA writes -- also when
 *           the mapping is already gone;
 *   fault   the first one after the WDMA wrote drops the buffer's lines
 *           (INVAL) before the PTEs come back. A fault while the buffer is
 *           still queued maps it without that, and marks it stale;
 *   DQBUF   stale PTEs go (ZAP), so the next access faults and invalidates.
 *
 * A client that never touches the pixels never faults and never pays a
 * cache operation. Pure state under the caller's lock; callers carry out
 * the returned actions in the order ZAP, CLEAN, INVAL. tests/ builds it on
 * the host against a cache-line model.
 */
#ifndef OVC_CPU_CACHE_H
#define OVC_CPU_CACHE_H

#include <linux/types.h>

#define OVC_CC_ZAP	1u	/* remove every user PTE of the buffer */
#define OVC_CC_CLEAN	2u	/* write back the buffer's dirty lines */
#define OVC_CC_INVAL	4u	/* drop the buffer's lines */

struct ovc_cc {
	bool queued;	/* the WDMA owns it: QBUF .. DQBUF / STREAMOFF */
	bool ptes;	/* user PTEs may exist */
	bool cpu_valid;	/* lines dropped since the WDMA last wrote */
	bool stale;	/* PTEs were inserted while queued */
	bool touched;	/* PTEs existed since the last QBUF: lines may be dirty */
};

/* QBUF (buf_prepare). */
static inline unsigned int ovc_cc_qbuf(struct ovc_cc *cc)
{
	unsigned int act = (cc->ptes ? OVC_CC_ZAP : 0) |
			   (cc->touched ? OVC_CC_CLEAN : 0);

	*cc = (struct ovc_cc){ .queued = true };
	return act;
}

/* DQBUF (buf_finish), also for buffers taken back at STREAMOFF. */
static inline unsigned int ovc_cc_dqbuf(struct ovc_cc *cc)
{
	unsigned int act = cc->stale ? OVC_CC_ZAP : 0;

	cc->queued = false;
	if (act)
		cc->ptes = cc->stale = false;
	return act;
}

/* A user access found no PTE; the caller then maps the buffer. */
static inline unsigned int ovc_cc_fault(struct ovc_cc *cc)
{
	unsigned int act = 0;

	if (cc->queued) {
		cc->stale = true;
	} else if (!cc->cpu_valid) {
		cc->cpu_valid = true;
		act = OVC_CC_INVAL;
	}
	cc->ptes = cc->touched = true;
	return act;
}

/* The last user mapping is gone; written lines may still be dirty. */
static inline void ovc_cc_unmapped(struct ovc_cc *cc)
{
	cc->ptes = cc->stale = false;
}

#endif /* OVC_CPU_CACHE_H */
