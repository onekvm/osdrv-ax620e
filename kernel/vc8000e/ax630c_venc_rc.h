/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * Integer port of the vcenc_rc.h frame-level rate controller (onekvm-video-ax630c
 * third_party/vcenc-ewl, from GoogleBot42/open-nanokvm-pro) for kernel use.
 *
 * Same laws, same constants: seed QP from bits per pixel, first P = IDR + 3
 * (CBR), the +-2 P-step law on bits/target of the previous frame, IDR QP
 * from the previous GOP's mean P QP, one-second budget windows rounded to
 * whole GOPs and a two-second leaky bucket without credit. Bit counts are
 * kept in Q16 so per-frame targets carry the fraction the float version
 * has, ratios are compared as cross products, rounding to the nearest
 * integer QP is half-to-even like lrint(), and log2 is Q16 fixed point.
 * The caller picks a QP on a copy and commits the copy only after a complete
 * access unit, so a rejected frame leaves no trace.
 */
#ifndef AX630C_VENC_RC_H
#define AX630C_VENC_RC_H

#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/string.h>
#include <linux/types.h>

#define VENC_RC_CBR		0u
#define VENC_RC_VBR		1u

#define VENC_RC_CPB_SEC		2	/* leaky bucket = 2 s of target bits */
#define VENC_RC_P_FLOOR_DELTA	11u	/* CBR: P QP >= IDR QP - 11 */
#define VENC_RC_FIRST_P_DELTA	3u	/* CBR: first P after an IDR = IDR + 3 */
#define VENC_RC_P_CEIL_LATER	46u	/* CBR ceiling from the second GOP on */
#define VENC_RC_I_STEP		2	/* IDR QP moves <= 2 per GOP */
#define VENC_RC_I_MEANP_DELTA	4	/* IDR = mean P QP of the last GOP - 4 */
#define VENC_RC_VBR_RESTART_QP	32u	/* VBR first IDR after a retarget */
#define VENC_RC_Q		16	/* bit counts below are in Q16 */

struct venc_rc {
	u32 codec;		/* 0 = H.264, 1 = HEVC (seed clamp only) */
	u32 w, h, mode, fps, gop, qp_min, qp_max;
	s64 bitrate;		/* bits per second */
	s64 bpf, cpb;		/* Q16 bits */
	s64 buf;		/* Q16 bits owed, 0..cpb */
	u32 win_len, win_pos;
	s64 win_budget, win_spent;
	u32 gop_p_qp_sum, gop_p_n, gop_index;
	s64 last_idr_bits;
	u32 qp_seed, qp_i, qp_last;
	s64 last_bits, last_target, prev_bits;
	bool last_dropped, retargeted, pending_idr;
	u32 since_idr, p_count, pending_qp, frames;
	s64 pending_target;
};

/* log2(v) in Q16, v > 0. */
static inline s64 venc_rc_log2_q16(u64 v)
{
	int ip = fls64(v) - 1, i;
	u64 m = ip >= 31 ? v >> (ip - 31) : v << (31 - ip);	/* [1,2) in Q31 */
	s64 r = (s64)ip << 16;

	for (i = 15; i >= 0; i--) {
		m = (m * m) >> 31;
		if (m >= (2ULL << 31)) {
			m >>= 1;
			r |= 1 << i;
		}
	}
	return r;
}

/* Nearest integer, ties to even (lrint), of n / d with n >= 0, d > 0. */
static inline s64 venc_rc_round_even(u64 n, u32 d)
{
	u64 q = div_u64(n, d), r = n - q * d;

	if (2 * r > d || (2 * r == d && (q & 1)))
		q++;
	return q;
}

static inline u32 venc_rc_clampu(s64 q, u32 lo, u32 hi)
{
	return q < (s64)lo ? lo : q > (s64)hi ? hi : (u32)q;
}

static inline u64 venc_rc_pixel_rate(const struct venc_rc *rc)
{
	return (u64)rc->fps * rc->w * rc->h;
}

/* CBR seed: 32 + 4.8 * log2((1/16) / bpp), in [32, 37 (H.264) / 35 (HEVC)]. */
static inline u32 venc_rc_seed_qp(const struct venc_rc *rc)
{
	u32 cap = rc->codec ? 35 : 37;
	s64 d, q;

	if (rc->bitrate <= 0)
		return cap;
	d = venc_rc_log2_q16(venc_rc_pixel_rate(rc)) -
	    venc_rc_log2_q16(16ULL * (u64)rc->bitrate);
	q = (32LL << 16) + div_s64(48 * d, 10);
	if (q < (32LL << 16))
		q = 32LL << 16;
	if (q > ((s64)cap << 16))
		q = (s64)cap << 16;
	return (u32)((q + 0x8000) >> 16);
}

/* VBR seed: the nominal pic_init_qp curve, 36 / 32 / 26 at 2 / 8 / 16 Mbps
 * 1080p60 (2 QP per octave below 0.0643 bpp, 6 above), in [16, 51]. */
static inline u32 venc_rc_nominal_qp(const struct venc_rc *rc)
{
	u64 ref = 643ULL * venc_rc_pixel_rate(rc);	/* 0.0643 bpp, x10000 */
	u64 have = 10000ULL * (u64)rc->bitrate;
	s64 q;

	if (rc->bitrate <= 0)
		return 36;
	if (have < ref)
		q = (32LL << 16) + 2 * (venc_rc_log2_q16(ref) - venc_rc_log2_q16(have));
	else
		q = (32LL << 16) - 6 * (venc_rc_log2_q16(have) - venc_rc_log2_q16(ref));
	if (q < (16LL << 16))
		q = 16LL << 16;
	if (q > (51LL << 16))
		q = 51LL << 16;
	return (u32)((q + 0x8000) >> 16);
}

/* Budget window: one second rounded to whole GOPs. */
static inline u32 venc_rc_win_len(const struct venc_rc *rc)
{
	u32 stat = rc->fps, n;

	if (!rc->gop)
		return stat ? stat : 1;
	n = (stat + rc->gop / 2) / rc->gop;
	return (n ? n : 1) * rc->gop;
}

static inline void venc_rc_retarget(struct venc_rc *rc, s64 bitrate, u32 fps)
{
	rc->bitrate = bitrate;
	rc->fps = fps ? fps : 1;
	rc->bpf = div_s64(bitrate << VENC_RC_Q, rc->fps);
	rc->cpb = (bitrate << VENC_RC_Q) * VENC_RC_CPB_SEC;
}

static inline void venc_rc_start_window(struct venc_rc *rc)
{
	u32 n = venc_rc_win_len(rc);

	rc->win_len = n;
	rc->win_pos = 0;
	rc->win_spent = 0;
	/* debt carried in is repaid within this window */
	rc->win_budget = (s64)n * rc->bpf - rc->buf;
	if (rc->win_budget < 0)
		rc->win_budget = 0;
}

static inline u32 venc_rc_idrs_left(const struct venc_rc *rc)
{
	if (!rc->gop || !rc->win_pos || rc->win_pos >= rc->win_len)
		return 0;
	return (rc->win_len - 1) / rc->gop - (rc->win_pos - 1) / rc->gop;
}

static inline void venc_rc_init(struct venc_rc *rc, u32 codec, u32 w, u32 h,
				u32 fps, s64 bitrate, u32 gop, u32 mode,
				u32 qp_min, u32 qp_max)
{
	memset(rc, 0, sizeof(*rc));
	rc->codec = codec;
	rc->w = w;
	rc->h = h;
	rc->mode = mode;
	rc->gop = gop;
	rc->qp_min = qp_min;
	rc->qp_max = qp_max;
	venc_rc_retarget(rc, bitrate, fps);
	rc->qp_seed = venc_rc_clampu(mode == VENC_RC_VBR ? venc_rc_nominal_qp(rc) :
				     venc_rc_seed_qp(rc), qp_min, qp_max);
	rc->qp_i = rc->qp_last = rc->qp_seed;
}

/* New bitrate and/or fps from the next frame on. */
static inline void venc_rc_set_target(struct venc_rc *rc, s64 bitrate, u32 fps)
{
	u32 left;

	if (bitrate == rc->bitrate && fps == rc->fps)
		return;
	venc_rc_retarget(rc, bitrate, fps);
	left = rc->win_len > rc->win_pos ? rc->win_len - rc->win_pos : 0;
	rc->win_budget = rc->win_spent + (s64)left * rc->bpf;
	if (rc->buf > rc->cpb)
		rc->buf = rc->cpb;
	rc->qp_seed = venc_rc_clampu(rc->mode == VENC_RC_VBR ? VENC_RC_VBR_RESTART_QP :
				     venc_rc_seed_qp(rc), rc->qp_min, rc->qp_max);
	rc->retargeted = true;
}

/* P-step law on r = last_bits / last_target, as cross products. */
static inline int venc_rc_step(const struct venc_rc *rc)
{
	s64 b = rc->last_bits * 100, t = rc->last_target;

	if (t <= 0) {
		b = 100;
		t = 1;
	}
	if (rc->mode == VENC_RC_VBR)
		return b >= 105 * t ? 2 : b < 95 * t ? -1 : 0;
	if (b >= 125 * t)
		return 2;
	if (b >= 105 * t)
		return 1;
	if (b >= 95 * t)
		return 0;
	if (b >= 85 * t)
		return -1;
	if (b >= 70 * t || rc->p_count == 1)
		return -2;
	return -1;
}

static inline u32 venc_rc_pick_qp(struct venc_rc *rc, bool is_idr)
{
	u32 qp;

	if (is_idr) {
		if (!rc->frames || rc->win_pos >= rc->win_len ||
		    (rc->gop && !rc->win_pos))
			venc_rc_start_window(rc);
		if (!rc->frames || rc->retargeted) {
			qp = rc->qp_seed;
			rc->retargeted = false;
		} else if (rc->mode == VENC_RC_VBR) {
			qp = venc_rc_clampu((s64)rc->qp_last + 1, rc->qp_min, rc->qp_max);
		} else {
			s64 want = rc->gop_p_n ?
				venc_rc_round_even(rc->gop_p_qp_sum, rc->gop_p_n) : rc->qp_i;
			s64 lo = (s64)rc->qp_i - VENC_RC_I_STEP;
			s64 hi = (s64)rc->qp_i + VENC_RC_I_STEP;

			want -= VENC_RC_I_MEANP_DELTA;
			if (hi > rc->qp_seed)
				hi = rc->qp_seed;
			/* an IDR that took more than half the GOP budget may
			 * climb past the initial QP */
			if (2 * rc->last_idr_bits > (s64)(rc->gop ? rc->gop : rc->fps) * rc->bpf)
				hi = (s64)rc->qp_i + VENC_RC_I_STEP;
			if (hi < lo)
				hi = lo;
			want = clamp(want, lo, hi);
			qp = venc_rc_clampu(want, rc->qp_min, rc->qp_max);
		}
		if (rc->frames)
			rc->gop_index++;
		rc->qp_i = qp;
		rc->since_idr = 0;
		rc->gop_p_qp_sum = 0;
		rc->gop_p_n = 0;
		rc->pending_target = 0;
	} else {
		u32 floor = rc->qp_min, ceil = rc->qp_max, idrs;
		s64 left, target;

		if (rc->win_pos >= rc->win_len)		/* IDR-once rollover */
			venc_rc_start_window(rc);
		/* the P frames left in the window, with the IDRs still due
		 * reserved at the last IDR's size */
		idrs = venc_rc_idrs_left(rc);
		left = (s64)rc->win_len - rc->win_pos - idrs;
		if (left < 1)
			left = 1;
		target = div64_s64(rc->win_budget - rc->win_spent -
				   (s64)idrs * rc->last_idr_bits, left);
		if (rc->mode == VENC_RC_VBR)
			target = div_s64(rc->bpf * 4, 5);
		if (target < div_s64(rc->bpf, 20))
			target = div_s64(rc->bpf, 20);
		if (target > 4 * rc->bpf)
			target = 4 * rc->bpf;
		rc->since_idr++;

		if (rc->mode == VENC_RC_CBR) {
			if (rc->qp_i > VENC_RC_P_FLOOR_DELTA + rc->qp_min)
				floor = rc->qp_i - VENC_RC_P_FLOOR_DELTA;
			if (rc->gop_index && ceil > VENC_RC_P_CEIL_LATER)
				ceil = VENC_RC_P_CEIL_LATER;
		}
		if (rc->since_idr == 1) {
			qp = rc->qp_i + (rc->mode == VENC_RC_CBR ? VENC_RC_FIRST_P_DELTA : 0);
		} else {
			int step = venc_rc_step(rc);

			/* A frame after our own QP decrease that costs >= 2.5x
			 * its predecessor is a static picture refining toward
			 * the new QP, not a rate signal: hold once. */
			if (step > 0 && rc->last_dropped && rc->prev_bits > 0 &&
			    2 * rc->last_bits >= 5 * rc->prev_bits)
				step = 0;
			qp = venc_rc_clampu((s64)rc->qp_last + step, rc->qp_min, rc->qp_max);
		}
		if (qp < floor)
			qp = floor;
		if (qp > ceil)
			qp = ceil;
		rc->pending_target = target;
	}
	qp = venc_rc_clampu(qp, rc->qp_min, rc->qp_max);
	rc->pending_idr = is_idr;
	rc->pending_qp = qp;
	return qp;
}

/* Feed back the coded size (bits, parameter sets included) and real QP. */
static inline void venc_rc_update(struct venc_rc *rc, s64 bits, u32 qp)
{
	bits <<= VENC_RC_Q;
	if (rc->pending_idr) {
		rc->last_idr_bits = bits;
		rc->qp_i = qp;
		rc->last_dropped = false;
		rc->prev_bits = 0;
	} else {
		rc->gop_p_qp_sum += qp;
		rc->gop_p_n++;
		rc->p_count++;
		rc->last_dropped = rc->since_idr > 1 && qp < rc->qp_last;
		rc->prev_bits = rc->since_idr > 1 ? rc->last_bits : 0;
		rc->last_bits = bits;
		rc->last_target = rc->pending_target;
	}
	rc->qp_last = qp;
	rc->win_spent += bits;
	rc->win_pos++;
	rc->buf = clamp(rc->buf + bits - rc->bpf, 0LL, rc->cpb);
	rc->frames++;
}

#endif
