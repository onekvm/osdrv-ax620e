/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Input frame interval for open_vin_capture's G_PARM and ENUM_FRAMEINTERVALS.
 *
 * The HDMI->CSI-2 bridge is not queryable from the capture driver, so the
 * interval comes from the frame-done stamps. Each gap counts as a whole
 * number of frames of the previous window's shortest gap, so a frame-done
 * missed under IRQ latency still counts its frame; the rate is the frame
 * count over a window of at least OVC_FI_WINDOW_NS. It is accepted once two
 * consecutive windows agree within OVC_FI_TOL_PPM, then snaps to n/1 or the
 * NTSC n*1000/1001 Hz when it is that close and is otherwise reported at
 * 1 mHz resolution. A window that disagrees (stall, rate change) keeps the
 * last accepted value. The value outlives STREAMOFF: it is the rate the
 * input had when it was last captured, 0/0 until then.
 *
 * Pure arithmetic under the caller's lock; tests/ builds it on the host.
 */
#ifndef OVC_FRAME_INTERVAL_H
#define OVC_FRAME_INTERVAL_H

#include <linux/gcd.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#define OVC_FI_WINDOW_NS	2000000000ULL
#define OVC_FI_TOL_PPM		500

struct ovc_fi {
	u64 start_ns;		/* window start */
	u64 last_ns;		/* previous frame-done; 0 = none this stream */
	u64 ref_ns;		/* one frame: last window's shortest gap */
	u64 min_ns;		/* shortest gap in this window */
	u32 frames;		/* input frames in this window */
	u32 last_mhz;		/* previous window's rate; 0 = none */
	struct v4l2_fract tpf;	/* accepted interval; 0/0 = unknown */
};

static inline bool ovc_fi_close(u64 mhz, u64 ref)
{
	return abs_diff(mhz, ref) * 1000000 <= ref * OVC_FI_TOL_PPM;
}

static inline struct v4l2_fract ovc_fi_nominal(u32 mhz)
{
	u32 n = DIV_ROUND_CLOSEST(mhz, 1000);
	unsigned long g;

	if (n) {
		u32 ntsc = DIV_ROUND_CLOSEST_ULL(n * 1000000ULL, 1001);
		bool is_ntsc = abs_diff(mhz, ntsc) < abs_diff(mhz, n * 1000);

		if (ovc_fi_close(mhz, is_ntsc ? ntsc : n * 1000))
			return is_ntsc ? (struct v4l2_fract){ 1001, n * 1000 } :
					 (struct v4l2_fract){ 1, n };
	}
	g = gcd(mhz, 1000);
	return (struct v4l2_fract){ 1000 / g, mhz / g };
}

/* A new stream: drop the windows, keep the accepted interval. */
static inline void ovc_fi_start(struct ovc_fi *fi)
{
	struct v4l2_fract tpf = fi->tpf;

	*fi = (struct ovc_fi){ .tpf = tpf };
}

/* One frame-done at @now_ns (monotonic). */
static inline void ovc_fi_sample(struct ovc_fi *fi, u64 now_ns)
{
	u64 gap = now_ns - fi->last_ns, ref, span;
	u32 mhz;

	if (!fi->last_ns) {
		fi->start_ns = fi->last_ns = now_ns;
		return;
	}
	if (!gap)
		return;
	fi->last_ns = now_ns;
	if (!fi->min_ns || gap < fi->min_ns)
		fi->min_ns = gap;
	ref = fi->ref_ns ?: fi->min_ns;
	fi->frames += max_t(u64, 1, div64_u64(gap + ref / 2, ref));

	span = now_ns - fi->start_ns;
	if (span < OVC_FI_WINDOW_NS)
		return;
	mhz = div64_u64((u64)fi->frames * 1000000000000ULL, span);
	if (mhz && fi->last_mhz && ovc_fi_close(mhz, fi->last_mhz))
		fi->tpf = ovc_fi_nominal((mhz + fi->last_mhz) / 2);
	fi->last_mhz = mhz;
	fi->ref_ns = fi->min_ns;
	fi->min_ns = 0;
	fi->frames = 0;
	fi->start_ns = now_ns;
}

#endif /* OVC_FRAME_INTERVAL_H */
