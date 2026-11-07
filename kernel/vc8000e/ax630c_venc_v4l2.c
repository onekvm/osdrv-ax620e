// SPDX-License-Identifier: GPL-2.0-only
/*
 * AX630C VC8000E V4L2 stateful encoder (mem2mem).
 *
 * Packed YUYV in, H.264 or H.265 Annex B out. Each frame is a fixed-QP
 * command buffer built by the vcenc headers the OneKVM backend uses, and is
 * submitted as an in-kernel client of ax630c_venc_vcmd: the core stays
 * shared with /dev/es_venc users instead of being claimed by one of them.
 * Frame-level rate control (ax630c_venc_rc.h) picks each frame's QP.
 *
 * The bitstream goes straight into the CAPTURE buffer. An IDR carries its
 * parameter sets in front of the slice, padded with leading zero bytes to the
 * 8-byte boundary where the hardware starts writing; P frames are bare
 * slices at offset 0. vb2 buffers and the encoder's reconstruction set come
 * from the default CMA area through a child device, never from the VCMD
 * frame-buffer pool the backend allocates from.
 */
#include <linux/dma-mapping.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "ax630c_vcmd_client.h"
#include "ax630c_venc_rc.h"
#include "vcenc_encode.h"
#include "vcenc_header.h"
#include "vcenc_hevc_header.h"

#define VENC_NAME		"ax630c-venc"
#define VENC_MAX_BUFS		16	/* per queue */
#define VENC_DEF_W		1920
#define VENC_DEF_H		1080
#define VENC_PPS_QP		32u	/* pic_init_qp; the core writes slice_qp_delta */
#define VENC_HDR_MAX		192
#define VENC_TIMEOUT_MS		2000
#define VENC_RETRIES		2	/* re-encode at QP + 6 when the slice overflows */
#define VENC_CAP_MIN		SZ_64K
#define VENC_CAP_SLACK		64
#define VENC_MAX_FPS		240
#define VENC_STATUS_REGS	(SUBMODULE_MAIN_ADDR / 2 / 4)
#define VENC_BUFF_FULL		0x020	/* swreg1 ASIC_STATUS_BUFF_FULL */

struct venc_dev {
	struct device *dev;		/* child device: CMA allocations */
	struct mutex lock;
	struct v4l2_device v4l2;
	struct video_device vfd;
	struct v4l2_m2m_dev *m2m;
	struct workqueue_struct *wq;
	struct work_struct work;
	atomic64_t frames, bytes, errors, timeouts, retries, encode_ns, latency_ns;
	u64 latency_max_ns;		/* this stream's worst, OUTPUT QBUF to done */
	u32 last_irq, last_cycles;
	/* OUTPUT QBUF to done in 0.5 ms buckets; the last one is open-ended. */
	atomic_t latency_hist[64];
};

struct venc_buf {
	struct v4l2_m2m_buffer m2m;
	u64 queued_ns;
};

struct venc_ctx {
	struct v4l2_fh fh;
	struct venc_dev *venc;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_pix_format src, dst;
	struct v4l2_fract tpf;
	u32 osequence, csequence;
	bool dst_size_set;		/* CAPTURE sizeimage chosen by the client */

	/* session: valid while OUTPUT streams; used by the job worker only */
	struct file *vcmd;
	vcenc_geom geom;
	void *wset;
	dma_addr_t wset_dma;
	size_t wset_size;
	u32 frame_num;
	bool rc_ready;
	struct venc_rc rc;

	/* control values, read by the worker */
	u32 bitrate, rc_mode, gop;
	bool rc_enable;
	u32 min_qp[2], max_qp[2], i_qp[2], p_qp[2];	/* [0] H.264, [1] HEVC */
	atomic_t force_idr, rc_reset;
};

static const struct v4l2_event venc_eos_event = { .type = V4L2_EVENT_EOS };

static struct venc_ctx *file_ctx(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct venc_ctx, fh);
}

static u32 venc_fps(const struct venc_ctx *ctx)
{
	return clamp_t(u32, DIV_ROUND_CLOSEST(ctx->tpf.denominator, ctx->tpf.numerator),
		       1, VENC_MAX_FPS);
}

static u32 venc_cap_default(u32 w, u32 h)
{
	return ALIGN(w * h + SZ_64K, SZ_4K);
}

static void venc_fill_src(struct v4l2_pix_format *p, const struct v4l2_pix_format_mplane *req)
{
	u32 w = clamp_t(u32, req->width, VCENC_GEOM_MIN_W, VCENC_GEOM_MAX_W) & ~1u;
	u32 h = clamp_t(u32, req->height, VCENC_GEOM_MIN_H, VCENC_GEOM_MAX_H) & ~1u;

	/* The core reads input lines packed at the true width. */
	*p = (struct v4l2_pix_format){
		.width = w, .height = h, .pixelformat = V4L2_PIX_FMT_YUYV,
		.field = V4L2_FIELD_NONE, .bytesperline = w * 2, .sizeimage = w * h * 2,
		.colorspace = req->colorspace, .ycbcr_enc = req->ycbcr_enc,
		.quantization = req->quantization, .xfer_func = req->xfer_func,
	};
	/* The application describes its frames and CAPTURE reports the same.
	 * The stream's VUI always signals limited-range BT.709, which is what
	 * the HDMI bridge delivers and the default here. */
	if (p->colorspace == V4L2_COLORSPACE_DEFAULT) {
		p->colorspace = V4L2_COLORSPACE_REC709;
		p->ycbcr_enc = V4L2_YCBCR_ENC_709;
		p->quantization = V4L2_QUANTIZATION_LIM_RANGE;
		p->xfer_func = V4L2_XFER_FUNC_709;
	}
}

static void venc_fill_dst(struct v4l2_pix_format *p, const struct v4l2_pix_format *src,
			  u32 fourcc, u32 size)
{
	u32 max = ALIGN(2 * src->width * src->height + SZ_64K, SZ_4K);
	u32 min = max_t(u32, ALIGN(src->width * src->height / 4, SZ_4K), VENC_CAP_MIN);

	*p = *src;
	p->pixelformat = fourcc == V4L2_PIX_FMT_HEVC ? V4L2_PIX_FMT_HEVC : V4L2_PIX_FMT_H264;
	p->bytesperline = 0;
	p->sizeimage = size ? clamp_t(u32, ALIGN(size, SZ_4K), min, max) :
		       venc_cap_default(src->width, src->height);
}

static void venc_to_mp(const struct v4l2_pix_format *p, struct v4l2_pix_format_mplane *mp)
{
	*mp = (struct v4l2_pix_format_mplane){
		.width = p->width, .height = p->height, .pixelformat = p->pixelformat,
		.field = p->field, .colorspace = p->colorspace, .num_planes = 1,
		.ycbcr_enc = p->ycbcr_enc, .quantization = p->quantization,
		.xfer_func = p->xfer_func,
		.plane_fmt[0] = { .sizeimage = p->sizeimage, .bytesperline = p->bytesperline },
	};
}

/* ---- rate control and frame encode (job worker only) ------------------- */

static void venc_qp_bounds(struct venc_ctx *ctx, bool hevc, u32 *lo, u32 *hi)
{
	*lo = clamp_t(u32, READ_ONCE(ctx->min_qp[hevc]), VCENC_QP_MIN, VCENC_QP_MAX);
	*hi = clamp_t(u32, READ_ONCE(ctx->max_qp[hevc]), *lo, VCENC_QP_MAX);
}

static int venc_run(struct venc_ctx *ctx, u32 codec, bool idr, u32 qp, u32 in,
		    u32 out, u32 limit, const u8 *slice, u32 *bytes)
{
	struct venc_dev *venc = ctx->venc;
	struct vcenc_frame fr = {
		.frame_num = ctx->frame_num, .qp = qp, .rc_mode = ENC_RC_FIXQP,
		.codec = codec, .pic_init_qp = VENC_PPS_QP,
		.input_phys = in, .output_phys = out,
	};
	vcenc_geom g = ctx->geom;
	u32 *cmd, *regs, st = 0, irq, nbytes, cycles;
	u16 id, size;
	ktime_t t0;
	int ret, nal;

	g.sw9 = limit;			/* output byte limit */
	ret = ax630c_vcmd_reserve(ctx->vcmd, &id);
	if (ret)
		return ret;
	cmd = ax630c_vcmd_cmdbuf(ctx->vcmd, id);
	regs = ax630c_vcmd_status(ctx->vcmd, id);
	if (!cmd || !regs) {
		ret = -EIO;
		goto release;
	}
	regs += VENC_STATUS_REGS;
	memset(regs, 0, 512 * 4);
	/* The working set skips the floorplan's own output area. */
	size = vcenc_build_encode_cmdbuf(cmd, (u32)ctx->wset_dma - ctx->geom.off_s10, &g,
					 ax630c_vcmd_status_bus(id) + SUBMODULE_MAIN_ADDR / 2,
					 &fr);
	if (!size) {
		ret = -EINVAL;
		goto release;
	}
	t0 = ktime_get();
	ret = ax630c_vcmd_link_run(ctx->vcmd, id, size);
	if (ret)
		goto release;
	ret = ax630c_vcmd_wait(ctx->vcmd, id, msecs_to_jiffies(VENC_TIMEOUT_MS), &st);
	if (ret)
		goto release;
	dma_rmb();
	irq = READ_ONCE(regs[1]);
	nbytes = READ_ONCE(regs[9]);
	cycles = READ_ONCE(regs[82]);
	nal = nbytes > (codec == ENC_CODEC_HEVC ? 5u : 4u) ?
	      (codec == ENC_CODEC_HEVC ? (slice[4] >> 1) & 0x3f : slice[4] & 0x1f) : -1;
	ax630c_vcmd_release(ctx->vcmd, id);
	WRITE_ONCE(venc->last_irq, irq);
	WRITE_ONCE(venc->last_cycles, cycles);
	if (st != CMDBUF_EXE_STATUS_OK || !cycles)
		return -EIO;
	if ((irq & VENC_BUFF_FULL) || nbytes > limit)
		return -ENOSPC;
	if (nbytes < 5 || nal != (codec == ENC_CODEC_HEVC ? (idr ? 19 : 1) : (idr ? 5 : 1)))
		return -EIO;
	atomic64_add(ktime_get_ns() - ktime_to_ns(t0), &venc->encode_ns);
	*bytes = nbytes;
	return 0;
release:
	ax630c_vcmd_release(ctx->vcmd, id);
	return ret;
}

static int venc_encode(struct venc_ctx *ctx, struct vb2_v4l2_buffer *src,
		       struct vb2_v4l2_buffer *dst, u32 *bytes, bool *key)
{
	struct venc_dev *venc = ctx->venc;
	bool hevc = ctx->dst.pixelformat == V4L2_PIX_FMT_HEVC;
	u32 codec = hevc ? ENC_CODEC_HEVC : ENC_CODEC_H264;
	u32 w = ctx->geom.w, h = ctx->geom.h, fps = venc_fps(ctx);
	u32 gop = READ_ONCE(ctx->gop), hlen = 0, off = 0, qp, lo, hi, limit, out_bytes = 0;
	size_t cap_size = vb2_plane_size(&dst->vb2_buf, 0);
	u8 *vaddr = vb2_plane_vaddr(&dst->vb2_buf, 0);
	bool rc = READ_ONCE(ctx->rc_enable), idr;
	u8 hdr[VENC_HDR_MAX];
	struct venc_rc next = {};
	dma_addr_t in, out;
	int ret, attempt;

	if (!ctx->vcmd) {
		struct file *client = ax630c_vcmd_client_open();

		if (IS_ERR(client))
			return PTR_ERR(client);
		ctx->vcmd = client;
	}
	if (atomic_xchg(&ctx->force_idr, 0) || (gop && ctx->frame_num >= gop))
		ctx->frame_num = 0;
	idr = !ctx->frame_num;

	venc_qp_bounds(ctx, hevc, &lo, &hi);
	if (rc) {
		s64 bitrate = READ_ONCE(ctx->bitrate);
		u32 mode = READ_ONCE(ctx->rc_mode);

		if (!ctx->rc_ready || atomic_xchg(&ctx->rc_reset, 0) || ctx->rc.codec != codec ||
		    ctx->rc.w != w || ctx->rc.h != h) {
			venc_rc_init(&ctx->rc, codec, w, h, fps, bitrate, gop, mode, lo, hi);
			ctx->rc_ready = true;
		} else {
			venc_rc_set_target(&ctx->rc, bitrate, fps);
			ctx->rc.gop = gop;
		}
		next = ctx->rc;
		qp = venc_rc_pick_qp(&next, idr);
	} else {
		qp = clamp_t(u32, idr ? READ_ONCE(ctx->i_qp[hevc]) : READ_ONCE(ctx->p_qp[hevc]),
			     VCENC_QP_MIN, VCENC_QP_MAX);
	}

	if (idr) {
		if (hevc) {
			hlen = vcenc_write_hevc_headers_fps(hdr, w, h, VENC_PPS_QP,
							    vcenc_hevc_level_idc_fps(w, h, fps), fps);
		} else {
			hlen = vcenc_write_sps_fps(hdr, w, h, fps);
			hlen += vcenc_write_pps(hdr + hlen, VENC_PPS_QP);
		}
		off = ALIGN(hlen, 8);
	}
	if (!vaddr || cap_size < off + VENC_CAP_MIN / 2)
		return -ENOSPC;
	in = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0) + src->vb2_buf.planes[0].data_offset;
	out = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	limit = min_t(size_t, ctx->geom.out_limit, cap_size - off - VENC_CAP_SLACK);

	for (attempt = 0;; attempt++) {
		ret = venc_run(ctx, codec, idr, qp, in, out + off, limit, vaddr + off, &out_bytes);
		if (ret != -ENOSPC || attempt >= VENC_RETRIES || qp >= VCENC_QP_MAX)
			break;
		qp = min_t(u32, qp + 6, VCENC_QP_MAX);
		atomic64_inc(&venc->retries);
	}
	if (ret == -ETIMEDOUT) {
		/* Closing the client aborts and delinks its queued buffer before
		 * either vb2 buffer is handed back. */
		atomic64_inc(&venc->timeouts);
		ax630c_vcmd_client_close(ctx->vcmd);
		ctx->vcmd = NULL;
	}
	if (ret)
		return ret;

	if (idr) {
		memset(vaddr, 0, off - hlen);
		memcpy(vaddr + off - hlen, hdr, hlen);
	}
	*bytes = off + out_bytes;
	*key = idr;
	if (rc) {
		venc_rc_update(&next, (s64)*bytes * 8, qp);
		ctx->rc = next;
	}
	/* frame_num is u(16) in the SPS; wrapping restarts with an IDR. */
	if (++ctx->frame_num > 0xffff)
		ctx->frame_num = 0;
	return 0;
}

static void venc_work(struct work_struct *work)
{
	struct venc_dev *venc = container_of(work, struct venc_dev, work);
	struct venc_ctx *ctx = v4l2_m2m_get_curr_priv(venc->m2m);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(m2m_ctx);
	u32 bytes = 0;
	bool key = false;
	int ret;

	ret = venc_encode(ctx, src, dst, &bytes, &key);
	if (ret) {
		/* The next frame restarts from an IDR. */
		atomic_set(&ctx->force_idr, 1);
		atomic64_inc(&venc->errors);
		dev_err_ratelimited(venc->dev, "frame %u not encoded: %d (status %#x)\n",
				    ctx->osequence, ret, READ_ONCE(venc->last_irq));
	} else {
		u64 lat = ktime_get_ns() -
			  container_of(src, struct venc_buf, m2m.vb)->queued_ns;

		atomic64_inc(&venc->frames);
		atomic64_add(bytes, &venc->bytes);
		atomic64_add(lat, &venc->latency_ns);
		if (lat > READ_ONCE(venc->latency_max_ns))
			WRITE_ONCE(venc->latency_max_ns, lat);
		atomic_inc(&venc->latency_hist[min_t(u64, div_u64(lat, 500000),
						     ARRAY_SIZE(venc->latency_hist) - 1)]);
	}

	src = v4l2_m2m_src_buf_remove(m2m_ctx);
	src->sequence = ctx->osequence++;
	if (ret && !v4l2_m2m_is_last_draining_src_buf(m2m_ctx, src)) {
		/* Drop the frame; the CAPTURE buffer takes the next one. */
		v4l2_m2m_buf_done(src, VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(venc->m2m, m2m_ctx);
		return;
	}
	dst = v4l2_m2m_dst_buf_remove(m2m_ctx);
	v4l2_m2m_buf_copy_metadata(src, dst, false);
	dst->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
			V4L2_BUF_FLAG_BFRAME | V4L2_BUF_FLAG_LAST);
	if (!ret)
		dst->flags |= key ? V4L2_BUF_FLAG_KEYFRAME : V4L2_BUF_FLAG_PFRAME;
	dst->field = V4L2_FIELD_NONE;
	dst->sequence = ctx->csequence++;
	vb2_set_plane_payload(&dst->vb2_buf, 0, ret ? 0 : bytes);
	if (v4l2_m2m_is_last_draining_src_buf(m2m_ctx, src)) {
		dst->flags |= V4L2_BUF_FLAG_LAST;
		v4l2_event_queue_fh(&ctx->fh, &venc_eos_event);
		v4l2_m2m_mark_stopped(m2m_ctx);
	}
	/* A failed last frame still ends the drain with an empty LAST buffer. */
	v4l2_m2m_buf_done(src, ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(dst, VB2_BUF_STATE_DONE);
	v4l2_m2m_job_finish(venc->m2m, m2m_ctx);
}

static void venc_device_run(void *priv)
{
	struct venc_ctx *ctx = priv;

	queue_work(ctx->venc->wq, &ctx->venc->work);
}

static void venc_job_abort(void *priv)
{
	struct venc_ctx *ctx = priv;

	/* A job ends within the completion timeout. */
	flush_work(&ctx->venc->work);
}

static const struct v4l2_m2m_ops venc_m2m_ops = {
	.device_run = venc_device_run, .job_abort = venc_job_abort,
};

/* ---- session ----------------------------------------------------------- */

static void venc_session_stop(struct venc_ctx *ctx)
{
	ax630c_vcmd_client_close(ctx->vcmd);
	ctx->vcmd = NULL;
	if (ctx->wset)
		dma_free_attrs(ctx->venc->dev, ctx->wset_size, ctx->wset, ctx->wset_dma,
			       DMA_ATTR_NO_KERNEL_MAPPING);
	ctx->wset = NULL;
}

static int venc_session_start(struct venc_ctx *ctx)
{
	struct device *dev = ctx->venc->dev;
	struct file *client;

	if (vcenc_geom_build_ex(&ctx->geom, ctx->src.width, ctx->src.height, 0))
		return -EINVAL;
	/* Reconstruction, reference and side buffers; the output area at the
	 * head of the floorplan is replaced by the CAPTURE buffer. */
	ctx->wset_size = PAGE_ALIGN(ctx->geom.span - ctx->geom.off_s10);
	ctx->wset = dma_alloc_attrs(dev, ctx->wset_size, &ctx->wset_dma, GFP_KERNEL,
				    DMA_ATTR_NO_KERNEL_MAPPING);
	if (!ctx->wset)
		return -ENOMEM;
	client = ax630c_vcmd_client_open();
	if (IS_ERR(client)) {
		venc_session_stop(ctx);
		return PTR_ERR(client);
	}
	ctx->vcmd = client;
	ctx->frame_num = 0;
	ctx->rc_ready = false;
	atomic_set(&ctx->force_idr, 1);
	return 0;
}

/* ---- vb2 --------------------------------------------------------------- */

static int venc_queue_setup(struct vb2_queue *q, unsigned int *nbufs,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct venc_ctx *ctx = vb2_get_drv_priv(q);
	u32 size = V4L2_TYPE_IS_OUTPUT(q->type) ? ctx->src.sizeimage : ctx->dst.sizeimage;
	unsigned int have = vb2_get_num_buffers(q);

	/*
	 * Both queues allocate from the default CMA area other drivers share;
	 * 16 frames each is ample for this pipeline. vb2 itself cannot be
	 * limited below 32 (max_num_buffers), so trim each request here.
	 */
	if (have >= VENC_MAX_BUFS)
		return -ENOBUFS;
	*nbufs = min(*nbufs, VENC_MAX_BUFS - have);
	if (*nplanes)
		return *nplanes != 1 || sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int venc_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;
	return vbuf->field == V4L2_FIELD_NONE ? 0 : -EINVAL;
}

static int venc_buf_prepare(struct vb2_buffer *vb)
{
	struct venc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	u64 addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	u32 off = vb->planes[0].data_offset;

	if (V4L2_TYPE_IS_OUTPUT(vb->type)) {
		u32 size = ctx->src.sizeimage;

		if (vb2_plane_size(vb, 0) < (u64)size + off || (off & 15) ||
		    addr + off + size > SZ_4G ||
		    vb2_get_plane_payload(vb, 0) < (u64)size + off)
			return -EINVAL;
	} else if (vb2_plane_size(vb, 0) < VENC_CAP_MIN ||
		   addr + vb2_plane_size(vb, 0) > SZ_4G) {
		return -EINVAL;
	}
	return 0;
}

static void venc_buf_queue(struct vb2_buffer *vb)
{
	struct venc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (V4L2_TYPE_IS_CAPTURE(vb->type) && vb2_is_streaming(vb->vb2_queue) &&
	    v4l2_m2m_dst_buf_is_last(ctx->fh.m2m_ctx)) {
		vb2_set_plane_payload(vb, 0, 0);
		vbuf->field = V4L2_FIELD_NONE;
		vbuf->sequence = ctx->csequence++;
		v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx, vbuf);
		v4l2_event_queue_fh(&ctx->fh, &venc_eos_event);
		return;
	}
	if (V4L2_TYPE_IS_OUTPUT(vb->type))
		container_of(vbuf, struct venc_buf, m2m.vb)->queued_ns = ktime_get_ns();
	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static void venc_return_bufs(struct venc_ctx *ctx, u32 type, enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *b;

	while ((b = V4L2_TYPE_IS_OUTPUT(type) ? v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		    v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(b, state);
}

static int venc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct venc_ctx *ctx = vb2_get_drv_priv(q);
	int ret = 0;

	v4l2_m2m_update_start_streaming_state(ctx->fh.m2m_ctx, q);
	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		ctx->osequence = 0;
		WRITE_ONCE(ctx->venc->latency_max_ns, 0);
		ret = venc_session_start(ctx);
	} else {
		/* A new CAPTURE stream (possibly another codec) starts clean. */
		ctx->csequence = 0;
		ctx->rc_ready = false;
		atomic_set(&ctx->force_idr, 1);
	}
	if (ret)
		venc_return_bufs(ctx, q->type, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void venc_stop_streaming(struct vb2_queue *q)
{
	struct venc_ctx *ctx = vb2_get_drv_priv(q);

	/* m2m STREAMOFF has already waited for a running job. */
	venc_return_bufs(ctx, q->type, VB2_BUF_STATE_ERROR);
	v4l2_m2m_update_stop_streaming_state(ctx->fh.m2m_ctx, q);
	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		venc_session_stop(ctx);
		if (v4l2_m2m_has_stopped(ctx->fh.m2m_ctx))
			v4l2_event_queue_fh(&ctx->fh, &venc_eos_event);
	}
}

static const struct vb2_ops venc_qops = {
	.queue_setup = venc_queue_setup, .buf_out_validate = venc_buf_out_validate,
	.buf_prepare = venc_buf_prepare,
	.buf_queue = venc_buf_queue, .start_streaming = venc_start_streaming,
	.stop_streaming = venc_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare, .wait_finish = vb2_ops_wait_finish,
};

static int venc_queue_init(void *priv, struct vb2_queue *src, struct vb2_queue *dst)
{
	struct venc_ctx *ctx = priv;
	struct vb2_queue *queues[] = { src, dst };
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(queues); i++) {
		struct vb2_queue *q = queues[i];

		q->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE : V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		/* The driver writes parameter sets through the CAPTURE
		 * buffer's kernel mapping; OUTPUT needs none. */
		q->io_modes = i ? VB2_MMAP : VB2_MMAP | VB2_DMABUF;
		q->dma_attrs = i ? 0 : DMA_ATTR_NO_KERNEL_MAPPING;
		q->drv_priv = ctx;
		q->buf_struct_size = sizeof(struct venc_buf);
		q->ops = &venc_qops;
		q->mem_ops = &vb2_dma_contig_memops;
		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		q->lock = &ctx->venc->lock;
		q->dev = ctx->venc->dev;
		ret = vb2_queue_init(q);
		if (ret)
			return ret;
	}
	return 0;
}

/* ---- ioctls ------------------------------------------------------------ */

static int venc_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, VENC_NAME, sizeof(cap->driver));
	strscpy(cap->card, "AX630C VC8000E", sizeof(cap->card));
	strscpy(cap->bus_info, "platform:" VENC_NAME, sizeof(cap->bus_info));
	return 0;
}

static int venc_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		if (f->index)
			return -EINVAL;
		f->pixelformat = V4L2_PIX_FMT_YUYV;
	} else {
		if (f->index > 1)
			return -EINVAL;
		f->pixelformat = f->index ? V4L2_PIX_FMT_HEVC : V4L2_PIX_FMT_H264;
	}
	return 0;
}

static int venc_enum_framesizes(struct file *file, void *priv, struct v4l2_frmsizeenum *fs)
{
	if (fs->index || (fs->pixel_format != V4L2_PIX_FMT_YUYV &&
			  fs->pixel_format != V4L2_PIX_FMT_H264 &&
			  fs->pixel_format != V4L2_PIX_FMT_HEVC))
		return -EINVAL;
	fs->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fs->stepwise = (struct v4l2_frmsize_stepwise){
		VCENC_GEOM_MIN_W, VCENC_GEOM_MAX_W, 2,
		VCENC_GEOM_MIN_H, VCENC_GEOM_MAX_H, 2,
	};
	return 0;
}

static int venc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct venc_ctx *ctx = file_ctx(file);

	venc_to_mp(V4L2_TYPE_IS_OUTPUT(f->type) ? &ctx->src : &ctx->dst, &f->fmt.pix_mp);
	return 0;
}

static int venc_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct venc_ctx *ctx = file_ctx(file);
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;
	struct v4l2_pix_format p;

	if (V4L2_TYPE_IS_OUTPUT(f->type))
		venc_fill_src(&p, mp);
	else
		/* The coded size follows OUTPUT. */
		venc_fill_dst(&p, &ctx->src, mp->pixelformat, mp->plane_fmt[0].sizeimage);
	venc_to_mp(&p, mp);
	return 0;
}

static int venc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct venc_ctx *ctx = file_ctx(file);
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct v4l2_pix_format_mplane *mp = &f->fmt.pix_mp;

	if (vb2_is_busy(v4l2_m2m_get_vq(m2m_ctx, f->type)))
		return -EBUSY;
	venc_try_fmt(file, priv, f);
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		venc_fill_src(&ctx->src, mp);
		/* Keep a CAPTURE size the client chose or allocated, within
		 * the new bounds; otherwise use the new default. */
		venc_fill_dst(&ctx->dst, &ctx->src, ctx->dst.pixelformat,
			      ctx->dst_size_set || vb2_is_busy(v4l2_m2m_get_dst_vq(m2m_ctx)) ?
			      ctx->dst.sizeimage : 0);
	} else {
		ctx->dst_size_set = mp->plane_fmt[0].sizeimage != 0;
		venc_fill_dst(&ctx->dst, &ctx->src, mp->pixelformat, mp->plane_fmt[0].sizeimage);
	}
	return 0;
}

static int venc_g_selection(struct file *file, void *priv, struct v4l2_selection *s)
{
	struct venc_ctx *ctx = file_ctx(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r = (struct v4l2_rect){ 0, 0, ctx->src.width, ctx->src.height };
		return 0;
	default:
		return -EINVAL;
	}
}

static int venc_s_selection(struct file *file, void *priv, struct v4l2_selection *s)
{
	struct venc_ctx *ctx = file_ctx(file);

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT || s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;
	/* The core encodes the whole input frame. */
	s->r = (struct v4l2_rect){ 0, 0, ctx->src.width, ctx->src.height };
	return 0;
}

static int venc_g_parm(struct file *file, void *priv, struct v4l2_streamparm *a)
{
	struct venc_ctx *ctx = file_ctx(file);

	/* Frame rate of the raw input; the coded rate is the same. */
	if (!V4L2_TYPE_IS_OUTPUT(a->type))
		return -EINVAL;
	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.output.timeperframe = ctx->tpf;
	return 0;
}

static int venc_s_parm(struct file *file, void *priv, struct v4l2_streamparm *a)
{
	struct venc_ctx *ctx = file_ctx(file);
	struct v4l2_fract *t = &a->parm.output.timeperframe;

	if (!V4L2_TYPE_IS_OUTPUT(a->type))
		return -EINVAL;
	if (t->numerator && t->denominator) {
		if (t->denominator > (u64)t->numerator * VENC_MAX_FPS)
			*t = (struct v4l2_fract){ 1, VENC_MAX_FPS };
		else if (t->denominator < t->numerator)
			*t = (struct v4l2_fract){ 1, 1 };
		ctx->tpf = *t;
	}
	return venc_g_parm(file, priv, a);
}

static int venc_encoder_cmd(struct file *file, void *priv, struct v4l2_encoder_cmd *ec)
{
	struct venc_ctx *ctx = file_ctx(file);
	int ret;

	ret = v4l2_m2m_ioctl_try_encoder_cmd(file, priv, ec);
	if (ret < 0)
		return ret;
	if (!vb2_is_streaming(&ctx->fh.m2m_ctx->out_q_ctx.q))
		return 0;
	ret = v4l2_m2m_ioctl_encoder_cmd(file, priv, ec);
	if (ret < 0)
		return ret;
	if (ec->cmd == V4L2_ENC_CMD_STOP && v4l2_m2m_has_stopped(ctx->fh.m2m_ctx))
		v4l2_event_queue_fh(&ctx->fh, &venc_eos_event);
	if (ec->cmd == V4L2_ENC_CMD_START && v4l2_m2m_has_stopped(ctx->fh.m2m_ctx))
		vb2_clear_last_buffer_dequeued(&ctx->fh.m2m_ctx->cap_q_ctx.q);
	if (ec->cmd == V4L2_ENC_CMD_START)
		atomic_set(&ctx->force_idr, 1);
	return 0;
}

static int venc_subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub)
{
	if (sub->type == V4L2_EVENT_EOS)
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	return v4l2_ctrl_subscribe_event(fh, sub);
}

static const struct v4l2_ioctl_ops venc_ioctl_ops = {
	.vidioc_querycap = venc_querycap,
	.vidioc_enum_fmt_vid_out = venc_enum_fmt, .vidioc_enum_fmt_vid_cap = venc_enum_fmt,
	.vidioc_enum_framesizes = venc_enum_framesizes,
	.vidioc_g_fmt_vid_out_mplane = venc_g_fmt, .vidioc_g_fmt_vid_cap_mplane = venc_g_fmt,
	.vidioc_try_fmt_vid_out_mplane = venc_try_fmt, .vidioc_try_fmt_vid_cap_mplane = venc_try_fmt,
	.vidioc_s_fmt_vid_out_mplane = venc_s_fmt, .vidioc_s_fmt_vid_cap_mplane = venc_s_fmt,
	.vidioc_g_selection = venc_g_selection, .vidioc_s_selection = venc_s_selection,
	.vidioc_g_parm = venc_g_parm, .vidioc_s_parm = venc_s_parm,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs, .vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf, .vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon, .vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_try_encoder_cmd = v4l2_m2m_ioctl_try_encoder_cmd,
	.vidioc_encoder_cmd = venc_encoder_cmd,
	.vidioc_subscribe_event = venc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

/* ---- controls ---------------------------------------------------------- */

static int venc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct venc_ctx *ctx = container_of(ctrl->handler, struct venc_ctx, hdl);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		WRITE_ONCE(ctx->bitrate, ctrl->val);
		return 0;
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		WRITE_ONCE(ctx->rc_mode, ctrl->val == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR ?
			   VENC_RC_VBR : VENC_RC_CBR);
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		WRITE_ONCE(ctx->rc_enable, ctrl->val);
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		WRITE_ONCE(ctx->gop, ctrl->val);
		return 0;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		atomic_set(&ctx->force_idr, 1);
		return 0;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
		WRITE_ONCE(ctx->min_qp[0], ctrl->val);
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
		WRITE_ONCE(ctx->max_qp[0], ctrl->val);
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
		WRITE_ONCE(ctx->i_qp[0], ctrl->val);
		return 0;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
		WRITE_ONCE(ctx->p_qp[0], ctrl->val);
		return 0;
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP:
		WRITE_ONCE(ctx->min_qp[1], ctrl->val);
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP:
		WRITE_ONCE(ctx->max_qp[1], ctrl->val);
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP:
		WRITE_ONCE(ctx->i_qp[1], ctrl->val);
		return 0;
	case V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP:
		WRITE_ONCE(ctx->p_qp[1], ctrl->val);
		return 0;
	default:
		/* fixed: B frames, header mode, profiles */
		return 0;
	}
	/* rate-control mode or QP bounds changed: restart the controller */
	atomic_set(&ctx->rc_reset, 1);
	return 0;
}

/* The level the parameter sets carry for the current size and frame rate. */
static s32 venc_level(struct venc_ctx *ctx, u32 id)
{
	u32 w = ctx->src.width, h = ctx->src.height, fps = venc_fps(ctx), idc;
	u8 sps[VENC_HDR_MAX];

	if (id == V4L2_CID_MPEG_VIDEO_HEVC_LEVEL) {
		idc = vcenc_hevc_level_idc_fps(w, h, fps);
		return idc == 123 ? V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1 :
		       idc == 150 ? V4L2_MPEG_VIDEO_HEVC_LEVEL_5 :
		       idc == 153 ? V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1 :
		       V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2;
	}
	/* start code, NAL header, profile_idc, constraint flags, level_idc */
	vcenc_write_sps_fps(sps, w, h, fps);
	idc = sps[7];
	return idc == 30 ? V4L2_MPEG_VIDEO_H264_LEVEL_3_0 :
	       idc == 31 ? V4L2_MPEG_VIDEO_H264_LEVEL_3_1 :
	       idc == 32 ? V4L2_MPEG_VIDEO_H264_LEVEL_3_2 :
	       idc == 40 ? V4L2_MPEG_VIDEO_H264_LEVEL_4_0 :
	       idc == 41 ? V4L2_MPEG_VIDEO_H264_LEVEL_4_1 :
	       idc == 42 ? V4L2_MPEG_VIDEO_H264_LEVEL_4_2 :
	       idc == 50 ? V4L2_MPEG_VIDEO_H264_LEVEL_5_0 :
	       V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
}

static int venc_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct venc_ctx *ctx = container_of(ctrl->handler, struct venc_ctx, hdl);

	ctrl->val = venc_level(ctx, ctrl->id);
	return 0;
}

/* Level follows the format: a requested level reads back as the one used. */
static int venc_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct venc_ctx *ctx = container_of(ctrl->handler, struct venc_ctx, hdl);

	if (ctrl->id == V4L2_CID_MPEG_VIDEO_H264_LEVEL ||
	    ctrl->id == V4L2_CID_MPEG_VIDEO_HEVC_LEVEL)
		ctrl->val = venc_level(ctx, ctrl->id);
	return 0;
}

static const struct v4l2_ctrl_ops venc_ctrl_ops = {
	.s_ctrl = venc_s_ctrl, .try_ctrl = venc_try_ctrl,
	.g_volatile_ctrl = venc_g_volatile_ctrl,
};

static int venc_ctrls_init(struct venc_ctx *ctx)
{
	struct v4l2_ctrl_handler *hdl = &ctx->hdl;
	const struct v4l2_ctrl_ops *ops = &venc_ctrl_ops;
	static const u32 qp_ids[2][4] = {
		{ V4L2_CID_MPEG_VIDEO_H264_MIN_QP, V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
		  V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP, V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP },
		{ V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP, V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP,
		  V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP, V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP },
	};
	static const s32 qp_def[4] = { VCENC_QP_MIN, VCENC_QP_MAX, 32, 32 };
	struct v4l2_ctrl *lvl[2];
	unsigned int c, i;

	v4l2_ctrl_handler_init(hdl, 21);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MIN_BUFFERS_FOR_OUTPUT, 1, 1, 1, 1);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_BITRATE, 100000, 50000000, 1, 8000000);
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR, 0,
			       V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, 0, 1, 1, 1);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_GOP_SIZE, 0, 65535, 1, 60);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_B_FRAMES, 0, 0, 1, 0);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 0, 0, 0, 0);
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_MPEG_VIDEO_HEADER_MODE,
			       V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
			       BIT(V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE),
			       V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR, 1, 1, 1, 1);
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			       V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
			       BIT(V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE) |
			       BIT(V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE),
			       V4L2_MPEG_VIDEO_H264_PROFILE_MAIN);
	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN, 0,
			       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);
	/* Follows the size and frame rate. */
	lvl[0] = v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_MPEG_VIDEO_H264_LEVEL,
					V4L2_MPEG_VIDEO_H264_LEVEL_6_2, 0,
					V4L2_MPEG_VIDEO_H264_LEVEL_4_2);
	lvl[1] = v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
					V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2, 0,
					V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1);
	for (i = 0; i < 2; i++)
		if (lvl[i])
			lvl[i]->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
	for (c = 0; c < 2; c++)
		for (i = 0; i < 4; i++)
			v4l2_ctrl_new_std(hdl, ops, qp_ids[c][i], 0, 51, 1, qp_def[i]);
	if (hdl->error) {
		int ret = hdl->error;

		v4l2_ctrl_handler_free(hdl);
		return ret;
	}
	return v4l2_ctrl_handler_setup(hdl);
}

/* ---- file operations --------------------------------------------------- */

static int venc_open(struct file *file)
{
	struct venc_dev *venc = video_drvdata(file);
	struct venc_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->venc = venc;
	venc_fill_src(&ctx->src, &(struct v4l2_pix_format_mplane){
		.width = VENC_DEF_W, .height = VENC_DEF_H });
	venc_fill_dst(&ctx->dst, &ctx->src, V4L2_PIX_FMT_H264, 0);
	ctx->tpf = (struct v4l2_fract){ 1, 30 };
	atomic_set(&ctx->force_idr, 1);
	ret = venc_ctrls_init(ctx);
	if (ret) {
		kfree(ctx);
		return ret;
	}
	v4l2_fh_init(&ctx->fh, &venc->vfd);
	ctx->fh.ctrl_handler = &ctx->hdl;
	file->private_data = &ctx->fh;
	mutex_lock(&venc->lock);
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(venc->m2m, ctx, venc_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		mutex_unlock(&venc->lock);
		v4l2_fh_exit(&ctx->fh);
		v4l2_ctrl_handler_free(&ctx->hdl);
		kfree(ctx);
		return ret;
	}
	v4l2_fh_add(&ctx->fh, file);
	mutex_unlock(&venc->lock);
	return 0;
}

static int venc_release(struct file *file)
{
	struct venc_ctx *ctx = file_ctx(file);
	struct venc_dev *venc = ctx->venc;

	mutex_lock(&venc->lock);
	/* Stops both queues; OUTPUT stop ends the session. */
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&venc->lock);
	venc_session_stop(ctx);
	v4l2_ctrl_handler_free(&ctx->hdl);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations venc_fops = {
	.owner = THIS_MODULE, .open = venc_open, .release = venc_release,
	.poll = v4l2_m2m_fop_poll, .unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

/* ---- device ------------------------------------------------------------ */

static ssize_t stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct venc_dev *venc = dev_get_drvdata(dev);
	u64 frames = atomic64_read(&venc->frames);
	u64 enc = atomic64_read(&venc->encode_ns), lat = atomic64_read(&venc->latency_ns);
	int len, i;

	/* Totals (*_sum, the histogram) give a test window's figures from deltas. */
	len = sysfs_emit(buf,
			  "frames %llu\nbytes %llu\nerrors %lld\ntimeouts %lld\nretries %lld\n"
			  "encode_us_avg %llu\nencode_us_sum %llu\n"
			  "latency_us_avg %llu\nlatency_us_sum %llu\nlatency_us_max %llu\n"
			  "last_status %#x\nlast_cycles %u\n",
			  frames, (u64)atomic64_read(&venc->bytes),
			  atomic64_read(&venc->errors), atomic64_read(&venc->timeouts),
			  atomic64_read(&venc->retries),
			  frames ? div64_u64(enc, frames * 1000) : 0, div64_u64(enc, 1000),
			  frames ? div64_u64(lat, frames * 1000) : 0, div64_u64(lat, 1000),
			  div64_u64(READ_ONCE(venc->latency_max_ns), 1000),
			  READ_ONCE(venc->last_irq), READ_ONCE(venc->last_cycles));
	len += sysfs_emit_at(buf, len, "latency_hist_500us");
	for (i = 0; i < ARRAY_SIZE(venc->latency_hist); i++)
		len += sysfs_emit_at(buf, len, " %d", atomic_read(&venc->latency_hist[i]));
	len += sysfs_emit_at(buf, len, "\n");
	return len;
}
static DEVICE_ATTR_RO(stats);
static struct attribute *venc_attrs[] = { &dev_attr_stats.attr, NULL };
ATTRIBUTE_GROUPS(venc);

/* Last reference: the device is gone and no file is open. */
static void venc_v4l2_release(struct v4l2_device *v4l2)
{
	struct venc_dev *venc = container_of(v4l2, struct venc_dev, v4l2);

	v4l2_device_unregister(&venc->v4l2);
	v4l2_m2m_release(venc->m2m);
	destroy_workqueue(venc->wq);
	mutex_destroy(&venc->lock);
	kfree(venc);
}

static int venc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct venc_dev *venc;
	int ret;

	/* The VCMD driver registers this device as its child. */
	if (!ax630c_vcmd_device())
		return -EPROBE_DEFER;
	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	venc = kzalloc(sizeof(*venc), GFP_KERNEL);
	if (!venc)
		return -ENOMEM;
	venc->dev = dev;
	mutex_init(&venc->lock);
	INIT_WORK(&venc->work, venc_work);
	venc->wq = alloc_ordered_workqueue("ax630c-venc", WQ_HIGHPRI);
	if (!venc->wq) {
		ret = -ENOMEM;
		goto err_free;
	}
	venc->m2m = v4l2_m2m_init(&venc_m2m_ops);
	if (IS_ERR(venc->m2m)) {
		ret = PTR_ERR(venc->m2m);
		goto err_wq;
	}
	ret = v4l2_device_register(dev, &venc->v4l2);
	if (ret)
		goto err_m2m;
	venc->v4l2.release = venc_v4l2_release;
	strscpy(venc->vfd.name, VENC_NAME, sizeof(venc->vfd.name));
	venc->vfd.v4l2_dev = &venc->v4l2;
	venc->vfd.fops = &venc_fops;
	venc->vfd.ioctl_ops = &venc_ioctl_ops;
	venc->vfd.release = video_device_release_empty;
	venc->vfd.lock = &venc->lock;
	venc->vfd.vfl_dir = VFL_DIR_M2M;
	venc->vfd.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(&venc->vfd, venc);
	platform_set_drvdata(pdev, venc);
	ret = video_register_device(&venc->vfd, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_device_put(&venc->v4l2);	/* frees through the release */
		return ret;
	}
	dev_info(dev, "V4L2 YUYV to H.264/H.265 encoder as %s\n",
		 video_device_node_name(&venc->vfd));
	return 0;
err_m2m:
	v4l2_m2m_release(venc->m2m);
err_wq:
	destroy_workqueue(venc->wq);
err_free:
	mutex_destroy(&venc->lock);
	kfree(venc);
	return ret;
}

static void venc_remove(struct platform_device *pdev)
{
	struct venc_dev *venc = platform_get_drvdata(pdev);

	/* Open files keep the context until they close; ioctls now fail. */
	v4l2_device_disconnect(&venc->v4l2);
	video_unregister_device(&venc->vfd);
	v4l2_device_put(&venc->v4l2);
}

static struct platform_driver venc_driver = {
	.probe = venc_probe, .remove = venc_remove,
	.driver = {
		.name = VENC_NAME, .dev_groups = venc_groups,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(venc_driver);

MODULE_DESCRIPTION("Axera AX630C VC8000E V4L2 H.264/H.265 encoder");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" VENC_NAME);
