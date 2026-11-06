/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef AX620E_LAZY_CMA_CLIENT_H
#define AX620E_LAZY_CMA_CLIENT_H

#include <linux/dma-buf.h>
#include <linux/iosys-map.h>
#include <linux/of.h>
#include <linux/scatterlist.h>
#ifdef ONEKVM_LAZY_CMA
#include <onekvm/lazy_cma.h>
#endif

struct ax_lazy_buffer {
	struct dma_buf *dbuf;
	struct dma_buf_attachment *att;
	struct sg_table *sgt;
	struct iosys_map map;
};

static inline bool ax_lazy_enabled(struct device *dev)
{
	return dev && of_property_present(dev->of_node, "onekvm,lazy-cma");
}

static inline int ax_lazy_ready(struct device *dev)
{
#ifdef ONEKVM_LAZY_CMA
	return onekvm_lazy_cma_ready(dev);
#else
	return -EOPNOTSUPP;
#endif
}

static inline void ax_lazy_free(struct ax_lazy_buffer *buf)
{
	if (!buf->dbuf)
		return;
	if (iosys_map_is_set(&buf->map))
		dma_buf_vunmap_unlocked(buf->dbuf, &buf->map);
	dma_buf_unmap_attachment_unlocked(buf->att, buf->sgt, DMA_BIDIRECTIONAL);
	dma_buf_detach(buf->dbuf, buf->att);
	dma_buf_put(buf->dbuf);
	buf->dbuf = NULL;
}

static inline int ax_lazy_alloc(struct device *dev, unsigned long size,
				struct ax_lazy_buffer *buf, bool vmap)
{
#ifdef ONEKVM_LAZY_CMA
	int ret;

	buf->dbuf = onekvm_lazy_cma_alloc(dev, size);
	if (IS_ERR(buf->dbuf)) {
		ret = PTR_ERR(buf->dbuf);
		buf->dbuf = NULL;
		return ret;
	}
	buf->att = dma_buf_attach(buf->dbuf, dev);
	if (IS_ERR(buf->att)) {
		ret = PTR_ERR(buf->att);
		goto err_put;
	}
	buf->sgt = dma_buf_map_attachment_unlocked(buf->att, DMA_BIDIRECTIONAL);
	if (IS_ERR(buf->sgt)) {
		ret = PTR_ERR(buf->sgt);
		goto err_detach;
	}
	if (buf->sgt->nents != 1 || sg_dma_len(buf->sgt->sgl) < size) {
		ret = -EINVAL;
		goto err_unmap;
	}
	if (vmap) {
		ret = dma_buf_vmap_unlocked(buf->dbuf, &buf->map);
		if (ret)
			goto err_unmap;
	}
	return 0;
err_unmap:
	dma_buf_unmap_attachment_unlocked(buf->att, buf->sgt, DMA_BIDIRECTIONAL);
err_detach:
	dma_buf_detach(buf->dbuf, buf->att);
err_put:
	dma_buf_put(buf->dbuf);
	buf->dbuf = NULL;
	return ret;
#else
	return -EOPNOTSUPP;
#endif
}

#endif
