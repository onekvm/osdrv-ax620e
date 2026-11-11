# osdrv-ax620e

English | [简体中文](README.zh-CN.md)

OneKVM's own video drivers for NanoKVM Pro (AX630C) and NanoKVM Go (AX620Q), built as out-of-tree kernel modules.

| Module | Source | Purpose |
| --- | --- | --- |
| `open_vin_csi2.ko` | `kernel/open_vin_csi2.c` | MIPI CSI-2 receiver and D-PHY (V4L2 subdev) |
| `open_vin_capture.ko` | `kernel/open_vin_capture.c` | VIN/IFE capture `/dev/video*`; buffers can be exported as dma-buf |
| `ax630c_venc_vcmd.ko` | `kernel/vc8000e/` | VC8000E encoder command engine, frame buffer allocation and dma-buf import |
| `ax630c_venc_v4l2.ko` | `kernel/vc8000e/ax630c_venc_v4l2.c` | Optional V4L2 mem2mem encoder: YUYV input, H.264/H.265 output, for GStreamer `v4l2h264enc`/`v4l2h265enc` and ffmpeg `*_v4l2m2m`; packaged separately as `kernel-module-ax630c-venc-v4l2` |
| `ax630c_tdp.ko` | `kernel/ax630c_tdp.c` | TDP V4L2 mem2mem scaling and color conversion |
| `onekvm_ax630c_npu.ko` | `kernel/npu/` | NPU: power-up, contiguous buffers, OCM mapping, program launch and completion-interrupt wait; `/dev/onekvm-ax-npu`, interface `include/uapi/onekvm/ax630c_npu.h`. Written from observed hardware behavior and contains no vendor code; for user space see the `ax` backend of onekvm-npu |

| Path | Contents |
| --- | --- |
| `tools/ax630c_npu_dtb.py` | Adds the NPU node (`onekvm,ax630c-npu`, registers 0x3800000, OCM 0x3000000, PMU 0x2100000, SPI 38/42) under `/soc` in the Pro DTB; the kernel dts does not describe the NPU, so the distro calls this when packaging the boot DTB |
| `tools/ax620e_media_dtb.py` | Rewrites the Pro kernel DTB: replaces the two private regions with a shared lazy-cma pool, or keeps the legacy separate reusable CMA regions; called by the distro when building the boot DTB |
| `kernel/vc8000e/vcenc/` | VC8000E register and bitstream-header generation code used by the V4L2 encoder, taken verbatim from onekvm-video-ax630c; provenance in `UPSTREAM.md` |
| `tests/` | Host tests: `make -C tests test` (includes DTB rewriting, and frame-by-frame comparison of the encoder rate control against the reference implementation) |

## Memory

On Pro, VIN and VENC share `onekvm-lazy-cma` by default. Capture frames are preferably migrated out of normal memory on demand, and large encoder blocks are preferably taken from the shared 96 MiB reusable CMA pool; freed buffers are kept for reuse and returned to Linux under memory pressure. The default 100 MiB CMA and the fixed 8 MiB command region are unchanged. The machine variables are `ONEKVM_PRO_LAZY_CMA` and `ONEKVM_PRO_LAZY_CMA_POOL_MIB`; the kernel dts is not modified. `--lazy-pool-mib` accepts multiples of 4 MiB from 0 to 96 MiB; 0 is only suitable for experiments that do not rely on large-block backing. 4K30 under stress still needs about 90.9 MiB of CMA, so the default shared pool stays at 96 MiB.

Devices select the shared supplier through the `onekvm,lazy-cma` phandle. VIN vb2 buffers are held and mapped by a dma-buf and exported as-is; VENC still returns bus addresses to user space, and its mmap keeps page references through the dma-buf. `vcmd_core.c` adapts the mmap entry point without modifying the `eswin/` vendor sources. After an allocation is FREEd or the device fd is closed, active VMAs keep holding the buffer; it enters the retained pool only when the last reference is dropped.

Without this phandle, the original device trees remain supported:

- VIN: `dma_alloc_coherent()` allocates from the device CMA region. The buffer-count limit is estimated from actual usage: in a `no-map` region each block is rounded up to a power of two, in a CMA region to `CONFIG_CMA_ALIGNMENT` (1 MiB). At 4096×2160 each block is 17 MiB, so 96 MiB holds 5 blocks (the old 32 MiB-per-block estimate allowed only 3).
- VC8000E: when the region is `reusable`, each `HANTRO_IOCH_ALLOC_FRAMEBUF` calls `dma_alloc_pages()` and freeing returns the pages immediately; when it is still `no-map`, addresses are allocated within the fixed range as before. The user-space ABI is unchanged.
- With `ONEKVM_PRO_MEDIA_REUSABLE=1` and `ONEKVM_PRO_LAZY_CMA=0`, two separate reusable pools are kept; with the former also disabled, the no-map layout is kept. Go keeps its original layout.

`tests/dma-lifetime.c` is an on-device test, built by the distro into the `ax620e-dma-lifetime-test` deploy artifact. With the video service stopped, it checks zeroing, rejection of cross-owner mappings, and active mappings during FREE/close/reclaim. Host layout tests still run with `make -C tests test`.

## V4L2 encoder

`ax630c_venc_v4l2` is an in-kernel client of `ax630c_venc_vcmd`: the VCMD driver registers the child device `ax630c-venc`, on which the encoder creates `/dev/videoN` and shares command scheduling with Core's user-space encoding sessions. The OUTPUT queue supports MMAP and DMABUF import; the CAPTURE queue is MMAP. Working buffers come from the default CMA. Rate control is an integer implementation (`ax630c_venc_rc.h`) that matches `vcenc_rc.h` in onekvm-video-ax630c frame for frame. Statistics are in `/sys/bus/platform/devices/ax630c-venc/stats`.

Zero-copy usage: `v4l2src io-mode=dmabuf ! v4l2h264enc output-io-mode=dmabuf-import` measured 60 fps at 1080p60 on device 105 with about 6% process CPU. ffmpeg 8's `v4l2` input and `*_v4l2m2m` only use MMAP copies, and VIN mappings are uncached; measured about 24.5 fps with about 117% process CPU. VIN does not implement `G_PARM`, so ffmpeg cannot get the input frame rate: `-r 60` inserts duplicate frames whenever encoding falls behind, and the backlog keeps growing; `-fps_mode passthrough` inserts no frames, but the encoder computes rate control and level at 240 fps. Each queue holds at most 16 buffers, all from the default CMA.

These drivers originally lived in `drivers/media/platform/axera/` of `linux-6.18-ax620e`; their commit history moved into this repository along with the directory.

Required kernel options (built-in or modular): `VIDEO_DEV`, `MEDIA_CONTROLLER`, `VIDEO_V4L2_SUBDEV_API`, `V4L2_FWNODE`, `V4L2_MEM2MEM_DEV`, `VIDEOBUF2_CORE`, `VIDEOBUF2_V4L2`, `VIDEOBUF2_MEMOPS`, `VIDEOBUF2_DMA_CONTIG`, `DMA_SHARED_BUFFER`, `MFD_SYSCON`.

See [AGENTS.md](AGENTS.md) for build instructions and constraints.
