# onekvm-ax620e-media

NanoKVM Pro（AX630C）与 NanoKVM Go（AX620Q）的自有视频驱动，作为树外内核模块构建。

| 模块 | 源码 | 作用 |
| --- | --- | --- |
| `open_vin_csi2.ko` | `kernel/open_vin_csi2.c` | MIPI CSI-2 接收与 D-PHY（V4L2 subdev） |
| `open_vin_capture.ko` | `kernel/open_vin_capture.c` | VIN/IFE 采集 `/dev/video*`，缓冲可导出 dma-buf |
| `ax630c_venc_vcmd.ko` | `kernel/vc8000e/` | VC8000E 编码命令引擎、帧缓冲分配与 dma-buf 导入 |
| `ax630c_tdp.ko` | `kernel/ax630c_tdp.c` | TDP V4L2 mem2mem 缩放与色彩转换 |

这些驱动原在 `linux-6.18-ax620e` 的 `drivers/media/platform/axera/`，历史提交已随目录迁入本仓库。

内核需提供（内置或模块）：`VIDEO_DEV`、`MEDIA_CONTROLLER`、`VIDEO_V4L2_SUBDEV_API`、`V4L2_FWNODE`、`V4L2_MEM2MEM_DEV`、`VIDEOBUF2_CORE`、`VIDEOBUF2_V4L2`、`VIDEOBUF2_MEMOPS`、`VIDEOBUF2_DMA_CONTIG`、`DMA_SHARED_BUFFER`、`MFD_SYSCON`。

构建与约束见 [AGENTS.md](AGENTS.md)。
