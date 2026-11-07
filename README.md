# osdrv-ax620e

NanoKVM Pro（AX630C）与 NanoKVM Go（AX620Q）的自有视频驱动，作为树外内核模块构建。

| 模块 | 源码 | 作用 |
| --- | --- | --- |
| `open_vin_csi2.ko` | `kernel/open_vin_csi2.c` | MIPI CSI-2 接收与 D-PHY（V4L2 subdev） |
| `open_vin_capture.ko` | `kernel/open_vin_capture.c` | VIN/IFE 采集 `/dev/video*`，缓冲可导出 dma-buf |
| `ax630c_venc_vcmd.ko` | `kernel/vc8000e/` | VC8000E 编码命令引擎、帧缓冲分配与 dma-buf 导入 |
| `ax630c_venc_v4l2.ko` | `kernel/vc8000e/ax630c_venc_v4l2.c` | 可选 V4L2 mem2mem 编码器：YUYV 输入，H.264/H.265 输出，供 GStreamer `v4l2h264enc`/`v4l2h265enc`、ffmpeg `*_v4l2m2m` 使用；单独打包为 `kernel-module-ax630c-venc-v4l2` |
| `ax630c_tdp.ko` | `kernel/ax630c_tdp.c` | TDP V4L2 mem2mem 缩放与色彩转换 |
| `onekvm_ax630c_npu.ko` | `kernel/npu/` | NPU：上电、连续缓冲、OCM 映射、启动程序并等完成中断；`/dev/onekvm-ax-npu`，接口 `include/uapi/onekvm/ax630c_npu.h`。按实测硬件行为编写，不含厂商代码；用户态见 onekvm-npu 的 `ax` 后端 |

| 路径 | 内容 |
| --- | --- |
| `tools/ax630c_npu_dtb.py` | 在 Pro DTB 的 `/soc` 下加入 NPU 节点（`onekvm,ax630c-npu`，寄存器 0x3800000、OCM 0x3000000、PMU 0x2100000，SPI 38/42）；内核 dts 不描述 NPU，由 distro 打包启动 DTB 时调用 |
| `tools/ax620e_media_dtb.py` | 改写 Pro 内核 DTB：两个私有区替换为共享 lazy-cma 池，或兼容原有的独立 reusable CMA；由 distro 构建启动 DTB 时调用 |
| `kernel/vc8000e/vcenc/` | V4L2 编码器使用的 VC8000E 寄存器与码流头生成代码，原样取自 onekvm-video-ax630c，来源见 `UPSTREAM.md` |
| `tests/` | 主机测试：`make -C tests test`（含 DTB 改写和编码器码控与参照实现逐帧对拍） |

## 内存

Pro 默认由 VIN 与 VENC 共用 `onekvm-lazy-cma`。采集帧优先从普通内存按需迁出，编码器大块优先从共享的 96 MiB reusable CMA 池取得；释放后保留复用，压力下归还 Linux。默认 100 MiB CMA 与固定 8 MiB 命令区不变。机型变量为 `ONEKVM_PRO_LAZY_CMA` 和 `ONEKVM_PRO_LAZY_CMA_POOL_MIB`，不改内核 dts。 `--lazy-pool-mib` 接受 0–96 MiB 的 4 MiB 倍数；0 只适合不依赖大块后备的实验。4K30 压力下仍需约 90.9 MiB CMA，默认共享池保持 96 MiB。

设备通过 `onekvm,lazy-cma` phandle 选择共享供应者。VIN 的 vb2 缓冲由 dma-buf 持有、映射并原样导出；VENC 仍向用户态返回总线地址，mmap 由 dma-buf 保持页的引用。`vcmd_core.c` 适配映射入口，不修改 `eswin/` 供应商源码。分配 FREE／关闭设备 fd 后，活跃 VMA 继续持有缓冲，最后一个引用关闭才进入保留池。

没有此 phandle 时继续兼容原有设备树：

- VIN：`dma_alloc_coherent()` 从设备 CMA 区分配。缓冲数上限按实际占用估算：`no-map` 区每块按 2 的幂取整，CMA 区按 `CONFIG_CMA_ALIGNMENT`（1 MiB）取整，4096×2160 每块 17 MiB，96 MiB 可放 5 块（原按 32 MiB 估只放 3 块）。
- VC8000E：区域为 `reusable` 时每次 `HANTRO_IOCH_ALLOC_FRAMEBUF` 调用 `dma_alloc_pages()`，释放即归还；仍为 `no-map` 时沿用固定区间内的地址分配。用户态 ABI 不变。
- `ONEKVM_PRO_MEDIA_REUSABLE=1` 且 `ONEKVM_PRO_LAZY_CMA=0` 时保留两个独立 reusable 池；关闭前者则保留 no-map 布局。Go 沿用原有布局。

`tests/dma-lifetime.c` 是实机测试，经 distro 编译为 `ax620e-dma-lifetime-test` 部署产物；停止视频服务后检查清零、跨所有者映射拒绝，以及 FREE／close／reclaim 期间的活跃映射。主机布局测试继续用 `make -C tests test`。

## V4L2 编码器

`ax630c_venc_v4l2` 是 `ax630c_venc_vcmd` 的内核内客户端：VCMD 驱动注册子设备 `ax630c-venc`，编码器在其上建立 `/dev/videoN`，与 Core 的用户态编码会话共享命令调度。OUTPUT 队列支持 MMAP 与 DMABUF 导入，CAPTURE 队列为 MMAP；工作缓冲取自默认 CMA。码控为整数实现（`ax630c_venc_rc.h`），与 onekvm-video-ax630c 的 `vcenc_rc.h` 逐帧一致。统计见 `/sys/bus/platform/devices/ax630c-venc/stats`。

零拷贝用法：`v4l2src io-mode=dmabuf ! v4l2h264enc output-io-mode=dmabuf-import`，105 上 1080p60 实测 60 fps、进程 CPU 约 6%。ffmpeg 8 的 `v4l2` 输入与 `*_v4l2m2m` 只走 MMAP 复制，VIN 映射不经缓存，实测约 24.5 fps、进程 CPU 约 117%。VIN 未实现 `G_PARM`，ffmpeg 拿不到输入帧率：加 `-r 60` 会在跟不上时补重复帧并越积越多；用 `-fps_mode passthrough` 不补帧，但编码器按 240 fps 计算码控和级别。每个队列最多 16 个缓冲，均取自默认 CMA。

这些驱动原在 `linux-6.18-ax620e` 的 `drivers/media/platform/axera/`，历史提交已随目录迁入本仓库。

内核需提供（内置或模块）：`VIDEO_DEV`、`MEDIA_CONTROLLER`、`VIDEO_V4L2_SUBDEV_API`、`V4L2_FWNODE`、`V4L2_MEM2MEM_DEV`、`VIDEOBUF2_CORE`、`VIDEOBUF2_V4L2`、`VIDEOBUF2_MEMOPS`、`VIDEOBUF2_DMA_CONTIG`、`DMA_SHARED_BUFFER`、`MFD_SYSCON`。

构建与约束见 [AGENTS.md](AGENTS.md)。
