# vcenc headers

These headers are copied unmodified from `third_party/vcenc-ewl` in
[`onekvm-video-ax630c`](https://github.com/onekvm/onekvm-video-ax630c)
(commit `efb892e58a5998a3329d72c5673a9424b3ea6bec`), which imported them from
the `pkgs/vcenc-ewl` implementation in
[GoogleBot42/open-nanokvm-pro](https://github.com/GoogleBot42/open-nanokvm-pro).
Their SPDX identifiers offer `GPL-2.0 OR MIT`; the kernel uses the GPL-2.0
option.

They are the register programs, geometry laws and parameter-set writers the
OneKVM user-space backend already drives through `/dev/es_venc`. Keep them
byte-identical to the backend copy and re-sync both together. `include/`
supplies `<stdint.h>` and `<string.h>` for a kernel build; only
`ax630c_venc_v4l2.c` adds it to its include path.

`vcenc_rc.h` (floating point) is not built into the module.
`ax630c_venc_rc.h` is an integer port of its frame controller; the host test
`tests/venc_rc/` keeps a copy of `vcenc_rc.h` as the reference and checks
both pick the same QP frame by frame.
