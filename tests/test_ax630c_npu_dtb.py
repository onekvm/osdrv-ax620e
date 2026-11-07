#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""ax630c_npu_dtb.py adds one NPU node under /soc and keeps the rest."""
import importlib.util
import pathlib
import struct
import sys
import tempfile

TOOL = pathlib.Path(__file__).resolve().parent.parent / 'tools' / 'ax630c_npu_dtb.py'
spec = importlib.util.spec_from_file_location('npu_dtb', TOOL)
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)

be = t.be32
root = ['', [('#address-cells', be(2)), ('#size-cells', be(2)), ('interrupt-parent', be(1)),
             ('model', b'test\0')],
        [['soc', [('compatible', b'simple-bus\0'), ('#address-cells', be(2)),
                  ('#size-cells', be(2)), ('ranges', b'')],
          [['serial@4880000', [('reg', be(0, 0x4880000, 0, 0x1000))], []]]],
         ['memory@40000000', [('device_type', b'memory\0')], []]]]
dtb = t.build(root, [(0x40000000, 0x1000)], 0)

with tempfile.TemporaryDirectory() as d:
    a, b, c = (pathlib.Path(d) / n for n in ('in.dtb', 'out.dtb', 'again.dtb'))
    a.write_bytes(dtb)
    sys.argv = ['x', str(a), str(b)]
    t.main()
    sys.argv = ['x', str(b), str(c)]
    t.main()
    assert b.read_bytes() == c.read_bytes(), 'not idempotent'
    out, rsv, _ = t.parse(b.read_bytes())
    soc = [n for n in out[2] if n[0] == 'soc'][0]
    names = [n[0] for n in soc[2]]
    assert names == ['serial@4880000', 'npu@3800000'], names
    npu = dict(soc[2][1][1])
    assert npu['compatible'] == b'onekvm,ax630c-npu\0'
    assert struct.unpack('>12I', npu['reg'])[1::2][::2] == (0x3800000, 0x3000000, 0x2100000)
    assert npu['interrupts'] == be(0, 38, 1, 0, 42, 1)
    assert rsv == [(0x40000000, 0x1000)]
    out[2][0][2].pop()
    assert out == root, 'other nodes changed'
print('ok   ax630c_npu_dtb adds the NPU node under /soc, idempotent')
