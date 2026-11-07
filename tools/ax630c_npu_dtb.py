#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Add the AX630C NPU node to a built NanoKVM Pro DTB.

The kernel device tree does not describe the NPU; the open driver
(kernel/npu, onekvm,ax630c-npu) binds to this node:

    npu@3800000 {
        compatible = "onekvm,ax630c-npu";
        reg = <0 0x3800000 0 0xb0000>, <0 0x3000000 0 0x280000>,
              <0 0x2100000 0 0x1000>;
        reg-names = "regs", "ocm", "pmu";
        interrupts = <GIC_SPI 38 IRQ_TYPE_EDGE_RISING>,
                     <GIC_SPI 42 IRQ_TYPE_EDGE_RISING>;
        interrupt-names = "done", "error";
    };

The node goes under /soc (a simple bus with an identity mapping and two
address and size cells; the GIC is the root's interrupt parent). An
existing npu@3800000 node is replaced.

usage: ax630c_npu_dtb.py IN.dtb OUT.dtb
"""
import struct
import sys

MAGIC = 0xD00DFEED
BEGIN, END, PROP, NOP, FDT_END = 1, 2, 3, 4, 9
NODE = 'npu@3800000'


def be32(*v):
    return struct.pack('>%dI' % len(v), *v)


NPU_PROPS = [
    ('compatible', b'onekvm,ax630c-npu\0'),
    ('reg', be32(0, 0x3800000, 0, 0xb0000, 0, 0x3000000, 0, 0x280000, 0, 0x2100000, 0, 0x1000)),
    ('reg-names', b'regs\0ocm\0pmu\0'),
    ('interrupts', be32(0, 38, 1, 0, 42, 1)),
    ('interrupt-names', b'done\0error\0'),
]


def parse(blob):
    magic, total, off_struct, off_strings, off_rsv, version, _, _, size_strings, size_struct = \
        struct.unpack_from('>10I', blob, 0)
    if magic != MAGIC or version < 17:
        raise SystemExit('not a version 17 FDT')
    strings = blob[off_strings:off_strings + size_strings]
    rsv = []
    o = off_rsv
    while True:
        a, s = struct.unpack_from('>QQ', blob, o)
        o += 16
        if a == 0 and s == 0:
            break
        rsv.append((a, s))
    o = off_struct
    stack, root = [], None
    while True:
        tok, = struct.unpack_from('>I', blob, o)
        o += 4
        if tok == BEGIN:
            end = blob.index(b'\0', o)
            node = [blob[o:end].decode(), [], []]
            o = (end + 4) & ~3
            if stack:
                stack[-1][2].append(node)
            else:
                root = node
            stack.append(node)
        elif tok == END:
            stack.pop()
        elif tok == PROP:
            ln, nameoff = struct.unpack_from('>II', blob, o)
            o += 8
            name = strings[nameoff:strings.index(b'\0', nameoff)].decode()
            stack[-1][1].append((name, blob[o:o + ln]))
            o = (o + ln + 3) & ~3
        elif tok == NOP:
            continue
        elif tok == FDT_END:
            break
        else:
            raise SystemExit('bad FDT token %#x' % tok)
    boot_cpu = struct.unpack_from('>I', blob, 28)[0]
    return root, rsv, boot_cpu


def build(root, rsv, boot_cpu):
    strings, offsets, body = bytearray(), {}, bytearray()

    def name_off(n):
        if n not in offsets:
            offsets[n] = len(strings)
            strings.extend(n.encode() + b'\0')
        return offsets[n]

    def emit(node):
        name = node[0].encode() + b'\0'
        body.extend(be32(BEGIN) + name + b'\0' * (-len(name) % 4))
        for n, v in node[1]:
            body.extend(be32(PROP, len(v), name_off(n)) + v + b'\0' * (-len(v) % 4))
        for c in node[2]:
            emit(c)
        body.extend(be32(END))

    emit(root)
    body.extend(be32(FDT_END))
    rsvmap = b''.join(struct.pack('>QQ', a, s) for a, s in rsv) + struct.pack('>QQ', 0, 0)
    off_rsv = 40
    off_struct = off_rsv + len(rsvmap)
    off_strings = off_struct + len(body)
    total = off_strings + len(strings)
    header = struct.pack('>10I', MAGIC, total, off_struct, off_strings, off_rsv, 17, 16,
                         boot_cpu, len(strings), len(body))
    return header + rsvmap + bytes(body) + bytes(strings)


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    root, rsv, boot_cpu = parse(open(sys.argv[1], 'rb').read())
    if 'interrupt-parent' not in dict(root[1]):
        raise SystemExit('root node has no interrupt parent')
    soc = [c for c in root[2] if c[0] == 'soc']
    if len(soc) != 1:
        raise SystemExit('no /soc bus')
    soc = soc[0]
    props = dict(soc[1])
    if (props.get('#address-cells') != be32(2) or props.get('#size-cells') != be32(2)
            or props.get('ranges') != b''):
        raise SystemExit('/soc must map 1:1 with 2 address and 2 size cells')
    soc[2] = [c for c in soc[2] if c[0] != NODE]
    soc[2].append([NODE, list(NPU_PROPS), []])
    open(sys.argv[2], 'wb').write(build(root, rsv, boot_cpu))


if __name__ == '__main__':
    main()
