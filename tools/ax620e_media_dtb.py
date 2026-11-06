#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Lend the AX630C capture and encoder frame-buffer pools to Linux.

The kernel device tree reserves the VIN capture pool and the VC8000E frame
buffer region as no-map memory.  This tool rewrites a built DTB so that both
become reusable CMA areas:

    capture-pool@... { compatible = "shared-dma-pool"; reusable; reg = ...; };
    venc-framebuf@... { compatible = "shared-dma-pool"; reusable; reg = ...; };

Each stays the private area of its device (`memory-region`), so the drivers
allocate from it with the DMA API while Linux keeps movable pages there when
no stream holds buffers.  Addresses and sizes are unchanged.  The input is
checked first: each region must be a single no-map entry inside /memory,
aligned for CMA, and clear of every other reservation.

usage: ax620e_media_dtb.py IN.dtb OUT.dtb
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

MIB = 1024 * 1024
# A CMA area must start and end on a pageblock boundary; 4 MiB covers the
# RISC-V and arm64 configurations OneKVM builds.
CMA_ALIGN = 4 * MIB

FDT_MAGIC = 0xD00DFEED
FDT_BEGIN_NODE, FDT_END_NODE, FDT_PROP, FDT_NOP, FDT_END = 1, 2, 3, 4, 9



class DtbError(Exception):
    pass


@dataclass
class Node:
    name: str
    props: list[tuple[str, bytes]] = field(default_factory=list)
    children: list["Node"] = field(default_factory=list)

    def prop(self, name: str) -> bytes | None:
        for key, value in self.props:
            if key == name:
                return value
        return None

    def set(self, name: str, value: bytes) -> None:
        for i, (key, _) in enumerate(self.props):
            if key == name:
                self.props[i] = (name, value)
                return
        self.props.append((name, value))

    def child(self, name: str) -> Node | None:
        return next((c for c in self.children if c.name == name), None)

    def compatible(self) -> list[str]:
        raw = self.prop("compatible") or b""
        return [s.decode() for s in raw.split(b"\0") if s]


@dataclass
class Fdt:
    root: Node
    reserve_map: list[tuple[int, int]]
    boot_cpuid: int


@dataclass(frozen=True)
class Region:
    start: int
    end: int

    @property
    def size(self) -> int:
        return self.end - self.start

    def overlaps(self, other: "Region") -> bool:
        return self.start < other.end and other.start < self.end

    def __str__(self) -> str:
        return f"{self.start:#010x}-{self.end - 1:#010x} ({self.size // MIB} MiB)"


def parse(blob: bytes) -> Fdt:
    if len(blob) < 40:
        raise DtbError("input is too short for an FDT header")
    (magic, total, off_struct, off_strings, off_rsv, version, _last_comp,
     boot_cpuid, size_strings, size_struct) = struct.unpack(">10I", blob[:40])
    if magic != FDT_MAGIC:
        raise DtbError("input is not a flattened device tree")
    if version < 17 or total > len(blob):
        raise DtbError(f"unsupported or truncated FDT (version {version})")
    strings = blob[off_strings:off_strings + size_strings]

    reserve_map = []
    pos = off_rsv
    while True:
        address, size = struct.unpack(">QQ", blob[pos:pos + 16])
        pos += 16
        if address == 0 and size == 0:
            break
        reserve_map.append((address, size))

    pos = off_struct
    end = off_struct + size_struct
    stack: list[Node] = []
    root = None
    while pos < end:
        (token,) = struct.unpack(">I", blob[pos:pos + 4])
        pos += 4
        if token == FDT_BEGIN_NODE:
            name_end = blob.index(b"\0", pos)
            node = Node(blob[pos:name_end].decode())
            pos = (name_end + 4) & ~3
            if stack:
                stack[-1].children.append(node)
            elif root is None:
                root = node
            else:
                raise DtbError("FDT has more than one root node")
            stack.append(node)
        elif token == FDT_END_NODE:
            stack.pop()
        elif token == FDT_PROP:
            length, name_off = struct.unpack(">II", blob[pos:pos + 8])
            pos += 8
            name = strings[name_off:strings.index(b"\0", name_off)].decode()
            stack[-1].props.append((name, blob[pos:pos + length]))
            pos = (pos + length + 3) & ~3
        elif token == FDT_NOP:
            continue
        elif token == FDT_END:
            break
        else:
            raise DtbError(f"bad FDT token {token:#x}")
    if root is None or stack:
        raise DtbError("unbalanced FDT structure block")
    return Fdt(root, reserve_map, boot_cpuid)


def serialize(fdt: Fdt) -> bytes:
    strings = bytearray()
    offsets: dict[str, int] = {}

    def string_offset(name: str) -> int:
        if name not in offsets:
            offsets[name] = len(strings)
            strings.extend(name.encode() + b"\0")
        return offsets[name]

    body = bytearray()

    def pad() -> None:
        while len(body) % 4:
            body.append(0)

    def emit(node: Node) -> None:
        body.extend(struct.pack(">I", FDT_BEGIN_NODE))
        body.extend(node.name.encode() + b"\0")
        pad()
        for key, value in node.props:
            body.extend(struct.pack(">III", FDT_PROP, len(value), string_offset(key)))
            body.extend(value)
            pad()
        for child in node.children:
            emit(child)
        body.extend(struct.pack(">I", FDT_END_NODE))

    emit(fdt.root)
    body.extend(struct.pack(">I", FDT_END))
    rsv = b"".join(struct.pack(">QQ", a, s) for a, s in fdt.reserve_map)
    rsv += struct.pack(">QQ", 0, 0)
    off_rsv = 40
    off_struct = off_rsv + len(rsv)
    off_strings = off_struct + len(body)
    total = off_strings + len(strings)
    header = struct.pack(">10I", FDT_MAGIC, total, off_struct, off_strings,
                         off_rsv, 17, 16, fdt.boot_cpuid, len(strings), len(body))
    return header + rsv + bytes(body) + bytes(strings)


def cells(node: Node, name: str, default: int) -> int:
    raw = node.prop(name)
    return struct.unpack(">I", raw)[0] if raw else default


def read_regs(node: Node, address_cells: int, size_cells: int) -> list[Region]:
    raw = node.prop("reg") or b""
    stride = 4 * (address_cells + size_cells)
    if not raw or len(raw) % stride:
        return []
    regions = []
    for off in range(0, len(raw), stride):
        words = struct.unpack(f">{address_cells + size_cells}I", raw[off:off + stride])
        address = 0
        for w in words[:address_cells]:
            address = (address << 32) | w
        size = 0
        for w in words[address_cells:]:
            size = (size << 32) | w
        regions.append(Region(address, address + size))
    return regions


def encode_reg(region: Region, address_cells: int, size_cells: int) -> bytes:
    def split(value: int, count: int) -> list[int]:
        return [(value >> (32 * (count - 1 - i))) & 0xFFFFFFFF for i in range(count)]

    words = split(region.start, address_cells) + split(region.size, size_cells)
    return struct.pack(f">{len(words)}I", *words)


def walk(node: Node):
    yield node
    for child in node.children:
        yield from walk(child)


def max_phandle(root: Node) -> int:
    highest = 0
    for node in walk(root):
        for key in ("phandle", "linux,phandle"):
            raw = node.prop(key)
            if raw and len(raw) == 4:
                highest = max(highest, struct.unpack(">I", raw)[0])
    return highest


def memory_regions(root: Node, address_cells: int, size_cells: int) -> list[Region]:
    regions = []
    for node in root.children:
        if node.prop("device_type") == b"memory\0":
            regions.extend(read_regs(node, address_cells, size_cells))
    return regions

POOL_COMPATIBLE = "shared-dma-pool"
# (device compatible, memory-region-names entry or None for the first region)
POOLS = (
    ("axera,ax630c-vin", None),
    ("axera,ax630c-vc8000e", "framebuf"),
)


def region_node(root: Node, resmem: Node, device: Node, name: str | None) -> Node:
    raw = device.prop("memory-region") or b""
    phandles = list(struct.unpack(f">{len(raw) // 4}I", raw)) if raw and len(raw) % 4 == 0 else []
    index = 0
    if name is not None:
        names = [s.decode() for s in (device.prop("memory-region-names") or b"").split(b"\0") if s]
        if name not in names:
            raise DtbError(f"{device.name}: no {name!r} memory-region")
        index = names.index(name)
    if index >= len(phandles):
        raise DtbError(f"{device.name}: missing memory-region entry {index}")
    for node in resmem.children:
        raw = node.prop("phandle") or node.prop("linux,phandle")
        if raw and struct.unpack(">I", raw)[0] == phandles[index]:
            return node
    raise DtbError(f"{device.name}: memory-region is not a /reserved-memory node")


def lend_pools(fdt: Fdt) -> list[tuple[str, Region]]:
    root = fdt.root
    resmem = root.child("reserved-memory")
    if resmem is None:
        raise DtbError("input has no /reserved-memory node")
    acells = cells(resmem, "#address-cells", 2)
    scells = cells(resmem, "#size-cells", 2)
    memory = memory_regions(root, cells(root, "#address-cells", 2),
                            cells(root, "#size-cells", 2))

    targets = []
    for compatible, name in POOLS:
        devices = [n for n in walk(root) if compatible in n.compatible()
                   and n.prop("status") in (None, b"okay\0", b"ok\0")]
        if len(devices) != 1:
            raise DtbError(f"expected one enabled {compatible} node, found {len(devices)}")
        node = region_node(root, resmem, devices[0], name)
        if node.prop("reusable") is not None:
            raise DtbError(f"/reserved-memory/{node.name} is already reusable")
        if node.prop("no-map") is None:
            raise DtbError(f"/reserved-memory/{node.name} is not a no-map region")
        regs = read_regs(node, acells, scells)
        if len(regs) != 1:
            raise DtbError(f"{node.name}: expected a single static reg entry")
        region = regs[0]
        if region.start % CMA_ALIGN or region.size % CMA_ALIGN:
            raise DtbError(f"{node.name} {region} is not {CMA_ALIGN // MIB} MiB aligned")
        if not any(m.start <= region.start and region.end <= m.end for m in memory):
            raise DtbError(f"{node.name} {region} is outside the /memory ranges")
        for other in resmem.children:
            if other is node:
                continue
            for reg in read_regs(other, acells, scells):
                if region.overlaps(reg):
                    raise DtbError(f"{node.name} {region} overlaps /reserved-memory/{other.name} {reg}")
        for address, size in fdt.reserve_map:
            if region.overlaps(Region(address, address + size)):
                raise DtbError(f"{node.name} {region} overlaps a /memreserve/ entry")
        targets.append((node, region))

    for node, _ in targets:
        node.props = [(k, v) for k, v in node.props if k not in ("no-map", "compatible")]
        node.props.insert(0, ("compatible", POOL_COMPATIBLE.encode() + b"\0"))
        node.props.append(("reusable", b""))
    return [(node.name, region) for node, region in targets]


def lazy_pool(fdt: Fdt, pool_mib: int) -> list[tuple[str, Region]]:
    """Replace both private media regions with one optional shared pool.

    Validate on a copy so a rejected layout never leaves partial mutations.
    The default CMA and the fixed VCMD command region remain untouched.
    """
    if pool_mib < 0 or pool_mib % 4:
        raise DtbError("lazy pool must be a nonnegative multiple of 4 MiB")
    out = parse(serialize(fdt))
    if any("onekvm,lazy-dma" in n.compatible() for n in walk(out.root)):
        raise DtbError("input already has a lazy DMA device")
    lent = dict(lend_pools(out))
    root = out.root
    resmem = root.child("reserved-memory")
    devices = [next(n for n in walk(root) if compat in n.compatible()
                    and n.prop("status") in (None, b"okay\0", b"ok\0"))
               for compat, _ in POOLS]
    vin, venc = devices
    capture = region_node(root, resmem, vin, None)
    framebuf = region_node(root, resmem, venc, "framebuf")
    removed_refs = {n.prop("phandle") or n.prop("linux,phandle")
                    for n in (capture, framebuf)}
    for node in walk(root):
        refs = node.prop("memory-region") or b""
        if node not in devices and any(refs[i:i + 4] in removed_refs
                                      for i in range(0, len(refs), 4)):
            raise DtbError(f"{node.name}: another device references a private media pool")
    old = lent[framebuf.name]
    if pool_mib * MIB > old.size:
        raise DtbError("lazy pool exceeds the former encoder frame-buffer region")
    phandle = max_phandle(root) + 1
    supplier = Node("onekvm-lazy-dma", [
        ("compatible", b"onekvm,lazy-dma\0"),
        ("phandle", struct.pack(">I", phandle)),
    ])
    vin.props = [(k, v) for k, v in vin.props
                 if k not in ("memory-region", "memory-region-names")]
    names = [s for s in venc.prop("memory-region-names").split(b"\0") if s]
    refs = struct.unpack(f">{len(names)}I", venc.prop("memory-region"))
    keep = [(name, ref) for name, ref in zip(names, refs) if name != b"framebuf"]
    venc.set("memory-region-names", b"".join(name + b"\0" for name, _ in keep))
    venc.set("memory-region", struct.pack(f">{len(keep)}I", *(ref for _, ref in keep)))
    for device in devices:
        device.set("onekvm,lazy-dma", struct.pack(">I", phandle))
    resmem.children = [n for n in resmem.children if n not in (capture, framebuf)]
    regions = []
    if pool_mib:
        region = Region(old.start, old.start + pool_mib * MIB)
        pool_phandle = phandle + 1
        pool = Node(f"onekvm-lazy-dma@{region.start:x}", [
            ("compatible", b"shared-dma-pool\0"), ("reusable", b""),
            ("reg", encode_reg(region, cells(resmem, "#address-cells", 2),
                               cells(resmem, "#size-cells", 2))),
            ("phandle", struct.pack(">I", pool_phandle)),
        ])
        resmem.children.append(pool)
        supplier.set("memory-region", struct.pack(">I", pool_phandle))
        regions.append((pool.name, region))
    root.children.append(supplier)
    fdt.root = root
    return regions


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--lazy-pool-mib", type=int,
                        help="use shared lazy DMA instead of private pools (0: ordinary memory only)")
    args = parser.parse_args(argv)
    try:
        fdt = parse(args.input.read_bytes())
        lent = (lend_pools(fdt) if args.lazy_pool_mib is None
                else lazy_pool(fdt, args.lazy_pool_mib))
        blob = serialize(fdt)
        parse(blob)
    except DtbError as err:
        print(f"ax620e_media_dtb: {args.input}: {err}", file=sys.stderr)
        return 1
    args.output.write_bytes(blob)
    for name, region in lent:
        print(f"ax620e_media_dtb: {name} {region} -> reusable CMA")
    return 0


if __name__ == "__main__":
    sys.exit(main())
