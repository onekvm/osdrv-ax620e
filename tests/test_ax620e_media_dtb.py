#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Host tests for tools/ax620e_media_dtb.py on a synthetic NanoKVM Pro DTB."""

import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import ax620e_media_dtb as tool  # noqa: E402
from ax620e_media_dtb import MIB, DtbError, Fdt, Node, Region  # noqa: E402


def u32(*values):
    return struct.pack(f">{len(values)}I", *values)


def reg64(start, size):
    return u32(start >> 32, start & 0xFFFFFFFF, size >> 32, size & 0xFFFFFFFF)


def pro_fdt(framebuf_start=0x73800000, framebuf_mib=96, vin_status=b"okay\0"):
    resmem = Node("reserved-memory", [
        ("#address-cells", u32(2)), ("#size-cells", u32(2)), ("ranges", b""),
    ], [
        Node("cmm-cma@6d400000", [("compatible", b"shared-dma-pool\0"), ("reusable", b""),
                                  ("reg", reg64(0x6D400000, 100 * MIB))]),
        Node("venc-framebuf@73800000", [("reg", reg64(framebuf_start, framebuf_mib * MIB)),
                                        ("no-map", b""), ("phandle", u32(10))]),
        Node("capture-pool@79800000", [("compatible", b"shared-dma-pool\0"),
                                       ("reg", reg64(0x79800000, 96 * MIB)),
                                       ("no-map", b""), ("phandle", u32(11))]),
        Node("venc-cmdbuf@7f800000", [("compatible", b"shared-dma-pool\0"),
                                      ("reg", reg64(0x7F800000, 8 * MIB)),
                                      ("no-map", b""), ("phandle", u32(12))]),
    ])
    root = Node("", [("#address-cells", u32(2)), ("#size-cells", u32(2))], [
        Node("memory@40001000", [("device_type", b"memory\0"),
                                 ("reg", reg64(0x46200000, 0x27200000) + reg64(0x6D400000, 0x12C00000))]),
        resmem,
        Node("soc", [], [
            Node("venc@4010000", [("compatible", b"axera,ax630c-vc8000e\0"),
                                  ("status", b"okay\0"),
                                  ("memory-region", u32(12, 10)),
                                  ("memory-region-names", b"cmdbuf\0framebuf\0")]),
            Node("video-capture@2400000", [("compatible", b"axera,ax630c-vin\0"),
                                           ("status", vin_status),
                                           ("memory-region", u32(11))]),
        ]),
    ])
    return Fdt(root, [], 0)


def flags(node):
    return [k for k, _ in node.props if k in ("compatible", "no-map", "reusable")]


class LendPoolsTest(unittest.TestCase):
    def test_lazy_dma_replaces_private_pools_and_preserves_command_memory(self):
        fdt = pro_fdt()
        tool.lazy_pool(fdt, 64)
        out = tool.parse(tool.serialize(fdt))
        resmem = out.root.child("reserved-memory")
        self.assertIsNone(resmem.child("capture-pool@79800000"))
        self.assertIsNone(resmem.child("venc-framebuf@73800000"))
        pool = resmem.child("onekvm-lazy-dma@73800000")
        self.assertEqual(tool.read_regs(pool, 2, 2),
                         [Region(0x73800000, 0x77800000)])
        self.assertIsNotNone(pool.prop("reusable"))
        cmd = resmem.child("venc-cmdbuf@7f800000")
        self.assertIsNotNone(cmd.prop("no-map"))
        vin = out.root.child("soc").child("video-capture@2400000")
        venc = out.root.child("soc").child("venc@4010000")
        supplier = out.root.child("onekvm-lazy-dma")
        self.assertEqual(vin.prop("onekvm,lazy-dma"), supplier.prop("phandle"))
        self.assertEqual(venc.prop("onekvm,lazy-dma"), supplier.prop("phandle"))
        self.assertIsNone(vin.prop("memory-region"))
        self.assertEqual(venc.prop("memory-region"), cmd.prop("phandle"))
        self.assertEqual(venc.prop("memory-region-names"), b"cmdbuf\0")
        self.assertEqual(supplier.prop("memory-region"), pool.prop("phandle"))

    def test_lazy_dma_without_cma(self):
        fdt = pro_fdt()
        tool.lazy_pool(fdt, 0)
        self.assertIsNone(fdt.root.child("onekvm-lazy-dma").prop("memory-region"))
        self.assertEqual(len(fdt.root.child("reserved-memory").children), 2)

    def test_lazy_dma_rejects_another_pool_consumer(self):
        fdt = pro_fdt()
        fdt.root.children.append(Node("other-device", [("memory-region", u32(10))]))
        before = tool.serialize(fdt)
        with self.assertRaisesRegex(DtbError, "another device"):
            tool.lazy_pool(fdt, 64)
        self.assertEqual(tool.serialize(fdt), before)

    def test_lazy_dma_invalid_size_leaves_input_unchanged(self):
        for size in (-4, 3, 100):
            fdt = pro_fdt()
            before = tool.serialize(fdt)
            with self.assertRaises(DtbError):
                tool.lazy_pool(fdt, size)
            self.assertEqual(tool.serialize(fdt), before)

    def test_both_pools_become_reusable_in_place(self):
        fdt = pro_fdt()
        lent = dict(tool.lend_pools(fdt))
        out = tool.parse(tool.serialize(fdt))
        resmem = out.root.child("reserved-memory")
        for name, start in (("venc-framebuf@73800000", 0x73800000),
                            ("capture-pool@79800000", 0x79800000)):
            node = resmem.child(name)
            self.assertEqual(flags(node), ["compatible", "reusable"])
            self.assertEqual(node.compatible(), ["shared-dma-pool"])
            self.assertEqual(tool.read_regs(node, 2, 2), [Region(start, start + 96 * MIB)])
            self.assertEqual(lent[name].start, start)
        # Untouched: the command pool stays no-map, phandles are kept.
        self.assertIn("no-map", flags(resmem.child("venc-cmdbuf@7f800000")))
        self.assertEqual(resmem.child("venc-framebuf@73800000").prop("phandle"), u32(10))

    def test_rejects_second_run(self):
        fdt = pro_fdt()
        tool.lend_pools(fdt)
        with self.assertRaisesRegex(DtbError, "already reusable"):
            tool.lend_pools(fdt)

    def test_rejects_unaligned_region(self):
        with self.assertRaisesRegex(DtbError, "not 4 MiB aligned"):
            tool.lend_pools(pro_fdt(framebuf_start=0x73900000, framebuf_mib=92))

    def test_rejects_overlap(self):
        fdt = pro_fdt()
        fdt.root.child("reserved-memory").children.append(
            Node("fb", [("reg", reg64(0x7A000000, MIB)), ("no-map", b"")]))
        with self.assertRaisesRegex(DtbError, "overlaps /reserved-memory/fb"):
            tool.lend_pools(fdt)

    def test_rejects_disabled_device(self):
        with self.assertRaisesRegex(DtbError, "axera,ax630c-vin"):
            tool.lend_pools(pro_fdt(vin_status=b"disabled\0"))

    def test_failure_changes_nothing(self):
        fdt = pro_fdt()
        fdt.reserve_map.append((0x7A000000, MIB))
        before = tool.serialize(fdt)
        with self.assertRaisesRegex(DtbError, "memreserve"):
            tool.lend_pools(fdt)
        self.assertEqual(tool.serialize(fdt), before)

    def test_cli_failure_keeps_no_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            src, dst, bad = (Path(tmp) / n for n in ("in.dtb", "out.dtb", "bad.dtb"))
            src.write_bytes(tool.serialize(pro_fdt()))
            self.assertEqual(tool.main([str(src), str(dst)]), 0)
            self.assertEqual(tool.main([str(dst), str(bad)]), 1)
            self.assertFalse(bad.exists())


if __name__ == "__main__":
    unittest.main()
