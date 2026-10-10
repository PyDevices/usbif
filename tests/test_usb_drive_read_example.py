# SPDX-FileCopyrightText: 2026 Brad Barnett
#
# SPDX-License-Identifier: MIT
"""examples/usb_drive_read.py finds the filesystem on a partitioned drive
and on one with no partition table.

MicroPython's ``vfs.VfsFat.mkfs()`` formats a small drive with no MBR, its
FAT boot sector at block 0, and a large one with an MBR and one FAT32
partition. The example has to mount either, and refuse a drive that is
neither.
"""

import importlib.util
import pathlib
import unittest

import _env  # noqa: F401

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load_example():
    spec = importlib.util.spec_from_file_location(
        "usb_drive_read_example", ROOT / "examples" / "usb_drive_read.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Host:
    def __init__(self, block0):
        self.block0 = block0

    def msc_read(self, lba, buf):
        assert lba == 0
        buf[:] = self.block0


def _mbr(ptype, start, count):
    b = bytearray(512)
    e = 446
    b[e + 4] = ptype
    b[e + 8:e + 12] = start.to_bytes(4, "little")
    b[e + 12:e + 16] = count.to_bytes(4, "little")
    b[510:512] = b"\x55\xaa"
    return b


def _boot_sector(label_at, label):
    b = bytearray(512)
    b[0:3] = b"\xeb\x3c\x90"
    b[label_at:label_at + 8] = label
    b[510:512] = b"\x55\xaa"
    return b


class FirstPartition(unittest.TestCase):
    def setUp(self):
        self.example = _load_example()

    def test_mbr_fat32_partition(self):
        host = _Host(_mbr(0x0C, 8192, 62676992))
        self.assertEqual(self.example.first_partition(host, 62685184, 512), (8192, 62676992))

    def test_no_partition_table_fat16(self):
        host = _Host(_boot_sector(54, b"FAT16   "))
        self.assertEqual(self.example.first_partition(host, 4096, 512), (0, 4096))

    def test_no_partition_table_fat32(self):
        host = _Host(_boot_sector(82, b"FAT32   "))
        self.assertEqual(self.example.first_partition(host, 4096, 512), (0, 4096))

    def test_blank_drive(self):
        self.assertIsNone(self.example.first_partition(_Host(bytearray(512)), 4096, 512))

    def test_non_fat_partition(self):
        host = _Host(_mbr(0x83, 2048, 100000))   # a Linux partition
        self.assertIsNone(self.example.first_partition(host, 102048, 512))


if __name__ == "__main__":
    unittest.main()
