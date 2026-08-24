#!/usr/bin/env python3
import argparse
import pathlib
import struct
import sys

RISCV_MAGIC = b"RISCV"
EXPECTED_TEXT_OFFSET = 0x00400000
GUEST_RAM_SIZE = 0x01E00000
DTB_OFFSET = 0x01D00000
KERNEL_PARTITION_SIZE = 0x01E00000


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate an RV32 Linux Image")
    parser.add_argument("image", type=pathlib.Path)
    args = parser.parse_args()

    data = args.image.read_bytes()
    if len(data) < 64:
        raise SystemExit("Image is shorter than its 64-byte RISC-V header")
    text_offset, effective_size = struct.unpack_from("<QQ", data, 8)
    if data[48:53] != RISCV_MAGIC:
        raise SystemExit(f"bad RISC-V magic: {data[48:53]!r}")
    if text_offset != EXPECTED_TEXT_OFFSET:
        raise SystemExit(
            f"text offset is 0x{text_offset:x}, expected 0x{EXPECTED_TEXT_OFFSET:x}"
        )
    if effective_size == 0:
        raise SystemExit("RISC-V Image header reports an empty effective image")
    if text_offset + effective_size > DTB_OFFSET:
        raise SystemExit(
            "effective kernel overlaps the DTB: "
            f"0x{text_offset + effective_size:x} > 0x{DTB_OFFSET:x}"
        )
    if len(data) > KERNEL_PARTITION_SIZE:
        raise SystemExit(
            f"flash payload {len(data)} exceeds {KERNEL_PARTITION_SIZE}-byte partition"
        )
    if text_offset + effective_size > GUEST_RAM_SIZE:
        raise SystemExit("effective kernel exceeds guest RAM")

    print(
        f"Image OK: payload={len(data)} bytes, effective={effective_size} bytes, "
        f"load=0x{EXPECTED_TEXT_OFFSET:x}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
