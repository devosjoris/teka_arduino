#!/usr/bin/env python3
"""Validate an ESP32 firmware .bin and generate OTA metadata for NFC transfer.

This project's NFC OTA receiver uses EEPROM-based transfer:
- totalSize: the firmware size in bytes
- crc32: standard Ethernet/ZIP CRC32 of each chunk
- Chunks are written to EEPROM at address 0xC8 (max 1800 bytes)

Usage examples:
  python src/tools/nfc_ota_check.py
  python src/tools/nfc_ota_check.py --bin .pio/build/esp32s3-dev/firmware.bin --write-manifest  (--gzip)
  python src/tools/nfc_ota_check.py --gzip --write-manifest
"""

from __future__ import annotations

import argparse
import gzip
import json
import os
import sys
import zlib
from dataclasses import asdict, dataclass
from hashlib import sha256


MAX_EEPROM_CHUNK = 1800  # Must match NFC_OTA_MAX_CHUNK_SIZE in nfc_ota.h


@dataclass(frozen=True)
class OtaManifest:
    bin_path: str
    size_bytes: int
    crc32: str  # 0x????????
    sha256: str
    chunk_size: int
    max_chunk_size: int
    data_packets: int
    compression: str = "none"


def compute_crc32_and_sha256(path: str, chunk: int = 1024 * 1024) -> tuple[int, str]:
    crc = 0
    h = sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            crc = zlib.crc32(b, crc)
            h.update(b)
    return crc & 0xFFFFFFFF, h.hexdigest()


def read_first_bytes(path: str, n: int) -> bytes:
    with open(path, "rb") as f:
        return f.read(n)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Check firmware.bin and generate NFC OTA manifest")
    parser.add_argument(
        "--bin",
        dest="bin_path",
        default=os.path.join(".pio", "build", "esp32s3-dev", "firmware.bin"),
        help="Path to firmware .bin (default: .pio/build/esp32s3-dev/firmware.bin)",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=1800,
        help=f"EEPROM chunk size per DATA packet (1..{MAX_EEPROM_CHUNK}, default: 1800)",
    )
    parser.add_argument(
        "--write-manifest",
        action="store_true",
        help="Write a JSON manifest with size/crc/chunking info",
    )
    parser.add_argument(
        "--out",
        default="",
        help="Manifest output path (default: <bin>.nfc_ota.json when --write-manifest is set)",
    )

    parser.add_argument(
        "--gzip",
        action="store_true",
        help="Compress firmware with gzip before processing",
    )

    args = parser.parse_args(argv)

    bin_path = args.bin_path
    if not os.path.isfile(bin_path):
        print(f"ERROR: firmware not found: {bin_path}", file=sys.stderr)
        print("Build it with: pio run -e esp32s3-dev", file=sys.stderr)
        return 2

    compression_type = "none"
    if args.gzip:
        print(f"Compressing {bin_path}...")
        with open(bin_path, "rb") as f:
            raw_data = f.read()
        compressed_data = gzip.compress(raw_data, compresslevel=9)
        bin_path = bin_path + ".gz"
        with open(bin_path, "wb") as f:
            f.write(compressed_data)
        compression_type = "gzip"
        print(f"  orig size: {len(raw_data)}")
        print(f"  gzip size: {len(compressed_data)} ({len(compressed_data)/len(raw_data)*100:.1f}%)")
        print(f"  Using {bin_path} for OTA generation")

    if args.chunk_size <= 0 or args.chunk_size > MAX_EEPROM_CHUNK:
        print(
            f"ERROR: --chunk-size must be 1..{MAX_EEPROM_CHUNK} (got {args.chunk_size})",
            file=sys.stderr,
        )
        return 2

    size = os.path.getsize(bin_path)
    if size <= 0:
        print("ERROR: firmware file is empty", file=sys.stderr)
        return 2

    # Basic ESP32 image sanity check: most ESP32 app images start with 0xE9.
    first = read_first_bytes(bin_path, 16)
    header_ok = len(first) >= 1 and first[0] == 0xE9

    crc, sha = compute_crc32_and_sha256(bin_path)

    data_packets = (size + args.chunk_size - 1) // args.chunk_size

    manifest = OtaManifest(
        bin_path=os.path.normpath(bin_path),
        size_bytes=size,
        crc32=f"0x{crc:08X}",
        sha256=sha,
        chunk_size=args.chunk_size,
        max_chunk_size=MAX_EEPROM_CHUNK,
        data_packets=data_packets,
        compression=compression_type,
    )

    print("NFC OTA firmware check")
    print(f"  bin:      {manifest.bin_path}")
    print(f"  size:     {manifest.size_bytes} bytes")
    print(f"  crc32:    {manifest.crc32}")
    print(f"  sha256:   {manifest.sha256}")
    print(f"  chunk:    {manifest.chunk_size} bytes per DATA packet (max {manifest.max_chunk_size})")
    print(f"  packets:  {manifest.data_packets} DATA packets")
    print(f"  compress: {manifest.compression}")
    if not header_ok:
        print("  warning:  file does not start with 0xE9; may not be an ESP32 app image")

    if args.write_manifest:
        out_path = args.out
        if not out_path:
            out_path = bin_path + ".nfc_ota.json"
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(asdict(manifest), f, indent=2)
            f.write("\n")
        print(f"  manifest: wrote {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
