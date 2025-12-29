#!/usr/bin/env python3
"""Validate an ESP32 firmware .bin and generate OTA metadata for NFC streaming.

This project’s NFC OTA receiver expects:
- totalSize: the firmware size in bytes
- crc32: standard Ethernet/ZIP CRC32 of the whole binary

It does NOT require PSRAM; it streams small mailbox payloads.

Usage examples:
  python src/tools/nfc_ota_check.py
  python src/tools/nfc_ota_check.py --bin .pio/build/esp32s3-dev/firmware.bin --write-manifest
    python src/tools/nfc_ota_check.py --write-stream
  python src/tools/nfc_ota_check.py --chunk-size 200 --write-manifest --out firmware.nfc_ota.json
    python src/tools/nfc_ota_check.py --chunk-size 200 --write-stream --stream-out firmware.nfc_ota.stream
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import zlib
from dataclasses import asdict, dataclass
from hashlib import sha256
import struct


MAX_MAILBOX_LEN = 255
HEADER_LEN = 16
MAX_PAYLOAD_LEN = MAX_MAILBOX_LEN - HEADER_LEN  # 239


# Protocol constants (must match src/nfc_ota.cpp)
MAGIC = 0x544F464E  # 'N''F''O''T' little-endian
VERSION = 1

MSG_START = 1
MSG_DATA = 2
MSG_END = 3
MSG_ABORT = 4


@dataclass(frozen=True)
class OtaManifest:
    bin_path: str
    size_bytes: int
    crc32: str  # 0x????????
    sha256: str
    chunk_size: int
    max_payload_size: int
    data_packets: int


def build_header(*, msg_type: int, seq: int, arg0: int, data_len: int) -> bytes:
    if not (0 <= seq <= 0xFFFF):
        raise ValueError(f"seq out of range: {seq}")
    if not (0 <= arg0 <= 0xFFFFFFFF):
        raise ValueError(f"arg0 out of range: {arg0}")
    if not (0 <= data_len <= 0xFFFF):
        raise ValueError(f"data_len out of range: {data_len}")

    # struct MsgHeader (little-endian, packed):
    #   u32 magic
    #   u8  version
    #   u8  type
    #   u16 seq
    #   u32 arg0
    #   u16 dataLen
    #   u16 reserved
    return struct.pack("<IBBH IHH", MAGIC, VERSION, msg_type, seq, arg0, data_len, 0)


def build_frame(*, msg_type: int, seq: int, arg0: int, payload: bytes) -> bytes:
    if payload is None:
        payload = b""
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError(f"payload too large for mailbox: {len(payload)} > {MAX_PAYLOAD_LEN}")
    header = build_header(msg_type=msg_type, seq=seq, arg0=arg0, data_len=len(payload))
    frame = header + payload
    if len(frame) > MAX_MAILBOX_LEN:
        raise ValueError(f"frame too large for mailbox: {len(frame)} > {MAX_MAILBOX_LEN}")
    return frame


def write_length_prefixed_stream(out_path: str, frames: list[bytes]) -> None:
    # Stream format: repeated records of [u16_le frame_len][frame_bytes]
    # frame_len is the mailbox message length (<=255)
    with open(out_path, "wb") as f:
        for frame in frames:
            f.write(struct.pack("<H", len(frame)))
            f.write(frame)


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
        default=200,
        help=f"Suggested NFC mailbox payload chunk size (1..{MAX_PAYLOAD_LEN}, default: 200)",
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
        "--write-stream",
        action="store_true",
        help="Write a compact length-prefixed stream of START/DATA/END frames for NFC uploading",
    )
    parser.add_argument(
        "--stream-out",
        default="",
        help="Stream output path (default: <bin>.nfc_ota.stream when --write-stream is set)",
    )
    parser.add_argument(
        "--start-seq",
        type=int,
        default=0,
        help="Sequence number for START packet (default: 0)",
    )
    parser.add_argument(
        "--skip-crc",
        action="store_true",
        help="Write END.arg0 = 0 (skip CRC check on device)",
    )

    args = parser.parse_args(argv)

    bin_path = args.bin_path
    if not os.path.isfile(bin_path):
        print(f"ERROR: firmware not found: {bin_path}", file=sys.stderr)
        print("Build it with: pio run -e esp32s3-dev", file=sys.stderr)
        return 2

    if args.chunk_size <= 0 or args.chunk_size > MAX_PAYLOAD_LEN:
        print(
            f"ERROR: --chunk-size must be 1..{MAX_PAYLOAD_LEN} (got {args.chunk_size})",
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
        max_payload_size=MAX_PAYLOAD_LEN,
        data_packets=data_packets,
    )

    print("NFC OTA firmware check")
    print(f"  bin:      {manifest.bin_path}")
    print(f"  size:     {manifest.size_bytes} bytes")
    print(f"  crc32:    {manifest.crc32}  (use as END.arg0; or set 0 to skip)")
    print(f"  sha256:   {manifest.sha256}")
    print(f"  payload:  {manifest.chunk_size} bytes per DATA packet (max {manifest.max_payload_size})")
    print(f"  packets:  {manifest.data_packets} DATA packets")
    if not header_ok:
        print("  warning:  file does not start with 0xE9; may not be an ESP32 app image")

    if args.write_stream:
        start_seq = args.start_seq
        if start_seq < 0 or start_seq > 0xFFFF:
            print("ERROR: --start-seq must be 0..65535", file=sys.stderr)
            return 2

        end_crc = 0 if args.skip_crc else (crc & 0xFFFFFFFF)

        frames: list[bytes] = []
        frames.append(build_frame(msg_type=MSG_START, seq=start_seq, arg0=size, payload=b""))

        seq = (start_seq + 1) & 0xFFFF
        offset = 0
        with open(bin_path, "rb") as f:
            while True:
                payload = f.read(args.chunk_size)
                if not payload:
                    break
                frames.append(build_frame(msg_type=MSG_DATA, seq=seq, arg0=offset, payload=payload))
                offset += len(payload)
                seq = (seq + 1) & 0xFFFF

        frames.append(build_frame(msg_type=MSG_END, seq=seq, arg0=end_crc, payload=b""))

        out_path = args.stream_out
        if not out_path:
            out_path = bin_path + ".nfc_ota.stream"
        write_length_prefixed_stream(out_path, frames)
        print(f"  stream:   wrote {out_path} ({len(frames)} frames)")

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
