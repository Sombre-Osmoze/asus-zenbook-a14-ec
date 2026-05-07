#!/usr/bin/env python3
"""
Test ASUS Zenbook A14 RA vendor HID 0B05:4543.

Report descriptor declares Report ID 0x5A as 16-byte feature
(1 byte ID + 15 bytes payload).

Usage:
    sudo ./test-4543.py init           # send both known init sequences
    sudo ./test-4543.py bl <0..3>      # set keyboard backlight level
    sudo ./test-4543.py fnlock <0|1>   # toggle fn-lock
    sudo ./test-4543.py raw <hex>      # send arbitrary payload (hex, no 5a prefix)
    sudo ./test-4543.py probe          # send a few innocuous reads, dump replies

Tries python3-hid first (ctypes), falls back to python3-hidapi (cython).
"""
import sys

import os
VID = 0x0B05
PID = int(os.environ.get("PID", "0x4543"), 0)
REPORT_ID = 0x5A
PAYLOAD_LEN = int(os.environ.get("PLEN", "15"))  # 16-byte report total, minus 1 ID byte


def open_device():
    try:
        import hid
        d = hid.Device(VID, PID)
        return ("hid", d)
    except ImportError:
        pass
    except Exception as e:
        print(f"python3-hid open failed: {e}", file=sys.stderr)

    try:
        import hidapi
        d = hidapi.Device(next(hidapi.enumerate(VID, PID)))
        return ("hidapi", d)
    except ImportError:
        pass
    except Exception as e:
        print(f"python3-hidapi open failed: {e}", file=sys.stderr)

    print("No working HID library found. Install python3-hid or python3-hidapi.",
          file=sys.stderr)
    sys.exit(2)


def send_feature(backend, dev, payload: bytes):
    """payload = bytes WITHOUT report ID prefix; will be padded to PAYLOAD_LEN."""
    if len(payload) > PAYLOAD_LEN:
        raise ValueError(f"payload too long ({len(payload)} > {PAYLOAD_LEN})")
    payload = payload.ljust(PAYLOAD_LEN, b"\x00")
    full = bytes([REPORT_ID]) + payload  # 16 bytes total

    print(f"  -> {full.hex(' ')}")
    if backend == "hid":
        # python3-hid: send_feature_report(data) where data[0] = report id
        n = dev.send_feature_report(full)
    else:
        # python3-hidapi (cython): send_feature_report(data, report_id)
        n = dev.send_feature_report(payload, bytes([REPORT_ID]))
    print(f"  <- wrote {n} bytes")


def get_feature(backend, dev):
    if backend == "hid":
        data = dev.get_feature_report(REPORT_ID, 16)
    else:
        data = dev.get_feature_report(bytes([REPORT_ID]), 16)
    print(f"  <- read {data.hex(' ') if data else '(empty)'}")
    return data


def cmd_init(backend, dev):
    """Full A14 init per icecream95 BPF (0010-ASUS__Zenbook_A14_kbd.bpf.c)."""
    print("init 0 (ASUS Tech.Inc. handshake):")
    send_feature(backend, dev, b"ASUS Tech.Inc.\x00")
    print("init 1 (5A 05 20 31 00 08):")
    send_feature(backend, dev, bytes.fromhex("0520310008"))
    print("init 2 (5A D0 8F 01):")
    send_feature(backend, dev, bytes.fromhex("d08f01"))
    print("init 3 (5A D0 85 FF):")
    send_feature(backend, dev, bytes.fromhex("d085ff"))


def cmd_bl(backend, dev, level: int):
    level = max(0, min(3, level))
    print(f"set backlight level {level}:")
    send_feature(backend, dev, bytes.fromhex(f"bac5c4{level:02x}"))


def cmd_fnlock(backend, dev, state: int):
    state = 1 if state else 0
    print(f"set fn-lock {state}:")
    send_feature(backend, dev, bytes.fromhex(f"d04e{state:02x}"))


def cmd_raw(backend, dev, hex_payload: str):
    payload = bytes.fromhex(hex_payload.replace(" ", ""))
    print(f"raw payload (no 5a prefix):")
    send_feature(backend, dev, payload)


def cmd_probe(backend, dev):
    print("read current feature report state:")
    get_feature(backend, dev)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    backend, dev = open_device()
    print(f"opened {VID:#06x}:{PID:#06x} via python3-{backend}")

    try:
        cmd = sys.argv[1]
        if cmd == "init":
            cmd_init(backend, dev)
        elif cmd == "bl":
            cmd_bl(backend, dev, int(sys.argv[2]))
        elif cmd == "fnlock":
            cmd_fnlock(backend, dev, int(sys.argv[2]))
        elif cmd == "raw":
            cmd_raw(backend, dev, sys.argv[2])
        elif cmd == "probe":
            cmd_probe(backend, dev)
        else:
            print(f"unknown command: {cmd}", file=sys.stderr)
            sys.exit(1)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
