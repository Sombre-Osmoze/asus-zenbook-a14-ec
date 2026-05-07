#!/usr/bin/env python3
"""
Capture vendor input events from ASUS keyboard 0B05:0220.

Reads from the device until Ctrl-C. Vendor INPUT report ID 0x5A
is 5 bytes per icecream95's BPF analysis (Vivobook S15 schema):
    [0x5A, key_code, 0x00, ...]

Press Fn+F1 .. Fn+F12 one at a time; the script prints each report.
Compare against icecream95's known codes:
    0x10 = Brightness-       0x4E = Fn-lock
    0x20 = Brightness+       0x7C = Mic mute
    0x7E = Emoji             0xCB = Mic mode
    0x86 = MyASUS            0x9D = Fn+F (profile cycle)
    0xC7 = Kbd backlight cycle
"""
import hid
import os
import sys

VID = 0x0B05
PID = int(os.environ.get("PID", "0x0220"), 0)

def main():
    d = hid.Device(VID, PID)
    print(f"reading from {VID:#06x}:{PID:#06x}, press Ctrl-C to stop")
    print("press Fn keys one at a time...")
    try:
        while True:
            r = d.read(64, timeout=5000)
            if r:
                print(f"{r.hex(' ')}")
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        d.close()

if __name__ == "__main__":
    main()
