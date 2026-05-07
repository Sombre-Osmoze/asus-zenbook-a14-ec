# asus-zenbook-a14-ec

Out-of-tree Linux kernel driver (PoC) for the Embedded Controller of the
**ASUS Zenbook A14 (UX3407RA)**, a Qualcomm X1E80100-based laptop.

## Status

**Step 4.** Driver registers a `hwmon` device with writable PWM control,
runs an EC-temperature watchdog kthread when in manual fan mode, and handles
suspend/resume safely.

**Profile write on A14 is disabled**: EC register `(0x01, 0x0b)` is read-only
(firmware-controlled). Writes succeed at the I²C level but produce no state
change. Profile likely controlled via ACPI WMI method or baked into firmware
thermal tables. See commit `6fef937` for investigation details.

### Exposed interfaces

| sysfs                                    | semantics                              |
|------------------------------------------|----------------------------------------|
| `hwmon/hwmonN/fan1_input`                | RPM (tach × 88, calibrated)            |
| `hwmon/hwmonN/pwm1`                      | 0-255 (RW; only takes effect manual)   |
| `hwmon/hwmonN/pwm1_enable`               | 1 = manual, 2 = auto (RW)              |
| `hwmon/hwmonN/temp1_input`               | EC thermistor, m°C                     |

Profile sysfs (`profile` / `profile_choices`) is commented out in code;
can be re-enabled when A14 profile-write protocol is discovered.

## Roadmap

1. ✅ Scaffold (probe/remove, adapter lookup, client instantiation)
2. ✅ EC protocol layer (`ec_settle`, `ecrb`/`ecwb`/`eccr`/`eccw`) with mutex
3. ✅ `hwmon` registration: `fan1_input`, `pwm1`, `pwm1_enable`, `temp1`
4. ✅ Writable PWM + temperature-watchdog kthread + suspend/resume PM ops
5. ⏸️ Profile write — A14 EC register read-only; needs ACPI method investigation
6. (Later) Keyboard backlight LED class — protocol cracked; integrate `hid-asus-ec` work
7. (Later) DKMS packaging
8. (Later) Proper DT bindings for upstream submission

## Build

```sh
make
```

Builds against the running kernel headers
(`/lib/modules/$(uname -r)/build`). To build against a specific tree:

```sh
make KDIR=/path/to/linux
```

## Load / unload

```sh
make load     # sudo insmod ./asus_zenbook_a14_ec.ko
make unload   # sudo rmmod asus_zenbook_a14_ec
make reload
make dmesg    # tail driver log lines
```

## Safety

- **EC watchdog**: when `pwm1_enable=1` (manual), a kthread feeds the EC
  the max SoC thermal-zone temperature every 2s. If this stream stops
  for ~2 minutes the EC will hard-reset the box. The driver:
  - sends one temp synchronously **before** flipping to manual
  - sets EC back to auto **before** stopping the kthread
  - forces auto + stops kthread on `rmmod` and on suspend
  - restores manual on resume only after restarting the kthread
- Companion `i2c-tools` / `tool.py` user-space access on `/dev/i2c-4` is
  **mutually exclusive** with this driver — either use the userspace
  tool *or* this module, not both.
- **Profile write disabled** on A14 — EC register is read-only. Writes
  via `eccw(0x01, 0x8b, n)` succeed but produce no state change. Profile
  appears firmware-controlled.
- If anything misbehaves: hard power-cycle the laptop and pick the
  working kernel from the bootloader. The module is out-of-tree and
  never installed to `/lib/modules/.../extra/` unless you run
  `make install` (not provided).

## License

GPL-2.0-only.
