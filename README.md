# asus-zenbook-a14-ec

Out-of-tree Linux kernel driver (PoC) for the Embedded Controller of the
**ASUS Zenbook A14 (UX3407RA)**, a Qualcomm X1E80100-based laptop.

## Status

**Step 4 complete.** Driver registers a `hwmon` device with **working manual PWM control**.

**A14 has no watchdog timeout** (verified 2026-05-07): 3+ min manual mode with
no temperature feed = no reboot. Watchdog kthread disabled. Manual fan control
works via direct PWM writes — no temperature babysitting needed.

**Profile write disabled**: EC register `(0x01, 0x0b)` is read-only
(firmware-controlled). Writes succeed at I²C but produce no state change.
Profile likely controlled via ACPI WMI or baked into firmware thermal tables.

### Exposed interfaces

| sysfs                                    | semantics                              |
|------------------------------------------|----------------------------------------|
| `hwmon/hwmonN/fan1_input`                | RPM (tach × 88, calibrated 1400-2200)  |
| `hwmon/hwmonN/pwm1`                      | 0-255 (RW; takes effect in manual)     |
| `hwmon/hwmonN/pwm1_enable`               | 1 = manual, 2 = auto (RW, **works**)   |
| `hwmon/hwmonN/temp1_input`               | EC thermistor, m°C                     |

**Tested end-to-end** (2026-05-07):
```bash
echo 1 > pwm1_enable   # manual mode
echo 80 > pwm1         # fan → 1408 RPM
echo 150 > pwm1        # fan → 2200 RPM (audible ramp-up)
echo 2 > pwm1_enable   # restore auto
```

Profile sysfs (`profile` / `profile_choices`) commented out; can be re-enabled
when A14 profile-write protocol is discovered.

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

- **A14 has no watchdog timeout** (verified 2026-05-07: 3+ min manual mode
  with no temp feed = no reboot). Manual PWM control is safe — no kthread,
  no temperature babysitting. Watchdog code disabled in driver.
- **Vivobook warning**: If porting this driver to Vivobook S15, re-enable
  watchdog kthread (it hard-resets after ~2 min without temp feed).
- Companion `i2c-tools` / `tool.py` user-space access on `/dev/i2c-4` is
  **mutually exclusive** with this driver — either use the userspace
  tool *or* this module, not both.
- **Profile write disabled** on A14 — EC register read-only. Writes
  via `eccw(0x01, 0x8b, n)` succeed but produce no state change. Profile
  appears firmware-controlled (ACPI WMI or thermal tables).
- If anything misbehaves: hard power-cycle and pick working kernel from
  bootloader. Module is out-of-tree, never installed to
  `/lib/modules/.../extra/` unless you run `make install` (not provided).

## License

GPL-2.0-only.
