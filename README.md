# asus-zenbook-a14-ec

Out-of-tree Linux kernel driver (PoC) for the Embedded Controller of the
**ASUS Zenbook A14 (UX3407RA)**, a Qualcomm X1E80100-based laptop.

## Status

**Step 5.** Driver registers a `hwmon` device with writable PWM control,
runs an EC-temperature watchdog kthread when in manual fan mode, handles
suspend/resume safely, and exposes `platform_profile` (quiet / balanced /
balanced-performance / performance).

### Exposed interfaces

| sysfs                                    | semantics                              |
|------------------------------------------|----------------------------------------|
| `hwmon/hwmonN/fan1_input`                | RPM (tach × 88, calibrated)            |
| `hwmon/hwmonN/pwm1`                      | 0-255 (RW; only takes effect manual)   |
| `hwmon/hwmonN/pwm1_enable`               | 1 = manual, 2 = auto (RW)              |
| `hwmon/hwmonN/temp1_input`               | EC thermistor, m°C                     |
| `/sys/devices/platform/asus_zenbook_a14_ec/profile`         | RW: quiet/balanced/balanced-performance/performance |
| `/sys/devices/platform/asus_zenbook_a14_ec/profile_choices` | RO: space-separated list               |

> **Profile interface note**: the kernel's `platform_profile` framework
> requires ACPI (`acpi_disabled` check + `acpi_kobj` sysfs root) and is
> therefore unusable on DT-based ARM64. We expose the same string
> vocabulary as a custom sysfs group on the platform device. KDE
> PowerDevil's "Power Profile" dropdown will not pick this up natively
> — use the sysfs path directly or write a small bridge to PPD. Once a
> non-ACPI `platform_profile` lands upstream, this swaps to a one-liner
> `devm_platform_profile_register()` call.

## Roadmap

1. ✅ Scaffold (probe/remove, adapter lookup, client instantiation)
2. ✅ EC protocol layer (`ec_settle`, `ecrb`/`ecwb`/`eccr`/`eccw`) with mutex
3. ✅ `hwmon` registration: `fan1_input`, `pwm1`, `pwm1_enable`, `temp1`
4. ✅ Writable PWM + temperature-watchdog kthread + suspend/resume PM ops
5. ✅ `platform_profile` integration (KDE Energy Saving dropdown)
6. (Later) Keyboard backlight LED class — blocked on protocol RE
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
- Profile-set protocol is the documented Vivobook approach
  (`Request(0x76).write(0x24, idx)`). The driver verifies via the
  `eccr(0x01, 0x0b)` readback hypothesis on every set — see source.
- If anything misbehaves: hard power-cycle the laptop and pick the
  working kernel from the bootloader. The module is out-of-tree and
  never installed to `/lib/modules/.../extra/` unless you run
  `make install` (not provided).

## License

GPL-2.0-only.
