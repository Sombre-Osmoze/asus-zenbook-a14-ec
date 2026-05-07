# asus-zenbook-a14-ec

Out-of-tree Linux kernel drivers for the **ASUS Zenbook A14 (UX3407RA)**,
a Qualcomm X1E80100-based laptop. Provides fan control, power profiles,
keyboard backlight, and Fn hotkeys.

## Modules

| Module                  | Function                                         |
|-------------------------|--------------------------------------------------|
| `asus_zenbook_a14_ec`   | EC hwmon (fan/PWM/temp) + platform_profile       |
| `hid_asus_ec`           | Keyboard backlight LED class + Fn hotkeys        |

## Status

### EC driver (`asus_zenbook_a14_ec`)

- **hwmon**: `fan1_input`, `pwm1`, `pwm1_enable`, `temp1_input`
- **Manual PWM**: works (no watchdog on A14 — safe indefinitely)
- **platform_profile**: `quiet` / `balanced` / `performance`
  - quiet/balanced → EC auto mode
  - performance → manual PWM 180 (~2400 RPM sustained)
  - Requires patched `platform_profile.ko` (see `patches/`)
- **Suspend/resume**: clean, no errors
- **Profile write disabled**: EC register `(0x01, 0x0b)` is read-only

### HID driver (`hid_asus_ec`)

- **Keyboard backlight**: LED class `asus::kbd_backlight`, 4 levels (0-3)
- **Fn hotkeys**: Fn+F4 (backlight cycle), Fn+F5/F6 (brightness), Fn+F8
  (emoji), Fn+F9 (micmute), Fn+F10 (camera), Fn+F11 (touchpad),
  Fn+F12 (PROG1), Fn+F (performance)
- **Suspend/resume**: saves/restores backlight level
- Target device: `0B05:0220` (I2C-HID keyboard)

### PPD bridge (`scripts/ppd-bridge.py`)

Temporary userspace replacement for `power-profiles-daemon`. Exposes all
3 profiles on D-Bus so KDE Plasma's Energy Saving dropdown works. Will be
obsoleted once the kernel `platform_profile` patch is applied and the real
PPD detects the class device.

## Kernel patch

`patches/0001-platform_profile-allow-non-ACPI-systems.patch`

Removes the `acpi_disabled` guard from `drivers/acpi/platform_profile.c`
so the class device registration works on DT-only ARM64 systems. The
legacy `/sys/firmware/acpi/platform_profile` node is skipped when
`acpi_kobj` is NULL; the class interface
(`/sys/class/platform-profile/platform-profile-0/`) works regardless.

Apply to your kernel tree before building:
```sh
cd /path/to/linux
git apply /path/to/patches/0001-platform_profile-allow-non-ACPI-systems.patch
```

## Build

```sh
make                # builds both .ko against running kernel
make KDIR=/path/to/linux   # build against specific tree
```

## Load / unload

```sh
# EC driver (load platform_profile first if not built-in)
sudo modprobe platform_profile
sudo insmod ./asus_zenbook_a14_ec.ko

# HID driver
sudo insmod ./hid_asus_ec.ko

# Shortcuts
make load     # insmod EC driver
make unload   # rmmod EC driver
make reload   # rmmod + insmod
make dmesg    # tail driver log
```

## Exposed interfaces

| sysfs                                    | semantics                              |
|------------------------------------------|----------------------------------------|
| `hwmon/hwmonN/fan1_input`                | RPM (tach × 88, calibrated 1400-2200)  |
| `hwmon/hwmonN/pwm1`                      | 0-255 (RW; takes effect in manual)     |
| `hwmon/hwmonN/pwm1_enable`               | 1 = manual, 2 = auto (RW)             |
| `hwmon/hwmonN/temp1_input`               | EC thermistor, m°C                     |
| `leds/asus::kbd_backlight/brightness`    | 0-3 (keyboard backlight)               |
| `class/platform-profile/platform-profile-0/profile` | quiet/balanced/performance |

## Safety

- **A14 has no watchdog timeout** (verified: 3+ min manual mode = no
  reboot). Manual PWM control is safe without temperature babysitting.
- **Vivobook warning**: If porting to Vivobook S15, re-enable watchdog
  kthread (hard-resets after ~2 min without temp feed).
- Companion `tool.py` user-space access on `/dev/i2c-4` is **mutually
  exclusive** with this driver.
- If anything misbehaves: hard power-cycle and pick working kernel from
  bootloader.

## Credits

- **Sombre-Osmoze** <sombre@osmoze.xyz> — EC reverse-engineering, hwmon
  driver, platform_profile integration, PPD bridge, kernel patch
- **Alexandru Marc Serdeliuc** <serdeliuk@yahoo.com> — HID keyboard
  backlight driver (`hid-asus-ec`), original QA work that confirmed the
  backlight protocol on Zenbook A14
- **icecream95** — udev-hid-bpf work on Vivobook S15/Zenbook A14,
  early EC protocol documentation

## License

GPL-2.0-only (EC driver, kernel patch).
GPL-2.0-or-later (HID driver, per serdeliuk's original).
