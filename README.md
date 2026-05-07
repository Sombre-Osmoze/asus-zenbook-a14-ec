# asus-zenbook-a14-ec

Out-of-tree Linux kernel driver (PoC) for the Embedded Controller of the
**ASUS Zenbook A14 (UX3407RA)**, a Qualcomm X1E80100-based laptop.

## Status

**Step 1 scaffold.** The module loads, finds the I²C adapter behind the
platform device `b94000.i2c`, instantiates two `i2c_client`s at the EC
address (`0x5b`) and the companion fan-controller address (`0x76`), and
exits cleanly on `rmmod`. **No EC I/O is performed yet.**

## Roadmap

1. ✅ Scaffold (probe/remove, adapter lookup, client instantiation)
2. EC protocol layer (`ec_settle`, `ecrb`/`ecwb`/`eccr`/`eccw`) with mutex
3. `hwmon` registration: `fan1_input`, `pwm1`, `pwm1_enable`, `temp1`
4. (Later) `platform_profile` integration → KDE Energy Saving dropdown
5. (Later) Temperature watchdog kthread (required for safe `mode auto`)
6. (Later) Keyboard backlight LED class
7. (Later) Proper DT bindings for upstream submission

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

- The driver is intentionally read-only of the bus until step 2 lands.
- All EC writes will go through a per-device mutex once added.
- Companion `i2c-tools` user-space access on `/dev/i2c-4` will be **mutually
  exclusive** with this driver; either use `tool.py` *or* this module, not
  both.
- If anything misbehaves: hard power-cycle the laptop and pick the working
  kernel from the bootloader. The module is out-of-tree and never
  installed to `/lib/modules/.../extra/` unless you run `make install`
  (not provided).

## License

GPL-2.0-only.
