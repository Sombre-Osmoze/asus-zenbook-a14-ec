#!/usr/bin/env python3
"""
ASUS Zenbook A14 — PPD replacement daemon.

Replaces power-profiles-daemon with a lightweight D-Bus service that:
- Exposes the same net.hadess.PowerProfiles interface
- Advertises 3 profiles: power-saver, balanced, performance
- Controls fan via hwmon sysfs (asus_zenbook_a14_ec driver)

KDE Plasma sees this identically to real PPD.

Usage:
    sudo sv down power-profiles-daemon  # stop real PPD
    sudo python3 ppd-bridge.py          # run this
"""

import os
import glob
import signal
import sys

import dbus
import dbus.service
import dbus.mainloop.glib
from gi.repository import GLib


# --- hwmon discovery ---

HWMON_DRIVER_NAME = "asus_zenbook_a14_ec"

def find_hwmon():
    """Find hwmon path by driver name (stable across reboots)."""
    for path in glob.glob("/sys/class/hwmon/hwmon*"):
        name_file = os.path.join(path, "name")
        try:
            with open(name_file) as f:
                if f.read().strip() == HWMON_DRIVER_NAME:
                    return path
        except OSError:
            continue
    return None


# --- Profile definitions ---

PROFILES = {
    "power-saver": {
        "pwm_enable": "2",   # auto (EC manages, tends conservative)
        "pwm": None,
    },
    "balanced": {
        "pwm_enable": "2",   # auto (EC default thermal curve)
        "pwm": None,
    },
    "performance": {
        "pwm_enable": "1",   # manual
        "pwm": "180",        # ~2400 RPM — sustained cooling
    },
}

# Ordered as PPD expects (power-saver first, then balanced, then performance)
PROFILE_ORDER = ["power-saver", "balanced", "performance"]


# --- D-Bus service ---

# PPD claims both bus names; powerprofilesctl uses the new one
BUS_NAME_OLD = "net.hadess.PowerProfiles"
BUS_NAME_NEW = "org.freedesktop.UPower.PowerProfiles"
OBJ_PATH_OLD = "/net/hadess/PowerProfiles"
OBJ_PATH_NEW = "/org/freedesktop/UPower/PowerProfiles"
IFACE_OLD = "net.hadess.PowerProfiles"
IFACE_NEW = "org.freedesktop.UPower.PowerProfiles"

# We respond to both interfaces on properties calls
KNOWN_IFACES = {IFACE_OLD, IFACE_NEW}


class PowerProfilesDaemon(dbus.service.Object):
    """Implements net.hadess.PowerProfiles + org.freedesktop.UPower.PowerProfiles."""

    # Register on both object paths
    SUPPORTS_MULTIPLE_OBJECT_PATHS = True

    def __init__(self, bus, hwmon_path):
        self._bus_name_old = dbus.service.BusName(BUS_NAME_OLD, bus)
        self._bus_name_new = dbus.service.BusName(BUS_NAME_NEW, bus)

        # Register on both paths
        dbus.service.Object.__init__(self, bus, OBJ_PATH_OLD)
        self.add_to_connection(bus, OBJ_PATH_NEW)

        self._hwmon = hwmon_path
        self._active_profile = "balanced"
        self._holds = []  # ActiveProfileHolds
        self._hold_counter = 0

        # Apply initial profile
        self._apply_profile("balanced")
        print(f"[ppd-bridge] hwmon: {hwmon_path}")
        print(f"[ppd-bridge] serving on {BUS_NAME_OLD} + {BUS_NAME_NEW}")

    def _apply_profile(self, name):
        """Write fan settings to hwmon sysfs."""
        cfg = PROFILES[name]
        pwm_enable = os.path.join(self._hwmon, "pwm1_enable")
        pwm_value = os.path.join(self._hwmon, "pwm1")

        try:
            with open(pwm_enable, "w") as f:
                f.write(cfg["pwm_enable"])

            if cfg["pwm"] is not None:
                with open(pwm_value, "w") as f:
                    f.write(cfg["pwm"])

            print(f"[ppd-bridge] applied: {name} "
                  f"(pwm_enable={cfg['pwm_enable']}, pwm={cfg['pwm']})")
        except OSError as e:
            print(f"[ppd-bridge] ERROR applying {name}: {e}", file=sys.stderr)

    def _compute_effective_profile(self):
        """With holds, the most restrictive held profile wins."""
        if not self._holds:
            return self._active_profile

        # If any hold requests power-saver, that wins over balanced
        held_profiles = [h["Profile"] for h in self._holds]
        if "power-saver" in held_profiles:
            return "power-saver"
        if "performance" in held_profiles:
            return "performance"
        return self._active_profile

    # --- D-Bus Properties interface ---

    @dbus.service.method("org.freedesktop.DBus.Properties",
                         in_signature="ss", out_signature="v")
    def Get(self, interface, prop):
        if interface not in KNOWN_IFACES:
            raise dbus.exceptions.DBusException(
                f"No such interface: {interface}",
                name="org.freedesktop.DBus.Error.UnknownInterface")
        return self._get_prop(prop)

    @dbus.service.method("org.freedesktop.DBus.Properties",
                         in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface not in KNOWN_IFACES:
            raise dbus.exceptions.DBusException(
                f"No such interface: {interface}",
                name="org.freedesktop.DBus.Error.UnknownInterface")
        props = {}
        for name in ("ActiveProfile", "Profiles", "PerformanceInhibited",
                     "PerformanceDegraded", "Actions", "ActiveProfileHolds",
                     "Version"):
            props[name] = self._get_prop(name)
        return props

    @dbus.service.method("org.freedesktop.DBus.Properties",
                         in_signature="ssv")
    def Set(self, interface, prop, value):
        if interface not in KNOWN_IFACES:
            raise dbus.exceptions.DBusException(
                f"No such interface: {interface}",
                name="org.freedesktop.DBus.Error.UnknownInterface")
        if prop == "ActiveProfile":
            profile = str(value)
            if profile not in PROFILES:
                raise dbus.exceptions.DBusException(
                    f"Invalid profile: {profile}",
                    name="net.hadess.PowerProfiles.Error.InvalidProfile")
            self._active_profile = profile
            effective = self._compute_effective_profile()
            self._apply_profile(effective)
            self.PropertiesChanged(IFACE_OLD, {"ActiveProfile": profile}, [])
            self.PropertiesChanged(IFACE_NEW, {"ActiveProfile": profile}, [])
        else:
            raise dbus.exceptions.DBusException(
                f"Property {prop} is read-only",
                name="org.freedesktop.DBus.Error.PropertyReadOnly")

    @dbus.service.signal("org.freedesktop.DBus.Properties",
                         signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass

    def _get_prop(self, prop):
        if prop == "ActiveProfile":
            return dbus.String(self._active_profile)
        elif prop == "Profiles":
            profiles = dbus.Array([], signature="a{sv}")
            for name in PROFILE_ORDER:
                entry = dbus.Dictionary({
                    "Profile": dbus.String(name),
                    "Driver": dbus.String("asus_zenbook_a14_ec"),
                    "PlatformDriver": dbus.String("asus_zenbook_a14_ec"),
                }, signature="sv")
                profiles.append(entry)
            return profiles
        elif prop == "PerformanceInhibited":
            return dbus.String("")
        elif prop == "PerformanceDegraded":
            return dbus.String("")
        elif prop == "Actions":
            return dbus.Array(["trickle_charge"], signature="s")
        elif prop == "ActiveProfileHolds":
            return dbus.Array([
                dbus.Dictionary({
                    "ApplicationId": dbus.String(h["ApplicationId"]),
                    "Profile": dbus.String(h["Profile"]),
                    "Reason": dbus.String(h["Reason"]),
                }, signature="sv")
                for h in self._holds
            ], signature="a{sv}")
        elif prop == "Version":
            return dbus.String("0.30")
        else:
            raise dbus.exceptions.DBusException(
                f"Unknown property: {prop}",
                name="org.freedesktop.DBus.Error.UnknownProperty")

    # --- PPD-specific methods (new interface — what powerprofilesctl uses) ---

    @dbus.service.method(IFACE_NEW, in_signature="sss", out_signature="u")
    def HoldProfile(self, profile, reason, application_id):
        """Hold a profile (used by apps like gaming mode)."""
        if profile not in PROFILES:
            raise dbus.exceptions.DBusException(
                f"Invalid profile: {profile}",
                name="net.hadess.PowerProfiles.Error.InvalidProfile")

        self._hold_counter += 1
        cookie = self._hold_counter
        self._holds.append({
            "Profile": profile,
            "Reason": reason,
            "ApplicationId": application_id,
            "Cookie": cookie,
        })

        effective = self._compute_effective_profile()
        self._apply_profile(effective)
        self.PropertiesChanged(IFACE_NEW,
                               {"ActiveProfileHolds": self._get_prop("ActiveProfileHolds")}, [])
        print(f"[ppd-bridge] hold: {application_id} → {profile} (cookie={cookie})")
        return dbus.UInt32(cookie)

    @dbus.service.method(IFACE_NEW, in_signature="u")
    def ReleaseProfile(self, cookie):
        """Release a profile hold."""
        cookie = int(cookie)
        self._holds = [h for h in self._holds if h["Cookie"] != cookie]

        effective = self._compute_effective_profile()
        self._apply_profile(effective)
        self.PropertiesChanged(IFACE_NEW,
                               {"ActiveProfileHolds": self._get_prop("ActiveProfileHolds")}, [])
        print(f"[ppd-bridge] released hold cookie={cookie}")


def main():
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

    hwmon = find_hwmon()
    if not hwmon:
        print(f"[ppd-bridge] FATAL: hwmon for '{HWMON_DRIVER_NAME}' not found",
              file=sys.stderr)
        print("[ppd-bridge] Is the EC driver loaded?", file=sys.stderr)
        sys.exit(1)

    bus = dbus.SystemBus()
    daemon = PowerProfilesDaemon(bus, hwmon)

    loop = GLib.MainLoop()

    def shutdown(signum, frame):
        print(f"\n[ppd-bridge] caught signal {signum}, restoring balanced...")
        daemon._apply_profile("balanced")
        loop.quit()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    print("[ppd-bridge] running (Ctrl+C or SIGTERM to stop)")
    loop.run()


if __name__ == "__main__":
    main()
