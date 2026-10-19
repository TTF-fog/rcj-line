#!/usr/bin/env python3
"""Buttonless flasher for the Cytron Motion 2350 Pro (rcj-line firmware).

Runs ON THE Raspberry Pi. Flow:
  1. find the Motion's serial port by probing PING on /dev/ttyACM*/USB*
  2. send BOOTLOAD -> the board reboots into its RP2350 UF2 mass-storage mode
  3. wait for the RP2350 drive to appear, mount it if necessary
  4. copy firmware.uf2, sync, wait for the board to reboot itself
  5. wait for the serial port to return, confirm PING -> PONG

No BOOTSEL button, no loader tools:
    .venv/bin/python flash_rp2350.py [--uf2 ~/rcj-line/firmware.uf2] [--port /dev/ttyACM1]
"""
import argparse
import glob
import os
import subprocess
import sys
import time

BOOT_DRIVE_LABELS = {"RP2350", "MOTION2350", "RPI-RP2"}  # accepted MSC labels
BAUD = 115200


def log(msg):
    print(msg, flush=True)


def open_probe(dev):
    import serial
    try:
        return serial.Serial(dev, BAUD, timeout=0.3)
    except Exception:
        return None


def pick_probe(port):
    """Send PING on one port; return the Serial handle on PONG."""
    s = open_probe(port)
    if s is None:
        return None
    try:
        s.reset_input_buffer()
        s.write(b"PING\n")
        deadline = time.time() + 1.2
        buf = b""
        while time.time() < deadline:
            buf += s.read(256)
            if b"PONG" in buf:
                return s
    except Exception:
        pass
    try:
        s.close()
    except Exception:
        pass
    return None


def find_port(preferred=None):
    import serial  # noqa: F401  (fail loudly if pyserial missing)
    cands = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))
    if preferred:
        cands = [preferred] + [c for c in cands if c != preferred]
    for dev in cands:
        s = pick_probe(dev)
        if s is not None:
            log(f"serial: robot answered PING on {dev}")
            return dev, s
    return None, None


def list_boot_drives():
    """Return [(dev_node, label, mountpoint_or_None)] for UF2 boot-mode drives."""
    try:
        out = subprocess.run(
            ["lsblk", "-n", "-r", "-o", "NAME,LABEL,MOUNTPOINT"],
            capture_output=True, text=True, timeout=10,
        ).stdout
    except Exception:
        return []
    drives = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[1] in BOOT_DRIVE_LABELS:
            name = parts[0]
            mnt = parts[2] if len(parts) > 2 else None
            if mnt in ("", "none"):
                mnt = None
            drives.append((f"/dev/{name}", parts[1], mnt))
    return drives


def wait_for_boot_drive(timeout=40):
    log("waiting for RP2350 boot drive...")
    end = time.time() + timeout
    while time.time() < end:
        drives = list_boot_drives()
        if drives:
            return drives[0]
        time.sleep(0.5)
    return None


def mount_via_udisks(dev_node):
    try:
        out = subprocess.run(
            ["udisksctl", "mount", "-b", dev_node],
            capture_output=True, text=True, timeout=15,
        )
        for tok in out.stdout.split():
            if tok.startswith("/media") or tok.startswith("/run/media"):
                return tok
    except Exception:
        pass
    return None


def flash_uf2(drive_mount, uf2):
    log(f"copying {uf2} -> {drive_mount}")
    rv = subprocess.run(["cp", uf2, drive_mount], capture_output=True, text=True)
    if rv.returncode != 0:
        # some auto-mounts are root-owned; retry with sudo
        rv2 = subprocess.run(
            ["/usr/bin/sudo", "-n", "cp", uf2, drive_mount],
            capture_output=True, text=True,
        )
        if rv2.returncode != 0:
            log("ERROR: copy failed: " + (rv.stderr.strip() or rv2.stderr.strip()))
            return False
    subprocess.run(["sync"], timeout=15)
    return True


def wait_boot_drive_gone(mount, timeout=30):
    end = time.time() + timeout
    while time.time() < end:
        drives = list_boot_drives()
        if not drives:
            return True
        time.sleep(0.5)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--uf2", default=os.path.expanduser("~/rcj-line/firmware.uf2"))
    ap.add_argument("--port", default=None, help="explicit serial device, e.g. /dev/ttyACM1")
    args = ap.parse_args()

    if not os.path.isfile(args.uf2):
        log(f"ERROR: uf2 not found: {args.uf2}")
        return 2

    dev, s = find_port(args.port)
    if dev is None:
        log("ERROR: no robot serial port answered PING "
            "(check USB cable; boot the firmware once with BOOTSEL if needed)")
        return 3

    try:
        log("sending BOOTLOAD -> rebooting into UF2 bootloader...")
        s.write(b"BOOTLOAD\n")
        s.flush()
        deadline = time.time() + 3.0
        got_ok = False
        buf = b""
        while time.time() < deadline:
            buf += s.read(256)
            if b"OK: rebooting into UF2 bootloader" in buf:
                got_ok = True
                break
        if not got_ok:
            log("WARN: no OK echo before the port dropped (continuing anyway)")
    finally:
        try:
            s.close()
        except Exception:
            pass

    drive = wait_for_boot_drive(40)
    if drive is None:
        log("ERROR: RP2350 boot drive never appeared")
        return 4
    dev_node, label, mount = drive
    log(f"boot drive: {dev_node} label={label}")

    if mount is None:
        mount = mount_via_udisks(dev_node)
        if mount is None:
            log("ERROR: could not mount the boot drive")
            return 5
    log(f"mounted at {mount}")

    if not flash_uf2(mount, args.uf2):
        return 6

    ok = wait_boot_drive_gone(mount, 30)
    if not ok:
        log("WARN: drive still mounted - board may already have rebooted")
    else:
        log("boot drive gone - board reset with new firmware")

    log("waiting for serial port to return...")
    port = None
    s2 = None
    end = time.time() + 20
    while time.time() < end and s2 is None:
        import serial
        cands = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))
        for c in cands:
            s2 = pick_probe(c)
            if s2 is not None:
                port = c
                log(f"firmware back up on {port} (PING -> PONG)")
                break
        if s2 is None:
            time.sleep(0.5)

    if s2 is None:
        log("ERROR: robot did not answer PING after flashing")
        return 7

    try:
        deadline = time.time() + 2.0
        s2.reset_input_buffer()
        s2.write(b"\nGSTATUS\n")
        buf = b""
        t_end = time.time() + 2.0
        while time.time() < t_end:
            buf += s2.read(256)
        for line in buf.decode(errors="replace").splitlines()[:8]:
            log(f"    {line}")
        s2.close()
    except Exception:
        pass

    log("FLASH OK")
    return 0


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    sys.exit(main())
