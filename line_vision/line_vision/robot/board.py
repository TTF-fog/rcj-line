import glob
import queue
import threading
import time
from pathlib import Path

import serial

from . import tof
from .log import log

TELEMETRY = ("Q,", "G,", "T,", "#")
MAX_PWM = 90


class Aborted(Exception):
    """Raised inside any board call once Lack of Progress is pressed."""


class Board:
    """Text commands to the Motion 2350 over USB serial (see rcj-line/docs/HARDWARE_GUIDE.md)."""

    def __init__(self, port, cfg, dry=False):
        self.cfg = cfg
        self.dry = dry
        self.abort = threading.Event()
        self.lines = queue.Queue()
        self.tof = None
        self.tof_time = 0.0
        self.port = "dry run" if dry else port
        if dry:
            return
        if not Path(port).exists():
            port = (sorted(glob.glob("/dev/ttyACM*")) or [port])[0]
        self.port = port
        self.ser = serial.Serial(port, 115200, timeout=0.2)
        threading.Thread(target=self._read, daemon=True).start()
        if not self.ping():
            raise RuntimeError(f"board on {port} does not answer PING")
        log(f"board on {port}")

    def _read(self):
        while True:
            line = self.ser.readline().decode(errors="replace").strip()
            if line.startswith("TOF,"):
                try:
                    self.tof = tof.parse(line)
                    self.tof_time = time.monotonic()
                except ValueError:
                    pass
                continue
            if line and not line.startswith(TELEMETRY):
                if line.startswith("ERR"):
                    log(f"board: {line}")
                self.lines.put(line)

    def _write(self, cmd):
        if self.dry:
            print(f"> {cmd}", flush=True)
        else:
            self.ser.write(f"{cmd}\r\n".encode())

    def check(self):
        if self.abort.is_set():
            raise Aborted

    def send(self, cmd):
        self.check()
        self._write(cmd)

    def wait_for(self, prefixes, timeout):
        """Block until the board prints a line starting with one of prefixes; None on timeout."""
        if self.dry:
            return prefixes[0]
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.check()
            try:
                line = self.lines.get(timeout=0.05)
            except queue.Empty:
                continue
            if line.startswith(prefixes):
                return line
        log(f"board: no {prefixes} within {timeout}s")
        return None

    def ping(self):
        self.lines.queue.clear()
        self._write("PING")
        return self.wait_for(("PONG",), 1.0) is not None

    def sleep(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.check()
            time.sleep(0.02)

    def drive(self, forward, turn):
        """Forward and turn in PWM; positive turn steers right."""
        left = max(-MAX_PWM, min(MAX_PWM, round(forward + turn)))
        right = max(-MAX_PWM, min(MAX_PWM, round(forward - turn)))
        self.send(f"SPEED {left} {right}")

    def stop(self):
        self.send("STOP")

    def turn(self, deg):
        """Gyro turn in place; positive = left."""
        self.lines.queue.clear()
        self.send(f"G TURN {deg:+.0f} {self.cfg.turn_speed}")
        self.wait_for(("TDONE", "TABORT"), 3 + abs(deg) / 30)

    def straight(self, cm):
        """Gyro straight-hold for a distance; negative = backwards."""
        if not cm:
            return
        self.send(f"G {'FWD' if cm > 0 else 'BACK'} {self.cfg.move_speed}")
        self.sleep(abs(cm) / self.cfg.move_cm_per_s)
        self.send("G STOP")

    def grab(self, side):
        """Full pickup; side L stores the ball left (silver), R right (black)."""
        self.lines.queue.clear()
        self.send(f"CLAW GRAB {side}")
        self.wait_for(("CLAWSEQ done",), 5)

    def drop(self, side):
        self.send(self.cfg.drop_cmds[side])

    def light(self, colour):
        self.send(f"LIGHT {colour}")

    def beep(self, ms=150, hz=2730):
        self._write(f"BEEP {ms} {hz}")

    def halt(self):
        """Stop everything, even after Lack of Progress was pressed."""
        for cmd in ("G STOP", "STOP", "CLAW ABORT", self.cfg.drop_cmds["N"]):
            self._write(cmd)
