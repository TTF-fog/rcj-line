#!/usr/bin/env python3
"""
Interactive & Scriptable CLI for the Robocup 4-Motor Controller (Teensy).
Connects over UART/USB-serial to send commands to the Teensy motor controller,
including the gyro straight-hold (GFWD/GBACK/...) and gyro turn
(GTURN/GTL90/GTR90/GTL45/GTR45/...) commands.
"""

import sys
import time
import argparse
import queue
import threading
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


# Lines streamed continuously by the MCU (not command responses)
TELEMETRY_PREFIXES = ("Q,", "G,", "T,", "#")
ACK_PREFIXES = ("OK:", "ERR:", "STATUS:", "GSTATUS:", "PONG")
TURN_DONE_PREFIXES = ("TDONE", "TABORT")

# Gyro turn commands report completion asynchronously with TDONE/TABORT
TURN_COMMANDS = ("GTURN", "GLEFT", "GRIGHT", "GTL90", "GTR90", "GTL45", "GTR45")


class MotorCLI:
    def __init__(self, port: Optional[str] = None, baud: int = 115200):
        self.port_name = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.running = True
        self.show_telemetry = False
        self._rx_q: "queue.Queue[str]" = queue.Queue(maxsize=256)
        self._reader: Optional[threading.Thread] = None

    def find_default_port(self) -> Optional[str]:
        """Auto-detect available serial ports."""
        if serial is None:
            return None
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            return None
        # Prefer ACM or USB ports
        for p in ports:
            if "ACM" in p.device or "USB" in p.device:
                return p.device
        return ports[0].device

    def connect(self):
        """Open serial connection and start the RX reader thread."""
        self.running = True

        if serial is None:
            raise RuntimeError("pyserial is not installed. Run with Python environment containing pyserial.")

        if not self.port_name:
            detected = self.find_default_port()
            if detected:
                print(f"[INFO] Auto-detected port: {detected}")
                self.port_name = detected
            else:
                print("[WARN] No serial port detected. Plug in the robot or use --port /dev/ttyACM0.")
                return

        try:
            print(f"[INFO] Connecting to {self.port_name} at {self.baud} baud...")
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.2)
            time.sleep(0.5)  # Let connection settle
            self._start_reader()
            print("[INFO] Connected successfully.")
        except Exception as e:
            print(f"[ERROR] Failed to open port {self.port_name}: {e}")

    # --------------------------------------------------------------------- rx
    def _start_reader(self):
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    def _reader_loop(self):
        ser = self.ser
        while self.running and ser is not None and ser.is_open:
            try:
                raw = ser.readline()
            except Exception:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            try:
                self._rx_q.put_nowait(line)
            except queue.Full:
                pass  # buffer full: drop stale telemetry rather than block

    def drain_lines(self) -> list:
        """Return and clear all lines buffered by the reader thread."""
        out = []
        while True:
            try:
                out.append(self._rx_q.get_nowait())
            except queue.Empty:
                return out

    @staticmethod
    def _is_telemetry(line: str) -> bool:
        return line.startswith(TELEMETRY_PREFIXES)

    def _getline(self, deadline: float) -> Optional[str]:
        remaining = deadline - time.time()
        if remaining <= 0:
            return None
        try:
            return self._rx_q.get(timeout=min(remaining, 0.05))
        except queue.Empty:
            return None

    def _echo(self, line: str, show: bool):
        if show:
            print(f"[RX] {line}")

    # --------------------------------------------------------------------- tx
    def send(self, command: str, wait_done_s: Optional[float] = None) -> Optional[str]:
        """Send command string to microcontroller and wait for response.

        For gyro turn commands pass wait_done_s (e.g. 8.0) to additionally
        wait for the asynchronous TDONE/TABORT completion event.
        """
        cmd = command.strip()
        if not cmd:
            return None

        if not self.ser or not self.ser.is_open:
            print("[ERROR] Serial port is not open.")
            return None

        try:
            # Clear stale incoming lines, then send
            self.drain_lines()
            self.ser.write((cmd + "\n").encode("utf-8"))
            self.ser.flush()
        except Exception as e:
            print(f"[ERROR] Communication error: {e}")
            return None

        # Wait up to 0.6s for command response from MCU
        deadline = time.time() + 0.6
        response_lines = []
        while time.time() < deadline:
            line = self._getline(deadline)
            if line is None:
                break
            if self._is_telemetry(line):
                self._echo(line, self.show_telemetry)
                continue
            response_lines.append(line)
            if line.startswith(ACK_PREFIXES):
                break

        # Turn completion arrives asynchronously (the turn runs on the MCU)
        if wait_done_s:
            deadline = time.time() + wait_done_s
            while time.time() < deadline:
                line = self._getline(deadline)
                if line is None:
                    break
                if self._is_telemetry(line):
                    self._echo(line, self.show_telemetry)
                    continue
                response_lines.append(line)
                if line.startswith(TURN_DONE_PREFIXES) or line.startswith("ERR:"):
                    break
            if not any(l.startswith(TURN_DONE_PREFIXES) for l in response_lines):
                print(f"[WARN] No TDONE/TABORT within {wait_done_s}s "
                      "(turn may still be running; check gstatus)")

        resp_str = "\n".join(response_lines)
        if resp_str:
            print(f"[RX] {resp_str}")
        return resp_str

    def print_help(self):
        print("""
==================== Robocup 4-Motor Commands =====================
  fwd [speed]            : Drive forward (speed 0..255, default 150)
  back [speed]           : Drive backward (speed 0..255, default 150)
  steer <speed> <ratio>  : Steer differential drive (-1.0=Left .. 1.0=Right)
  left [speed]           : Pivot/spin left in place (default 120)
  right [speed]          : Pivot/spin right in place (default 120)
  speed <left> <right>   : Set left (M1,M2) and right (M3,M4) speeds (-255..255)
  motor <1..4> <speed>   : Drive an individual motor directly
  motors <m1..m4>        : Set speeds for all 4 motors (M1 M2 M3 M4)
  stop                   : Coast all motors to stop
  brake                  : Active dynamic short-brake
  status                 : Query current speeds and status
  ---- Gyro straight-hold (P on heading error) ----
  gfwd [speed] [kp]      : Drive straight fwd, heading latched (default 150)
  gback [speed] [kp]     : Drive straight rev, heading latched (default 150)
  gstop                  : Exit gyro hold + stop motors
  gkp <kp>               : Set straight P gain, PWM per deg (0..20)
  gmax <max>             : Set max correction PWM (0..255)
  gstatus                : Query gyro hold/turn state + heading
  ---- Gyro turns (in-place, heading target; waits for TDONE) ----
  gturn <+/-deg> [s] [k] : Relative turn, +deg=left/CCW, -deg=right/CW
  gleft <deg> [s] [k]    : Gyro turn left by deg (e.g. gleft 90)
  gright <deg> [s] [k]   : Gyro turn right by deg (e.g. gright 45)
  gtl90 [speed] [kp]     : Gyro turn exactly 90 deg left
  gtr90 [speed] [kp]     : Gyro turn exactly 90 deg right
  gtl45 [speed] [kp]     : Gyro turn exactly 45 deg left
  gtr45 [speed] [kp]     : Gyro turn exactly 45 deg right
  gtkp <kp>              : Turn P gain, PWM per deg (0..50)
  gtmin <pwm>            : Turn stall-floor PWM near target (0..255)
  gttol <deg>            : Turn done tolerance (0.5..20)
  help                   : Display this help message
  quit / exit            : Exit CLI
=====================================================================
""")

    def interactive_mode(self):
        """Interactive Read-Eval-Print Loop."""
        self.print_help()
        while self.running:
            try:
                self._echo_pending()
                line = input("motor> ").strip()
                if not line:
                    continue
                cmd_lower = line.lower()
                if cmd_lower in ("exit", "quit", "q"):
                    break
                elif cmd_lower in ("help", "h", "?"):
                    self.print_help()
                    continue

                # Shortcut expansions
                parts = line.split()
                c = parts[0].lower()
                args = parts[1:]

                if c == "fwd" or c == "forward":
                    spd = args[0] if args else "150"
                    self.send(f"FWD {spd}")
                elif c in ("back", "rev", "backward"):
                    spd = args[0] if args else "150"
                    self.send(f"BACK {spd}")
                elif c == "steer":
                    if len(args) == 2:
                        self.send(f"STEER {args[0]} {args[1]}")
                    elif len(args) == 1:
                        self.send(f"STEER 150 {args[0]}")
                    else:
                        print("Usage: steer <speed> <ratio> (e.g. steer 180 0.5)")
                elif c == "left":
                    spd = args[0] if args else "120"
                    self.send(f"LEFT {spd}")
                elif c == "right":
                    spd = args[0] if args else "120"
                    self.send(f"RIGHT {spd}")
                elif c in ("speed", "m"):
                    if len(args) == 2:
                        self.send(f"SPEED {args[0]} {args[1]}")
                    else:
                        print("Usage: speed <left [-255..255]> <right [-255..255]>")
                elif c == "gfwd":
                    spd = args[0] if len(args) >= 1 else "150"
                    kp = f" {args[1]}" if len(args) >= 2 else ""
                    self.send(f"GFWD {spd}{kp}")
                elif c == "gback":
                    spd = args[0] if len(args) >= 1 else "150"
                    kp = f" {args[1]}" if len(args) >= 2 else ""
                    self.send(f"GBACK {spd}{kp}")
                elif c == "gstop":
                    self.send("GSTOP")
                elif c == "gkp":
                    if len(args) == 1:
                        self.send(f"GKP {args[0]}")
                    else:
                        print("Usage: gkp <0.0..20.0> (e.g. gkp 2.5)")
                elif c == "gmax":
                    if len(args) == 1:
                        self.send(f"GMAX {args[0]}")
                    else:
                        print("Usage: gmax <0..255> (e.g. gmax 80)")
                elif c == "gstatus":
                    self.send("GSTATUS")
                elif c == "gturn":
                    if args:
                        self._send_turn("GTURN " + " ".join(args))
                    else:
                        print("Usage: gturn <+/-deg> [speed] [kp] (e.g. gturn -90 150)")
                elif c in ("gleft", "gright"):
                    if args:
                        self._send_turn(f"{c.upper()} " + " ".join(args))
                    else:
                        print(f"Usage: {c} <deg> [speed] [kp] (e.g. {c} 45 150)")
                elif c in ("gtl90", "gtr90", "gtl45", "gtr45"):
                    extra = (" " + " ".join(args)) if args else ""
                    self._send_turn(c.upper() + extra)
                elif c == "gtkp":
                    if len(args) == 1:
                        self.send(f"GTKP {args[0]}")
                    else:
                        print("Usage: gtkp <0.0..50.0> (e.g. gtkp 4)")
                elif c == "gtmin":
                    if len(args) == 1:
                        self.send(f"GTMIN {args[0]}")
                    else:
                        print("Usage: gtmin <0..255> (e.g. gtmin 60)")
                elif c == "gttol":
                    if len(args) == 1:
                        self.send(f"GTTOL {args[0]}")
                    else:
                        print("Usage: gttol <0.5..20> (e.g. gttol 2)")
                elif c == "stop":
                    self.send("STOP")
                elif c in ("brake", "brk"):
                    self.send("BRAKE")
                elif c == "status":
                    self.send("STATUS")
                else:
                    # Pass raw command directly
                    self.send(line)

            except (EOFError, KeyboardInterrupt):
                print("\n[INFO] Exiting Motor CLI.")
                break

    def _send_turn(self, cmd: str):
        """Send a gyro turn command and wait for its TDONE/TABORT event."""
        self.send(cmd, wait_done_s=8.0)

    def _echo_pending(self):
        """Print any lines that arrived asynchronously (e.g. TDONE events)."""
        for line in self.drain_lines():
            if self._is_telemetry(line):
                self._echo(line, self.show_telemetry)
            else:
                print(f"[RX] {line}")

    def close(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()


def main():
    parser = argparse.ArgumentParser(description="Robocup 4-Motor Controller CLI")
    parser.add_argument("--port", "-p", type=str, default=None,
                        help="Serial port (e.g. /dev/ttyACM0 or /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Echo G,/T, gyro telemetry lines while waiting")
    parser.add_argument("--done-timeout", "-t", type=float, default=8.0,
                        help="Seconds to wait for TDONE after gyro turn commands (default: 8)")
    parser.add_argument("command", nargs="*",
                        help="Optional command to execute once and exit (e.g. 'fwd 150', 'gtl90', 'stop')")

    args = parser.parse_args()

    cli = MotorCLI(port=args.port, baud=args.baud)
    cli.show_telemetry = args.verbose
    cli.connect()

    if args.command:
        # Run one-off command
        full_cmd = " ".join(args.command)
        first = full_cmd.split()[0].upper()
        if first in TURN_COMMANDS:
            cli.send(full_cmd, wait_done_s=args.done_timeout)
        else:
            cli.send(full_cmd)
        cli.close()
    else:
        # Enter interactive mode
        try:
            cli.interactive_mode()
        finally:
            cli.close()


if __name__ == "__main__":
    main()
