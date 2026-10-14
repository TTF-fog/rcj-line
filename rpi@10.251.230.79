#!/usr/bin/env python3
"""
Interactive & Scriptable CLI for TB6612 Motor Control
Connects over UART to send commands to the Teensy motor controller.
"""

import sys
import time
import argparse
import threading
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


class MotorCLI:
    def __init__(self, port: Optional[str] = None, baud: int = 115200, mock: bool = False):
        self.port_name = port
        self.baud = baud
        self.mock = mock
        self.ser: Optional[serial.Serial] = None
        self.running = True

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
        """Open serial connection."""
        if self.mock:
            print("[INFO] Running in MOCK mode (no physical serial port used).")
            return

        if serial is None:
            raise RuntimeError("pyserial is not installed. Run with Python environment containing pyserial.")

        if not self.port_name:
            detected = self.find_default_port()
            if detected:
                print(f"[INFO] Auto-detected port: {detected}")
                self.port_name = detected
            else:
                print("[WARN] No serial port detected. Switching to MOCK mode.")
                self.mock = True
                return

        try:
            print(f"[INFO] Connecting to {self.port_name} at {self.baud} baud...")
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.2)
            time.sleep(0.5)  # Let connection settle
            print("[INFO] Connected successfully.")
        except Exception as e:
            print(f"[ERROR] Failed to open port {self.port_name}: {e}")
            print("[INFO] Switching to MOCK mode.")
            self.mock = True

    def send(self, command: str) -> Optional[str]:
        """Send command string to microcontroller and wait for response."""
        cmd = command.strip()
        if not cmd:
            return None

        if self.mock:
            print(f"[MOCK TX] -> {cmd}")
            # Simulate quick acknowledgment
            response = f"[MOCK RX] OK: {cmd}"
            print(response)
            return response

        if not self.ser or not self.ser.is_open:
            print("[ERROR] Serial port is not open.")
            return None

        try:
            # Clear stale incoming buffer before sending
            self.ser.reset_input_buffer()
            self.ser.write((cmd + "\n").encode("utf-8"))
            self.ser.flush()

            # Wait up to 0.6s for command response from MCU
            deadline = time.time() + 0.6
            response_lines = []
            while time.time() < deadline:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        if line.startswith("OK:") or line.startswith("ERR:") or line.startswith("STATUS:") or line.startswith("PONG"):
                            response_lines.append(line)
                            break
                        elif not line.startswith("Q,") and not line.startswith("#"):
                            response_lines.append(line)
                else:
                    time.sleep(0.005)

            resp_str = "\n".join(response_lines)
            if resp_str:
                print(f"[RX] {resp_str}")
            return resp_str
        except Exception as e:
            print(f"[ERROR] Communication error: {e}")
            return None

    def print_help(self):
        print("""
======================= TB6612 Motor Commands =======================
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
  help                   : Display this help message
  quit / exit            : Exit CLI
=====================================================================
""")

    def interactive_mode(self):
        """Interactive Read-Eval-Print Loop."""
        self.print_help()
        while self.running:
            try:
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
                elif c == "stop":
                    self.send("STOP")
                elif c in ("brake", "brk"):
                    self.send("BRAKE")
                elif c == "enable":
                    self.send("ENABLE")
                elif c == "disable":
                    self.send("DISABLE")
                elif c == "status":
                    self.send("STATUS")
                else:
                    # Pass raw command directly
                    self.send(line)

            except (EOFError, KeyboardInterrupt):
                print("\n[INFO] Exiting Motor CLI.")
                break

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


def main():
    parser = argparse.ArgumentParser(description="TB6612 Motor Controller CLI")
    parser.add_argument("--port", "-p", type=str, default=None,
                        help="Serial port (e.g. /dev/ttyACM0 or /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--mock", "-m", action="store_true",
                        help="Run in mock mode without physical serial connection")
    parser.add_argument("command", nargs="*",
                        help="Optional command to execute once and exit (e.g. 'fwd 150', 'stop')")

    args = parser.parse_args()

    cli = MotorCLI(port=args.port, baud=args.baud, mock=args.mock)
    cli.connect()

    if args.command:
        # Run one-off command
        full_cmd = " ".join(args.command)
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
