#!/usr/bin/env python3
"""
Rerun Visualizer for Robocup Line Robot
- 3D View: Live 3D orientation vector computed from BNO055 IMU quaternions (Q,w,x,y,z)
- TimeSeries Graph: Left and Right motor speeds over time
- Dataframe View & Live Table: History and live logging of motor commands and acknowledgments
"""

import sys
import time
import argparse
import threading
import math
from datetime import datetime
from typing import Optional, List, Dict, Any

try:
    import serial
except ImportError:
    serial = None

try:
    import rerun as rr
    import rerun.blueprint as rrb
    import pandas as pd
    import numpy as np
except ImportError as e:
    print(f"Error importing required libraries: {e}")
    print("Please install requirements or run with the project Python environment:")
    print("  /home/ttf/.venv/bin/python tools/visualizer.py")
    sys.exit(1)


def quat_to_vectors(w: float, x: float, y: float, z: float):
    """
    Compute forward (heading), lateral, and up 3D vectors from a normalized quaternion.
    Forward = (1, 0, 0) rotated by q
    Lateral = (0, 1, 0) rotated by q
    Up      = (0, 0, 1) rotated by q
    """
    # Normalize quaternion just in case
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm > 0.0:
        w, x, y, z = w / norm, x / norm, y / norm, z / norm

    # Forward vector (X-axis rotated)
    vx = 1.0 - 2.0 * (y * y + z * z)
    vy = 2.0 * (x * y + w * z)
    vz = 2.0 * (x * z - w * y)

    # Lateral vector (Y-axis rotated)
    sx = 2.0 * (x * y - w * z)
    sy = 1.0 - 2.0 * (x * x + z * z)
    sz = 2.0 * (y * z + w * x)

    # Up vector (Z-axis rotated)
    ux = 2.0 * (x * z + w * y)
    uy = 2.0 * (y * z - w * x)
    uz = 1.0 - 2.0 * (x * x + y * y)

    return [vx, vy, vz], [sx, sy, sz], [ux, uy, uz]


class RobocupVisualizer:
    def __init__(self, imu_port: Optional[str] = None, motor_port: Optional[str] = None,
                 baud: int = 115200, demo: bool = False):
        self.imu_port_name = imu_port
        self.motor_port_name = motor_port
        self.baud = baud
        self.demo = demo
        self.running = True

        self.ser_imu: Optional[serial.Serial] = None
        self.ser_motor: Optional[serial.Serial] = None

        self.left_speed: int = 0
        self.right_speed: int = 0
        self.command_history: List[Dict[str, Any]] = []
        self.history_lock = threading.Lock()

    def setup_rerun(self):
        """Initialize Rerun stream and configure Blueprint layout."""
        rr.init("Robocup_Robot_Monitor", spawn=True)

        # Build custom blueprint layout with origin at world root "/"
        # 1. 3D View of Orientation Vector
        # 2. Time-series graph for Motor Speeds
        # 3. Tabbed Dataframe and Table View for Motor Commands
        blueprint = rrb.Blueprint(
            rrb.Grid(
                rrb.Spatial3DView(
                    name="3D IMU Orientation Vector",
                    origin="/"
                ),
                rrb.TimeSeriesView(
                    name="Motor Speeds (Left & Right)",
                    origin="/motor/speed"
                ),
                rrb.Tabs(
                    rrb.DataframeView(
                        name="Live Commands Dataframe",
                        origin="/motor",
                        query=rrb.archetypes.DataframeQuery(auto_scroll=True)
                    ),
                    rrb.TextDocumentView(
                        name="Commands Table",
                        origin="/motor/table_md"
                    )
                ),
            )
        )
        rr.send_blueprint(blueprint)

        # Log static reference origin coordinate axes in the world frame
        rr.log(
            "world/reference_frame",
            rr.Arrows3D(
                origins=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                vectors=[[0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.3]],
                colors=[[150, 150, 150], [150, 150, 150], [150, 150, 150]],
                radii=[0.005],
                labels=["Ref X", "Ref Y", "Ref Z"]
            ),
            static=True
        )

        # Populate initial state so dataframe and table are immediately visible
        self.record_command("SYSTEM_START", 0, 0, "READY")
        self.record_command("STANDBY", 0, 0, "AWAITING_COMMANDS")

    def open_serial_ports(self):
        """Open serial connections for IMU and Motor UARTs."""
        if self.demo:
            print("[INFO] Running in DEMO mode with simulated IMU and motor data.")
            return

        if serial is None:
            raise RuntimeError("pyserial is not installed. Run with Python environment containing pyserial.")

        if not self.imu_port_name:
            print("[WARN] No serial port specified. Falling back to DEMO mode.")
            self.demo = True
            return

        try:
            print(f"[INFO] Opening IMU serial port: {self.imu_port_name} at {self.baud} baud...")
            self.ser_imu = serial.Serial(self.imu_port_name, self.baud, timeout=0.1)
        except Exception as e:
            print(f"[ERROR] Failed to open IMU port {self.imu_port_name}: {e}")
            print("[INFO] Switching to DEMO mode.")
            self.demo = True
            return

        # If motor port is separate
        if self.motor_port_name and self.motor_port_name != self.imu_port_name:
            try:
                print(f"[INFO] Opening Motor serial port: {self.motor_port_name} at {self.baud} baud...")
                self.ser_motor = serial.Serial(self.motor_port_name, self.baud, timeout=0.1)
            except Exception as e:
                print(f"[WARN] Failed to open separate motor port {self.motor_port_name}: {e}")
                self.ser_motor = None
        else:
            # Shared port
            self.ser_motor = self.ser_imu

    def record_command(self, cmd_text: str, left: int, right: int, status: str = "OK"):
        """Record command in dataframe, table, and time-series plots."""
        self.left_speed = left
        self.right_speed = right

        timestamp_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        entry = {
            "Timestamp": timestamp_str,
            "Command": cmd_text,
            "Left Target": left,
            "Right Target": right,
            "Status": status
        }

        with self.history_lock:
            self.command_history.append(entry)
            if len(self.command_history) > 60:
                self.command_history.pop(0)

            df = pd.DataFrame(self.command_history)

            # Build markdown table representation for live viewing
            md_table = "| Timestamp | Command | Left Speed | Right Speed | Status |\n"
            md_table += "| :--- | :--- | :---: | :---: | :--- |\n"
            for item in reversed(self.command_history[-20:]):
                md_table += f"| {item['Timestamp']} | **{item['Command']}** | {item['Left Target']} | {item['Right Target']} | `{item['Status']}` |\n"

        # 1. Log components for Rerun's DataframeView (queried under /motor)
        rr.log("motor/command", rr.TextLog(f"{cmd_text} | L:{left} R:{right} | {status}", level="INFO"))
        rr.log("motor/command_name", rr.TextLog(cmd_text))
        rr.log("motor/status", rr.TextLog(status))

        # 2. Log motor speeds in TimeSeries
        rr.log("motor/speed/left", rr.Scalars(float(left)))
        rr.log("motor/speed/right", rr.Scalars(float(right)))

        # 3. Log live formatted markdown table in TextDocumentView
        rr.log("motor/table_md", rr.TextDocument(md_table, media_type="text/markdown"))

        # 4. Also send dataframe table directly
        rr.send_dataframe(df, index=None, entity_path="motor/table")

    def parse_imu_line(self, line: str):
        """Parse 'Q,w,x,y,z,CAL,sys,g,a,m' CSV line."""
        line = line.strip()
        if not line:
            return

        if line.startswith("Q,"):
            parts = line.split(",")
            if len(parts) >= 11:
                try:
                    w = float(parts[1])
                    x = float(parts[2])
                    y = float(parts[3])
                    z = float(parts[4])
                    sys_cal = int(parts[6])
                    g_cal = int(parts[7])
                    a_cal = int(parts[8])
                    m_cal = int(parts[9])

                    # 1. Compute rotated 3D direction vectors from quaternion
                    f_vec, s_vec, u_vec = quat_to_vectors(w, x, y, z)

                    # 2. Log the 3D Heading Vector directly in the 3D view
                    rr.log(
                        "imu/heading_vector",
                        rr.Arrows3D(
                            origins=[[0.0, 0.0, 0.0]],
                            vectors=[f_vec],
                            colors=[[0, 220, 255]],  # Bright cyan heading vector
                            radii=[0.025],
                            labels=[f"Heading: ({f_vec[0]:.2f}, {f_vec[1]:.2f}, {f_vec[2]:.2f})"]
                        )
                    )

                    # 3. Log full 3D orientation tri-vector (Forward=Red, Lateral=Green, Up=Blue)
                    rr.log(
                        "imu/orientation_axes",
                        rr.Arrows3D(
                            origins=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                            vectors=[f_vec, s_vec, u_vec],
                            colors=[[255, 40, 40], [40, 220, 40], [50, 120, 255]],
                            radii=[0.018, 0.014, 0.014],
                            labels=["Forward (X)", "Side (Y)", "Up (Z)"]
                        )
                    )

                    # 4. Also keep Transform3D on robot entity with a child heading arrow
                    rr.log(
                        "robot",
                        rr.Transform3D(
                            quaternion=rr.Quaternion(xyzw=[x, y, z, w])
                        )
                    )
                    rr.log(
                        "robot/heading",
                        rr.Arrows3D(
                            origins=[[0.0, 0.0, 0.0]],
                            vectors=[[1.0, 0.0, 0.0]],
                            colors=[[255, 215, 0]],
                            radii=[0.02]
                        )
                    )

                    # 5. Log calibration indicators
                    rr.log("imu/calibration/sys", rr.Scalars(sys_cal))
                    rr.log("imu/calibration/gyro", rr.Scalars(g_cal))
                    rr.log("imu/calibration/accel", rr.Scalars(a_cal))
                    rr.log("imu/calibration/mag", rr.Scalars(m_cal))

                except (ValueError, IndexError):
                    pass
        elif line.startswith("OK:"):
            # Motor feedback line (e.g. "OK: SPEED 100 -100" or "OK: FWD 150")
            self.parse_motor_feedback(line)

    def parse_motor_feedback(self, line: str):
        """Parse motor feedback lines like 'OK: SPEED 100 200' or 'OK: FWD 150'."""
        parts = line.replace("OK:", "").strip().split()
        if not parts:
            return

        cmd = parts[0].upper()
        if cmd == "SPEED" and len(parts) >= 3:
            try:
                l, r = int(parts[1]), int(parts[2])
                self.record_command(f"SPEED {l} {r}", l, r, "ACK")
            except ValueError:
                pass
        elif cmd == "FWD" and len(parts) >= 2:
            try:
                spd = int(parts[1])
                self.record_command(f"FWD {spd}", spd, spd, "ACK")
            except ValueError:
                pass
        elif cmd == "BACK" and len(parts) >= 2:
            try:
                spd = int(parts[1])
                self.record_command(f"BACK {spd}", -spd, -spd, "ACK")
            except ValueError:
                pass
        elif cmd == "STOP":
            self.record_command("STOP", 0, 0, "ACK")
        elif cmd == "BRAKE":
            self.record_command("BRAKE", 0, 0, "ACK")
        elif cmd == "STEER" and len(parts) >= 3:
            try:
                spd = int(parts[1])
                ratio = float(parts[2])
                self.record_command(f"STEER {spd} {ratio}", self.left_speed, self.right_speed, "ACK")
            except ValueError:
                pass
        elif cmd == "MOTOR" and len(parts) >= 3:
            try:
                idx, spd = int(parts[1]), int(parts[2])
                self.record_command(f"MOTOR {idx} {spd}", self.left_speed, self.right_speed, "ACK")
            except ValueError:
                pass
        elif cmd == "MOTORS" and len(parts) >= 5:
            try:
                m1, m2, m3, m4 = int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4])
                l = (m1 + m2) // 2
                r = (m3 + m4) // 2
                self.record_command(f"MOTORS {m1} {m2} {m3} {m4}", l, r, "ACK")
            except ValueError:
                pass

    def send_motor_command(self, cmd_str: str):
        """Send command string over motor UART bus."""
        cmd_clean = cmd_str.strip()
        if not cmd_clean:
            return

        target_ser = self.ser_motor or self.ser_imu
        if target_ser and target_ser.is_open:
            try:
                target_ser.write((cmd_clean + "\n").encode("utf-8"))
                target_ser.flush()
            except Exception as e:
                print(f"[ERROR] Failed to send serial command: {e}")

        # Update local speed representation based on sent command
        parts = cmd_clean.split()
        cmd = parts[0].upper()
        l_spd, r_spd = self.left_speed, self.right_speed

        try:
            if cmd in ("SPEED", "M") and len(parts) >= 3:
                l_spd, r_spd = int(parts[1]), int(parts[2])
            elif cmd in ("FWD", "FORWARD") and len(parts) >= 2:
                s = int(parts[1])
                l_spd, r_spd = s, s
            elif cmd in ("BACK", "REV", "BACKWARD") and len(parts) >= 2:
                s = int(parts[1])
                l_spd, r_spd = -s, -s
            elif cmd == "STEER" and len(parts) >= 3:
                s = int(parts[1])
                ratio = float(parts[2])
                if ratio > 0:
                    l_spd = s
                    r_spd = int(round(s * (1.0 - 2.0 * ratio)))
                else:
                    r_spd = s
                    l_spd = int(round(s * (1.0 + 2.0 * ratio)))
            elif cmd == "LEFT" and len(parts) >= 2:
                s = int(parts[1])
                l_spd, r_spd = -s, s
            elif cmd == "RIGHT" and len(parts) >= 2:
                s = int(parts[1])
                l_spd, r_spd = s, -s
            elif cmd in ("STOP", "BRAKE"):
                l_spd, r_spd = 0, 0
        except ValueError:
            pass

        self.record_command(cmd_clean, l_spd, r_spd, "SENT")

    def run_demo_simulation(self):
        """Simulate robot yaw/pitch motion and motor cycles for demo."""
        t = 0.0
        step = 0
        while self.running:
            # Simulate smooth roll/pitch/yaw quaternion
            roll = 0.1 * math.sin(t * 1.5)
            pitch = 0.05 * math.cos(t * 1.2)
            yaw = t * 0.5  # continuous rotation

            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)

            w = cr * cp * cy + sr * sp * sy
            x = sr * cp * cy - cr * sp * sy
            y = cr * sp * cy + sr * cp * sy
            z = cr * cp * sy - sr * sp * cy

            # Compute and log orientation vectors
            f_vec, s_vec, u_vec = quat_to_vectors(w, x, y, z)
            rr.log(
                "imu/heading_vector",
                rr.Arrows3D(
                    origins=[[0.0, 0.0, 0.0]],
                    vectors=[f_vec],
                    colors=[[0, 220, 255]],
                    radii=[0.025],
                    labels=[f"Heading: ({f_vec[0]:.2f}, {f_vec[1]:.2f}, {f_vec[2]:.2f})"]
                )
            )
            rr.log(
                "imu/orientation_axes",
                rr.Arrows3D(
                    origins=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                    vectors=[f_vec, s_vec, u_vec],
                    colors=[[255, 40, 40], [40, 220, 40], [50, 120, 255]],
                    radii=[0.018, 0.014, 0.014],
                    labels=["Forward (X)", "Side (Y)", "Up (Z)"]
                )
            )
            rr.log("robot", rr.Transform3D(quaternion=rr.Quaternion(xyzw=[x, y, z, w])))

            # Periodic simulated motor commands
            step += 1
            if step % 75 == 0:  # ~every 1.5s
                phase = (step // 75) % 5
                if phase == 0:
                    self.send_motor_command("FWD 180")
                elif phase == 1:
                    self.send_motor_command("STEER 150 0.4")
                elif phase == 2:
                    self.send_motor_command("RIGHT 120")
                elif phase == 3:
                    self.send_motor_command("BACK 100")
                elif phase == 4:
                    self.send_motor_command("STOP")

            t += 0.02
            time.sleep(0.02)

    def run_serial_listener(self):
        """Read incoming serial lines and update visualization."""
        while self.running:
            line_read = False
            # Read IMU stream
            if self.ser_imu and self.ser_imu.is_open:
                try:
                    if self.ser_imu.in_waiting > 0:
                        raw = self.ser_imu.readline().decode("utf-8", errors="ignore")
                        self.parse_imu_line(raw)
                        line_read = True
                except Exception as e:
                    print(f"[WARN] Serial read error on IMU port: {e}")

            # Read separate motor port if active
            if self.ser_motor and self.ser_motor != self.ser_imu and self.ser_motor.is_open:
                try:
                    if self.ser_motor.in_waiting > 0:
                        raw = self.ser_motor.readline().decode("utf-8", errors="ignore")
                        self.parse_motor_feedback(raw)
                        line_read = True
                except Exception as e:
                    print(f"[WARN] Serial read error on Motor port: {e}")

            if not line_read:
                time.sleep(0.005)

    def interactive_cli_thread(self):
        """Allows typing motor commands directly in the visualizer terminal."""
        time.sleep(0.5)
        print("\n" + "=" * 60)
        print("  Robocup Robot Visualizer Running!")
        print("  Type motor commands here while viewing Rerun:")
        print("    fwd <speed>          |  back <speed>      |  steer <spd> <ratio>")
        print("    speed <left> <right> |  stop | brake     |  quit")
        print("=" * 60 + "\n")

        while self.running:
            try:
                user_input = input("robot> ").strip()
                if not user_input:
                    continue
                if user_input.lower() in ("exit", "quit", "q"):
                    print("[INFO] Exiting...")
                    self.running = False
                    break
                self.send_motor_command(user_input)
            except (EOFError, KeyboardInterrupt):
                self.running = False
                break

    def run(self):
        self.setup_rerun()
        self.open_serial_ports()

        # Start data acquisition thread
        if self.demo:
            data_thread = threading.Thread(target=self.run_demo_simulation, daemon=True)
        else:
            data_thread = threading.Thread(target=self.run_serial_listener, daemon=True)
        data_thread.start()

        # Start interactive CLI in current thread
        try:
            self.interactive_cli_thread()
        except KeyboardInterrupt:
            self.running = False


def main():
    parser = argparse.ArgumentParser(description="Robocup Line Robot Rerun Visualizer")
    parser.add_argument("--port", "-p", type=str, default=None,
                        help="Shared serial port for both IMU and Motor UART (e.g. /dev/ttyACM0)")
    parser.add_argument("--imu-port", type=str, default=None,
                        help="Serial port dedicated to IMU stream (e.g. /dev/ttyACM0)")
    parser.add_argument("--motor-port", type=str, default=None,
                        help="Serial port dedicated to TB6612 Motor commands (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="Serial baud rate (default: 115200)")
    parser.add_argument("--demo", "-d", action="store_true",
                        help="Run in demo simulation mode without hardware")

    args = parser.parse_args()

    imu_port = args.imu_port or args.port
    motor_port = args.motor_port or args.port

    # If no port provided and not demo, check if /dev/ttyACM0 is available
    if not args.demo and not imu_port:
        import os
        if os.path.exists("/dev/ttyACM0"):
            print("[INFO] Found /dev/ttyACM0, connecting as default port.")
            imu_port = "/dev/ttyACM0"
            motor_port = "/dev/ttyACM0"

    visualizer = RobocupVisualizer(
        imu_port=imu_port,
        motor_port=motor_port,
        baud=args.baud,
        demo=args.demo
    )
    visualizer.run()


if __name__ == "__main__":
    main()
