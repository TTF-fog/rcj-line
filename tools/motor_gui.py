#!/usr/bin/env python3
"""
CustomTkinter Motor Control Panel for the Robocup Line robot.

Drive controls (FWD/BACK/LEFT/RIGHT/STOP/BRAKE/STEER) plus the gyro
straight-hold P controller (GFWD/GBACK/GSTOP/GKP/GMAX/GSTATUS).

Usage:
    /home/ttf/.venv/bin/python tools/motor_gui.py [--port /dev/ttyACM0] [--baud 115200]

Requires a display (Tecnomatix / local X session). Uses the MotorCLI
serial layer from tools/motor_cli.py.
"""

import argparse
import threading
import tkinter as tk
from tkinter import messagebox

import customtkinter as ctk

from motor_cli import MotorCLI

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")


class MotorGUI(ctk.CTk):
    def __init__(self, port=None, baud=115200):
        super().__init__()
        self.title("Robocup Motor Panel")
        self.geometry("720x640")
        self.resizable(False, False)

        self.cli = MotorCLI(port=port, baud=baud, mock=False)
        self.connected = False

        self._build_connection_frame()
        self._build_drive_frame()
        self._build_gyro_frame()
        self._build_log_frame()

        self._bind_keys()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ layout
    def _build_connection_frame(self):
        frm = ctk.CTkFrame(self)
        frm.pack(fill="x", padx=10, pady=(10, 5))

        ctk.CTkLabel(frm, text="Port:").pack(side="left", padx=(10, 2))
        detected = self.cli.find_default_port() if not self.cli.port_name else self.cli.port_name
        self.port_var = tk.StringVar(value=self.cli.port_name or detected or "")
        ctk.CTkEntry(frm, textvariable=self.port_var, width=160).pack(side="left", padx=2)

        ctk.CTkLabel(frm, text="Baud:").pack(side="left", padx=(10, 2))
        self.baud_var = tk.StringVar(value=str(self.cli.baud))
        ctk.CTkEntry(frm, textvariable=self.baud_var, width=80).pack(side="left", padx=2)

        self.mock_var = tk.BooleanVar(value=False)
        ctk.CTkCheckBox(frm, text="Mock", variable=self.mock_var, width=70).pack(side="left", padx=10)

        self.conn_btn = ctk.CTkButton(frm, text="Connect", width=100, command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=5)
        self.conn_label = ctk.CTkLabel(frm, text="Disconnected", text_color="gray")
        self.conn_label.pack(side="left", padx=10)

    def _build_drive_frame(self):
        frm = ctk.CTkFrame(self)
        frm.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(frm, text="Drive", font=ctk.CTkFont(weight="bold")).pack(anchor="w", padx=10, pady=(5, 0))

        row = ctk.CTkFrame(frm, fg_color="transparent")
        row.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(row, text="Speed:").pack(side="left")
        self.speed_slider = ctk.CTkSlider(row, from_=0, to=255, number_of_steps=255,
                                          width=300, command=self._on_speed_slider)
        self.speed_slider.set(150)
        self.speed_slider.pack(side="left", padx=10)
        self.speed_label = ctk.CTkLabel(row, text="150", width=40)
        self.speed_label.pack(side="left")

        btns = ctk.CTkFrame(frm, fg_color="transparent")
        btns.pack(padx=10, pady=5)
        ctk.CTkButton(btns, text="▲ FWD", width=100, command=lambda: self._send("FWD", True)).grid(row=0, column=1, padx=4, pady=4)
        ctk.CTkButton(btns, text="◀ LEFT", width=100, command=lambda: self._send("LEFT", True)).grid(row=1, column=0, padx=4, pady=4)
        ctk.CTkButton(btns, text="STOP", width=100, fg_color="#a33", hover_color="#c44",
                      command=lambda: self._send("STOP", False)).grid(row=1, column=1, padx=4, pady=4)
        ctk.CTkButton(btns, text="RIGHT ▶", width=100, command=lambda: self._send("RIGHT", True)).grid(row=1, column=2, padx=4, pady=4)
        ctk.CTkButton(btns, text="▼ BACK", width=100, command=lambda: self._send("BACK", True)).grid(row=2, column=1, padx=4, pady=4)
        ctk.CTkButton(btns, text="BRAKE", width=100, fg_color="#a33", hover_color="#c44",
                      command=lambda: self._send("BRAKE", False)).grid(row=2, column=2, padx=4, pady=4)

        steer = ctk.CTkFrame(frm, fg_color="transparent")
        steer.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(steer, text="Steer ratio:").pack(side="left")
        self.ratio_slider = ctk.CTkSlider(steer, from_=-1.0, to=1.0, number_of_steps=40, width=200)
        self.ratio_slider.set(0.0)
        self.ratio_slider.pack(side="left", padx=10)
        self.ratio_label = ctk.CTkLabel(steer, text="0.00", width=45)
        self.ratio_label.pack(side="left")
        self.ratio_slider.configure(command=lambda v: self.ratio_label.configure(text=f"{float(v):.2f}"))
        ctk.CTkButton(steer, text="STEER", width=90, command=self._send_steer).pack(side="left", padx=10)

    def _build_gyro_frame(self):
        frm = ctk.CTkFrame(self)
        frm.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(frm, text="Gyro straight-hold (P on heading error)", font=ctk.CTkFont(weight="bold")).pack(anchor="w", padx=10, pady=(5, 0))

        row1 = ctk.CTkFrame(frm, fg_color="transparent")
        row1.pack(fill="x", padx=10, pady=5)
        ctk.CTkButton(row1, text="GFWD (straight)", width=140, fg_color="#2a7",
                      hover_color="#3b8", command=lambda: self._send("GFWD", True)).pack(side="left", padx=4)
        ctk.CTkButton(row1, text="GBACK (straight)", width=140,
                      command=lambda: self._send("GBACK", True)).pack(side="left", padx=4)
        ctk.CTkButton(row1, text="GSTOP", width=100, fg_color="#a33", hover_color="#c44",
                      command=lambda: self._send("GSTOP", False)).pack(side="left", padx=4)
        ctk.CTkButton(row1, text="GSTATUS", width=100, command=self._send_gstatus).pack(side="left", padx=4)

        row2 = ctk.CTkFrame(frm, fg_color="transparent")
        row2.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(row2, text="Kp:").pack(side="left")
        self.kp_slider = ctk.CTkSlider(row2, from_=0.0, to=10.0, number_of_steps=100, width=170,
                                       command=lambda v: self.kp_label.configure(text=f"{float(v):.1f}"))
        self.kp_slider.set(2.0)
        self.kp_slider.pack(side="left", padx=10)
        self.kp_label = ctk.CTkLabel(row2, text="2.0", width=35)
        self.kp_label.pack(side="left")
        ctk.CTkButton(row2, text="Set Kp", width=70, command=self._send_kp).pack(side="left", padx=5)

        ctk.CTkLabel(row2, text="GMAX:").pack(side="left", padx=(15, 2))
        self.gmax_var = tk.StringVar(value="80")
        ctk.CTkEntry(row2, textvariable=self.gmax_var, width=60).pack(side="left", padx=2)
        ctk.CTkButton(row2, text="Set", width=60, command=self._send_gmax).pack(side="left", padx=5)

        self.gyro_label = ctk.CTkLabel(frm, text="Gyro: idle", text_color="gray", anchor="w")
        self.gyro_label.pack(fill="x", padx=10, pady=(0, 5))

    def _build_log_frame(self):
        frm = ctk.CTkFrame(self)
        frm.pack(fill="both", expand=True, padx=10, pady=5)
        top = ctk.CTkFrame(frm, fg_color="transparent")
        top.pack(fill="x", padx=10, pady=(5, 0))
        ctk.CTkLabel(top, text="Log", font=ctk.CTkFont(weight="bold")).pack(side="left")
        ctk.CTkButton(top, text="Clear", width=70, command=self._clear_log).pack(side="right")
        self.log_box = ctk.CTkTextbox(frm, height=130, state="disabled")
        self.log_box.pack(fill="both", expand=True, padx=10, pady=5)

    def _bind_keys(self):
        self.bind("<Up>", lambda _e: self._send("FWD", True))
        self.bind("<Down>", lambda _e: self._send("BACK", True))
        self.bind("<Left>", lambda _e: self._send("LEFT", True))
        self.bind("<Right>", lambda _e: self._send("RIGHT", True))
        self.bind("<space>", lambda _e: self._send("STOP", False))

    # ------------------------------------------------------------------ actions
    def _toggle_connect(self):
        if self.connected:
            self.cli.close()
            self.connected = False
            self.conn_btn.configure(text="Connect")
            self.conn_label.configure(text="Disconnected", text_color="gray")
            self._log("SYS", "Disconnected.")
            return
        self.cli.port_name = self.port_var.get().strip() or None
        try:
            self.cli.baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Baud", "Baud rate must be an integer.")
            return
        self.cli.mock = self.mock_var.get()
        self.cli.connect()
        self.connected = True
        mode = "MOCK" if self.cli.mock else (self.cli.port_name or "auto")
        self.conn_btn.configure(text="Disconnect")
        self.conn_label.configure(text=f"Connected ({mode})", text_color="green")
        self._log("SYS", f"Connected ({mode}).")

    def _require_connection(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the robot first (or tick Mock).")
            return False
        return True

    def _send(self, base_cmd, with_speed):
        if not self._require_connection():
            return
        cmd = f"{base_cmd} {self._speed_int()}" if with_speed else base_cmd
        self._send_async(cmd)

    def _send_steer(self):
        if not self._require_connection():
            return
        self._send_async(f"STEER {self._speed_int()} {float(self.ratio_slider.get()):.2f}")

    def _send_kp(self):
        if not self._require_connection():
            return
        self._send_async(f"GKP {float(self.kp_slider.get()):.1f}")

    def _send_gmax(self):
        if not self._require_connection():
            return
        self._send_async(f"GMAX {self.gmax_var.get().strip()}")

    def _send_gstatus(self):
        if not self._require_connection():
            return
        threading.Thread(target=self._gstatus_worker, daemon=True).start()

    def _gstatus_worker(self):
        resp = self.cli.send("GSTATUS")
        self.after(0, self._on_gstatus, resp or "(no response)")

    def _on_gstatus(self, resp):
        self.gyro_label.configure(text=f"Gyro: {resp}", text_color="white")

    def _send_async(self, cmd):
        self._log("TX", cmd)
        threading.Thread(target=self._send_worker, args=(cmd,), daemon=True).start()

    def _send_worker(self, cmd):
        resp = self.cli.send(cmd)
        if resp:
            self.after(0, self._log, "RX", resp)

    def _speed_int(self):
        return int(round(float(self.speed_slider.get())))

    def _on_speed_slider(self, value):
        self.speed_label.configure(text=str(int(round(float(value)))))

    # ------------------------------------------------------------------ log
    def _log(self, tag, msg):
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{tag}] {msg}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def _clear_log(self):
        self.log_box.configure(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.configure(state="disabled")

    def _on_close(self):
        try:
            self.cli.close()
        finally:
            self.destroy()


def main():
    parser = argparse.ArgumentParser(description="Robocup Motor Control Panel (CustomTkinter)")
    parser.add_argument("--port", "-p", type=str, default=None,
                        help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", "-b", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    args = parser.parse_args()

    app = MotorGUI(port=args.port, baud=args.baud)
    app.mainloop()


if __name__ == "__main__":
    main()
