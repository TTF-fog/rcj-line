#!/usr/bin/env python3
"""Control panel for the RCJ Line robot (Cytron Motion 2350 Pro over USB serial).

    python tools/robot_gui.py [--port /dev/ttyACM0]

Keyboard (when not typing in a field): W/A/S/D or arrows drive while held,
Q/E turn 90° left/right, Space or Esc = STOP.
"""

import argparse
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import colorchooser, messagebox

import customtkinter as ctk
import serial
import serial.tools.list_ports

# Colors. Mode colors match the robot's own status pixels.
BG = "#14171c"
CARD = "#1d2127"
CARD_INNER = "#252a31"
TEXT = "#e6e6e6"
MUTED = "#8b949e"
BLUE = "#3b82f6"
CYAN = "#06b6d4"
GREEN = "#22c55e"
AMBER = "#f59e0b"
RED = "#ef4444"
PURPLE = "#a855f7"

MODE_COLORS = {"IDLE": GREEN, "DRIVE": BLUE, "STRAIGHT": CYAN, "TURN": AMBER}
GRAB_STAGES = ["ARM_DOWN", "OPEN", "NEUTRAL", "BIAS", "ARM_UP", "CREEP"]
TOF_STATE_COLORS = {"ok": GREEN, "missing": MUTED, "lost": AMBER,
                    "init_failed": RED, "address_failed": RED}
TOF_RANGE_MM = 1200
STREAM_CHOICES = ["Off", "5", "10", "20", "50"]

SETTING_LABELS = {
    "motor.invert1": "Reverse M1", "motor.invert2": "Reverse M2",
    "motor.invert3": "Reverse M3", "motor.invert4": "Reverse M4",
    "motor.swap_sides": "Swap left / right", "motor.ramp": "Ramp, % per 10 ms",
    "heading.invert": "Invert heading",
    "straight.kp": "Kp", "straight.kd": "Kd", "straight.max": "Max correction %",
    "turn.kp": "Kp", "turn.kd": "Kd", "turn.min": "Min power %", "turn.max": "Max power %",
    "turn.tol": "Tolerance °", "turn.timeout": "Timeout ms",
    "claw.arm_up": "Arm up °", "claw.arm_down": "Arm down °",
    "claw.open_l": "Open left °", "claw.open_r": "Open right °",
    "claw.neutral_l": "Neutral left °", "claw.neutral_r": "Neutral right °",
    "claw.bias": "Bias ° (L/R grab)", "claw.arm_ms": "Arm wait ms", "claw.jaw_ms": "Jaw wait ms",
    "claw.creep": "Creep open °/s",
}
FLAG_SETTINGS = {"motor.invert1", "motor.invert2", "motor.invert3", "motor.invert4",
                 "motor.swap_sides", "heading.invert"}


def parse_fields(line):
    """'STAT mode=TURN left=45' -> {'mode': 'TURN', 'left': '45'}"""
    return dict(token.split("=", 1) for token in line.split() if "=" in token)


def to_float(text, default=None):
    try:
        value = float(text)
    except (TypeError, ValueError):
        return default
    return default if math.isnan(value) else value


def format_mm(mm):
    if mm < 0:
        return "—"
    return "far" if mm >= 8190 else f"{mm} mm"


def find_ports():
    ports = list(serial.tools.list_ports.comports())
    # The board (Raspberry Pi vendor id) first, then other USB serial ports.
    ports.sort(key=lambda p: (p.vid != 0x2E8A, "ACM" not in p.device and "USB" not in p.device))
    return [p.device for p in ports if "ttyS" not in p.device]


class RobotLink:
    """Serial connection. A background thread puts received lines on `inbox`."""

    def __init__(self):
        self.port = None
        self.inbox = queue.Queue()

    @property
    def connected(self):
        return self.port is not None

    def connect(self, device):
        self.port = serial.Serial(device, 115200, timeout=0.1, exclusive=True)
        threading.Thread(target=self._read_loop, args=(self.port,), daemon=True).start()

    def disconnect(self):
        port, self.port = self.port, None
        if port:
            port.close()

    def send(self, command):
        if not self.port:
            return False
        try:
            self.port.write((command + "\n").encode())
            return True
        except serial.SerialException as error:
            self.inbox.put(("closed", str(error)))
            return False

    def _read_loop(self, port):
        while self.port is port:
            try:
                raw = port.readline()
            except (serial.SerialException, TypeError, OSError) as error:
                if self.port is port:
                    self.inbox.put(("closed", str(error)))
                return
            if raw:
                self.inbox.put(("line", raw.decode(errors="replace").strip()))


# ----------------------------------------------------------------- widgets

def card(parent, title=None):
    frame = ctk.CTkFrame(parent, fg_color=CARD, corner_radius=10)
    if title:
        ctk.CTkLabel(frame, text=title, text_color=MUTED,
                     font=ctk.CTkFont(size=12, weight="bold")).pack(anchor="w", padx=14, pady=(10, 2))
    return frame


def button(parent, text, command, color=CARD_INNER, width=90, height=34, **kwargs):
    return ctk.CTkButton(parent, text=text, command=command, width=width, height=height,
                         fg_color=color, hover_color=_shade(color), corner_radius=8, **kwargs)


def _shade(hex_color, factor=1.25):
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (1, 3, 5))
    return "#%02x%02x%02x" % tuple(min(255, int(c * factor) + 8) for c in (r, g, b))


class LabeledSlider(ctk.CTkFrame):
    """Label, slider and live value on one row."""

    def __init__(self, parent, label, lo, hi, value, steps=None, fmt="{:.0f}", command=None):
        super().__init__(parent, fg_color="transparent")
        self.fmt = fmt
        self.on_change = command
        ctk.CTkLabel(self, text=label, width=90, anchor="w").pack(side="left")
        self.slider = ctk.CTkSlider(self, from_=lo, to=hi, number_of_steps=steps or int(hi - lo),
                                    width=260, command=self._changed)
        self.slider.pack(side="left", padx=8, fill="x", expand=True)
        self.value_label = ctk.CTkLabel(self, width=52, anchor="e", font=ctk.CTkFont(family="monospace"))
        self.value_label.pack(side="left")
        self.set(value)

    def _changed(self, value):
        self.value_label.configure(text=self.fmt.format(value))
        if self.on_change:
            self.on_change(value)

    def get(self):
        return self.slider.get()

    def set(self, value):
        self.slider.set(value)
        self.value_label.configure(text=self.fmt.format(self.slider.get()))


class Compass(tk.Canvas):
    def __init__(self, parent, size=170):
        super().__init__(parent, width=size, height=size, bg=CARD, highlightthickness=0)
        self.size = size
        self.heading = None
        self.target = None
        self.redraw()

    def show(self, heading, target):
        self.heading, self.target = heading, target
        self.redraw()

    def _point(self, degrees, radius):
        c = self.size / 2
        a = math.radians(degrees)
        return c + radius * math.sin(a), c - radius * math.cos(a)

    def redraw(self):
        self.delete("all")
        c, r = self.size / 2, self.size / 2 - 12
        self.create_oval(c - r, c - r, c + r, c + r, outline="#3a414b", width=2, fill=CARD_INNER)
        for deg in range(0, 360, 30):
            inner = r - (10 if deg % 90 == 0 else 5)
            self.create_line(*self._point(deg, inner), *self._point(deg, r), fill="#56606c", width=2)
        if self.target is not None:
            x, y = self._point(self.target, r)
            self.create_oval(x - 6, y - 6, x + 6, y + 6, fill=AMBER, outline="")
        if self.heading is not None:
            self.create_line(c, c, *self._point(self.heading, r - 14), fill=BLUE, width=5,
                             capstyle="round", arrow="last", arrowshape=(12, 14, 6))
            self.create_text(c, c + r * 0.45, text=f"{self.heading:.1f}°", fill=TEXT,
                             font=("monospace", 13, "bold"))
            self.create_oval(c - 4, c - 4, c + 4, c + 4, fill=TEXT, outline="")
        else:
            self.create_text(c, c, text="no IMU data", fill=MUTED, font=("sans", 10))


class CenterBars(tk.Canvas):
    """Horizontal bars for -100..100 values, zero in the middle."""

    def __init__(self, parent, labels, width=300):
        self.row_h = 26
        super().__init__(parent, width=width, height=self.row_h * len(labels) + 4, bg=CARD,
                         highlightthickness=0)
        self.labels = labels
        self.values = [0.0] * len(labels)
        self.bind("<Configure>", lambda e: self.redraw())

    def show(self, values):
        self.values = values
        self.redraw()

    def redraw(self):
        self.delete("all")
        w = self.winfo_width() if self.winfo_width() > 10 else int(self["width"])
        left, right = 30, w - 48
        mid = (left + right) / 2
        for i, (label, value) in enumerate(zip(self.labels, self.values)):
            y = 4 + i * self.row_h
            self.create_text(4, y + 10, text=label, anchor="w", fill=MUTED, font=("sans", 10, "bold"))
            self.create_rectangle(left, y + 3, right, y + 17, fill=CARD_INNER, outline="")
            end = mid + (right - mid) * max(-100, min(100, value)) / 100
            color = GREEN if value >= 0 else RED
            self.create_rectangle(min(mid, end), y + 3, max(mid, end), y + 17, fill=color, outline="")
            self.create_line(mid, y, mid, y + 20, fill="#56606c")
            self.create_text(w - 4, y + 10, text=f"{value:+.0f}", anchor="e", fill=TEXT,
                             font=("monospace", 10))


class TofBars(tk.Canvas):
    def __init__(self, parent, width=300):
        self.row_h = 22
        super().__init__(parent, width=width, height=self.row_h * 4 + 4, bg=CARD, highlightthickness=0)
        self.values = [-1] * 4
        self.bind("<Configure>", lambda e: self.redraw())

    def show(self, values):
        self.values = values
        self.redraw()

    def redraw(self):
        self.delete("all")
        w = self.winfo_width() if self.winfo_width() > 10 else int(self["width"])
        left, right = 30, w - 64
        for i, mm in enumerate(self.values):
            y = 4 + i * self.row_h
            self.create_text(4, y + 8, text=f"T{i + 1}", anchor="w", fill=MUTED, font=("sans", 10, "bold"))
            self.create_rectangle(left, y + 2, right, y + 14, fill=CARD_INNER, outline="")
            text, fill = format_mm(mm), MUTED
            if mm >= 8190:
                self.create_rectangle(left, y + 2, right, y + 14, fill="#2f3640", outline="")
            elif mm >= 0:
                fill = TEXT
                end = left + (right - left) * min(mm, TOF_RANGE_MM) / TOF_RANGE_MM
                self.create_rectangle(left, y + 2, end, y + 14, fill=CYAN, outline="")
            self.create_text(w - 4, y + 8, text=text, anchor="e", fill=fill, font=("monospace", 10))


# --------------------------------------------------------------------- app

class RobotGui(ctk.CTk):
    def __init__(self, port=None):
        super().__init__(fg_color=BG)
        self.title("RCJ Line robot")
        self.geometry("1320x860")
        self.minsize(1150, 760)

        self.link = RobotLink()
        self.settings = {}        # key -> value as last reported by the robot
        self.setting_rows = {}    # key -> (StringVar or BooleanVar, widget)
        self.setting_groups = {}  # "turn" -> (frame, column, rows used)
        self.synced_sliders = set()
        self.history, self.history_pos = [], 0
        self.held_keys = set()
        self.key_drive = None     # (speed, steer) currently sent from the keyboard
        self.key_release_job = None
        self.last_servo_send = 0.0
        self.quiet_replies = 0    # replies to the automatic connect-time commands stay out of the console

        self.grid_columnconfigure(0, weight=3, uniform="cols")
        self.grid_columnconfigure(1, weight=2, uniform="cols")
        self.grid_rowconfigure(1, weight=1)

        self._build_top_bar(port)
        tabs = ctk.CTkTabview(self, fg_color=CARD, corner_radius=10,
                              segmented_button_font=ctk.CTkFont(size=15, weight="bold"),
                              segmented_button_selected_color=BLUE,
                              segmented_button_selected_hover_color=_shade(BLUE))
        tabs.grid(row=1, column=0, sticky="nsew", padx=(14, 7), pady=(0, 8))
        for name, build in (("Drive", self._build_drive_tab), ("Gyro", self._build_gyro_tab),
                            ("Claw", self._build_claw_tab), ("Sensors", self._build_sensors_tab),
                            ("Settings", self._build_settings_tab)):
            build(tabs.add(name))

        right = ctk.CTkFrame(self, fg_color="transparent")
        right.grid(row=1, column=1, sticky="nsew", padx=(7, 14), pady=(0, 8))
        right.grid_columnconfigure(0, weight=1)
        right.grid_rowconfigure(1, weight=1)
        self._build_telemetry(right)
        self._build_console(right)
        self._build_status_bar()

        self.bind("<KeyPress>", self._on_key_press)
        self.bind("<KeyRelease>", self._on_key_release)
        self.bind("<Escape>", lambda e: self.stop())
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(30, self._poll_inbox)
        if port or self.port_box.get():
            self.after(200, self.toggle_connection)

    # ---------------------------------------------------------- top bar
    def _build_top_bar(self, port):
        bar = ctk.CTkFrame(self, fg_color="transparent")
        bar.grid(row=0, column=0, columnspan=2, sticky="ew", padx=14, pady=(12, 8))
        ctk.CTkLabel(bar, text="RCJ Line", font=ctk.CTkFont(size=20, weight="bold")).pack(side="left")
        ctk.CTkLabel(bar, text="Motion 2350 Pro", text_color=MUTED).pack(side="left", padx=(8, 24))

        ports = find_ports()
        self.port_box = ctk.CTkComboBox(bar, values=ports or [""], width=170)
        self.port_box.set(port or (ports[0] if ports else ""))
        self.port_box.pack(side="left")
        button(bar, "↻", self._refresh_ports, width=34).pack(side="left", padx=4)
        self.connect_button = button(bar, "Connect", self.toggle_connection, color=BLUE, width=110)
        self.connect_button.pack(side="left", padx=4)
        self.conn_dot = ctk.CTkLabel(bar, text="●", text_color=MUTED, font=ctk.CTkFont(size=18))
        self.conn_dot.pack(side="left", padx=(12, 4))
        self.conn_label = ctk.CTkLabel(bar, text="Disconnected", text_color=MUTED)
        self.conn_label.pack(side="left")

        ctk.CTkButton(bar, text="■  STOP", command=self.stop, width=150, height=42, corner_radius=10,
                      fg_color=RED, hover_color="#dc2626",
                      font=ctk.CTkFont(size=16, weight="bold")).pack(side="right")
        self.mode_label = ctk.CTkLabel(bar, text="IDLE", width=110, height=32, corner_radius=8,
                                       fg_color=CARD, text_color=GREEN,
                                       font=ctk.CTkFont(size=14, weight="bold"))
        self.mode_label.pack(side="right", padx=12)

    def _refresh_ports(self):
        ports = find_ports()
        self.port_box.configure(values=ports or [""])
        if ports and self.port_box.get() not in ports:
            self.port_box.set(ports[0])

    # ---------------------------------------------------------- drive tab
    def _build_drive_tab(self, tab):
        tab.grid_columnconfigure(0, weight=1)

        drive = card(tab, "STEERING DRIVE")
        drive.grid(row=0, column=0, sticky="ew", pady=(4, 8))
        self.drive_speed = LabeledSlider(drive, "Speed %", 0, 100, 40)
        self.drive_speed.pack(fill="x", padx=14, pady=4)
        steer_row = ctk.CTkFrame(drive, fg_color="transparent")
        steer_row.pack(fill="x", padx=14, pady=4)
        self.drive_steer = LabeledSlider(steer_row, "Steer", -1, 1, 0, steps=40, fmt="{:+.2f}")
        self.drive_steer.pack(side="left", fill="x", expand=True)
        button(steer_row, "Center", lambda: self.drive_steer.set(0), width=70).pack(side="left", padx=(8, 0))
        ms_row = ctk.CTkFrame(drive, fg_color="transparent")
        ms_row.pack(fill="x", padx=14, pady=(4, 12))
        ctk.CTkLabel(ms_row, text="Duration ms", width=90, anchor="w").pack(side="left")
        self.drive_ms = ctk.CTkEntry(ms_row, width=90, placeholder_text="until stop")
        self.drive_ms.pack(side="left", padx=8)
        button(ms_row, "Drive  ▶", lambda: self._drive(1), color=BLUE, width=110).pack(side="right")
        button(ms_row, "◀  Reverse", lambda: self._drive(-1), width=110).pack(side="right", padx=8)

        pad_card = card(tab, "QUICK MOVES  ·  uses speed and duration above")
        pad_card.grid(row=1, column=0, sticky="ew", pady=8)
        pad = ctk.CTkFrame(pad_card, fg_color="transparent")
        pad.pack(pady=(4, 12))
        size = dict(width=110, height=46)
        button(pad, "▲  Forward", lambda: self._drive(1, steer=0), **size).grid(row=0, column=1, padx=4, pady=4)
        button(pad, "⟲  Spin left", lambda: self._drive(1, steer=-1), **size).grid(row=1, column=0, padx=4, pady=4)
        button(pad, "■  STOP", self.stop, color=RED, **size).grid(row=1, column=1, padx=4, pady=4)
        button(pad, "Spin right  ⟳", lambda: self._drive(1, steer=1), **size).grid(row=1, column=2, padx=4, pady=4)
        button(pad, "▼  Back", lambda: self._drive(-1, steer=0), **size).grid(row=2, column=1, padx=4, pady=4)
        ctk.CTkLabel(pad_card, text="Keyboard: W A S D / arrows while held  ·  Q E turn 90°  ·  Space / Esc stop",
                     text_color=MUTED).pack(pady=(0, 10))

        light = card(tab, "LIGHT BAR")
        light.grid(row=2, column=0, sticky="ew", pady=8)
        row = ctk.CTkFrame(light, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=(4, 12))
        for name, rgb in (("White", (255, 255, 255)), ("Red", (255, 0, 0)), ("Green", (0, 255, 0)),
                          ("Blue", (0, 0, 255)), ("Warm", (255, 160, 60))):
            swatch = "#%02x%02x%02x" % tuple(int(c * 0.7) for c in rgb)
            button(row, name, lambda c=rgb: self.send("LIGHT %d %d %d" % c), color=swatch,
                   width=72, text_color="#111" if name in ("White", "Warm") else TEXT).pack(side="left", padx=3)
        button(row, "Pick…", self._pick_light, width=72).pack(side="left", padx=3)
        button(row, "Off", lambda: self.send("LIGHT OFF"), color="#3a3a3a", width=72).pack(side="left", padx=3)

    def _duration(self, entry):
        text = entry.get().strip()
        if not text:
            return ""
        if not text.isdigit() or int(text) <= 0:
            self._set_status(f"Duration must be a whole number of ms, got '{text}'", RED)
            return None
        return f" {text}"

    def _drive(self, direction, steer=None):
        ms = self._duration(self.drive_ms)
        if ms is None:
            return
        steer = self.drive_steer.get() if steer is None else steer
        self.send(f"DRIVE {direction * self.drive_speed.get():.0f} {steer:.2f}{ms}")

    def _pick_light(self):
        rgb, _ = colorchooser.askcolor(title="Light bar color", parent=self)
        if rgb:
            self.send("LIGHT %d %d %d" % tuple(int(c) for c in rgb))

    # ----------------------------------------------------------- gyro tab
    def _build_gyro_tab(self, tab):
        tab.grid_columnconfigure(0, weight=1)

        straight = card(tab, "GYRO STRAIGHT  ·  holds the heading it starts with")
        straight.grid(row=0, column=0, sticky="ew", pady=(4, 8))
        self.straight_speed = LabeledSlider(straight, "Speed %", -100, 100, 40, steps=40)
        self.straight_speed.pack(fill="x", padx=14, pady=4)
        row = ctk.CTkFrame(straight, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=(4, 12))
        ctk.CTkLabel(row, text="Duration ms", width=90, anchor="w").pack(side="left")
        self.straight_ms = ctk.CTkEntry(row, width=90, placeholder_text="until stop")
        self.straight_ms.pack(side="left", padx=8)
        button(row, "Go straight  ▶", self._straight, color=CYAN, width=140, text_color="#062a30").pack(side="right")

        turn = card(tab, "GYRO TURN  ·  spin in place, + = right")
        turn.grid(row=1, column=0, sticky="ew", pady=8)
        quick = ctk.CTkFrame(turn, fg_color="transparent")
        quick.pack(fill="x", padx=14, pady=4)
        for degrees in (-180, -90, -45, 45, 90, 180):
            label = f"⟲ {-degrees}°" if degrees < 0 else f"{degrees}° ⟳"
            button(quick, label, lambda d=degrees: self._turn(d), color=CARD_INNER, height=42).pack(
                side="left", padx=3, expand=True, fill="x")
        row = ctk.CTkFrame(turn, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=4)
        ctk.CTkLabel(row, text="Degrees", width=90, anchor="w").pack(side="left")
        self.turn_degrees = ctk.CTkEntry(row, width=90, placeholder_text="e.g. 135")
        self.turn_degrees.pack(side="left", padx=8)
        self.turn_degrees.bind("<Return>", lambda e: self._turn(None))
        button(row, "Turn", lambda: self._turn(None), color=AMBER, width=110, text_color="#2a1a00").pack(side="right")
        self.turn_power = LabeledSlider(turn, "Max power %", 1, 100, 60, steps=99)
        self.turn_power.pack(fill="x", padx=14, pady=(4, 12))

        result = card(tab, "LAST MOVE RESULT")
        result.grid(row=2, column=0, sticky="ew", pady=8)
        self.result_label = ctk.CTkLabel(result, text="—", text_color=MUTED,
                                         font=ctk.CTkFont(family="monospace", size=15, weight="bold"))
        self.result_label.pack(anchor="w", padx=14, pady=(2, 6))
        self.target_label = ctk.CTkLabel(result, text="target —   error —", text_color=MUTED,
                                         font=ctk.CTkFont(family="monospace"))
        self.target_label.pack(anchor="w", padx=14, pady=(0, 12))
        ctk.CTkLabel(result, text="Tune gains in Settings › turn / straight. See docs/HARDWARE_GUIDE.md.",
                     text_color=MUTED).pack(anchor="w", padx=14, pady=(0, 10))

    def _straight(self):
        ms = self._duration(self.straight_ms)
        if ms is not None:
            self.send(f"STRAIGHT {self.straight_speed.get():.0f}{ms}")

    def _turn(self, degrees):
        if degrees is None:
            degrees = to_float(self.turn_degrees.get().strip())
            if not degrees:
                self._set_status("Enter a non-zero number of degrees", RED)
                return
        self.send(f"TURN {degrees:g} {self.turn_power.get():.0f}")

    # ----------------------------------------------------------- claw tab
    def _build_claw_tab(self, tab):
        tab.grid_columnconfigure(0, weight=1)

        grab = card(tab, "PICKUP SEQUENCE")
        grab.grid(row=0, column=0, sticky="ew", pady=(4, 8))
        row = ctk.CTkFrame(grab, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=4)
        for side, label in (("L", "◀  Grab left"), ("C", "Grab center"), ("R", "Grab right  ▶")):
            button(row, label, lambda s=side: self.send(f"GRAB {s}"), color=PURPLE, height=46).pack(
                side="left", expand=True, fill="x", padx=3)
        stages = ctk.CTkFrame(grab, fg_color="transparent")
        stages.pack(fill="x", padx=14, pady=(6, 12))
        self.stage_labels = {}
        for stage in GRAB_STAGES:
            label = ctk.CTkLabel(stages, text=stage.replace("_", " ").title(), height=26, corner_radius=6,
                                 fg_color=CARD_INNER, text_color=MUTED)
            label.pack(side="left", expand=True, fill="x", padx=2)
            self.stage_labels[stage] = label

        poses = card(tab, "STORED POSES")
        poses.grid(row=1, column=0, sticky="ew", pady=8)
        row = ctk.CTkFrame(poses, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=(4, 12))
        for text, command in (("Arm up", "ARM UP"), ("Arm down", "ARM DOWN"),
                              ("Jaws open", "JAW OPEN"), ("Jaws neutral", "JAW NEUTRAL")):
            button(row, text, lambda c=command: self.send(c), height=38).pack(side="left", expand=True,
                                                                               fill="x", padx=3)

        raw = card(tab, "TUNE ANGLES  ·  sliders move the servo live, then store the pose")
        raw.grid(row=2, column=0, sticky="ew", pady=8)
        self.servo_sliders = {}
        for servo, label in ((1, "Left jaw"), (2, "Right jaw"), (3, "Arm")):
            slider = LabeledSlider(raw, label, 0, 180, 90, command=lambda v, s=servo: self._servo_moved(s))
            slider.pack(fill="x", padx=14, pady=3)
            slider.slider.bind("<ButtonRelease-1>", lambda e, s=servo: self._servo_moved(s, force=True))
            self.servo_sliders[servo] = slider
        row = ctk.CTkFrame(raw, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=(6, 12))
        jaws = lambda: (self.servo_sliders[1].get(), self.servo_sliders[2].get())
        arm = lambda: (self.servo_sliders[3].get(),)
        for text, keys, values in (("Store jaws as open", ("claw.open_l", "claw.open_r"), jaws),
                                   ("Store jaws as neutral", ("claw.neutral_l", "claw.neutral_r"), jaws),
                                   ("Store arm as up", ("claw.arm_up",), arm),
                                   ("Store arm as down", ("claw.arm_down",), arm)):
            button(row, text, lambda k=keys, v=values: self._store(k, v()), height=32).pack(
                side="left", expand=True, fill="x", padx=3)
        ctk.CTkLabel(raw, text="Stored values apply immediately. Press SAVE in Settings to keep them.",
                     text_color=MUTED).pack(anchor="w", padx=14, pady=(0, 10))

    def _servo_moved(self, servo, force=False):
        now = time.monotonic()
        if not force and now - self.last_servo_send < 0.05:
            return
        self.last_servo_send = now
        self.send(f"SERVO {servo} {self.servo_sliders[servo].get():.0f}", echo=force)

    def _store(self, keys, values):
        for key, value in zip(keys, values):
            self.send(f"SET {key} {value:.0f}")
            self.send(f"GET {key}")

    # -------------------------------------------------------- sensors tab
    def _build_sensors_tab(self, tab):
        tab.grid_columnconfigure((0, 1), weight=1)

        tof = card(tab, "DISTANCE SENSORS (VL53L0X)")
        tof.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(4, 8))
        slots = ctk.CTkFrame(tof, fg_color="transparent")
        slots.pack(fill="x", padx=10, pady=4)
        slots.grid_columnconfigure(tuple(range(4)), weight=1, uniform="slots")
        self.tof_cards = []
        for i in range(4):
            slot = ctk.CTkFrame(slots, fg_color=CARD_INNER, corner_radius=8)
            slot.grid(row=0, column=i, sticky="nsew", padx=4)
            ctk.CTkLabel(slot, text=f"Slot {i + 1}", font=ctk.CTkFont(weight="bold")).pack(pady=(8, 0))
            state = ctk.CTkLabel(slot, text="unknown", text_color=MUTED)
            state.pack()
            distance = ctk.CTkLabel(slot, text="—", font=ctk.CTkFont(family="monospace", size=18, weight="bold"))
            distance.pack()
            detail = ctk.CTkLabel(slot, text="", text_color=MUTED, font=ctk.CTkFont(size=11))
            detail.pack(pady=(0, 8))
            self.tof_cards.append((state, distance, detail))
        row = ctk.CTkFrame(tof, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=(6, 12))
        button(row, "Refresh", lambda: self.send("TOF"), width=100).pack(side="left")
        button(row, "Scan for sensors", lambda: self.send("TOF SCAN"), color=BLUE, width=150).pack(side="left", padx=8)
        ctk.CTkLabel(row, text="Scan only while stopped (blocks ~0.5 s).", text_color=MUTED).pack(side="left", padx=8)

        imu = card(tab, "IMU (BNO055)")
        imu.grid(row=1, column=0, columnspan=2, sticky="ew", pady=8)
        grid = ctk.CTkFrame(imu, fg_color="transparent")
        grid.pack(fill="x", padx=14, pady=4)
        self.imu_labels = {}
        fields = (("heading", "Heading °"), ("roll", "Roll °"), ("pitch", "Pitch °"),
                  ("gx", "Gyro X °/s"), ("gy", "Gyro Y °/s"), ("gz", "Gyro Z °/s"),
                  ("qw", "Quat W"), ("qx", "Quat X"), ("qy", "Quat Y"), ("qz", "Quat Z"))
        for n, (key, label) in enumerate(fields):
            r, c = divmod(n, 5)
            cell = ctk.CTkFrame(grid, fg_color=CARD_INNER, corner_radius=6)
            cell.grid(row=r, column=c, sticky="ew", padx=3, pady=3)
            grid.grid_columnconfigure(c, weight=1)
            ctk.CTkLabel(cell, text=label, text_color=MUTED, font=ctk.CTkFont(size=11)).pack(pady=(4, 0))
            value = ctk.CTkLabel(cell, text="—", font=ctk.CTkFont(family="monospace", size=14, weight="bold"))
            value.pack(pady=(0, 4))
            self.imu_labels[key] = value
        cal = ctk.CTkFrame(imu, fg_color="transparent")
        cal.pack(fill="x", padx=14, pady=(8, 4))
        self.cal_bars = []
        for n, name in enumerate(("System", "Gyro", "Accel", "Mag (off)")):
            ctk.CTkLabel(cal, text=name, text_color=MUTED, width=70, anchor="e").grid(row=0, column=2 * n, padx=(6, 4))
            bar = ctk.CTkProgressBar(cal, width=80, progress_color=GREEN)
            bar.set(0)
            bar.grid(row=0, column=2 * n + 1, sticky="ew")
            cal.grid_columnconfigure(2 * n + 1, weight=1)
            self.cal_bars.append(bar)
        row = ctk.CTkFrame(imu, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=(6, 12))
        button(row, "Read once", lambda: self.send("IMU"), width=100).pack(side="left")
        button(row, "Status once", lambda: self.send("STATUS"), width=100).pack(side="left", padx=8)
        ctk.CTkLabel(row, text="Heading zero is the BNO055's last power-up.", text_color=MUTED).pack(side="left", padx=8)

    # ------------------------------------------------------- settings tab
    def _build_settings_tab(self, tab):
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(2, weight=1)

        actions = ctk.CTkFrame(tab, fg_color="transparent")
        actions.grid(row=0, column=0, sticky="ew", pady=(4, 6))
        actions.grid_columnconfigure(tuple(range(5)), weight=1, uniform="actions")
        self.apply_button = button(actions, "Apply changes", self._apply_settings, color=BLUE, width=0)
        for column, widget in enumerate((
                button(actions, "Reload", lambda: self.send("GET"), width=0),
                self.apply_button,
                button(actions, "SAVE to flash", lambda: self.send("SAVE"), color=GREEN, width=0,
                       text_color="#062a12"),
                button(actions, "Defaults", self._defaults, width=0),
                button(actions, "Bootloader", self._bootload, color="#4b2a2a", width=0))):
            widget.grid(row=0, column=column, sticky="ew", padx=3)

        test = card(tab, "MOTOR DIRECTION TEST  ·  wheels off the ground")
        test.grid(row=1, column=0, sticky="ew", pady=6)
        row = ctk.CTkFrame(test, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=(4, 12))
        ctk.CTkLabel(row, text="Power %", text_color=MUTED).pack(side="left")
        self.test_power = ctk.CTkSegmentedButton(row, values=["20", "30", "50"])
        self.test_power.set("30")
        self.test_power.pack(side="left", padx=(6, 14))
        for motor, label in ((0, "M1 · left"), (1, "M2 · left"), (2, "M3 · right"), (3, "M4 · right")):
            button(row, label, lambda m=motor: self._test_motor(m), width=96).pack(side="left", padx=3)
        button(row, "Stop", self.stop, color=RED, width=70).pack(side="left", padx=(10, 0))

        self.settings_frame = ctk.CTkScrollableFrame(tab, fg_color=CARD, corner_radius=10)
        self.settings_frame.grid(row=2, column=0, sticky="nsew", pady=(6, 4))
        self.settings_frame.grid_columnconfigure((0, 1), weight=1)
        self.settings_hint = ctk.CTkLabel(self.settings_frame, text="Connect to load settings.", text_color=MUTED)
        self.settings_hint.grid(row=0, column=0, columnspan=2, pady=20)

    def _test_motor(self, motor):
        speeds = ["0"] * 4
        speeds[motor] = self.test_power.get()
        self.send("MOTORS " + " ".join(speeds))

    def _add_setting_row(self, key):
        group = key.split(".")[0]
        groups = self.setting_groups
        if group not in groups:
            # Groups flow into two columns: motor/heading/straight left, turn/claw/stream right.
            column = 0 if group in ("motor", "heading", "straight") else 1
            box = ctk.CTkFrame(self.settings_frame, fg_color=CARD_INNER, corner_radius=8)
            box.grid(row=len([g for g in groups.values() if g[1] == column]), column=column,
                     sticky="new", padx=6, pady=6)
            box.grid_columnconfigure(1, weight=1)
            ctk.CTkLabel(box, text=group.upper(), text_color=MUTED,
                         font=ctk.CTkFont(size=12, weight="bold")).grid(row=0, column=0, columnspan=2,
                                                                        sticky="w", padx=10, pady=(6, 2))
            groups[group] = (box, column, 0)
        box, column, rows = groups[group]
        r = rows + 1
        groups[group] = (box, column, r)
        ctk.CTkLabel(box, text=SETTING_LABELS.get(key, key), anchor="w").grid(row=r, column=0, sticky="w",
                                                                             padx=10, pady=3)
        if key in FLAG_SETTINGS:
            var = tk.BooleanVar()
            widget = ctk.CTkSwitch(box, text="", variable=var, progress_color=BLUE,
                                   command=lambda k=key, v=var: self.send(f"SET {k} {int(v.get())}"))
            widget.grid(row=r, column=1, sticky="e", padx=10)
        else:
            var = tk.StringVar()
            widget = ctk.CTkEntry(box, textvariable=var, width=90, justify="right")
            widget.grid(row=r, column=1, sticky="e", padx=10, pady=3)
            widget.bind("<Return>", lambda e, k=key: self._apply_settings([k]))
            var.trace_add("write", lambda *a, k=key: self._mark_setting(k))
        self.setting_rows[key] = (var, widget)

    def _mark_setting(self, key):
        var, widget = self.setting_rows[key]
        changed = to_float(var.get()) != to_float(self.settings.get(key))
        widget.configure(border_color=AMBER if changed else ctk.ThemeManager.theme["CTkEntry"]["border_color"])
        pending = sum(1 for k, (v, w) in self.setting_rows.items()
                      if k not in FLAG_SETTINGS and to_float(v.get()) != to_float(self.settings.get(k)))
        self.apply_button.configure(text=f"Apply {pending} change{'s' if pending != 1 else ''}"
                                    if pending else "Apply changes")

    def _apply_settings(self, keys=None):
        for key in keys or list(self.setting_rows):
            var, _ = self.setting_rows[key]
            if key in FLAG_SETTINGS or to_float(var.get()) == to_float(self.settings.get(key)):
                continue
            self.send(f"SET {key} {var.get().strip()}")
            self.send(f"GET {key}")

    def _defaults(self):
        if messagebox.askyesno("Restore defaults", "Reset every setting to the built-in defaults?\n"
                               "They are only kept after SAVE.", parent=self):
            self.send("DEFAULTS")
            self.send("GET")

    def _bootload(self):
        if messagebox.askyesno("Bootloader", "Reboot the board into its UF2 bootloader?\n"
                               "It disconnects until it is flashed or reset.", parent=self):
            self.send("BOOTLOAD")

    # ---------------------------------------------------------- telemetry
    def _build_telemetry(self, parent):
        tele = card(parent)
        tele.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        tele.grid_columnconfigure(1, weight=1)

        head = ctk.CTkFrame(tele, fg_color="transparent")
        head.grid(row=0, column=0, columnspan=2, sticky="ew", padx=14, pady=(10, 0))
        ctk.CTkLabel(head, text="LIVE", text_color=MUTED, font=ctk.CTkFont(size=12, weight="bold")).pack(side="left")
        self.stream_select = ctk.CTkSegmentedButton(head, values=STREAM_CHOICES,
                                                    command=self._set_stream)
        self.stream_select.set("20")
        self.stream_select.pack(side="right")
        ctk.CTkLabel(head, text="Stream Hz", text_color=MUTED).pack(side="right", padx=8)

        self.compass = Compass(tele)
        self.compass.grid(row=1, column=0, rowspan=2, padx=(14, 6), pady=10)

        info = ctk.CTkFrame(tele, fg_color="transparent")
        info.grid(row=1, column=1, sticky="new", padx=(6, 14), pady=(10, 0))
        self.tele_labels = {}
        for n, (key, label) in enumerate((("target", "Target"), ("err", "Error"),
                                          ("remain", "Time left"), ("grab", "Claw"), ("cal", "Calibration"))):
            ctk.CTkLabel(info, text=label, text_color=MUTED, anchor="w").grid(row=n, column=0, sticky="w")
            value = ctk.CTkLabel(info, text="—", anchor="e", font=ctk.CTkFont(family="monospace", weight="bold"))
            value.grid(row=n, column=1, sticky="e")
            info.grid_columnconfigure(1, weight=1)
            self.tele_labels[key] = value

        self.motor_bars = CenterBars(tele, ["L", "R"])
        self.motor_bars.grid(row=2, column=1, sticky="ew", padx=(6, 14), pady=(4, 10))
        self.tof_bars = TofBars(tele)
        self.tof_bars.grid(row=3, column=0, columnspan=2, sticky="ew", padx=14, pady=(0, 12))

    def _set_stream(self, value, echo=True):
        self.send(f"STREAM {0 if value == 'Off' else value}", echo=echo)

    # ------------------------------------------------------------ console
    def _build_console(self, parent):
        console = card(parent)
        console.grid(row=1, column=0, sticky="nsew")
        console.grid_columnconfigure(0, weight=1)
        console.grid_rowconfigure(1, weight=1)

        head = ctk.CTkFrame(console, fg_color="transparent")
        head.grid(row=0, column=0, sticky="ew", padx=14, pady=(10, 4))
        ctk.CTkLabel(head, text="CONSOLE", text_color=MUTED, font=ctk.CTkFont(size=12, weight="bold")).pack(side="left")
        button(head, "Clear", lambda: self.log.delete("1.0", "end"), width=60, height=26).pack(side="right")
        button(head, "Help", lambda: self.send("HELP"), width=60, height=26).pack(side="right", padx=6)
        self.show_stream = ctk.CTkSwitch(head, text="Show stream", progress_color=BLUE)
        self.show_stream.pack(side="right", padx=6)

        self.log = ctk.CTkTextbox(console, fg_color=BG, font=ctk.CTkFont(family="monospace", size=12),
                                  wrap="none", corner_radius=8)
        self.log.grid(row=1, column=0, sticky="nsew", padx=10)
        for tag, color in (("sent", BLUE), ("ok", "#7d8b99"), ("err", RED), ("done", GREEN),
                           ("fail", RED), ("cancel", AMBER), ("evt", PURPLE), ("stream", "#5b6672"),
                           ("data", TEXT), ("info", MUTED)):
            self.log.tag_config(tag, foreground=color)

        row = ctk.CTkFrame(console, fg_color="transparent")
        row.grid(row=2, column=0, sticky="ew", padx=10, pady=10)
        self.command_entry = ctk.CTkEntry(row, placeholder_text="Type a command, e.g. TURN 90   (↑/↓ history)")
        self.command_entry.pack(side="left", fill="x", expand=True)
        self.command_entry.bind("<Return>", self._send_typed)
        self.command_entry.bind("<Up>", lambda e: self._history_step(-1))
        self.command_entry.bind("<Down>", lambda e: self._history_step(1))
        button(row, "Send", self._send_typed, color=BLUE, width=70).pack(side="left", padx=(8, 0))

    def _send_typed(self, _event=None):
        text = self.command_entry.get().strip()
        if not text:
            return
        self.history.append(text)
        self.history_pos = len(self.history)
        self.command_entry.delete(0, "end")
        self.send(text)

    def _history_step(self, step):
        if not self.history:
            return "break"
        self.history_pos = max(0, min(len(self.history), self.history_pos + step))
        self.command_entry.delete(0, "end")
        if self.history_pos < len(self.history):
            self.command_entry.insert(0, self.history[self.history_pos])
        return "break"

    def _log(self, text, tag):
        at_bottom = self.log.yview()[1] > 0.98
        self.log.insert("end", text + "\n", tag)
        if int(self.log.index("end-1c").split(".")[0]) > 3000:
            self.log.delete("1.0", "500.0")
        if at_bottom:
            self.log.see("end")

    def _build_status_bar(self):
        self.status = ctk.CTkLabel(self, text="", text_color=MUTED, anchor="w")
        self.status.grid(row=2, column=0, columnspan=2, sticky="ew", padx=18, pady=(0, 8))

    def _set_status(self, text, color=MUTED):
        self.status.configure(text=text, text_color=color)

    # ------------------------------------------------------ connection
    def toggle_connection(self):
        if self.link.connected:
            self.link.disconnect()
            self._show_connection(False, "Disconnected")
            return
        device = self.port_box.get().strip()
        try:
            self.link.connect(device)
        except (serial.SerialException, ValueError) as error:
            self._show_connection(False, "Connect failed")
            self._set_status(str(error), RED)
            return
        self._show_connection(True, f"Connected · {device}")
        self.quiet_replies = 3
        self._set_stream(self.stream_select.get(), echo=False)
        self.send("GET", echo=False)
        self.send("TOF", echo=False)

    def _show_connection(self, connected, text):
        self.conn_dot.configure(text_color=GREEN if connected else MUTED)
        self.conn_label.configure(text=text, text_color=TEXT if connected else MUTED)
        self.connect_button.configure(text="Disconnect" if connected else "Connect",
                                      fg_color=CARD_INNER if connected else BLUE)

    def send(self, command, echo=True):
        if not self.link.send(command):
            self._set_status("Not connected", RED)
            return
        if echo:
            self._log(f"> {command}", "sent")

    def stop(self):
        self.key_drive = None
        self.send("STOP")

    def _on_close(self):
        if self.link.connected:
            self.link.send("STOP")
            self.link.disconnect()
        self.destroy()

    # ------------------------------------------------------ incoming lines
    def _poll_inbox(self):
        for _ in range(200):
            try:
                kind, payload = self.link.inbox.get_nowait()
            except queue.Empty:
                break
            if kind == "closed":
                self.link.disconnect()
                self._show_connection(False, "Connection lost")
                self._set_status(payload, RED)
            else:
                self._handle_line(payload)
        self.after(30, self._poll_inbox)

    def _handle_line(self, line):
        word = line.split(" ", 1)[0]
        if word == "IMU":
            self._on_imu(line)
        elif word == "STAT":
            self._on_stat(parse_fields(line))
        if word in ("IMU", "STAT"):
            if self.show_stream.get():
                self._log(line, "stream")
            return

        if word == "CFG":
            self._on_cfg(line)
        elif word == "TOF":
            self._on_tof(line)
        elif word in ("DONE", "FAIL", "CANCEL"):
            color = {"DONE": GREEN, "FAIL": RED, "CANCEL": AMBER}[word]
            self.result_label.configure(text=line, text_color=color)
            self._set_status(line, color)
        elif word == "ERR":
            self._set_status(line, RED)
        elif word == "READY":
            self._set_status(f"Robot booted: {line}", GREEN)
            self._set_stream(self.stream_select.get(), echo=False)

        if self.quiet_replies:
            if word in ("OK", "ERR"):
                self.quiet_replies -= 1
                if not self.quiet_replies:
                    self._log(f"Loaded {len(self.settings)} settings and ToF status", "ok")
            if word in ("OK", "CFG", "TOF"):
                return
        tag = {"OK": "ok", "ERR": "err", "DONE": "done", "FAIL": "fail", "CANCEL": "cancel",
               "EVT": "evt", "READY": "evt"}.get(word, "data")
        self._log(line, tag)

    def _on_imu(self, line):
        fields = parse_fields(line)
        if "heading" not in fields:
            self.compass.show(None, None)
            return
        for key, label in self.imu_labels.items():
            value = to_float(fields.get(key))
            label.configure(text="—" if value is None else f"{value:.{4 if key.startswith('q') else 1}f}")
        self.compass.show(to_float(fields["heading"]), self.compass.target)
        cal = fields.get("cal", "0,0,0,0").split(",")
        for bar, level in zip(self.cal_bars, cal):
            bar.set(int(level) / 3 if level.isdigit() else 0)
        self.tele_labels["cal"].configure(text=fields.get("cal", "—"))

    def _on_stat(self, fields):
        mode = fields.get("mode", "IDLE")
        grab = fields.get("grab", "IDLE")
        shown = "GRAB" if grab != "IDLE" and mode == "IDLE" else mode
        self.mode_label.configure(text=shown, text_color=PURPLE if shown == "GRAB" else MODE_COLORS.get(mode, TEXT))

        target, err = to_float(fields.get("target")), to_float(fields.get("err"))
        self.compass.show(self.compass.heading, target)
        self.tele_labels["target"].configure(text="—" if target is None else f"{target:.1f}°")
        self.tele_labels["err"].configure(text="—" if err is None else f"{err:+.1f}°")
        remain = int(to_float(fields.get("remain"), 0))
        self.tele_labels["remain"].configure(text=f"{remain / 1000:.1f} s" if remain else "—")
        self.tele_labels["grab"].configure(text=grab.replace("_", " ").lower())
        self.target_label.configure(text=f"target {'—' if target is None else f'{target:6.1f}°'}   "
                                         f"error {'—' if err is None else f'{err:+6.1f}°'}")

        self.motor_bars.show([to_float(fields.get("left"), 0), to_float(fields.get("right"), 0)])
        tof = [int(to_float(v, -1)) for v in fields.get("tof", "-1,-1,-1,-1").split(",")]
        self.tof_bars.show(tof)
        for (state, distance, _), mm in zip(self.tof_cards, tof):
            if state.cget("text") == "ok":
                distance.configure(text=format_mm(mm))

        for stage, label in self.stage_labels.items():
            active = stage == grab
            label.configure(fg_color=PURPLE if active else CARD_INNER, text_color=TEXT if active else MUTED)

    def _on_cfg(self, line):
        key, _, value = line[4:].partition("=")
        self.settings[key] = value
        if key == "stream.hz":  # shown by the Stream Hz buttons instead of a settings row
            choice = "Off" if to_float(value) == 0 else value
            if choice in STREAM_CHOICES:
                self.stream_select.set(choice)
            return
        if key not in self.setting_rows:
            self.settings_hint.grid_forget()
            self._add_setting_row(key)
        var, widget = self.setting_rows[key]
        if key in FLAG_SETTINGS:
            var.set(to_float(value, 0) != 0)
        else:
            if self.focus_get() is not widget._entry:  # don't overwrite what the user is typing
                var.set(value)
            self._mark_setting(key)
        self._sync_from_settings(key, value)

    def _sync_from_settings(self, key, value):
        """Start sliders at the robot's values: turn power at turn.max, servos at their rest pose."""
        number = to_float(value)
        if number is None:
            return
        targets = {"turn.max": self.turn_power, "claw.neutral_l": self.servo_sliders[1],
                   "claw.neutral_r": self.servo_sliders[2], "claw.arm_up": self.servo_sliders[3]}
        if key in targets and key not in self.synced_sliders:
            targets[key].set(number)
            self.synced_sliders.add(key)

    def _on_tof(self, line):
        parts = line.split()
        if len(parts) < 2 or not parts[1].isdigit():
            return
        index = int(parts[1]) - 1
        if not 0 <= index < 4:
            return
        fields = parse_fields(line)
        state_name = fields.get("state", "?")
        state, distance, detail = self.tof_cards[index]
        state.configure(text=state_name, text_color=TOF_STATE_COLORS.get(state_name, TEXT))
        mm = int(to_float(fields.get("mm"), -1))
        distance.configure(text=format_mm(mm))
        detail.configure(text=f"{fields.get('bus', '')} · {fields.get('addr', '')} · {fields.get('xshut', '')}")

    # ------------------------------------------------------ keyboard driving
    DRIVE_KEYS = {"w": "fwd", "Up": "fwd", "s": "back", "Down": "back",
                  "a": "left", "Left": "left", "d": "right", "Right": "right"}

    def _typing(self, event):
        return event.widget.winfo_class() in ("Entry", "Text")

    def _on_key_press(self, event):
        if self._typing(event):
            return
        key = event.keysym if event.keysym in self.DRIVE_KEYS else event.keysym.lower()
        if key == "space":
            self.stop()
        elif key in ("q", "e"):
            self.send(f"TURN {-90 if key == 'q' else 90} {self.turn_power.get():.0f}")
        elif key in self.DRIVE_KEYS:
            if self.key_release_job:  # X11 auto-repeat sends release+press pairs
                self.after_cancel(self.key_release_job)
                self.key_release_job = None
            self.held_keys.add(self.DRIVE_KEYS[key])
            self._drive_from_keys()

    def _on_key_release(self, event):
        key = event.keysym if event.keysym in self.DRIVE_KEYS else event.keysym.lower()
        if key in self.DRIVE_KEYS and not self._typing(event):
            action = self.DRIVE_KEYS[key]
            self.key_release_job = self.after(60, lambda: self._release_key(action))

    def _release_key(self, action):
        self.key_release_job = None
        self.held_keys.discard(action)
        if self.held_keys:
            self._drive_from_keys()
        elif self.key_drive is not None:
            self.stop()

    def _drive_from_keys(self):
        keys = self.held_keys
        direction = (1 if "fwd" in keys else 0) - (1 if "back" in keys else 0)
        side = (1 if "right" in keys else 0) - (1 if "left" in keys else 0)
        speed = self.drive_speed.get()
        if direction == 0:
            drive = (speed, float(side)) if side else None   # spin in place
        else:
            drive = (direction * speed, 0.5 * side)          # arc
        if drive is None:
            if self.key_drive is not None:
                self.stop()
        elif drive != self.key_drive:
            self.key_drive = drive
            self.send(f"DRIVE {drive[0]:.0f} {drive[1]:.2f}", echo=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="serial port, default: first robot-looking port")
    args = parser.parse_args()
    ctk.set_appearance_mode("dark")
    RobotGui(args.port).mainloop()


if __name__ == "__main__":
    main()
