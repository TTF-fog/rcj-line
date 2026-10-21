"""Desktop dashboard for the robot, in CustomTkinter.

    ~/rcj-line/.venv/bin/python robot/ctk_dash.py [--host 192.168.1.4:8080]

Standalone (needs only customtkinter and Pillow) and talks to the robot's --view server, so the file can be
copied to a laptop. Space = Start / Lack of Progress.
"""
import argparse
import io
import json
import threading
import time
import urllib.request

import customtkinter as ctk
from PIL import Image

STEPS = ["enter", "collect", "green", "red", "exit"]
RUN_S = 480
BG, CARD, LINE, TEXT, DIM = "#0d1014", "#161b22", "#262d36", "#e6edf3", "#8b949e"
GREEN, AMBER, RED, BLUE, VIOLET = "#3fb950", "#d29922", "#f85149", "#58a6ff", "#bc8cff"
PHASE = {"ready": BLUE, "line": GREEN, "zone": VIOLET, "stuck": RED, "finished": AMBER}
LOG_TAGS = [("lop", "lack of progress #", RED), ("warn", "WARNING", AMBER), ("warn", "ERR", AMBER),
            ("warn", "Traceback", AMBER), ("warn", "stuck", AMBER), ("warn", "lost", AMBER),
            ("warn", "timed out", AMBER), ("warn", "not found", AMBER), ("goal", "goal", GREEN),
            ("zone", "[zone", VIOLET), ("line", "[line]", DIM)]


class Link(threading.Thread):
    """Polls the robot in the background; the UI only reads state, frame and online."""

    def __init__(self, host):
        super().__init__(daemon=True)
        self.host = host
        self.state = None
        self.frame = None
        self.online = False

    def get(self, path):
        return urllib.request.urlopen(f"http://{self.host}{path}", timeout=1).read()

    def run(self):
        last_state = 0.0
        while True:
            try:
                frame = self.get("/frame.jpg")
                if frame:
                    self.frame = frame
                if time.monotonic() - last_state > 0.25:
                    self.state = json.loads(self.get("/state"))
                    last_state = time.monotonic()
                self.online = True
                time.sleep(0.07)
            except (OSError, ValueError):
                self.online = False
                time.sleep(0.5)

    def post(self, path):
        request = urllib.request.Request(f"http://{self.host}{path}", method="POST")
        threading.Thread(target=lambda: urllib.request.urlopen(request, timeout=1), daemon=True).start()


def card(parent, title):
    frame = ctk.CTkFrame(parent, fg_color=CARD, border_color=LINE, border_width=1, corner_radius=10)
    ctk.CTkLabel(frame, text=title.upper(), text_color=DIM, font=ctk.CTkFont(size=11, weight="bold"),
                 anchor="w").pack(fill="x", padx=14, pady=(10, 0))
    return frame


class App(ctk.CTk):
    def __init__(self, link):
        super().__init__(fg_color=BG)
        self.link = link
        self.title("Robot dashboard")
        self.geometry("1360x920")
        self.frame_shown = None
        self.log_shown = None
        big = ctk.CTkFont(size=40, weight="bold")

        top = ctk.CTkFrame(self, fg_color=BG)
        top.pack(fill="x", padx=16, pady=(12, 4))
        ctk.CTkLabel(top, text="RCJ Rescue Line", font=ctk.CTkFont(size=16, weight="bold")).pack(side="left")
        self.conn = ctk.CTkLabel(top, text="● offline", text_color=RED)
        self.conn.pack(side="right", padx=(10, 0))
        ctk.CTkButton(top, text="Connect", width=80, command=self.connect).pack(side="right", padx=(6, 0))
        self.host = ctk.CTkEntry(top, width=200)
        self.host.insert(0, link.host)
        self.host.pack(side="right")
        self.sysinfo = ctk.CTkLabel(top, text="", text_color=DIM)
        self.sysinfo.pack(side="right", padx=16)

        log = card(self, "Log")
        log.pack(side="bottom", fill="x", padx=16, pady=(6, 14))
        self.log = ctk.CTkTextbox(log, height=120, fg_color=CARD, font=ctk.CTkFont(family="monospace", size=12),
                                  wrap="word")
        self.log.pack(fill="x", padx=10, pady=(4, 10))
        for tag, _, colour in LOG_TAGS:
            self.log.tag_config(tag, foreground=colour)

        body = ctk.CTkFrame(self, fg_color=BG)
        body.pack(fill="both", expand=True, padx=16, pady=6)
        body.grid_columnconfigure(0, weight=3)
        body.grid_columnconfigure(1, weight=2, minsize=380)
        body.grid_rowconfigure(0, weight=1)

        self.cam_card = card(body, "Camera")
        self.cam_card.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        self.cam = ctk.CTkLabel(self.cam_card, text="waiting for frames", text_color=DIM, fg_color="#000",
                                corner_radius=8)
        self.cam.pack(fill="both", expand=True, padx=14, pady=(8, 4))
        self.cmd = ctk.CTkLabel(self.cam_card, text="", text_color=DIM, font=ctk.CTkFont(family="monospace", size=12),
                                anchor="w")
        self.cmd.pack(fill="x", padx=14, pady=(0, 10))

        side = ctk.CTkFrame(body, fg_color=BG)
        side.grid(row=0, column=1, sticky="nsew")

        state = card(side, "State")
        state.pack(fill="x", pady=(0, 10))
        self.phase = ctk.CTkLabel(state, text="-", font=ctk.CTkFont(size=28, weight="bold"), anchor="w")
        self.phase.pack(fill="x", padx=14)
        steps = ctk.CTkFrame(state, fg_color=CARD)
        steps.pack(fill="x", padx=10, pady=(4, 12))
        self.steps = []
        for i, name in enumerate(STEPS):
            steps.grid_columnconfigure(i, weight=1)
            label = ctk.CTkLabel(steps, text=f"●\n{name}", text_color=LINE, font=ctk.CTkFont(size=11))
            label.grid(row=0, column=i)
            self.steps.append(label)

        run = card(side, "Run")
        run.pack(fill="x", pady=(0, 10))
        row = ctk.CTkFrame(run, fg_color=CARD)
        row.pack(fill="x", padx=14)
        self.clock = ctk.CTkLabel(row, text="0:00", font=big)
        self.clock.pack(side="left")
        ctk.CTkLabel(row, text=" of 8:00", text_color=DIM).pack(side="left", anchor="s", pady=8)
        self.lops = ctk.CTkLabel(row, text="0 LoP", font=ctk.CTkFont(size=22, weight="bold"))
        self.lops.pack(side="right")
        self.bar = ctk.CTkProgressBar(run, progress_color=GREEN, fg_color=LINE, height=8)
        self.bar.set(0)
        self.bar.pack(fill="x", padx=14, pady=(4, 14))

        victims = card(side, "Victims")
        victims.pack(fill="x", pady=(0, 10))
        self.victims = ctk.CTkCanvas(victims, height=88, bg=CARD, highlightthickness=0)
        self.victims.pack(fill="x", padx=14, pady=(4, 10))

        control = card(side, "Control")
        control.pack(fill="x", pady=(0, 10))
        buttons = ctk.CTkFrame(control, fg_color=CARD)
        buttons.pack(fill="x", padx=14, pady=(6, 4))
        self.press = ctk.CTkButton(buttons, text="Start", height=44, font=ctk.CTkFont(size=15, weight="bold"),
                                   command=lambda: link.post("/press"))
        self.press.pack(side="left", fill="x", expand=True)
        self.new = ctk.CTkButton(buttons, text="New run", height=44, width=110, fg_color="#21262d",
                                 hover_color=LINE, command=lambda: link.post("/hold"))
        self.new.pack(side="left", padx=(10, 0))
        ctk.CTkLabel(control, text="Bench use. During an official run only the robot's own buttons may be used.",
                     text_color=DIM, font=ctk.CTkFont(size=11), wraplength=340, justify="left",
                     anchor="w").pack(fill="x", padx=14, pady=(0, 10))

        sensors = card(side, "TOF (mm)")
        sensors.pack(fill="x")
        grid = ctk.CTkFrame(sensors, fg_color=CARD)
        grid.pack(pady=(4, 12))
        mono = ctk.CTkFont(family="monospace", size=14)
        self.tof = {}
        for side_name, r, c in (("front", 0, 1), ("left", 1, 0), ("right", 1, 2), ("back", 2, 1)):
            self.tof[side_name] = ctk.CTkLabel(grid, text="-", font=mono, width=70)
            self.tof[side_name].grid(row=r, column=c)
        ctk.CTkLabel(grid, text="robot", width=70, height=30, fg_color=LINE, corner_radius=8,
                     text_color=DIM).grid(row=1, column=1, padx=6, pady=2)

        self.bind("<space>", lambda e: link.post("/press") if self.focus_get() is not self.host._entry else None)
        self.after(100, self.tick)

    def connect(self):
        self.link.host = self.host.get().strip()
        self.link.state = self.link.frame = None

    def tick(self):
        self.conn.configure(text="● live" if self.link.online else "● offline",
                            text_color=GREEN if self.link.online else RED)
        frame = self.link.frame
        if frame is not None and frame is not self.frame_shown:
            self.show_frame(frame)
        if self.link.state:
            self.render(self.link.state)
        self.after(50, self.tick)

    def show_frame(self, jpeg):
        self.frame_shown = jpeg
        image = Image.open(io.BytesIO(jpeg))
        # Sized from the card, not the label, so the image can't grow the label that sizes it.
        w = max(160, self.cam_card.winfo_width() - 40)
        h = max(120, self.cam_card.winfo_height() - 90)
        scale = min(w / image.width, h / image.height)
        size = (int(image.width * scale), int(image.height * scale))
        self.cam.configure(image=ctk.CTkImage(dark_image=image, light_image=image, size=size), text="")

    def render(self, s):
        running = s["running"]
        self.phase.configure(text=s["phase"].upper(), text_color=PHASE.get(s["phase"], TEXT))
        now = STEPS.index(s["zone_step"]) if s["phase"] == "zone" and s["zone_step"] in STEPS else -1
        for i, label in enumerate(self.steps):
            label.configure(text_color=TEXT if i == now else VIOLET if i < now else LINE)
        clock = int(s["clock"])
        self.clock.configure(text=f"{clock // 60}:{clock % 60:02d}")
        frac = min(1.0, s["clock"] / RUN_S)
        self.bar.set(frac)
        self.bar.configure(progress_color=RED if frac > 0.9 else AMBER if frac > 0.75 else GREEN)
        self.lops.configure(text=f"{s['lops']} LoP", text_color=RED if s["lops"] else TEXT)
        self.cmd.configure(text=s["line"] if s["phase"] == "line" else f"zone {s['zone_step']}" if s["phase"] == "zone" else "")
        self.press.configure(text="Lack of Progress" if running else "Start",
                             fg_color=RED if running else GREEN, hover_color="#da3633" if running else "#2ea043")
        self.new.configure(state="disabled" if running else "normal")
        for side_name, label in self.tof.items():
            label.configure(text=str(s["tof"][side_name]) if s["tof"] else "-")
        temp = s["temp"]
        self.sysinfo.configure(text=f"board {s['board']}   down cam {s['cameras']['down']}   cpu {temp:.0f} °C",
                               text_color=RED if temp > 80 else AMBER if temp > 70 else DIM)
        self.draw_victims(s)
        if s["log"] != self.log_shown:
            self.draw_log(s["log"])

    def draw_victims(self, s):
        c = self.victims
        c.delete("all")
        slots = []
        for colour in ("silver", "black"):
            done, held = s["delivered"].get(colour, 0), s["held"].get(colour, 0)
            for i in range(s["want"][colour]):
                slots.append((colour, "rescued" if i < done else "held" if i < done + held else "missing"))
        width = max(c.winfo_width(), 300)
        for i, (colour, status) in enumerate(slots):
            x = width * (i + 0.5) / len(slots)
            fill = {"silver": "#c9d1d9", "black": "#30363d"}[colour] if status != "missing" else ""
            outline = {"rescued": GREEN, "held": BLUE, "missing": LINE}[status]
            c.create_oval(x - 20, 4, x + 20, 44, fill=fill, outline=outline, width=3,
                          dash=(4, 3) if status == "missing" else None)
            c.create_text(x, 58, text=colour, fill=DIM, font=("sans", 9))
            c.create_text(x, 74, text=status, fill=outline if status != "missing" else DIM, font=("sans", 9, "bold"))

    def draw_log(self, lines):
        self.log_shown = lines
        at_bottom = self.log.yview()[1] > 0.98
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        for line in lines:
            tag = next((t for t, needle, _ in LOG_TAGS if needle in line), None)
            self.log.insert("end", line + "\n", tag)
        self.log.configure(state="disabled")
        if at_bottom:
            self.log.see("end")


def main():
    ap = argparse.ArgumentParser(description="Desktop dashboard for the robot")
    ap.add_argument("--host", default="localhost:8080", help="robot address:port of its --view server")
    args = ap.parse_args()
    ctk.set_appearance_mode("dark")
    link = Link(args.host)
    link.start()
    App(link).mainloop()


if __name__ == "__main__":
    main()
