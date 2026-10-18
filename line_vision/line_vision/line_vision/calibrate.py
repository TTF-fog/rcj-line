"""Guided calibration: python3 -m line_vision.calibrate"""
import argparse
import math
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

from . import overlay
from .config import DEFAULT_PATH, Config
from .vision import LineVision, _runs, background_kernel, floor_ratio

MARKER_MM = 25.0
STRIP_MM = 25.0


class CalibrationError(Exception):
    pass


def _roi(frame, v):
    proc = cv2.resize(frame, (v.proc_width, v.proc_height), interpolation=cv2.INTER_AREA)
    x0, y0, x1, y1 = LineVision(v).roi_rect()
    return proc[y0:y1, x0:x1]


def _gray(roi):
    return cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (5, 5), 0)


def dark_mask(roi):
    """Largest dark region by Otsu threshold, or None if the frame has no clear dark/bright split."""
    gray = _gray(roi)
    t, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    dark, bright = gray[mask > 0], gray[mask == 0]
    if dark.size < 0.01 * gray.size or bright.size < 0.05 * gray.size or bright.mean() - dark.mean() < 40:
        return None
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if n < 2:
        return None
    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return (labels == best).astype(np.uint8) * 255


def sharpness(roi):
    return float(cv2.Laplacian(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())


def needs_hflip(frames, v):
    """Frames taken with the line under the robot's left side."""
    xs = []
    for f in frames:
        m = dark_mask(_roi(f, v))
        if m is not None:
            mom = cv2.moments(m, binaryImage=True)
            xs.append(mom["m10"] / mom["m00"] / m.shape[1])
    if len(xs) < len(frames) // 2:
        raise CalibrationError("no line found; put the black line in view")
    x = float(np.median(xs))
    if abs(x - 0.5) < 0.1:
        raise CalibrationError(f"line is near the centre ({x:.0%} across); move it further to the robot's left")
    return x > 0.5, [f"line seen at {x:.0%} across the image -> hflip={x > 0.5}"]


def needs_vflip(frames, v):
    """Frames of a line running left-right across the view while the robot is pushed forward."""
    ys, vertical = [], 0
    for f in frames:
        m = dark_mask(_roi(f, v))
        if m is None:
            continue
        mom = cv2.moments(m, binaryImage=True)
        vertical += mom["mu02"] > mom["mu20"]
        ys.append(mom["m01"] / mom["m00"] / m.shape[0])
    if len(ys) < len(frames) // 3:
        raise CalibrationError("no line found; put the line across the view")
    if vertical > len(ys) / 2:
        raise CalibrationError("the line looks vertical in the image. Either the robot was not turned 90 degrees, or the "
                               "camera is mounted rotated 90 degrees (flips cannot fix that; remount it)")
    slope = np.polyfit(np.arange(len(ys)), ys, 1)[0] * len(ys)
    if abs(slope) < 0.08:
        raise CalibrationError(f"line barely moved ({slope:+.0%} of the image height); push the robot further forward")
    return slope < 0, [f"line moved {slope:+.0%} of the image height (down = positive) -> vflip={slope < 0}"]


def measure_geometry(frames, tape_mm, v):
    """Line width in processed pixels near (bottom) and far (top) of the ROI, from a straight line."""
    near, far = [], []
    for f in frames:
        m = dark_mask(_roi(f, v))
        if m is None:
            continue
        h, w = m.shape
        pts = []
        for i in range(8):
            y = int(h - (i + 0.5) * h / 8)
            runs = [r for r in _runs(m[y] > 0) if r[0] > 0 and r[1] < w]
            if len(runs) == 1:
                pts.append(((runs[0][0] + runs[0][1]) / 2, y, runs[0][1] - runs[0][0]))
        if len(pts) < 3:
            continue
        slope = np.polyfit([p[1] for p in pts], [p[0] for p in pts], 1)[0]
        widths = [p[2] * math.cos(math.atan(slope)) for p in pts]
        near.append(np.median(widths[:3]))
        far.append(np.median(widths[-3:]))
    if len(near) < max(3, len(frames) // 3):
        raise CalibrationError("could not see a single straight line fully inside the view "
                               "(both edges visible, not touching the image sides)")
    h, w = _roi(frames[0], v).shape[:2]
    ppm_near, ppm_far = float(np.median(near)) / tape_mm, float(np.median(far)) / tape_mm
    return {
        "tape_mm": tape_mm, "roi_w": w, "roi_h": h,
        "ppm_near": ppm_near, "ppm_far": ppm_far,
        "fov_w_near_mm": w / ppm_near, "fov_w_far_mm": w / ppm_far,
        "fov_h_mm": h / ((ppm_near + ppm_far) / 2),
    }


def geometry_settings(g):
    w, h = g["roi_w"], g["roi_h"]
    line_min = g["tape_mm"] * min(g["ppm_near"], g["ppm_far"])
    line_max = g["tape_mm"] * max(g["ppm_near"], g["ppm_far"])
    marker_far = MARKER_MM * g["ppm_far"]
    marker_near = MARKER_MM * g["ppm_near"]
    return {
        "background_kernel": min(int(2.5 * line_max) | 1, 2 * max(w, h) + 1),
        "min_line_area_frac": round(float(np.clip(0.5 * line_min ** 2 / (w * h), 0.001, 0.05)), 4),
        "min_exit_px": max(3, int(0.5 * line_min)),
        "marker_min_area_frac": round(min(0.5, 0.35 * marker_far ** 2 / (w * h)), 4),
        "marker_max_area_frac": round(min(1.0, 2.0 * marker_near ** 2 / (w * h)), 4),
        "strip_min_height_px": max(3, int(0.6 * STRIP_MM * min(g["ppm_near"], g["ppm_far"]))),
    }


def geometry_report(g):
    lines = [
        f"tape {g['tape_mm']:.0f} mm = {g['tape_mm'] * g['ppm_near']:.0f} px near, {g['tape_mm'] * g['ppm_far']:.0f} px far",
        f"field of view: {g['fov_w_near_mm']:.0f} mm wide (near), {g['fov_w_far_mm']:.0f} mm wide (far), "
        f"{g['fov_h_mm']:.0f} mm deep",
    ]
    need_w = 2 * MARKER_MM + g["tape_mm"] + 10
    need_h = 2.5 * MARKER_MM + g["tape_mm"]
    if g["fov_w_near_mm"] < need_w:
        lines.append(f"WARNING: the view must be at least {need_w:.0f} mm wide to see a green marker beside the line. "
                     "Marker and intersection decisions will not work. Tilt the camera forward or fit a wider lens.")
    if g["fov_h_mm"] < need_h:
        lines.append(f"WARNING: the view must be at least {need_h:.0f} mm deep to see a marker and the crossing line "
                     "beyond it in one frame. Marker decisions will be unreliable.")
    if g["tape_mm"] * g["ppm_near"] > 0.3 * g["roi_w"]:
        lines.append("NOTE: the line fills a large part of the view; sharp curves leave the view quickly, so drive slowly.")
    return lines


def measure_black(frames, v):
    kernel = background_kernel(v.background_kernel)
    lg, fg, lr, fr = [], [], [], []
    for f in frames:
        roi = _roi(f, v)
        m = dark_mask(roi)
        if m is None:
            continue
        gray = _gray(roi)
        ratio = floor_ratio(gray, kernel)
        core = cv2.erode(m, np.ones((5, 5), np.uint8)) > 0
        floor = cv2.dilate(m, np.ones((9, 9), np.uint8)) == 0
        lg.append(gray[core][::4]); fg.append(gray[floor][::4])
        lr.append(ratio[core][::4]); fr.append(ratio[floor][::4])
    if len(lg) < len(frames) // 2:
        raise CalibrationError("no clear black line on white floor in view")
    lg, fg, lr, fr = map(np.concatenate, (lg, fg, lr, fr))
    lr95, fr1, fr5 = np.percentile(lr, 95), np.percentile(fr, 1), np.percentile(fr, 5)
    settings = {
        "black_rel": round(float((lr95 + fr5) / 2), 2),
        "black_rel_max_gray": int((np.percentile(lg, 99) + np.median(fg)) / 2),
        "black_abs": int(min(np.percentile(lg, 75), np.percentile(fg, 1) * 0.6)),
    }
    lines = [f"line gray median {np.median(lg):.0f}, floor gray median {np.median(fg):.0f}",
             f"line/floor brightness ratio: line <= {lr95:.2f}, floor >= {fr5:.2f}"]
    if lr95 >= fr5:
        lines.append("WARNING: line and floor brightness overlap; improve or even out the lighting under the robot.")
    return settings, lines, float(fr1)


def measure_colour(frames, v, kind):
    hs, ss, vs = [], [], []
    for f in frames:
        hsv = cv2.cvtColor(cv2.GaussianBlur(_roi(f, v), (5, 5), 0), cv2.COLOR_BGR2HSV)
        mask = ((hsv[:, :, 1] >= 70) & (hsv[:, :, 2] >= 40)).astype(np.uint8)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        if n < 2:
            continue
        best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        if stats[best, cv2.CC_STAT_AREA] < 0.005 * mask.size:
            continue
        px = hsv[labels == best]
        hs.append(px[:, 0]); ss.append(px[:, 1]); vs.append(px[:, 2])
    if len(hs) < len(frames) // 2:
        raise CalibrationError(f"no clearly coloured patch in view; put the {kind} item fully in view")
    h, s, val = (np.concatenate(a).astype(np.int32) for a in (hs, ss, vs))
    s_lo = max(40, int(np.percentile(s, 5) * 0.6))
    v_lo = max(25, int(np.percentile(val, 5) * 0.5))
    if kind == "green":
        lo, hi = np.percentile(h, 2) - 8, np.percentile(h, 98) + 8
        if not 30 <= np.median(h) <= 100:
            raise CalibrationError(f"most coloured thing in view has hue {np.median(h):.0f}, not green (40-90)")
        settings = {"green_lo": (int(max(0, lo)), s_lo, v_lo), "green_hi": (int(min(180, hi)), 255, 255)}
    else:
        signed = np.where(h > 90, h - 180, h)
        if abs(np.median(signed)) > 15:
            raise CalibrationError(f"most coloured thing in view has hue {np.median(h):.0f}, not red (near 0/180)")
        lo, hi = int(np.percentile(signed, 2)) - 6, int(np.percentile(signed, 98)) + 6
        settings = {"red_lo1": (max(0, lo), s_lo, v_lo), "red_hi1": (max(0, hi), 255, 255),
                    "red_lo2": (180 + min(0, lo), s_lo, v_lo), "red_hi2": (180, 255, 255)}
    return settings, [f"{kind}: hue {np.percentile(h, 2):.0f}-{np.percentile(h, 98):.0f}, "
                      f"saturation >= {s_lo}, brightness >= {v_lo}"]


def measure_silver(frames, v, floor_ratio_p1):
    kernel = background_kernel(v.background_kernel)
    bright, dim = [], []
    for f in frames:
        roi = _roi(f, v)
        gray = _gray(roi)
        sat = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)[:, :, 1]
        ratio = floor_ratio(gray, kernel)
        candidate = (sat < v.silver_max_sat) & (ratio > v.black_rel)
        bright.append(float((candidate & (gray >= 245)).mean()))
        dim.append(ratio[candidate & (ratio < floor_ratio_p1)][::4])
    if np.median(bright) >= 0.08:
        return ({"silver_specular": 245, "silver_rel_band": None},
                [f"silver shows as bright highlights ({np.median(bright):.0%} of view); using highlight detection"])
    dim = np.concatenate(dim)
    if dim.size < 0.08 * len(frames) * v.proc_width * v.proc_height / 4:
        raise CalibrationError("silver tape is not distinguishable from the floor under this lighting")
    band = (round(float(np.percentile(dim, 5)) - 0.02, 2), round(float(np.percentile(dim, 95)) + 0.02, 2))
    return ({"silver_specular": 256, "silver_rel_band": band},
            [f"silver shows as gray at {band[0]:.2f}-{band[1]:.2f} of floor brightness",
             "WARNING: shadows of the same darkness will look like silver; the false-detection check below matters."])


def false_detection_check(frames, cfg):
    vision = LineVision(cfg.vision)
    counts = {"line lost": 0, "junction": 0, "valid marker": 0, "any marker": 0, "red strip": 0, "silver strip": 0}
    for i, f in enumerate(frames):
        obs = vision.process(f, i / 30)
        counts["line lost"] += not obs.line_found
        counts["junction"] += obs.is_junction
        counts["valid marker"] += bool(obs.valid_marker_sides())
        counts["any marker"] += bool(obs.markers)
        counts["red strip"] += obs.red_strip is not None
        counts["silver strip"] += obs.silver_strip is not None
    hints = {
        "line lost": "black_rel too low or black_rel_max_gray too low",
        "junction": "noise touching the border; raise min_exit_px",
        "any marker": "green_lo saturation/brightness too low",
        "red strip": "red_lo1/red_lo2 saturation too low",
        "silver strip": "silver settings too loose",
    }
    lines = []
    for name, n in counts.items():
        line = f"{name:13} {n:4d}/{len(frames)} frames"
        if n > 0.05 * len(frames) and name in hints:
            line += f"   <- too many; {hints[name]}"
        lines.append(line)
    return lines


class Pump(threading.Thread):
    """Feeds the live view continuously and hands frames to whichever step is capturing."""

    def __init__(self, camera, viewer, cfg):
        super().__init__(daemon=True)
        self.camera, self.viewer, self.cfg = camera, viewer, cfg
        self.vision = LineVision(cfg.vision)
        self.title = ""
        self.sharpness = 0.0
        self._lock = threading.Lock()
        self._buffer = None
        self._running = True
        self._i = 0

    def run(self):
        while self._running:
            item = self.camera.read(0.5)
            if item is None:
                continue
            frame, ts = item
            with self._lock:
                if self._buffer is not None:
                    self._buffer.append(frame)
            self._i += 1
            if self._i % 2:
                continue
            obs = self.vision.process(frame, ts)
            x0, y0, x1, y1 = self.vision.roi_rect()
            self.sharpness = sharpness(obs.debug["proc"][y0:y1, x0:x1])
            self.viewer.publish(overlay.draw(obs, self.vision.roi_rect(),
                                             [self.title, f"sharpness {self.sharpness:.0f}"]))

    def reload(self, cfg):
        self.cfg = cfg
        self.vision = LineVision(cfg.vision)

    def collect(self, n, timeout=None):
        with self._lock:
            self._buffer = []
        deadline = time.monotonic() + (timeout or n / 10 + 5)
        while time.monotonic() < deadline:
            with self._lock:
                if len(self._buffer) >= n:
                    break
            time.sleep(0.02)
        with self._lock:
            frames, self._buffer = self._buffer, None
        if len(frames) < n:
            raise CalibrationError(f"camera delivered only {len(frames)}/{n} frames")
        return frames

    def stop(self):
        self._running = False
        self.join(timeout=2)


class Wizard:
    def __init__(self, path, port):
        from .viewer import Viewer

        self.path = path
        self.cfg = Config.load(path) if path.exists() else Config()
        self.cfg.camera.fixed_controls = None
        self.floor_ratio_p1 = 0.7
        self.viewer = Viewer(port)
        self.camera = self.pump = None

    def open_camera(self):
        from .camera import Camera

        if self.pump:
            self.pump.stop()
            self.camera.stop()
        self.camera = Camera(self.cfg.camera)
        self.camera.start()
        self.pump = Pump(self.camera, self.viewer, self.cfg)
        self.pump.start()

    def update_vision(self, settings):
        self.cfg.vision = replace(self.cfg.vision, **settings)
        self.pump.reload(self.cfg)

    def step(self, title, instructions, action):
        """action() -> (report lines, apply callable). Returns True if accepted."""
        print(f"\n=== {title} ===")
        for line in instructions:
            print(f"  {line}")
        self.pump.title = title
        while True:
            if input("Press Enter to capture (s = skip): ").strip().lower() == "s":
                return False
            try:
                lines, apply = action()
            except CalibrationError as e:
                print(f"  ! {e}")
                continue
            for line in lines:
                print(f"  {line}")
            if input("Enter = accept, r = redo: ").strip().lower() == "r":
                continue
            apply()
            return True

    def run(self):
        self.cfg.camera = replace(self.cfg.camera, hflip=False, vflip=False)
        self.open_camera()
        print(f"Live view: open {self.viewer.url()} in a browser on the same network.")
        print("Stop with Ctrl+C at any time; nothing is saved until the end.")

        print("\n=== 1. Focus ===")
        print("  Put the robot on the field with the line in view. Turn the lens until the 'sharpness'")
        print("  number in the live view is as high as it gets (or skip if the lens is fixed).")
        self.pump.title = "1. Focus"
        input("Press Enter when done: ")
        print(f"  sharpness {self.pump.sharpness:.0f}")

        v = lambda: self.cfg.vision
        flips = {"hflip": False, "vflip": False}

        def left():
            flip, lines = needs_hflip(self.pump.collect(20), v())
            return lines, lambda: flips.update(hflip=flip)

        self.step("2. Left/right", [
            "Place the robot along a straight line, then shift it sideways to its RIGHT",
            "so the line is under the LEFT half of the camera view (robot's left, seen from behind).",
        ], left)

        def forward():
            print("  Push the robot slowly straight forward about 2 cm now...")
            flip, lines = needs_vflip(self.pump.collect(75), v())
            return lines, lambda: flips.update(vflip=flip)

        self.step("3. Forward/back", [
            "Turn the robot 90 degrees so the line runs left-right across the camera view,",
            "slightly ahead of the view's centre. After pressing Enter, push the robot forward ~2 cm",
            "over about 2 seconds.",
        ], forward)
        if flips["hflip"] or flips["vflip"]:
            print(f"  reopening camera with {flips}")
            self.cfg.camera = replace(self.cfg.camera, **flips)
            self.open_camera()

        def exposure():
            self.camera.set_auto()
            controls = self.camera.settle_and_lock()
            frames = self.pump.collect(10)
            floor = []
            for f in frames:
                roi = _roi(f, v())
                m = dark_mask(roi)
                floor.append(np.median(_gray(roi) if m is None else _gray(roi)[m == 0]))
            lines = [f"exposure {controls['ExposureTime']} us, gain {controls['AnalogueGain']:.1f}, "
                     f"floor brightness {np.median(floor):.0f}/255"]
            if controls["AnalogueGain"] >= self.cfg.camera.max_analogue_gain - 0.01:
                lines.append("WARNING: gain is at its limit, so the image is under-exposed. Add light under the robot.")
            if np.median(floor) < 90:
                lines.append("WARNING: the floor is dark; black/white separation will be weak. Add light under the robot.")
            if np.median(floor) > 240:
                lines.append("WARNING: the floor is saturated; silver and white cannot be told apart.")
            fixed = {"ExposureTime": controls["ExposureTime"], "AnalogueGain": controls["AnalogueGain"],
                     "ColourGains": list(controls["ColourGains"])}
            return lines, lambda: setattr(self.cfg.camera, "fixed_controls", fixed)

        self.step("4. Exposure", [
            "Use the venue's lighting with the robot's own lights on. Put the camera over white floor",
            "with the line in view. Exposure and white balance are measured and then fixed.",
        ], exposure)

        def geometry():
            tape = input("  Measured width of your black tape in mm [15]: ").strip()
            tape_mm = float(tape) if tape else 15.0
            print("  Slowly slide the robot along the line now...")
            frames = self.pump.collect(60)
            g = measure_geometry(frames, tape_mm, v())
            size = geometry_settings(g)
            black, black_lines, fr1 = measure_black(frames, replace(v(), **size))
            lines = geometry_report(g) + black_lines

            def apply():
                self.floor_ratio_p1 = fr1
                self.update_vision({**size, **black})
            return lines, apply

        self.step("5. Line size and black level", [
            "Place the robot on a STRAIGHT line, pointing along it, with the whole line width",
            "inside the view. You will be asked for the tape width, then slide the robot along the line.",
        ], geometry)

        def colour(kind):
            def action():
                settings, lines = measure_colour(self.pump.collect(20), v(), kind)
                return lines, lambda: self.update_vision(settings)
            return action

        self.step("6. Green marker", ["Put a green marker fully in view (off the line is fine)."], colour("green"))
        self.step("7. Red goal strip", ["Put the red goal strip in view."], colour("red"))

        def silver():
            settings, lines = measure_silver(self.pump.collect(20), v(), self.floor_ratio_p1)
            return lines, lambda: self.update_vision(settings)

        self.step("8. Silver entrance strip", ["Put the silver evacuation-zone entrance tape across the view."], silver)

        def check():
            print("  Keep moving the robot along the line for 5 seconds...")
            return false_detection_check(self.pump.collect(150), self.cfg), lambda: None

        self.step("9. False-detection check", [
            "Move the robot by hand along plain line WITHOUT markers or strips: straight parts,",
            "curves, tile seams, and its own shadow. Every count should be close to 0.",
        ], check)

        self.cfg.save(self.path)
        print(f"\nSaved {self.path}")
        print("Next: python3 -m line_vision --view   (watch the decisions live, then run without --view)")

    def close(self):
        if self.pump:
            self.pump.stop()
            self.camera.stop()


def main():
    ap = argparse.ArgumentParser(prog="python3 -m line_vision.calibrate", description=__doc__)
    ap.add_argument("--config", type=Path, default=DEFAULT_PATH)
    ap.add_argument("--port", type=int, default=8080, help="live view port")
    args = ap.parse_args()
    wizard = Wizard(args.config, args.port)
    try:
        wizard.run()
    except (KeyboardInterrupt, EOFError):
        print("\naborted; nothing saved", file=sys.stderr)
    finally:
        wizard.close()


if __name__ == "__main__":
    main()
