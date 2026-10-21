"""Guided zone calibration: python -m robot.calibrate [step ...]. Writes the accepted values to robot.json."""
import argparse
import json
import threading
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from line_vision.calibrate import CalibrationError, measure_colour
from line_vision.camera import Camera
from line_vision.config import VisionConfig
from line_vision.viewer import Viewer

from . import tof, zone_vision
from .balls import AI_CAMERA, Detector, floor_balls
from .board import Board
from .config import DEFAULT_PATH, RobotConfig

STEPS = ["tof", "wall", "move", "grab", "far", "green", "red", "triangle"]


class Pump(threading.Thread):
    """Runs the detector on every front-camera frame for the live view and keeps the latest results."""

    def __init__(self, camera, detector, viewer, cfg):
        super().__init__(daemon=True)
        self.camera, self.detector, self.viewer, self.cfg = camera, detector, viewer, cfg
        self.title = ""
        self.colour = None
        self.latest = None
        self.count = 0
        self.cond = threading.Condition()

    def run(self):
        while True:
            item = self.camera.read(1.0)
            if item is None:
                continue
            frame = item[0]
            boxes = self.detector.predict(frame, self.cfg.ball_conf)
            img = self.detector.draw(frame, boxes)
            if self.colour:
                mask = zone_vision.colour_mask(frame, self.cfg, self.colour)
                img[mask > 0] = img[mask > 0] // 2 + (127, 0, 127)
            cv2.putText(img, self.title, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            self.viewer.publish(img)
            with self.cond:
                self.latest = (frame, boxes)
                self.count += 1
                self.cond.notify_all()

    def collect(self, n):
        """The next n (frame, boxes) results."""
        out = []
        with self.cond:
            while len(out) < n:
                seen = self.count
                if not self.cond.wait_for(lambda: self.count > seen, 2):
                    raise CalibrationError("front camera stalled")
                out.append(self.latest)
        return out


class Wizard:
    def __init__(self, path, port):
        self.path = path
        self.saved = json.loads(path.read_text()) if path.exists() else {}
        self.cfg = RobotConfig.load(path)
        self.changes = {}
        self.board = None
        self.viewer = Viewer(port)
        self.camera = Camera(AI_CAMERA)
        self.camera.start()
        self.camera.set_auto()
        self.pump = Pump(self.camera, Detector(self.cfg.model, self.cfg.yolo_threads), self.viewer, self.cfg)
        self.pump.start()

    def set(self, **values):
        self.changes.update(values)
        for key, value in values.items():
            setattr(self.cfg, key, value)

    def get_board(self):
        if self.board is None:
            self.board = Board(self.cfg.port, self.cfg)
            tof.setup(self.board)
            time.sleep(0.3)
        return self.board

    def step(self, title, instructions, action):
        """action() -> (report lines, settings dict). Returns True if accepted."""
        print(f"\n=== {title} ===")
        for line in instructions:
            print(f"  {line}")
        self.pump.title = title
        while True:
            if input("Press Enter to measure (s = skip): ").strip().lower() == "s":
                return False
            try:
                lines, settings = action()
            except (CalibrationError, RuntimeError, OSError) as e:
                print(f"  ! {e}")
                continue
            for line in lines:
                print(f"  {line}")
            if input("Enter = accept, r = redo: ").strip().lower() == "r":
                continue
            self.set(**settings)
            return True

    def tof_samples(self, n=20):
        self.get_board()
        samples = []
        for _ in range(n):
            d = tof.read()
            if d is None:
                raise CalibrationError("no TOF lines from the board; check the firmware and sensors")
            samples.append(d)
            time.sleep(0.05)
        return {k: int(np.median([s[k] for s in samples])) for k in ("front", "back", "left", "right")}

    def ball_bottoms(self, n=15):
        votes, bottoms = Counter(), []
        for frame, boxes in self.pump.collect(n):
            balls = floor_balls(self.pump.detector, boxes, frame.shape[0], 0.0)
            if balls:
                name, _, bottom, _ = balls[0]
                votes[name] += 1
                bottoms.append(bottom / frame.shape[0])
        if len(bottoms) < n // 2:
            raise CalibrationError(f"ball seen in only {len(bottoms)} of {n} frames; check the live view")
        return float(np.median(bottoms)), votes

    def run(self, steps):
        print(f"Live view: open {self.viewer.url()} in a browser on the same network.")
        print("Ctrl+C stops at any time; nothing is saved until the end.")
        c = self.cfg

        def check_tof():
            d = self.tof_samples()
            return [f"front {d['front']}  back {d['back']}  left {d['left']}  right {d['right']} mm"], {}

        def wall():
            d = self.tof_samples()
            return [f"right {d['right']} mm -> wall_mm, gap_mm {d['right'] + 200}"], \
                {"wall_mm": d["right"], "gap_mm": d["right"] + 200}

        def move():
            self.get_board().straight(2 * c.move_cm_per_s)
            cm = float(input("  Distance driven in cm: "))
            return [f"{cm / 2:.1f} cm/s at speed {c.move_speed}"], {"move_cm_per_s": round(cm / 2, 1)}

        def grab():
            bottom, votes = self.ball_bottoms()
            return [f"box bottom at {bottom:.2f} of the image, seen as {dict(votes)}"], {"grab_line": round(bottom, 3)}

        def far():
            bottom, votes = self.ball_bottoms()
            limit = round(bottom - 0.04, 3)
            if limit >= c.grab_line:
                raise CalibrationError(f"far ball at {bottom:.2f} is below the grab line {c.grab_line:.2f}")
            return [f"box bottom at {bottom:.2f}, seen as {dict(votes)}; ignoring boxes above {limit:.2f}"], \
                {"ball_min_bottom": limit}

        def colour(kind):
            def action():
                frames = [f for f, _ in self.pump.collect(10)]
                settings, lines = measure_colour(frames, VisionConfig(), kind)
                return lines, settings
            return action

        def triangle():
            areas = []
            for frame, _ in self.pump.collect(10):
                blob = zone_vision.triangle(zone_vision.colour_mask(frame, c, "green"))
                areas.append(blob[1] if blob else 0.0)
            area = float(np.median(areas))
            if area < c.triangle_min_area:
                raise CalibrationError(f"green covers only {area:.3f} of the image; calibrate green first")
            return [f"green covers {area:.2f} of the image"], {"triangle_close_area": round(area * 0.95, 3)}

        wizard = {
            "tof": ("TOF check", ["Robot anywhere in the zone. All four readings should look right."], check_tof),
            "wall": ("Wall distance", ["Robot parallel to a wall, right side at the distance it should follow at",
                                       "(about 10 cm)."], wall),
            "move": ("Straight speed", ["Clear 1 m ahead and mark the robot's front. It drives straight for 2 s,",
                                        "then you type the distance in cm."], move),
            "grab": ("Grab line", [f"Put a ball {c.grab_creep_cm:.0f} cm in front of where the claw picks it up,",
                                   "straight ahead. The robot stops here before its final creep."], grab),
            "far": ("Far limit", ["Put a ball against the far wall of the zone, about 110 cm away."], far),
            "green": ("Green", ["Face the green triangle from about 50 cm, in the zone lighting."], colour("green")),
            "red": ("Red", ["Face the red triangle from about 50 cm, in the zone lighting."], colour("red")),
            "triangle": ("Triangle stop", ["Put the robot where it should stop facing the green triangle,",
                                           "before it turns around to back in."], triangle),
        }
        for name in steps:
            self.pump.colour = {"green": "green", "red": "red", "triangle": "green"}.get(name)
            self.step(*wizard[name])
        self.pump.colour = None

        if not self.changes:
            print("\nNothing changed.")
            return
        print("\nNew values:")
        for key, value in self.changes.items():
            print(f"  {key} = {value}")
        if input(f"Save to {self.path}? [Y/n] ").strip().lower() != "n":
            self.saved.update({k: list(v) if isinstance(v, tuple) else v for k, v in self.changes.items()})
            self.path.write_text(json.dumps(self.saved, indent=2) + "\n")
            print("saved")

    def close(self):
        if self.board:
            self.board.halt()
        self.camera.stop()


def main():
    ap = argparse.ArgumentParser(prog="python -m robot.calibrate", description=__doc__)
    ap.add_argument("steps", nargs="*", choices=STEPS, help=f"steps to run (default all: {' '.join(STEPS)})")
    ap.add_argument("--config", type=Path, default=DEFAULT_PATH)
    ap.add_argument("--port", type=int, default=8080, help="live view port")
    args = ap.parse_args()
    cv2.setNumThreads(1)
    wizard = Wizard(args.config, args.port)
    try:
        wizard.run(args.steps or STEPS)
    except KeyboardInterrupt:
        print("\nstopped, nothing saved")
    finally:
        wizard.close()


if __name__ == "__main__":
    main()
