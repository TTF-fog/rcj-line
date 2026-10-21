import time
from collections import Counter

import cv2

from . import tof, zone_vision
from .balls import floor_balls
from .log import log

STEPS = ["enter", "collect", "green", "red", "exit"]
SIDE = {"silver": "L", "black": "R"}
TRIANGLE = {"silver": "green", "black": "red"}


class Zone:
    """The evacuation zone, from the silver entry strip to the black exit strip."""

    def __init__(self, cfg, board, front, down, detector, line_vision, viewer=None, debug_dir=None):
        self.cfg = cfg
        self.board = board
        self.front = front
        self.down = down
        self.detector = detector
        self.line_vision = line_vision
        self.viewer = viewer
        self.debug_dir = debug_dir
        # Delivered victims are removed by the referee, so this survives a Lack of Progress; held does not.
        self.delivered = Counter()
        self.held = Counter()
        self.step = ""
        self.last_img = None

    def run(self, start="enter"):
        """True once the robot is through the exit and back on the line."""
        steps = STEPS[STEPS.index(start):]
        if start in ("green", "red"):
            self.held = Counter(silver=self.cfg.want_silver, black=self.cfg.want_black)
        if "enter" in steps:
            self.set_step("enter")
            self.enter()
        if "collect" in steps:
            self.set_step("collect")
            self.collect()
        if "green" in steps:
            self.set_step("green")
            self.deliver("silver")
        if "red" in steps:
            self.set_step("red")
            self.deliver("black")
        self.set_step("exit")
        return self.leave()

    # Steps

    def enter(self):
        if tof.read() is None:
            self.board.straight(self.cfg.enter_cm)
        else:
            self.forward_until(lambda d: d["front"] <= d["back"])

    def collect(self):
        c = self.cfg
        end = time.monotonic() + c.collect_budget_s
        empty_scans = 0
        while self.wanted() and empty_scans < c.max_empty_scans and time.monotonic() < end:
            if not self.scan(self.see_ball):
                empty_scans += 1
                if empty_scans < c.max_empty_scans:
                    self.relocate()
                continue
            empty_scans = 0
            colour = self.approach_ball()
            if colour in self.wanted():
                self.board.straight(c.grab_creep_cm)
                self.board.grab(SIDE[colour])
                self.held[colour] += 1
                self.board.beep()
                self.log(f"picked {colour}")
            elif colour:
                self.log(f"{colour} is not needed, turning away")
                self.board.straight(-10)
                self.board.turn(90)

    def deliver(self, colour):
        c = self.cfg
        if not self.held[colour]:
            return
        tri = TRIANGLE[colour]
        for _ in range(c.max_empty_scans):
            if self.scan(lambda f: self.measure_triangle(f, tri) is not None) and \
                    self.drive_to(lambda f: self.measure_triangle(f, tri)):
                break
            self.relocate()
        else:
            self.log(f"{tri} triangle not found")
            return
        # Storage is at the back, so reverse into the triangle.
        self.board.turn(180)
        self.board.straight(-c.drop_back_cm)
        self.board.drop(SIDE[colour])
        self.board.sleep(c.drop_s)
        self.board.drop("N")
        self.board.straight(c.drop_leave_cm)
        self.delivered[colour] += self.held[colour]
        self.held[colour] = 0
        self.board.beep()
        self.log(f"dropped {colour}")

    def leave(self):
        """Right-hand wall follow until a gap with the black exit strip."""
        c = self.cfg
        if tof.read() is None:
            self.log("no TOF sensors, cannot wall follow")
            self.board.stop()
            return False
        end = time.monotonic() + c.leave_timeout_s
        self.forward_until(lambda d: d["front"] < c.corner_mm)
        self.board.turn(90)
        gap_reads = 0
        while time.monotonic() < end:
            self.board.check()
            d = tof.read()
            if d["front"] < c.corner_mm:
                self.board.stop()
                self.board.turn(90)
                continue
            gap_reads = gap_reads + 1 if d["right"] > c.gap_mm else 0
            if gap_reads >= c.gap_reads:
                self.board.stop()
                if self.try_exit():
                    return True
                gap_reads = 0
                continue
            self.board.drive(c.wall_speed, c.wall_kp * (d["right"] - c.wall_mm))
            time.sleep(0.03)
        self.board.stop()
        self.log("exit not found in time")
        return False

    # Building blocks

    def try_exit(self):
        """Turn into the gap on the right and look at the floor: black strip = exit, silver = the entrance."""
        c = self.cfg
        self.board.straight(c.gap_center_cm)
        self.board.turn(-90)
        self.board.send(f"G FWD {c.move_speed}")
        start = time.monotonic()
        floor = None
        while floor is None and time.monotonic() - start < c.exit_probe_cm / c.move_cm_per_s:
            floor = zone_vision.exit_floor(self.line_vision, self.down_frame(), c)
        self.board.send("G STOP")
        travelled_cm = (time.monotonic() - start) * c.move_cm_per_s
        self.log(f"gap floor: {floor}")
        if floor == "black":
            self.board.straight(c.exit_clear_cm)
            self.board.beep(400)
            return True
        self.board.straight(-travelled_cm)
        self.board.turn(90)
        self.board.straight(c.gap_skip_cm)
        return False

    def wanted(self):
        have = self.held + self.delivered
        return {colour for colour, want in (("silver", self.cfg.want_silver), ("black", self.cfg.want_black))
                if have[colour] < want}

    def scan(self, see):
        """Gyro-step one full turn until see(frame) is true on look_frames frames in a row while standing still."""
        for _ in range(round(360 / self.cfg.scan_step_deg)):
            if self.looks(see):
                return True
            self.board.turn(self.cfg.scan_step_deg)
        return False

    def looks(self, see):
        frame = self.still_frame()
        for _ in range(self.cfg.look_frames):
            if not see(frame):
                return False
            frame = self.frame()
        return True

    def relocate(self):
        self.board.turn(90)
        self.board.straight(self.cfg.relocate_cm)

    def forward_until(self, done, timeout=8.0):
        self.board.send(f"G FWD {self.cfg.move_speed}")
        end = time.monotonic() + timeout
        while time.monotonic() < end and not done(tof.read()):
            self.board.sleep(0.03)
        self.board.send("G STOP")

    def drive_to(self, measure):
        """P-steer on measure(frame) -> (x_error -1..1, closeness), None = not seen. True once closeness >= 1."""
        c = self.cfg
        end = time.monotonic() + c.approach_timeout_s
        lost = 0
        while time.monotonic() < end:
            m = measure(self.frame())
            if m is None:
                lost += 1
                self.board.stop()
                if lost > c.lost_frames:
                    self.log("target lost")
                    return False
                continue
            lost = 0
            x_error, closeness = m
            if closeness >= 1:
                self.board.stop()
                return True
            self.board.drive(c.approach_speed * (1 - 0.5 * closeness), c.approach_kp * x_error)
        self.board.stop()
        self.log("approach timed out")
        return False

    def balls(self, frame):
        boxes = self.detector.predict(frame, self.cfg.ball_conf)
        self.show(frame, boxes)
        return floor_balls(self.detector, boxes, frame.shape[0], self.cfg.ball_min_bottom)

    def see_ball(self, frame):
        return any(name in self.wanted() for name, *_ in self.balls(frame))

    def approach_ball(self):
        """Drive to the closest wanted ball. Returns its class by vote over the approach, None if lost."""
        votes = Counter()
        target_x = None

        def measure(frame):
            nonlocal target_x
            balls = self.balls(frame)
            if target_x is None:
                balls = [b for b in balls if b[0] in self.wanted()]
            if not balls:
                return None
            name, x, bottom, _ = min(balls, key=lambda b: 0 if target_x is None else abs(b[1] - target_x))
            target_x = x
            votes[name] += 1
            h, w = frame.shape[:2]
            return (x - w / 2) / (w / 2), bottom / h / self.cfg.grab_line

        if not self.drive_to(measure):
            return None
        self.log(f"votes {dict(votes)}")
        return votes.most_common(1)[0][0]

    def measure_triangle(self, frame, colour):
        mask = zone_vision.colour_mask(frame, self.cfg, colour)
        self.show(frame, mask=mask)
        blob = zone_vision.triangle(mask)
        if blob is None or blob[1] < self.cfg.triangle_min_area:
            return None
        x, area = blob
        return (x - 0.5) * 2, area / self.cfg.triangle_close_area

    # Camera, logging, debug view

    def frame(self):
        self.board.check()
        item = self.front.read(1.0)
        if item is None:
            raise RuntimeError("front camera stalled")
        return item[0]

    def still_frame(self):
        """First frame exposed after the robot has settled."""
        settled = time.monotonic() + self.cfg.settle_s
        self.board.sleep(self.cfg.settle_s)
        while True:
            self.board.check()
            item = self.front.read(1.0)
            if item is None:
                raise RuntimeError("front camera stalled")
            if item[1] >= settled:
                return item[0]

    def down_frame(self):
        item = self.down.read(1.0)
        if item is None:
            raise RuntimeError("down camera stalled")
        return item[0]

    def show(self, frame, boxes=(), mask=None):
        if not (self.viewer or self.debug_dir):
            return
        img = self.detector.draw(frame, boxes)
        if mask is not None:
            img[mask > 0] = img[mask > 0] // 2 + (127, 0, 127)
        text = (f"zone {self.step}  held S{self.held['silver']} B{self.held['black']}  "
                f"done S{self.delivered['silver']} B{self.delivered['black']}")
        cv2.putText(img, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(img, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        self.last_img = img
        if self.viewer:
            self.viewer.publish(img)

    def set_step(self, name):
        self.step = name
        self.log("start")

    def log(self, msg):
        log(f"[zone {self.step}] {msg}")
        if self.debug_dir and self.last_img is not None:
            cv2.imwrite(str(self.debug_dir / f"{time.monotonic():.2f}_zone_{self.step}.jpg"), self.last_img)
