import time

import cv2

from line_vision import overlay
from line_vision.navigator import EVACUATION_ZONE, GOAL, Navigator

from .log import log


class Run:
    """State that survives a Lack of Progress, and one attempt from a (re)start until the goal.

    After any LoP the referee puts the robot on the previous checkpoint, so every attempt starts with line
    following. If that leads into the zone again, the zone skips the victims already delivered.
    """

    def __init__(self, cfg, board, down, vision, nav_cfg, zone, viewer=None, debug_dir=None):
        self.cfg = cfg
        self.board = board
        self.down = down
        self.vision = vision
        self.nav_cfg = nav_cfg
        self.zone = zone
        self.viewer = viewer
        self.debug_dir = debug_dir
        self.lops = 0
        self.started = None
        self.finished = False
        # ready, line, zone, stuck or finished; read by the dashboard.
        self.phase = "ready"
        self.line_cmd = ""

    def new(self):
        self.zone.delivered.clear()
        self.zone.held.clear()
        self.lops = 0
        self.started = None
        self.finished = False
        self.phase = "ready"
        log("new run")

    def attempt(self, start="line"):
        if self.finished:
            self.new()
        if self.started is None:
            self.started = time.monotonic()
        log(f"attempt from {start}: {self.status()}")
        self.board.light("WHITE")
        self.board.beep()
        if start != "line":
            self.phase = "zone"
            if not self.zone.run(start):
                self.stuck()
        while True:
            self.phase = "line"
            if self.follow_line() == "goal":
                self.finished = True
                self.phase = "finished"
                self.board.beep(800)
                log(f"goal, run finished: {self.status()}")
                return
            self.phase = "zone"
            if not self.zone.run():
                self.stuck()

    def lop(self):
        self.lops += 1
        # Held victims are put back on the floor by the referee.
        self.zone.held.clear()
        self.phase = "ready"
        log(f"lack of progress #{self.lops}: {self.status()}")

    def stuck(self):
        self.phase = "stuck"
        log("stuck, waiting for Lack of Progress")
        self.board.halt()
        while True:
            self.board.sleep(1)

    def clock(self):
        return 0.0 if self.started is None else time.monotonic() - self.started

    def status(self):
        d = self.zone.delivered
        return f"t={self.clock():.0f}s lops={self.lops} rescued S{d['silver']} B{d['black']}"

    def follow_line(self):
        """Drive the line follower until it confirms the silver entrance ("zone") or finishes the goal hold ("goal")."""
        nav = Navigator(self.nav_cfg)
        last_state = None
        frames = 0
        while True:
            self.board.check()
            item = self.down.read(self.nav_cfg.frame_timeout_s)
            if item is None:
                self.board.stop()
                continue
            frame, timestamp = item
            obs = self.vision.process(frame, timestamp)
            now = time.monotonic()
            cmd = nav.update(obs, now)

            changed = cmd.state != last_state
            if changed:
                log(f"[line] {cmd}")
            last_state = cmd.state
            self.line_cmd = str(cmd)
            frames += 1
            if self.viewer and frames % 2 == 0 or self.debug_dir and changed:
                self.show(obs, cmd, changed)

            if cmd.state == EVACUATION_ZONE:
                self.board.stop()
                return "zone"
            if cmd.state == GOAL and now - nav.state_since >= self.nav_cfg.goal_hold_s:
                return "goal"
            if cmd.action == "STOP":
                self.board.stop()
            else:
                self.board.drive(self.cfg.line_pwm * cmd.speed, self.cfg.line_turn_pwm * cmd.turn)

    def show(self, obs, cmd, save):
        img = overlay.draw(obs, self.vision.roi_rect(), [f"line {cmd.state} {cmd.action}", self.status(), cmd.note])
        if self.viewer:
            self.viewer.publish(img)
        if save and self.debug_dir:
            cv2.imwrite(str(self.debug_dir / f"{time.monotonic():.2f}_line_{cmd.state}.jpg"), img)
