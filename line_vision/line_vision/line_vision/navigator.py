import math
from dataclasses import dataclass

from .config import NavConfig
from .vision import Observation

FOLLOW_LINE = "FOLLOW_LINE"
APPROACH_INTERSECTION = "APPROACH_INTERSECTION"
EXECUTE_TURN = "EXECUTE_TURN"
DEAD_END_TURN = "DEAD_END_TURN"
GAP_RECOVERY = "GAP_RECOVERY"
LINE_LOST = "LINE_LOST"
GOAL = "GOAL"
EVACUATION_ZONE = "EVACUATION_ZONE"


@dataclass
class Command:
    state: str
    action: str
    speed: float = 0.0
    turn: float = 0.0
    note: str = ""

    def __str__(self):
        text = f"{self.state:<22} {self.action:<12} speed={self.speed:4.2f} turn={self.turn:+5.2f}"
        return f"{text}  {self.note}" if self.note else text


def _clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


class Navigator:
    def __init__(self, cfg: NavConfig):
        self.cfg = cfg
        self.state = FOLLOW_LINE
        self.state_since = 0.0
        self.cooldown_until = -math.inf
        self.prev_err = None
        self.prev_t = None
        self.counter = 0
        self.lost_count = 0
        self.red_count = 0
        self.silver_count = 0
        self.votes = {"left": 0, "right": 0}
        self.choice = None
        self.target_angle = 0.0
        self.last_good = None
        self.search_dir = 0.0
        self.last_turn = 0.0

    def _enter(self, state, now):
        self.state = state
        self.state_since = now
        self.counter = 0
        self.prev_err = None

    def update(self, obs: Observation, now: float) -> Command:
        c = self.cfg
        if now - obs.timestamp > c.max_frame_age_s:
            return Command(self.state, "STOP", note=f"stale frame ({(now - obs.timestamp) * 1000:.0f} ms)")
        if self.state == GOAL:
            return self._goal(now)
        if self.state == EVACUATION_ZONE:
            return Command(EVACUATION_ZONE, "STOP", note="silver entrance confirmed; zone behaviour not implemented")

        strip = self._check_strips(obs, now)
        if strip:
            return strip
        if obs.line_found:
            self.lost_count = 0
            self.last_good = obs
        else:
            self.lost_count += 1

        handler = {
            FOLLOW_LINE: self._follow,
            APPROACH_INTERSECTION: self._approach,
            EXECUTE_TURN: self._turn,
            DEAD_END_TURN: self._dead_end,
            GAP_RECOVERY: self._gap,
            LINE_LOST: self._lost,
        }[self.state]
        cmd = handler(obs, now)
        self.last_turn = cmd.turn
        return cmd

    def _check_strips(self, obs, now):
        self.red_count = self.red_count + 1 if obs.red_strip else 0
        self.silver_count = self.silver_count + 1 if obs.silver_strip else 0
        if self.state in (DEAD_END_TURN, EXECUTE_TURN):
            return None
        if self.red_count >= self.cfg.strip_frames:
            self._enter(GOAL, now)
            return Command(GOAL, "STOP", note="red goal strip confirmed; holding")
        if self.silver_count >= self.cfg.strip_frames:
            self._enter(EVACUATION_ZONE, now)
            return Command(EVACUATION_ZONE, "STOP", note="silver entrance confirmed; zone behaviour not implemented")
        return None

    def _goal(self, now):
        if now - self.state_since < self.cfg.goal_hold_s:
            return Command(GOAL, "STOP", note=f"holding {now - self.state_since:.1f}s")
        return Command(GOAL, "STOP", note="run finished")

    def _steer(self, lateral, angle_deg, now, base_speed):
        c = self.cfg
        err = c.kp_lateral * lateral + c.kp_angle * angle_deg / 90
        deriv = 0.0
        if self.prev_err is not None and now > self.prev_t:
            deriv = (err - self.prev_err) / (now - self.prev_t)
        self.prev_err, self.prev_t = err, now
        turn = _clamp(err + c.kd * deriv)
        speed = max(c.min_speed, base_speed * (1 - c.slow_gain * min(1.0, abs(err))))
        return min(speed, base_speed), turn

    def _lost_line_step(self, obs, now):
        if self.lost_count < self.cfg.lost_frames:
            return None
        last = self.last_good
        if last is not None and not last.line_end and abs(last.lateral_error) >= self.cfg.search_lateral:
            self.search_dir = math.copysign(1.0, last.lateral_error)
        else:
            self.search_dir = 0.0
        self._enter(GAP_RECOVERY, now)
        return self._gap(obs, now)

    def _follow(self, obs, now):
        c = self.cfg
        if not obs.line_found:
            return self._lost_line_step(obs, now) or Command(
                FOLLOW_LINE, "FOLLOW", c.min_speed, self.last_turn, "line flicker")

        in_cooldown = now < self.cooldown_until
        if not in_cooldown and (obs.is_junction or obs.valid_marker_sides()):
            self.counter += 1
            if self.counter >= c.junction_enter_frames:
                self._enter(APPROACH_INTERSECTION, now)
                self.votes = {"left": 0, "right": 0}
                return self._approach(obs, now)
        else:
            self.counter = 0

        lateral, angle = obs.lateral_error, obs.target_angle_deg
        if in_cooldown and obs.is_junction:
            angle = obs.heading_error_deg
        speed, turn = self._steer(lateral, angle, now, c.base_speed)
        return Command(FOLLOW_LINE, "FOLLOW", speed, turn)

    def _approach(self, obs, now):
        c = self.cfg
        if not obs.line_found:
            return self._lost_line_step(obs, now) or Command(
                APPROACH_INTERSECTION, "SLOW", c.approach_speed, 0.0, "line flicker")
        for side in obs.valid_marker_sides():
            self.votes[side] += 1

        if obs.is_junction:
            self.counter = 0
            if obs.junction_y_frac is not None and obs.junction_y_frac >= c.commit_y_frac:
                return self._commit(obs, now)
        else:
            self.counter += 1
            if self.counter >= c.junction_enter_frames:
                if any(v >= c.marker_min_frames for v in self.votes.values()):
                    return self._commit(obs, now)
                self._enter(FOLLOW_LINE, now)
                return Command(FOLLOW_LINE, "FOLLOW", c.approach_speed, 0.0, "junction not confirmed")

        speed, turn = self._steer(obs.lateral_error, obs.heading_error_deg, now, c.approach_speed)
        return Command(APPROACH_INTERSECTION, "SLOW", speed, turn, self._votes_note())

    def _votes_note(self):
        return f"markers L={self.votes['left']} R={self.votes['right']}"

    def _commit(self, obs, now):
        c = self.cfg
        left = self.votes["left"] >= c.marker_min_frames
        right = self.votes["right"] >= c.marker_min_frames
        note = self._votes_note()
        if left and right:
            self._enter(DEAD_END_TURN, now)
            return Command(DEAD_END_TURN, "U_TURN", 0.0, 1.0, f"double marker, {note}")
        self.choice = "left" if left else "right" if right else "forward"
        self.target_angle = {"left": -90.0, "right": 90.0, "forward": 0.0}[self.choice]
        if self.choice == "forward" and not any(e.side == "forward" for e in obs.exits):
            note += "; no marker but no straight branch seen"
        self._enter(EXECUTE_TURN, now)
        cmd = self._turn(obs, now)
        cmd.note = f"chose {self.choice}, {note}"
        return cmd

    def _turn(self, obs, now):
        c = self.cfg
        action = {"left": "TURN_LEFT", "right": "TURN_RIGHT", "forward": "STRAIGHT"}[self.choice]
        elapsed = now - self.state_since
        if elapsed > c.turn_max_s:
            self._enter(LINE_LOST, now)
            return self._lost(obs, now)
        if elapsed < c.commit_advance_s:
            return Command(EXECUTE_TURN, action, c.approach_speed, 0.0, "advancing to junction")
        if not obs.line_found:
            if self.choice == "forward":
                return self._lost_line_step(obs, now) or Command(EXECUTE_TURN, action, c.approach_speed, 0.0)
            return Command(EXECUTE_TURN, action, 0.0, math.copysign(1.0, self.target_angle), "searching branch")

        matching = [e for e in obs.exits if e.side == self.choice]
        candidates = matching or obs.exits
        if candidates:
            self.target_angle = min(candidates, key=lambda e: abs(e.angle_deg - self.target_angle)).angle_deg
        else:
            self.target_angle = obs.target_angle_deg

        if elapsed >= c.commit_advance_s + c.turn_min_s and not obs.is_junction and abs(obs.target_angle_deg) < 30:
            self.counter += 1
            if self.counter >= c.turn_done_frames:
                self.cooldown_until = now + c.junction_cooldown_s
                self._enter(FOLLOW_LINE, now)
                return self._follow(obs, now)
        else:
            self.counter = 0

        if self.choice == "forward":
            speed, turn = self._steer(obs.lateral_error, self.target_angle, now, c.approach_speed)
        else:
            turn = _clamp(self.target_angle / 60)
            speed = c.turn_speed * max(0.0, math.cos(math.radians(self.target_angle)))
        return Command(EXECUTE_TURN, action, speed, turn, f"target {self.target_angle:+.0f} deg")

    def _dead_end(self, obs, now):
        c = self.cfg
        elapsed = now - self.state_since
        if elapsed > c.turn_max_s:
            self._enter(LINE_LOST, now)
            return self._lost(obs, now)
        aligned = (obs.line_found and not obs.is_junction
                   and abs(obs.lateral_error) < 0.3 and abs(obs.target_angle_deg) < 20)
        if elapsed >= c.u_turn_min_s and aligned:
            self.counter += 1
            if self.counter >= c.reacquire_frames:
                self.cooldown_until = now + c.junction_cooldown_s
                self._enter(FOLLOW_LINE, now)
                return self._follow(obs, now)
        else:
            self.counter = 0
        return Command(DEAD_END_TURN, "U_TURN", 0.0, 1.0, f"{elapsed:.1f}s")

    def _gap(self, obs, now):
        c = self.cfg
        if obs.line_found:
            self.counter += 1
            if self.counter >= c.reacquire_frames:
                self._enter(FOLLOW_LINE, now)
                return self._follow(obs, now)
        else:
            self.counter = 0
        elapsed = now - self.state_since
        if elapsed > c.gap_max_s:
            self._enter(LINE_LOST, now)
            return self._lost(obs, now)
        if self.search_dir:
            action = "SEARCH_LEFT" if self.search_dir < 0 else "SEARCH_RIGHT"
            return Command(GAP_RECOVERY, action, 0.0, 0.6 * self.search_dir, f"{elapsed:.1f}s")
        return Command(GAP_RECOVERY, "GAP_FORWARD", c.gap_speed, 0.0, f"hold heading {elapsed:.1f}s")

    def _lost(self, obs, now):
        if obs.line_found:
            self.counter += 1
            if self.counter >= self.cfg.reacquire_frames:
                self._enter(FOLLOW_LINE, now)
                return self._follow(obs, now)
        else:
            self.counter = 0
        return Command(LINE_LOST, "STOP", note="line not reacquired; wait for Lack of Progress restart")
