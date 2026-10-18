import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from .config import VisionConfig


@dataclass
class Exit:
    side: str  # "left", "forward" or "right"
    x: float
    y: float
    angle_deg: float  # from the robot's bottom-centre point; negative = left
    width: int


@dataclass
class Marker:
    x: float
    y: float
    w: int
    h: int
    side: str | None
    before_crossing: bool


@dataclass
class Strip:
    x: int
    y: int
    w: int
    h: int


@dataclass
class Observation:
    timestamp: float
    roi_w: int
    roi_h: int
    line_found: bool = False
    lateral_error: float = 0.0  # -1 = line at left ROI edge, +1 = right edge
    heading_error_deg: float = 0.0  # fitted direction of the near track; positive = line bends right
    target_angle_deg: float = 0.0  # angle to the point the line leaves the ROI
    track: list = field(default_factory=list)  # (x, y, width) per band, bottom first
    entry: Exit | None = None
    exits: list = field(default_factory=list)
    line_end: bool = False  # line ends inside the ROI: gap or end of line ahead
    junction_y_frac: float | None = None
    markers: list = field(default_factory=list)
    red_strip: Strip | None = None
    silver_strip: Strip | None = None
    line_width: float = 0.0
    debug: dict = field(default_factory=dict)

    @property
    def is_junction(self):
        return len(self.exits) >= 2

    def valid_marker_sides(self):
        return {m.side for m in self.markers if m.before_crossing and m.side}


def _runs(values):
    """[start, end) index pairs of True runs in a 1-D bool array."""
    d = np.diff(np.concatenate(([0], values.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def _angle_from_robot(x, y, w, h):
    return math.degrees(math.atan2(x - w / 2, (h - 1) - y))


def floor_ratio(gray, bg_kernel):
    """Brightness relative to the local floor white, so shading and vignetting cancel out."""
    local_white = cv2.blur(cv2.dilate(gray, bg_kernel), (15, 15))
    return gray.astype(np.float32) / np.maximum(local_white, 1).astype(np.float32)


def background_kernel(size):
    k = int(size) | 1
    return cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))


class LineVision:
    def __init__(self, cfg: VisionConfig):
        self.cfg = cfg
        self.last_x = None
        self._bg_kernel = background_kernel(cfg.background_kernel)
        self._k3 = np.ones((3, 3), np.uint8)
        self._k5 = np.ones((5, 5), np.uint8)

    def roi_rect(self):
        c = self.cfg
        x0, x1 = int(c.roi_left * c.proc_width), int(c.roi_right * c.proc_width)
        y0, y1 = int(c.roi_top * c.proc_height), int(c.roi_bottom * c.proc_height)
        return x0, y0, x1, y1

    def process(self, frame_bgr, timestamp):
        c = self.cfg
        proc = cv2.resize(frame_bgr, (c.proc_width, c.proc_height), interpolation=cv2.INTER_AREA)
        x0, y0, x1, y1 = self.roi_rect()
        roi = proc[y0:y1, x0:x1]
        h, w = roi.shape[:2]
        obs = Observation(timestamp=timestamp, roi_w=w, roi_h=h)
        obs.debug["proc"] = proc

        gray = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        hsv = cv2.cvtColor(cv2.GaussianBlur(roi, (5, 5), 0), cv2.COLOR_BGR2HSV)
        ratio = floor_ratio(gray, self._bg_kernel)

        green = self._clean(cv2.inRange(hsv, c.green_lo, c.green_hi))
        red = self._clean(cv2.inRange(hsv, c.red_lo1, c.red_hi1) | cv2.inRange(hsv, c.red_lo2, c.red_hi2))
        coloured = cv2.dilate(green | red, self._k5) > 0

        black = ((ratio < c.black_rel) & (gray < c.black_rel_max_gray)) | (gray < c.black_abs)
        black &= ~coloured
        black = cv2.morphologyEx(black.astype(np.uint8) * 255, cv2.MORPH_OPEN, self._k3)
        black = cv2.morphologyEx(black, cv2.MORPH_CLOSE, self._k3)
        line_mask, main = self._line_components(black, w, h)
        obs.debug.update(line=line_mask, main=main, green=green, red=red)

        if main is not None:
            self._measure_line(obs, main)
        obs.markers = self._markers(green, line_mask, obs)
        obs.red_strip = self._strip(red, obs)

        sat = hsv[:, :, 1]
        silver = gray >= c.silver_specular
        if c.silver_rel_band:
            silver |= (ratio >= c.silver_rel_band[0]) & (ratio <= c.silver_rel_band[1])
        silver &= sat < c.silver_max_sat
        silver &= cv2.dilate(line_mask, self._k5) == 0
        silver = cv2.morphologyEx(silver.astype(np.uint8) * 255, cv2.MORPH_OPEN, self._k5)
        obs.debug["silver"] = silver
        obs.silver_strip = self._strip(silver, obs)
        return obs

    def _clean(self, mask):
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._k3)

    def _line_components(self, black, w, h):
        n, labels, stats, _ = cv2.connectedComponentsWithStats(black, connectivity=8)
        min_area = self.cfg.min_line_area_frac * w * h
        keep = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= min_area]
        if not keep:
            return np.zeros_like(black), None
        line_mask = np.isin(labels, keep).astype(np.uint8) * 255

        ref_x = self.last_x if self.last_x is not None else w / 2
        bottom = labels[int(h * (1 - self.cfg.entry_zone_frac)):, :]
        best, best_dist = None, None
        for i in keep:
            xs = np.flatnonzero((bottom == i).any(axis=0))
            if xs.size:
                dist = np.abs(xs - ref_x).min()
                if best_dist is None or dist < best_dist:
                    best, best_dist = i, dist
        if best is None:
            best = max(keep, key=lambda i: stats[i, cv2.CC_STAT_AREA])
        return line_mask, (labels == best).astype(np.uint8) * 255

    def _measure_line(self, obs, main):
        c = self.cfg
        h, w = main.shape
        obs.line_found = True
        ref_x = self.last_x if self.last_x is not None else w / 2

        crossings = self._border_crossings(main)
        entry_candidates = [r for r in crossings if r.y >= h * (1 - c.entry_zone_frac)]
        if entry_candidates:
            obs.entry = min(entry_candidates, key=lambda r: math.hypot(r.x - ref_x, r.y - (h - 1)))
        obs.exits = [r for r in crossings if r is not obs.entry]
        if obs.entry is not None:
            for e in obs.exits:
                if e.side == "back":
                    e.side = "left" if e.x < obs.entry.x else "right"

        track_ref = obs.entry.x if obs.entry else ref_x
        band_h = max(1, h // c.bands)
        for i in range(c.bands):
            yb1 = h - i * band_h
            yb0 = max(0, yb1 - band_h)
            runs = _runs(main[yb0:yb1].any(axis=0))
            if not runs:
                if obs.track:
                    break
                continue
            s, e = min(runs, key=lambda r: abs((r[0] + r[1]) / 2 - track_ref))
            track_ref = (s + e) / 2
            obs.track.append((track_ref, (yb0 + yb1) / 2, int(e - s)))

        if obs.track:
            near_x = obs.track[0][0]
            obs.lateral_error = float(np.clip((near_x - w / 2) / (w / 2), -1, 1))
            self.last_x = near_x
            obs.line_width = float(np.median([t[2] for t in obs.track]))
        if len(obs.track) >= 2:
            ys = np.array([t[1] for t in obs.track])
            xs = np.array([t[0] for t in obs.track])
            slope = np.polyfit(ys, xs, 1)[0]
            obs.heading_error_deg = math.degrees(math.atan(-slope))

        forward = [e for e in obs.exits if e.side == "forward"]
        if len(obs.exits) == 1:
            obs.target_angle_deg = obs.exits[0].angle_deg
        elif forward:
            obs.target_angle_deg = min(forward, key=lambda e: abs(e.angle_deg)).angle_deg
        elif obs.track:
            tx, ty, _ = obs.track[-1]
            obs.target_angle_deg = _angle_from_robot(tx, ty, w, h)
        obs.line_end = not obs.exits

        if obs.is_junction:
            side_ys = [e.y for e in obs.exits if e.side in ("left", "right")]
            obs.junction_y_frac = (float(np.mean(side_ys)) if side_ys else 0.0) / h

    def _border_crossings(self, mask):
        """Runs where the line touches the ROI border, walking clockwise from the top-left corner."""
        h, w = mask.shape
        b = self.cfg.border_px
        on = np.concatenate([
            mask[:b, :].any(axis=0),
            mask[:, w - b:].any(axis=1),
            mask[h - b:, :].any(axis=0)[::-1],
            mask[:, :b].any(axis=1)[::-1],
        ])
        xs = np.concatenate([np.arange(w), np.full(h, w - 1), np.arange(w)[::-1], np.zeros(h)])
        ys = np.concatenate([np.zeros(w), np.arange(h), np.full(w, h - 1), np.arange(h)[::-1]])
        edges = np.concatenate([["forward"] * w, ["right"] * h, ["back"] * w, ["left"] * h])
        if on.all() or not on.any():
            return []
        shift = int(np.flatnonzero(~on)[0])
        rolled = np.roll(on, -shift)
        out = []
        for s, e in _runs(rolled):
            if e - s < self.cfg.min_exit_px:
                continue
            mid = (shift + (s + e - 1) // 2) % on.size
            x, y = float(xs[mid]), float(ys[mid])
            out.append(Exit(str(edges[mid]), x, y, _angle_from_robot(x, y, w, h), int(e - s)))
        return out

    def _markers(self, green, line_mask, obs):
        c = self.cfg
        h, w = green.shape
        n, _, stats, cents = cv2.connectedComponentsWithStats(green, connectivity=8)
        out = []
        for i in range(1, n):
            x, y, bw, bh, area = stats[i]
            if not (c.marker_min_area_frac * w * h <= area <= c.marker_max_area_frac * w * h):
                continue
            if area / (bw * bh) < c.marker_min_fill or max(bw, bh) / min(bw, bh) > c.marker_max_aspect:
                continue
            cx, cy = cents[i]
            reach = int(bh * 1.5)
            above = line_mask[max(0, y - reach):y, x:x + bw]
            below = line_mask[y + bh:y + bh + reach, x:x + bw]
            above_frac = above.mean() / 255 if above.size and y - reach >= 0 else 0.0
            below_frac = below.mean() / 255 if below.size else 0.0
            side = None
            if obs.track:
                line_x = min(obs.track, key=lambda t: abs(t[1] - cy))[0]
                side = "left" if cx < line_x else "right"
            out.append(Marker(float(cx), float(cy), int(bw), int(bh), side,
                              bool(above_frac > 0.15 and above_frac > below_frac)))
        return out

    def _strip(self, mask, obs):
        """A wide band across the line's path where the line stops (goal or evacuation-zone entrance)."""
        c = self.cfg
        h, w = mask.shape
        n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        best = None
        for i in range(1, n):
            x, y, bw, bh, area = stats[i]
            if bw < c.strip_min_width_frac * w or bh < c.strip_min_height_px or bw / bh < c.strip_min_aspect:
                continue
            if area / (bw * bh) < c.strip_min_fill:
                continue
            if any(e.y < y for e in obs.exits):
                continue
            if best is None or area > best[4]:
                best = stats[i]
        return Strip(*map(int, best[:4])) if best is not None else None
