import cv2
import numpy as np


def colour_mask(frame, cfg, colour):
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    if colour == "green":
        mask = cv2.inRange(hsv, cfg.green_lo, cfg.green_hi)
    else:
        mask = cv2.inRange(hsv, cfg.red_lo1, cfg.red_hi1) | cv2.inRange(hsv, cfg.red_lo2, cfg.red_hi2)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))


def triangle(mask):
    """(centre_x_fraction, area_fraction) of the largest blob, or None."""
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return cents[i][0] / mask.shape[1], stats[i, cv2.CC_STAT_AREA] / mask.size


def exit_floor(vision, frame, cfg):
    """"black" for the exit strip, "silver" for the entrance strip, None if neither is under the down camera."""
    obs = vision.process(frame, 0.0)
    if obs.silver_strip:
        return "silver"
    row_fill = (obs.debug["line"] > 0).mean(axis=1)
    if (row_fill >= cfg.strip_row_fill).sum() >= cfg.strip_min_rows:
        return "black"
    return None
