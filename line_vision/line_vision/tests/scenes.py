import cv2
import numpy as np

W, H = 640, 480
LINE = 40
MARKER = 66
BLACK = (25, 25, 25)
GREEN = (40, 150, 30)
RED = (30, 30, 200)


def floor(seed=0):
    rng = np.random.default_rng(seed)
    img = np.full((H, W, 3), 215, np.float32)
    yy, xx = np.mgrid[0:H, 0:W]
    img *= (1 - 0.25 * (((xx - W / 2) / W) ** 2 + ((yy - H / 2) / H) ** 2))[..., None]
    img += rng.normal(0, 4, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def vline(img, x=W // 2, y0=0, y1=H):
    cv2.rectangle(img, (x - LINE // 2, y0), (x + LINE // 2, y1), BLACK, -1)
    return img


def hline(img, y, x0=0, x1=W):
    cv2.rectangle(img, (x0, y - LINE // 2), (x1, y + LINE // 2), BLACK, -1)
    return img


def marker(img, side, crossing_y, before=True, x=W // 2):
    """Green square in the corner between the incoming line and the crossing line."""
    gap = 4
    mx0 = x - LINE // 2 - gap - MARKER if side == "left" else x + LINE // 2 + gap
    my0 = crossing_y + LINE // 2 + gap if before else crossing_y - LINE // 2 - gap - MARKER
    cv2.rectangle(img, (mx0, my0), (mx0 + MARKER, my0 + MARKER), GREEN, -1)
    return img


def strip(img, y, colour, height=MARKER, x=W // 2, width=500):
    cv2.rectangle(img, (x - width // 2, y), (x + width // 2, y + height), colour, -1)
    return img


def straight(x=W // 2):
    return vline(floor(), x)


def corner(side, y=160):
    img = vline(floor(), y0=y - LINE // 2)
    return hline(img, y, *((0, W // 2) if side == "left" else (W // 2, W)))


def junction(crossing_y, markers=(), before=True, forward=True):
    img = vline(floor(), y0=0 if forward else crossing_y - LINE // 2)
    hline(img, crossing_y)
    for side in markers:
        marker(img, side, crossing_y, before)
    return img


def line_end(y=220):
    return vline(floor(), y0=y)


def goal(y=150):
    img = vline(floor(), y0=y + MARKER)
    return strip(img, y, RED)


def silver_entrance(y=150):
    img = vline(floor(), y0=y + MARKER)
    return strip(img, y, (255, 255, 255))
