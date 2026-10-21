"""TOF distances streamed by the Motion 2350 firmware as `TOF,<front>,<back>,<left>,<right>` lines in mm.

-1 (or anything past FAR_MM) means nothing in range. Readings older than STALE_S count as no TOF.
"""
import time

SIDES = ("front", "back", "left", "right")
# Reported when a sensor sees nothing in range, so an open gap reads as far away.
FAR_MM = 2000
STALE_S = 0.5

# Last reading, for the dashboard.
last = None
_board = None


def setup(board):
    global _board
    _board = board


def parse(line):
    values = [int(v) for v in line.split(",")[1:5]]
    return {side: v if 0 <= v < FAR_MM else FAR_MM for side, v in zip(SIDES, values)}


def read():
    """{"front", "back", "left", "right"} distances in mm, or None without fresh TOF lines."""
    global last
    fresh = _board is not None and _board.tof and time.monotonic() - _board.tof_time < STALE_S
    last = dict(_board.tof) if fresh else None
    return last
