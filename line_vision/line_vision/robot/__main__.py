import argparse
import signal
import sys
import threading
import time
import traceback
from pathlib import Path

import cv2

from line_vision.camera import Camera
from line_vision.config import DEFAULT_PATH as LINE_CONFIG
from line_vision.config import Config
from line_vision.vision import LineVision

from . import log as logfile
from . import tof
from .balls import AI_CAMERA, Detector
from .board import Aborted, Board
from .config import DEFAULT_PATH, RobotConfig
from .dash import Dash
from .log import log
from .run import Run
from .zone import STEPS, Zone


def open_cameras(line_cfg):
    front = Camera(AI_CAMERA)
    front.start()
    front.set_auto()
    try:
        down = Camera(line_cfg.camera)
    except RuntimeError as e:
        log(f"WARNING: {e}; the front camera stands in for the down camera")
        return front, front
    log(f"down camera controls: {down.start()}")
    return front, down


def temperature():
    return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000


def main():
    ap = argparse.ArgumentParser(prog="python -m robot", description="RCJ Rescue Line run.")
    ap.add_argument("--config", type=Path, default=DEFAULT_PATH)
    ap.add_argument("--from", dest="start", choices=["line"] + STEPS, default="line",
                    help="where every attempt starts; zone steps are for bench testing")
    ap.add_argument("--view", type=int, nargs="?", const=8080, metavar="PORT", help="dashboard in the browser")
    ap.add_argument("--debug-dir", type=Path, help="save a frame at every logged event")
    ap.add_argument("--dry", action="store_true", help="print board commands instead of sending them")
    args = ap.parse_args()
    # robot.sh stop sends SIGTERM; exiting through SystemExit runs the finally that halts the motors.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    cfg = RobotConfig.load(args.config)
    log(f"log file {logfile.open_file(cfg.log_dir)}")
    if LINE_CONFIG.exists():
        line_cfg = Config.load(LINE_CONFIG)
    else:
        line_cfg = Config()
        log(f"WARNING: {LINE_CONFIG} missing, line follower uncalibrated (python3 -m line_vision.calibrate)")
    cv2.setNumThreads(1)

    board = Board(cfg.port, cfg, dry=args.dry)
    tof.setup(board)
    time.sleep(0.3)
    log(f"TOF: {tof.read()}" if tof.read() else "WARNING: no TOF lines from the board; running without TOF")
    # The down camera's exposure is calibrated with the light bar on.
    board.light("WHITE")
    front, down = open_cameras(line_cfg)
    if args.debug_dir:
        args.debug_dir.mkdir(parents=True, exist_ok=True)

    start, running = threading.Event(), threading.Event()

    def press():
        (board.abort if running.is_set() else start).set()

    def hold():
        if not running.is_set():
            run.new()
            board.beep(600, 1500)

    def state():
        return {
            "running": running.is_set(), "phase": run.phase, "line": run.line_cmd, "zone_step": zone.step,
            "clock": run.clock(), "lops": run.lops, "delivered": dict(zone.delivered), "held": dict(zone.held),
            "want": {"silver": cfg.want_silver, "black": cfg.want_black},
            "temp": temperature(), "tof": tof.last, "board": board.port,
            "cameras": {"front": "imx500", "down": "ov5647" if down is not front else "missing, using front"},
            "log": list(logfile.recent)[-150:],
        }

    vision = LineVision(line_cfg.vision)
    zone = Zone(cfg, board, front, down, Detector(cfg.model, cfg.yolo_threads), vision, debug_dir=args.debug_dir)
    run = Run(cfg, board, down, vision, line_cfg.nav, zone, debug_dir=args.debug_dir)
    if args.view:
        zone.viewer = run.viewer = Dash(args.view, state, press, hold)
        log(f"dashboard {run.viewer.url()}")
    log(f"cpu {temperature():.0f} C")

    def watch(button):
        held = False

        def on_held():
            nonlocal held
            held = True
            hold()

        def on_released():
            nonlocal held
            if not held:
                press()
            held = False

        button.when_held = on_held
        button.when_released = on_released

    if cfg.buttons:
        from gpiozero import Button

        buttons = [Button(pin, hold_time=cfg.hold_s, bounce_time=0.05) for pin in cfg.buttons]
        for b in buttons:
            watch(b)

    def keyboard():
        for line in sys.stdin:
            hold() if line.strip() == "r" else press()

    threading.Thread(target=keyboard, daemon=True).start()

    try:
        while True:
            log("ready: press = start / Lack of Progress, hold (or r + Enter) = new run")
            start.wait()
            start.clear()
            board.abort.clear()
            running.set()
            try:
                run.attempt(args.start)
            except Aborted:
                run.lop()
            except Exception:
                log(traceback.format_exc())
            finally:
                running.clear()
                board.halt()
    except KeyboardInterrupt:
        pass
    finally:
        board.halt()
        front.stop()
        if down is not front:
            down.stop()


if __name__ == "__main__":
    main()
