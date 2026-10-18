import argparse
import sys
import time
from pathlib import Path

import cv2

from . import overlay
from .config import DEFAULT_PATH, Config
from .navigator import Command, Navigator
from .vision import LineVision

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}


def file_frames(path, fps):
    """Yield (frame, timestamp) from an image, a directory of images, or a video, on a synthetic clock."""
    path = Path(path)
    if path.is_dir():
        files = sorted(p for p in path.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES and not p.name.startswith("dbg_"))
        for i, f in enumerate(files):
            yield cv2.imread(str(f)), i / fps
    elif path.suffix.lower() in IMAGE_SUFFIXES:
        yield cv2.imread(str(path)), 0.0
    else:
        cap = cv2.VideoCapture(str(path))
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield frame, i / fps
            i += 1


def main():
    ap = argparse.ArgumentParser(prog="python3 -m line_vision", description="Camera line following; prints drive commands.")
    ap.add_argument("--input", help="image, image directory or video to replay instead of the live camera")
    ap.add_argument("--config", type=Path, default=DEFAULT_PATH, help=f"calibration file (default {DEFAULT_PATH})")
    ap.add_argument("--view", type=int, nargs="?", const=8080, metavar="PORT",
                    help="serve a live debug view in the browser (default port 8080)")
    ap.add_argument("--debug-dir", type=Path, help="save raw frames and annotated overlays here")
    ap.add_argument("--save-every", type=int, default=15, help="frames between saves (state changes always save)")
    ap.add_argument("--frames", type=int, default=0, help="stop after this many frames (0 = run forever)")
    args = ap.parse_args()

    if args.config.exists():
        cfg = Config.load(args.config)
        print(f"loaded {args.config}", file=sys.stderr)
    else:
        cfg = Config()
        print(f"WARNING: {args.config} not found, using uncalibrated defaults. "
              "Run: python3 -m line_vision.calibrate", file=sys.stderr)
    vision = LineVision(cfg.vision)
    nav = Navigator(cfg.nav)
    if args.debug_dir:
        args.debug_dir.mkdir(parents=True, exist_ok=True)
    viewer = None
    if args.view:
        from .viewer import Viewer

        viewer = Viewer(args.view)
        print(f"live view: {viewer.url()}", file=sys.stderr)

    camera = None
    if args.input:
        source = file_frames(args.input, cfg.camera.fps)
    else:
        from .camera import Camera

        camera = Camera(cfg.camera)
        locked = camera.start()
        print(f"camera controls: {locked}", file=sys.stderr)

        def live():
            while True:
                yield camera.read(cfg.nav.frame_timeout_s)

        source = live()

    last_state = None
    try:
        for i, item in enumerate(source):
            if args.frames and i >= args.frames:
                break
            if item is None:
                print(Command(nav.state, "STOP", note="camera stalled"), flush=True)
                continue
            frame, timestamp = item
            t0 = time.monotonic()
            obs = vision.process(frame, timestamp)
            now = timestamp if camera is None else time.monotonic()
            cmd = nav.update(obs, now)
            proc_ms = (time.monotonic() - t0) * 1000
            print(f"{i:06d} {cmd}  [{proc_ms:.1f} ms]", flush=True)

            save = args.debug_dir and (i % args.save_every == 0 or cmd.state != last_state)
            if save or (viewer and i % 2 == 0):
                info = (f"lat={obs.lateral_error:+.2f} head={obs.heading_error_deg:+.0f} "
                        f"tgt={obs.target_angle_deg:+.0f} exits={''.join(e.side[0] for e in obs.exits)}")
                img = overlay.draw(obs, vision.roi_rect(), [f"{cmd.state} {cmd.action}", info, cmd.note])
                if viewer:
                    viewer.publish(img)
                if save:
                    cv2.imwrite(str(args.debug_dir / f"dbg_{i:06d}.jpg"), img)
                    if camera is not None:
                        cv2.imwrite(str(args.debug_dir / f"raw_{i:06d}.jpg"), frame)
            last_state = cmd.state
    except KeyboardInterrupt:
        pass
    finally:
        print(Command(nav.state, "STOP", note="shutdown"), flush=True)
        if camera is not None:
            camera.stop()


if __name__ == "__main__":
    main()
