import os
import threading
import time

from .config import CameraConfig

os.environ.setdefault("LIBCAMERA_LOG_LEVELS", "*:ERROR")


class Camera:
    """Picamera2 capture thread that keeps only the newest frame."""

    def __init__(self, cfg: CameraConfig):
        from libcamera import Transform
        from picamera2 import Picamera2

        models = [info["Model"] for info in Picamera2.global_camera_info()]
        if cfg.model not in models:
            raise RuntimeError(f"camera {cfg.model!r} not found; connected: {models}")
        self.cfg = cfg
        self.cam = Picamera2(models.index(cfg.model))
        self.cam.configure(self.cam.create_video_configuration(
            main={"size": (cfg.width, cfg.height), "format": "RGB888"},
            buffer_count=3,
            transform=Transform(hflip=cfg.hflip, vflip=cfg.vflip),
            controls={"FrameRate": cfg.fps},
        ))
        self._cond = threading.Condition()
        self._latest = None
        self._seq = 0
        self._read_seq = 0
        self._running = False
        self._thread = None

    def start(self):
        """Start streaming with calibrated controls, or auto-settle and lock if there are none."""
        self.cam.start()
        locked = self.apply_fixed(self.cfg.fixed_controls) if self.cfg.fixed_controls else self.settle_and_lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return locked

    def apply_fixed(self, fixed):
        controls = {
            "AeEnable": False,
            "AwbEnable": False,
            "ExposureTime": int(fixed["ExposureTime"]),
            "AnalogueGain": float(fixed["AnalogueGain"]),
            "ColourGains": tuple(fixed["ColourGains"]),
        }
        self.cam.set_controls(controls)
        time.sleep(0.3)
        return controls

    def set_auto(self):
        self.cam.set_controls({"AeEnable": True, "AwbEnable": True})

    def settle_and_lock(self):
        """Freeze AE/AWB after settling on the floor so black/white classification doesn't drift."""
        time.sleep(self.cfg.settle_s)
        md = self.cam.capture_metadata()
        exposure, gain = md["ExposureTime"], md["AnalogueGain"]
        new_exposure = min(exposure, self.cfg.max_exposure_us)
        new_gain = min(exposure * gain / new_exposure, self.cfg.max_analogue_gain)
        return self.apply_fixed({"ExposureTime": new_exposure, "AnalogueGain": new_gain, "ColourGains": md["ColourGains"]})

    def _loop(self):
        while self._running:
            request = self.cam.capture_request()
            try:
                frame = request.make_array("main")
                timestamp = request.get_metadata()["SensorTimestamp"] / 1e9
            finally:
                request.release()
            with self._cond:
                self._latest = (frame, timestamp)
                self._seq += 1
                self._cond.notify_all()

    def read(self, timeout):
        """Next unseen (frame_bgr, monotonic_timestamp), or None if no frame arrives in time."""
        with self._cond:
            if not self._cond.wait_for(lambda: self._seq > self._read_seq, timeout):
                return None
            self._read_seq = self._seq
            return self._latest

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)
        self.cam.stop()
        self.cam.close()
