import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

DEFAULT_PATH = Path(__file__).resolve().parent.parent / "calibration.json"


@dataclass
class CameraConfig:
    model: str = "ov5647"
    width: int = 640
    height: int = 480
    fps: int = 30
    hflip: bool = False
    vflip: bool = False
    settle_s: float = 1.5
    # Long exposures blur the line at speed; trade exposure for gain instead.
    max_exposure_us: int = 10000
    max_analogue_gain: float = 16.0
    # Set by calibration: {"ExposureTime", "AnalogueGain", "ColourGains"}. None = auto-settle then lock at startup.
    fixed_controls: dict | None = None


@dataclass
class VisionConfig:
    proc_width: int = 320
    proc_height: int = 240
    # ROI as fractions of the processed frame. Image top is "ahead", image left is robot left.
    roi_top: float = 0.0
    roi_bottom: float = 1.0
    roi_left: float = 0.0
    roi_right: float = 1.0

    # Black = darker than black_rel x local floor brightness, or darker than black_abs.
    # background_kernel must be wider than the line in processed pixels.
    background_kernel: int = 121
    black_rel: float = 0.6
    black_rel_max_gray: int = 140
    black_abs: int = 50
    min_line_area_frac: float = 0.004

    bands: int = 8
    border_px: int = 3
    min_exit_px: int = 6
    entry_zone_frac: float = 0.35

    # HSV in OpenCV units (H 0-180).
    green_lo: tuple = (40, 70, 40)
    green_hi: tuple = (90, 255, 255)
    red_lo1: tuple = (0, 110, 60)
    red_hi1: tuple = (8, 255, 255)
    red_lo2: tuple = (172, 110, 60)
    red_hi2: tuple = (180, 255, 255)
    silver_max_sat: int = 45
    silver_specular: int = 250
    # Optional (lo, hi) band of brightness relative to local floor. Off by default: floor shading and the
    # robot's shadow fall in the same band. Enable only after tuning on real silver tape.
    silver_rel_band: tuple | None = None

    marker_min_area_frac: float = 0.003
    marker_max_area_frac: float = 0.12
    marker_min_fill: float = 0.55
    marker_max_aspect: float = 2.5

    strip_min_width_frac: float = 0.45
    strip_min_height_px: int = 8
    strip_min_aspect: float = 3.0
    strip_min_fill: float = 0.5


@dataclass
class NavConfig:
    base_speed: float = 0.6
    min_speed: float = 0.2
    approach_speed: float = 0.3
    turn_speed: float = 0.2
    gap_speed: float = 0.25
    kp_lateral: float = 0.6
    kp_angle: float = 0.9
    kd: float = 0.08
    slow_gain: float = 0.8

    junction_enter_frames: int = 3
    marker_min_frames: int = 2
    # Commit to a branch once the crossing line is this far down the ROI; tune to the camera-to-axle offset.
    commit_y_frac: float = 0.6
    # Drive straight this long after committing before turning; covers the camera-to-axle offset.
    commit_advance_s: float = 0.0
    turn_min_s: float = 0.4
    turn_max_s: float = 4.0
    turn_done_frames: int = 4
    junction_cooldown_s: float = 1.0
    # Without a gyro, this stops the U-turn from locking onto the crossing line at 90 degrees.
    u_turn_min_s: float = 1.2
    reacquire_frames: int = 3

    lost_frames: int = 3
    gap_max_s: float = 2.0
    search_lateral: float = 0.5

    strip_frames: int = 4
    goal_hold_s: float = 6.0
    max_frame_age_s: float = 0.2
    frame_timeout_s: float = 0.5


@dataclass
class Config:
    camera: CameraConfig = field(default_factory=CameraConfig)
    vision: VisionConfig = field(default_factory=VisionConfig)
    nav: NavConfig = field(default_factory=NavConfig)

    @classmethod
    def load(cls, path):
        cfg = cls()
        for section, values in json.loads(Path(path).read_text()).items():
            target = getattr(cfg, section)
            for key, value in values.items():
                if not hasattr(target, key):
                    raise KeyError(f"unknown setting {section}.{key} in {path}")
                setattr(target, key, tuple(value) if isinstance(value, list) else value)
        return cfg

    def save(self, path):
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")
