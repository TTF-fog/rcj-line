import json
from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_PATH = Path(__file__).resolve().parent.parent / "robot.json"


@dataclass
class RobotConfig:
    # Falls back to the first /dev/ttyACM* if this one is missing.
    port: str = "/dev/ttyACM0"
    # Pi GPIO (BCM) pins of the run buttons; all behave the same. Press: start when idle, Lack of Progress
    # while running. Hold: start a new run (clears delivered victims and the clock). Enter in the terminal = press.
    buttons: tuple = ()
    hold_s: float = 2.0
    log_dir: str = "runs/logs"

    # Line following: navigator speed 1.0 and turn 1.0 map to these PWM values (firmware caps at 90).
    line_pwm: int = 90
    line_turn_pwm: int = 60

    model: str = "models/victims_320_ncnn_model"
    yolo_threads: int = 2
    ball_conf: float = 0.45
    # Balls sit on the floor; anything with its bottom edge above this image fraction is on a wall (LEDs, pictures).
    ball_min_bottom: float = 0.35
    # Fixed moves (gyro straight/back) all run at one speed so a single measured cm/s converts distance to time.
    move_speed: int = 70
    move_cm_per_s: float = 25.0
    turn_speed: int = 70

    enter_cm: float = 45.0
    collect_budget_s: float = 180.0
    want_silver: int = 2
    want_black: int = 1
    scan_step_deg: float = 30.0
    settle_s: float = 0.15
    look_frames: int = 3
    max_empty_scans: int = 2
    relocate_cm: float = 30.0

    approach_speed: int = 60
    approach_kp: float = 50.0
    approach_timeout_s: float = 8.0
    lost_frames: int = 10
    # Ball is in reach when its box bottom reaches this image fraction.
    grab_line: float = 0.92
    grab_creep_cm: float = 4.0

    # HSV in OpenCV units (H 0-180). Red wraps around 0.
    green_lo: tuple = (40, 70, 40)
    green_hi: tuple = (90, 255, 255)
    red_lo1: tuple = (0, 110, 60)
    red_hi1: tuple = (8, 255, 255)
    red_lo2: tuple = (172, 110, 60)
    red_hi2: tuple = (180, 255, 255)
    triangle_min_area: float = 0.01
    # Triangle is reached when its mask covers this fraction of the image.
    triangle_close_area: float = 0.30
    drop_back_cm: float = 12.0
    drop_s: float = 1.5
    drop_leave_cm: float = 20.0
    # Board commands for the storage stick: L = silver out, R = black out, N = neutral (no drop).
    drop_cmds: dict = field(default_factory=lambda: {"L": "DROP L", "R": "DROP R", "N": "DROP N"})

    # Distances in mm. The wall is followed on the right side.
    wall_mm: int = 100
    wall_kp: float = 0.3
    wall_speed: int = 55
    corner_mm: int = 120
    gap_mm: int = 300
    gap_reads: int = 3
    gap_center_cm: float = 10.0
    exit_probe_cm: float = 15.0
    exit_clear_cm: float = 10.0
    gap_skip_cm: float = 30.0
    leave_timeout_s: float = 90.0
    # Rows of the down-camera line mask that must be this full to count as the black exit strip.
    strip_row_fill: float = 0.45
    strip_min_rows: int = 8

    @classmethod
    def load(cls, path):
        cfg = cls()
        if Path(path).exists():
            for key, value in json.loads(Path(path).read_text()).items():
                if not hasattr(cfg, key):
                    raise KeyError(f"unknown setting {key} in {path}")
                setattr(cfg, key, tuple(value) if isinstance(value, list) else value)
        return cfg
