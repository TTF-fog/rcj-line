import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

from line_vision import calibrate
from line_vision.config import Config, VisionConfig
from line_vision.vision import LineVision
from tests import scenes

V = VisionConfig()


class OrientationTest(unittest.TestCase):
    def test_line_on_robot_left(self):
        self.assertFalse(calibrate.needs_hflip([scenes.straight(x=160)] * 5, V)[0])
        self.assertTrue(calibrate.needs_hflip([scenes.straight(x=480)] * 5, V)[0])

    def test_push_forward_moves_line_down(self):
        moving_down = [scenes.hline(scenes.floor(), y) for y in range(150, 330, 6)]
        self.assertFalse(calibrate.needs_vflip(moving_down, V)[0])
        self.assertTrue(calibrate.needs_vflip(moving_down[::-1], V)[0])

    def test_rotated_camera_is_rejected(self):
        with self.assertRaises(calibrate.CalibrationError):
            calibrate.needs_vflip([scenes.straight(x) for x in range(200, 400, 10)], V)


class MeasurementTest(unittest.TestCase):
    def test_geometry_from_tape_width(self):
        g = calibrate.measure_geometry([scenes.straight()] * 5, 15.0, V)
        expected_ppm = scenes.LINE / 2 / 15.0
        self.assertAlmostEqual(g["ppm_near"], expected_ppm, delta=0.1)
        self.assertAlmostEqual(g["fov_w_near_mm"], 320 / expected_ppm, delta=15)

    def test_narrow_view_warns_about_markers(self):
        g = calibrate.measure_geometry([scenes.straight()] * 5, 2.0, V)
        self.assertTrue(any("green marker" in line for line in calibrate.geometry_report(g)))

    def test_calibrated_settings_still_detect_everything(self):
        g = calibrate.measure_geometry([scenes.straight()] * 5, 15.0, V)
        v = replace(V, **calibrate.geometry_settings(g))
        black, _, _ = calibrate.measure_black([scenes.straight(), scenes.corner("left")] * 3, v)
        v = replace(v, **black)
        green, _ = calibrate.measure_colour([scenes.junction(200, ["left"])] * 3, v, "green")
        red, _ = calibrate.measure_colour([scenes.goal()] * 3, v, "red")
        v = replace(v, **green, **red)

        obs = LineVision(v).process(scenes.junction(200, ["left"]), 0)
        self.assertEqual(obs.valid_marker_sides(), {"left"})
        self.assertEqual(sorted(e.side for e in obs.exits), ["forward", "left", "right"])
        self.assertIsNotNone(LineVision(v).process(scenes.goal(), 0).red_strip)
        self.assertEqual(LineVision(v).process(scenes.straight(), 0).markers, [])

    def test_colour_step_rejects_wrong_colour(self):
        with self.assertRaises(calibrate.CalibrationError):
            calibrate.measure_colour([scenes.goal()] * 3, V, "green")

    def test_specular_silver(self):
        settings, _ = calibrate.measure_silver([scenes.silver_entrance()] * 3, V, 0.7)
        self.assertIsNone(settings["silver_rel_band"])

    def test_false_detection_check_is_clean_on_plain_line(self):
        lines = calibrate.false_detection_check([scenes.straight()] * 10, Config())
        self.assertFalse(any("too many" in line for line in lines))


class ConfigFileTest(unittest.TestCase):
    def test_round_trip(self):
        cfg = Config()
        cfg.vision.green_lo = (50, 60, 70)
        cfg.camera.fixed_controls = {"ExposureTime": 5000, "AnalogueGain": 2.0, "ColourGains": [1.5, 1.2]}
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "c.json"
            cfg.save(path)
            self.assertEqual(Config.load(path), cfg)


if __name__ == "__main__":
    unittest.main()
