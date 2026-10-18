import unittest

from line_vision.config import VisionConfig
from line_vision.vision import LineVision
from tests import scenes


def observe(img):
    return LineVision(VisionConfig()).process(img, 0.0)


def sides(obs):
    return sorted(e.side for e in obs.exits)


class LineTest(unittest.TestCase):
    def test_centred_straight_line(self):
        obs = observe(scenes.straight())
        self.assertTrue(obs.line_found)
        self.assertEqual(sides(obs), ["forward"])
        self.assertAlmostEqual(obs.lateral_error, 0, delta=0.05)
        self.assertAlmostEqual(obs.target_angle_deg, 0, delta=3)

    def test_offset_line_reports_lateral_error(self):
        obs = observe(scenes.straight(x=480))
        self.assertGreater(obs.lateral_error, 0.4)
        self.assertGreater(obs.target_angle_deg, 0)

    def test_corners_target_their_side(self):
        for side, sign in (("left", -1), ("right", 1)):
            obs = observe(scenes.corner(side))
            self.assertEqual(sides(obs), [side])
            self.assertGreater(sign * obs.target_angle_deg, 30)
            self.assertFalse(obs.is_junction)

    def test_four_way_and_t_junctions(self):
        self.assertEqual(sides(observe(scenes.junction(200))), ["forward", "left", "right"])
        obs = observe(scenes.junction(200, forward=False))
        self.assertEqual(sides(obs), ["left", "right"])
        self.assertAlmostEqual(obs.junction_y_frac, 200 / scenes.H, delta=0.05)

    def test_line_ending_in_view(self):
        obs = observe(scenes.line_end())
        self.assertTrue(obs.line_end)
        self.assertEqual(obs.exits, [])

    def test_empty_floor(self):
        self.assertFalse(observe(scenes.floor()).line_found)


class MarkerTest(unittest.TestCase):
    def test_marker_before_crossing_gives_side(self):
        for side in ("left", "right"):
            obs = observe(scenes.junction(200, [side]))
            self.assertEqual(obs.valid_marker_sides(), {side})

    def test_double_marker(self):
        self.assertEqual(observe(scenes.junction(250, ["left", "right"])).valid_marker_sides(), {"left", "right"})

    def test_marker_after_crossing_is_ignored(self):
        obs = observe(scenes.junction(250, ["left"], before=False))
        self.assertEqual(len(obs.markers), 1)
        self.assertEqual(obs.valid_marker_sides(), set())

    def test_markers_are_not_line(self):
        obs = observe(scenes.junction(250, ["left", "right"]))
        self.assertLess(obs.line_width, scenes.LINE)


class StripTest(unittest.TestCase):
    def test_red_goal_strip(self):
        obs = observe(scenes.goal())
        self.assertIsNotNone(obs.red_strip)
        self.assertIsNone(obs.silver_strip)

    def test_silver_entrance_strip(self):
        obs = observe(scenes.silver_entrance())
        self.assertIsNotNone(obs.silver_strip)
        self.assertIsNone(obs.red_strip)

    def test_plain_crossing_is_not_a_strip(self):
        obs = observe(scenes.junction(200))
        self.assertIsNone(obs.red_strip)
        self.assertIsNone(obs.silver_strip)


if __name__ == "__main__":
    unittest.main()
