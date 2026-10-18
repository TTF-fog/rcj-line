import unittest

from line_vision.config import Config
from line_vision.navigator import Navigator
from line_vision.vision import LineVision
from tests import scenes

FPS = 30


class Run:
    def __init__(self):
        cfg = Config()
        self.vision = LineVision(cfg.vision)
        self.nav = Navigator(cfg.nav)
        self.t = 0.0
        self.commands = []

    def feed(self, images):
        for img in images:
            self.commands.append(self.nav.update(self.vision.process(img, self.t), self.t))
            self.t += 1 / FPS
        return self.commands[-1]

    def actions(self):
        return [c.action for c in self.commands]


def approach(markers=(), forward=True):
    return [scenes.junction(y, markers, forward=forward) for y in range(40, 360, 12)]


class NavigatorTest(unittest.TestCase):
    def test_follows_straight_line(self):
        run = Run()
        cmd = run.feed([scenes.straight()] * 5)
        self.assertEqual((cmd.state, cmd.action), ("FOLLOW_LINE", "FOLLOW"))
        self.assertAlmostEqual(cmd.turn, 0, delta=0.05)

    def test_steers_toward_offset_line(self):
        self.assertGreater(Run().feed([scenes.straight(x=480)] * 5).turn, 0.2)
        self.assertLess(Run().feed([scenes.corner("left")] * 5).turn, -0.3)

    def test_left_marker_turns_left_then_resumes(self):
        run = Run()
        run.feed([scenes.straight()] * 3 + approach(["left"]))
        self.assertIn("SLOW", run.actions())
        self.assertEqual(run.commands[-1].action, "TURN_LEFT")
        self.assertLess(run.commands[-1].turn, -0.5)
        cmd = run.feed([scenes.straight()] * 20)
        self.assertEqual(cmd.state, "FOLLOW_LINE")

    def test_right_marker_on_t_junction(self):
        run = Run()
        run.feed([scenes.straight()] * 3 + approach(["right"], forward=False))
        self.assertEqual(run.commands[-1].action, "TURN_RIGHT")

    def test_no_marker_goes_straight(self):
        run = Run()
        run.feed([scenes.straight()] * 3 + approach())
        self.assertEqual(run.commands[-1].action, "STRAIGHT")
        self.assertAlmostEqual(run.commands[-1].turn, 0, delta=0.1)

    def test_double_marker_u_turn(self):
        run = Run()
        run.feed([scenes.straight()] * 3 + approach(["left", "right"]))
        self.assertEqual(run.commands[-1].action, "U_TURN")
        run.feed([scenes.floor()] * 10)
        self.assertEqual(run.commands[-1].action, "U_TURN")
        self.assertEqual(run.feed([scenes.straight()] * 45).state, "FOLLOW_LINE")

    def test_gap_then_reacquire(self):
        run = Run()
        run.feed([scenes.straight()] * 3 + [scenes.line_end()] * 3 + [scenes.floor()] * 10)
        self.assertEqual(run.commands[-1].action, "GAP_FORWARD")
        self.assertEqual(run.commands[-1].turn, 0)
        self.assertEqual(run.feed([scenes.straight()] * 4).state, "FOLLOW_LINE")

    def test_gap_too_long_stops(self):
        run = Run()
        cmd = run.feed([scenes.straight()] * 3 + [scenes.floor()] * 90)
        self.assertEqual((cmd.state, cmd.action), ("LINE_LOST", "STOP"))

    def test_goal_stops_and_holds(self):
        run = Run()
        run.feed([scenes.straight()] * 3 + [scenes.goal()] * 5)
        self.assertEqual((run.commands[-1].state, run.commands[-1].action), ("GOAL", "STOP"))
        cmd = run.feed([scenes.straight()] * FPS * 7)
        self.assertEqual((cmd.state, cmd.action, cmd.note), ("GOAL", "STOP", "run finished"))

    def test_silver_entrance_stops(self):
        run = Run()
        cmd = run.feed([scenes.straight()] * 3 + [scenes.silver_entrance()] * 5)
        self.assertEqual((cmd.state, cmd.action), ("EVACUATION_ZONE", "STOP"))

    def test_stale_frame_stops(self):
        nav = Navigator(Config().nav)
        obs = LineVision(Config().vision).process(scenes.straight(), 0.0)
        self.assertEqual(nav.update(obs, 1.0).action, "STOP")


if __name__ == "__main__":
    unittest.main()
