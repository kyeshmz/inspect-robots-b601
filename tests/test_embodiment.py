import sys
import unittest
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from inspect_robots import Action, Scene
from inspect_robots.conformance import assert_embodiment_conformant
from inspect_robots.errors import EmbodimentFault, SafetyAbort
from inspect_robots_b601 import B601Embodiment
from inspect_robots_b601.embodiment import JOINTS, SIM_LOW, SIM_HIGH

ROOT = Path(__file__).resolve().parents[1]


class Session:
    def __init__(self, config):
        self.position = np.array([0., 1., 1., 0., 0., 0.] + ([0.2] if config.gripper else []))
        self.moves = []
        self.closed = False

    def observe(self):
        return self.position.copy(), 1.0, {}, {}

    def move(self, target, duration):
        self.moves.append((target.copy(), duration))
        self.position = target.copy()

    def close(self):
        self.closed = True


class EmbodimentTests(unittest.TestCase):
    def setUp(self):
        self.patch = patch.dict(sys.modules, {"inspect_robots_b601._ros": SimpleNamespace(RosSession=Session)})
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.robot = B601Embodiment()
        self.addCleanup(self.robot.close)

    def test_network_free_spaces_conform(self):
        self.assertIsNone(self.robot._session)
        assert_embodiment_conformant(self.robot.info)
        robot = B601Embodiment(simulated=False, action_low=SIM_LOW, action_high=SIM_HIGH,
                              gripper_open=-0.5, gripper_closed=0.5, gripper_max_effort=0.1)
        assert_embodiment_conformant(robot.info)

    def test_sim_bounds_match_pinned_urdf(self):
        limits = json.loads((ROOT / "tests/fixtures/mock-joint-limits.json").read_text())
        self.assertEqual(tuple(limits["joints"]), JOINTS)
        self.assertEqual(tuple(limits["lower"]), SIM_LOW)
        self.assertEqual(tuple(limits["upper"]), SIM_HIGH)

    def test_hardware_requires_explicit_bounds_and_gripper_calibration(self):
        for kwargs in (
            {"simulated": False}, {"simulated": "false"}, {"control_hz": float("nan")},
            {"action_low": [0]}, {"action_low": SIM_HIGH},
            {"gripper_open": 1.0},
            {"simulated": False, "action_low": SIM_LOW, "action_high": SIM_HIGH, "gripper_open": 1.0},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                B601Embodiment(**kwargs)

    def test_reset_only_observes_and_step_is_measured_speed_bounded(self):
        self.robot.reset(Scene(id="test", instruction="hold"))
        session = self.robot._session
        self.assertEqual(session.moves, [])
        target = session.position.copy()
        target[0] = 0.04
        with patch("inspect_robots_b601.embodiment.time.sleep"):
            result = self.robot.step(Action(target))
        np.testing.assert_array_equal(result.observation.state["joint_pos"], target)
        self.assertGreaterEqual(session.moves[0][1], 0.5)

    def test_invalid_commands_never_reach_transport_and_latch(self):
        for target in ([0] * 5, [float("nan")] * 6, [0, -0.01, 1, 0, 0, 0], [0.06, 1, 1, 0, 0, 0]):
            robot = B601Embodiment()
            robot.reset(Scene(id="test", instruction="hold"))
            with self.assertRaises(SafetyAbort):
                robot.step(Action(np.array(target)))
            self.assertEqual(robot._session.moves, [])
            with self.assertRaises(EmbodimentFault):
                robot.reset(Scene(id="next", instruction="hold"))
            robot.close()

    def test_false_success_is_detected_from_measurements(self):
        self.robot.reset(Scene(id="test", instruction="hold"))
        with patch.object(self.robot._session, "move"), patch("inspect_robots_b601.embodiment.time.sleep"):
            with self.assertRaisesRegex(EmbodimentFault, "tolerance"):
                self.robot.step(Action(np.array([0.04, 1, 1, 0, 0, 0])))

    def test_gripper_uses_motor_radians_and_explicit_polarity(self):
        robot = B601Embodiment(simulated=False, action_low=SIM_LOW, action_high=SIM_HIGH,
                              gripper_open=-0.5, gripper_closed=0.5, gripper_max_effort=0.1)
        try:
            obs = robot.reset(Scene(id="test", instruction="hold"))
            self.assertAlmostEqual(obs.state["gripper"][0], 0.3)
            self.assertAlmostEqual(obs.state["joint_pos"][-1], 0.2)
        finally:
            robot.close()


if __name__ == "__main__":
    unittest.main()
