import hashlib
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from inspect_robots import Action, Scene
from inspect_robots.conformance import assert_embodiment_conformant
from inspect_robots.errors import EmbodimentFault, SafetyAbort

from inspect_robots_b601.motorbridge import MotorbridgeB601Embodiment

ROOT = Path(__file__).resolve().parents[1]


class FakeSession:
    def __init__(self, *args):
        self.positions = [0.0] * 6
        self.opened = self.activated = self.closed = False
        self.moves = []
        self.stopped = False

    def open(self):
        self.opened = True

    def activate(self):
        self.activated = True

    def observe(self):
        return tuple(self.positions), time.monotonic()

    def move(self, target, duration):
        self.moves.append((list(target), duration))
        self.positions = list(target)

    def close(self):
        self.closed = True

    def stop(self, reason):
        self.stopped = True


class MotorbridgeTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.report = Path(self.directory.name) / "trial.json"
        self.profile = ROOT / "tests/fixtures/joint1.synthetic.json"
        self.report.write_text(
            json.dumps(
                {
                    "schema": "b601-direct-joint1-test-v1",
                    "success": True,
                    "cleanup": [],
                    "profile_sha256": hashlib.sha256(self.profile.read_bytes()).hexdigest(),
                    "recorded_at_unix_s": time.time(),
                    "requested_delta_rad": 0.005,
                    "duration_s": 2.0,
                    "disable_acknowledged": True,
                    "measured_delta_rad": 0.005,
                }
            )
        )

    def robot(self, **kwargs):
        robot = MotorbridgeB601Embodiment(profile=str(self.profile), **kwargs)
        self.addCleanup(robot.close)
        return robot

    def activated(self):
        return self.robot(
            allow_motion=True,
            activate_on_reset=True,
            operator_supported_arm=True,
            tracking_report=str(self.report),
        )

    def test_constructor_is_hardware_free_and_one_axis_is_honest(self):
        robot = self.robot()
        self.assertIsNone(robot._session)
        self.assertEqual(robot.info.action_space.shape, (1,))
        self.assertEqual(tuple(robot.info.action_space.semantics.dim_labels), ("joint1",))
        assert_embodiment_conformant(robot.info)

    def test_profile_must_be_supplied_explicitly(self):
        with self.assertRaises(TypeError):
            MotorbridgeB601Embodiment()

    def test_activation_needs_all_explicit_flags_and_tracking_report(self):
        for options in (
            {"activate_on_reset": True},
            {"allow_motion": True, "activate_on_reset": True},
            {"allow_motion": True, "activate_on_reset": True, "operator_supported_arm": True},
            {"allow_motion": "true"},
        ):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.robot(**options)

    @patch("b601_hardware.direct.DirectSession", FakeSession)
    def test_readonly_reset_never_activates(self):
        robot = self.robot()
        observation = robot.reset(Scene(id="observe", instruction="observe"))
        self.assertFalse(robot._session.activated)
        self.assertEqual(observation.state["arm_joint_pos"].shape, (6,))
        with self.assertRaises(EmbodimentFault):
            robot.step(Action(np.array([0.001])))

    @patch("b601_hardware.direct.DirectSession", FakeSession)
    def test_step_maps_only_joint1_and_waits_for_measured_convergence(self):
        robot = self.activated()
        robot.reset(Scene(id="nudge", instruction="small nudge"))
        result = robot.step(Action(np.array([0.001])))
        target, duration = robot._session.moves[0]
        self.assertEqual(target, [0.001, 0, 0, 0, 0, 0])
        self.assertGreaterEqual(duration, 0.5)
        self.assertAlmostEqual(result.observation.state["joint_pos"][0], 0.001)

    @patch("b601_hardware.direct.DirectSession", FakeSession)
    def test_invalid_actions_never_reach_transport(self):
        for target in ([0.003], [float("nan")], [0, 0]):
            robot = self.activated()
            robot.reset(Scene(id="invalid", instruction="test"))
            with self.assertRaises(SafetyAbort):
                robot.step(Action(np.array(target)))
            self.assertEqual(robot._session.moves, [])
            self.assertTrue(robot._session.stopped)

    @patch("b601_hardware.direct.DirectSession", FakeSession)
    def test_failed_or_old_report_blocks_can_open(self):
        robot = self.activated()
        report = json.loads(self.report.read_text())
        report["success"] = False
        self.report.write_text(json.dumps(report))
        with self.assertRaises(EmbodimentFault):
            robot.reset(Scene(id="blocked", instruction="blocked"))
        self.assertIsNone(robot._session)


if __name__ == "__main__":
    unittest.main()
