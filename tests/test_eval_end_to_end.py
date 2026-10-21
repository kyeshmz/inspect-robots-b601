"""End-to-end: a full eval() through the b601 embodiment and the real ArmSession on fake motors."""

import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import patch

from inspect_robots import Action, ActionChunk, Scene, Task
from inspect_robots import eval as rl_eval
from inspect_robots.approver import DeltaLimitApprover
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.scorer import success_at_end

from inspect_robots_b601 import B601Embodiment
from inspect_robots_b601.hardware.session import ArmSession
from inspect_robots_b601.hardware.state import StopLedger
from test_session import Dummy, FakeController


class RaiseJoint3(PolicyBase):
    def __init__(self, robot):
        self.info = PolicyInfo(name="raise-joint3", action_space=robot.info.action_space,
                               observation_space=robot.info.observation_space)

    def act(self, observation):
        target = observation.state["joint_pos"].copy()
        target[2] += 0.02
        return ActionChunk([Action(target)])


class EndToEndTests(unittest.TestCase):
    def test_eval_moves_then_walks_home_before_disabling(self):
        controller = FakeController()

        def session(profile, state_dir):
            return ArmSession(profile, state_dir, controller_factory=lambda channel: controller,
                              bus_factory=lambda profile: Dummy(), period=0.005, home_wait=0.0)

        with tempfile.TemporaryDirectory() as directory:
            state_dir = Path(directory) / "state"
            StopLedger(state_dir).acknowledge(details={"test": "fresh arm"})
            robot = B601Embodiment(state_dir=str(state_dir), allow_motion=True, activate_on_reset=True,
                                   operator_supported_arm=True)
            # Only the injected motors are simulated; label the run so.
            robot.info = replace(robot.info, is_simulated=True)
            task = Task(name="b601-fake-raise", max_steps=3, scorer=success_at_end(),
                        scenes=[Scene(id="raise", instruction="raise joint3 a little")])
            with patch("inspect_robots_b601.hardware.session.ArmSession", session), robot:
                log, = rl_eval(task, RaiseJoint3(robot), robot, log_dir=str(Path(directory) / "eval"),
                               approver=DeltaLimitApprover(robot.info.action_space))
            trial, = (json.loads(line) for line in
                      (Path(directory) / "logs/motorbridge-trials.jsonl").read_text().splitlines())

        self.assertEqual(log.status, "success")
        self.assertIsNone(trial["fault"])
        self.assertAlmostEqual(trial["final"][2], 0.06, delta=0.005)
        for motor in controller.motors:
            self.assertFalse(motor.enabled)
            self.assertAlmostEqual(motor.disabled_at, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
