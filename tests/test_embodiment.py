"""Hardware-free tests for the six-joint Inspect adapter."""

import configparser
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
from inspect_robots import Action, Scene
from inspect_robots.conformance import assert_embodiment_conformant
from inspect_robots.errors import EmbodimentFault, SafetyAbort

from inspect_robots_b601.hardware.session import MoveError
from inspect_robots_b601.cli import AGENT_CONFIG
from inspect_robots_b601.embodiment import DEFAULT_PROFILE, B601Embodiment



class FakeCamera:
    width, height = 640, 480

    def __init__(self):
        self.started = self.closed = False

    def start(self):
        self.started = True

    def read(self):
        return np.zeros((self.height, self.width, 3), dtype=np.uint8)

    def close(self):
        self.closed = True


class DepthCamera(FakeCamera):
    depth = True

    def __init__(self, distance=0.15):
        super().__init__()
        self.distance = distance

    def read_with_depth(self):
        depth = np.zeros((self.height, self.width), dtype=np.float64)
        depth[250:310, 290:360] = self.distance  # a surface just past the jaw tips
        return self.read(), depth


class FakeSession:
    follow = 1.0  # fraction of each commanded change the joints actually make

    def __init__(self, *args, **kwargs):
        self.positions = [0.0] * 7
        self.opened = self.activated = self.closed = False
        self.moves = []
        self.paths = []
        self.stopped = None
        self.homes = 0
        self._home = tuple(self.positions)

    def open(self):
        self.opened = True

    def activate(self):
        self.activated = True

    def observe(self):
        return tuple(self.positions)

    def move(self, target, duration):
        self.moves.append((list(target), duration))
        self.positions = [p + (t - p) * self.follow for p, t in zip(self.positions, target)]
        return tuple(self.positions)

    def go_to(self, pose, *, timeout=None, recovery=False):
        self.paths.append([round(float(v), 6) for v in pose])
        self.positions = [p + (t - p) * self.follow for p, t in zip(self.positions, pose)]
        if self.follow < 0.5:
            raise RuntimeError("joint3 stalled")
        return tuple(self.positions)

    def go_home(self):
        self.homes += 1
        self.positions = list(self._home)
        return tuple(self.positions)

    def stop(self, reason):
        self.stopped = reason

    def close(self):
        self.closed = True


class SaggingSession(FakeSession):
    follow = 0.6


class BlockedSession(FakeSession):
    follow = 0.0


class EmbodimentTests(unittest.TestCase):
    def setUp(self):
        self.profile = DEFAULT_PROFILE
        self.use_session(FakeSession)

    def use_session(self, session):
        self.patch = patch("inspect_robots_b601.hardware.session.ArmSession", session)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def calibrate_wrist_camera(self):
        """The shipped profile has no unit calibration; jaw_range needs one."""
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        profile = json.loads(self.profile.read_text())
        profile["wrist_camera"] = {"jaw_window_px": [300, 272, 345, 296], "jaw_tip_depth_m": 0.092}
        self.profile = Path(directory.name) / "calibrated.json"
        self.profile.write_text(json.dumps(profile))

    def robot(self, **kwargs):
        # Trial logs and images land beside the state directory: keep them out of the checkout.
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        kwargs.setdefault("state_dir", str(Path(directory.name) / "state"))
        kwargs.setdefault("max_step", 0.02)
        kwargs.setdefault("step_size", 0.01)
        robot = B601Embodiment(
            profile=str(self.profile), allow_motion=True, activate_on_reset=True,
            operator_supported_arm=True, control_hz=0.5, **kwargs)
        self.addCleanup(robot.close)
        return robot

    def test_spaces_conform_and_session_is_lazy(self):
        robot = self.robot()
        self.assertIsNone(robot._session)
        self.assertEqual(robot.info.action_space.shape, (7,))
        self.assertEqual(tuple(robot.info.action_space.semantics.dim_labels),
                         tuple(f"joint{i}" for i in range(1, 7)) + ("gripper",))
        assert_embodiment_conformant(robot.info)
        self.assertNotIn("camera is wrist-mounted", robot.info.docs)

    def test_reset_activates_and_step_moves_all_joints(self):
        robot = self.robot()
        observation = robot.reset(Scene(id="test", instruction="move"))
        self.assertEqual(observation.state["joint_pos"].shape, (7,))
        result = robot.step(Action(np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])))
        np.testing.assert_array_equal(result.observation.state["joint_pos"],
                                      [0.01, 0, 0, 0, 0, 0, 0])
        self.assertEqual(robot._session.paths, [[0.01, 0, 0, 0, 0, 0, 0]])
        self.assertEqual(result.info, {})
        np.testing.assert_allclose(observation.state["tip"], [0.302, 0.0, 0.218, 0.0], atol=1e-3)

    def test_tip_reports_height_so_a_rising_move_is_visible(self):
        robot = self.robot(max_step=0.5, step_size=0.1)
        robot.reset(Scene(id="tip", instruction="lower"))
        # The failed tape run's ready pose and the pose where it closed: higher, not lower.
        robot._session.positions = [0.0004, 0.3085, 0.8065, -0.5268, 0.0, 0.0, 0.8]
        ready = robot._observe().state["tip"]
        robot._session.positions = [0.0585, 0.6911, 1.0724, -0.4946, 0.0, 0.0, 0.8]
        closed = robot._observe().state["tip"]
        self.assertAlmostEqual(ready[2], 0.381, places=3)
        self.assertAlmostEqual(closed[2], 0.415, places=3)
        self.assertIn("state[tip]", robot.info.docs)

    def test_oversize_and_out_of_range_targets_are_shortened_not_refused(self):
        robot = self.robot()
        robot.reset(Scene(id="test", instruction="move"))
        result = robot.step(Action(np.array([0.05, -1.0, 0.0, 0.0, 0.0, 0.0, 0.8])))
        np.testing.assert_allclose(robot._session.paths[-1], [0.02, -0.01, 0, 0, 0, 0, 0.8])
        self.assertEqual(result.info["limited"], ["joint1 +0.050 -> +0.020", "joint2 -1.000 -> -0.010"])
        robot.step(Action(np.array([0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8])))
        self.assertIsNone(robot._session.stopped)

    def test_declared_step_size_reaches_the_agent_and_limiter(self):
        robot = self.robot()
        self.assertEqual(robot.info.action_space.semantics.max_step, (0.01,) * 6 + (0.3,))
        with self.assertRaises(ValueError):
            self.robot(step_size=0.03)  # larger than the 0.02 rad clamp

    def stall_joint(self, robot, index):
        def go_to(target, timeout=None):
            robot._session.paths.append([round(float(v), 6) for v in target])
            stuck = robot._session.positions[index]
            robot._session.positions = list(target)
            robot._session.positions[index] = stuck
            raise MoveError(f"joint{index + 1} stalled")
        robot._session.go_to = go_to

    def test_stall_holds_marks_the_joint_and_keeps_the_trial_going(self):
        robot = self.robot(max_step=0.1)
        robot.reset(Scene(id="stall", instruction="pan"))
        self.stall_joint(robot, 0)
        result = robot.step(Action(np.array([0.06, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0])))
        self.assertEqual(result.info["stalled"], "joint1 stalled")
        np.testing.assert_array_equal(result.observation.state["blocked"], [1, 0, 0, 0, 0, 0])
        self.assertIsNone(robot._session.stopped)
        self.assertFalse(robot._faulted)
        del robot._session.go_to  # the arm moves normally again
        # Pushing further into the stop is held at the stop; other joints still move.
        result = robot.step(Action(np.array([0.08, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0])))
        self.assertEqual(robot._session.paths[-1], [0.0, 0.02, 0, 0, 0, 0, 0])
        self.assertIn("joint1 +0.080 -> +0.000", result.info["limited"])
        np.testing.assert_array_equal(result.observation.state["blocked"], [1, 0, 0, 0, 0, 0])
        # Backing off the other way releases the block.
        result = robot.step(Action(np.array([-0.02, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0])))
        self.assertEqual(robot._session.paths[-1], [-0.02, 0.02, 0, 0, 0, 0, 0])
        np.testing.assert_array_equal(result.observation.state["blocked"], [0] * 6)

    def test_each_trial_starts_from_home_and_is_recorded(self):
        robot = self.robot(ready="joint3=0.3")
        robot.reset(Scene(id="one", instruction="first"))
        session = robot._session
        robot.step(Action(np.array([0.0, 0.0, 0.32, 0.0, 0.0, 0.0, 0.0])))
        robot.reset(Scene(id="two", instruction="second"))
        ready = [0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0]
        self.assertEqual(session.homes, 1)
        self.assertEqual(session.paths, [ready, [0.0, 0.0, 0.32, 0.0, 0.0, 0.0, 0.0], ready])
        robot.close()
        records = [json.loads(line) for line in robot.trial_log.read_text().splitlines()]
        self.assertEqual([record["instruction"] for record in records], ["first", "second"])
        self.assertEqual([record["start"] for record in records], [[0.0] * 7, [0.0] * 7])

    def test_operator_session_confirms_each_trial_and_carries_status(self):
        events = []

        class Console:
            def gate(self, prompt, *, hint=None):
                events.append(("gate", prompt))

            def write_line(self, text):
                events.append(("line", text))

        self.use_session(type("OrderedSession", (FakeSession,), {
            "activate": lambda self: events.append(("activate",))}))
        robot = self.robot(hold_seconds=1.0)
        robot.connect_operator_session(Console())
        robot.reset(Scene(id="one", instruction="first"))
        robot.reset(Scene(id="two", instruction="second"))
        self.assertEqual([event[0] for event in events], ["gate", "activate", "gate"])
        self.assertIn("enable the motors", events[0][1])
        self.assertIn("start the next trial", events[2][1])
        self.assertEqual(robot._session.notify, robot._say)
        robot._session._enabled = True
        with patch("inspect_robots_b601.embodiment.time.sleep"):
            robot.close()
        self.assertIn(("line", "holding final pose for 1 s"), events)

    def test_read_only_runs_never_gate_or_move(self):
        events = []

        class Console:
            def gate(self, prompt, *, hint=None):
                events.append(prompt)

            def write_line(self, text):
                events.append(text)

        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        robot = B601Embodiment(state_dir=str(Path(directory.name) / "state"))
        self.addCleanup(robot.close)
        robot.connect_operator_session(Console())
        robot.reset(Scene(id="one", instruction="look"))
        robot.reset(Scene(id="two", instruction="look again"))
        self.assertEqual(events, [])
        self.assertEqual((robot._session.activated, robot._session.homes, robot._session.paths), (False, 0, []))

    def test_new_trial_clears_blocks(self):
        robot = self.robot(max_step=0.1)
        robot.reset(Scene(id="one", instruction="lower"))
        self.stall_joint(robot, 0)
        robot.step(Action(np.array([0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])))
        self.assertEqual(robot._blocked[0], 1.0)
        observation = robot.reset(Scene(id="two", instruction="lower"))
        np.testing.assert_array_equal(observation.state["blocked"], [0] * 6)

    def test_non_finite_targets_never_reach_session(self):
        robot = self.robot()
        robot.reset(Scene(id="test", instruction="move"))
        with self.assertRaises(SafetyAbort):
            robot.step(Action(np.array([np.nan, 0, 0, 0, 0, 0, 0])))
        self.assertEqual(robot._session.paths, [])
        with self.assertRaises(EmbodimentFault):
            robot.reset(Scene(id="next", instruction="move"))

    def test_default_config_and_reset_make_no_startup_move(self):
        config = configparser.ConfigParser()
        config.read(AGENT_CONFIG)
        self.assertNotIn("startup", config["embodiment.args"])
        robot = self.robot()
        robot.reset(Scene(id="test", instruction="move"))
        self.assertEqual(robot._session.moves, [])

    def test_startup_rejects_unknown_zero_or_large_deltas(self):
        for startup in ("joint9=0.05", "joint2", "joint2=0", "joint2=0.11", "joint2=nan"):
            with self.subTest(startup=startup), self.assertRaises(ValueError):
                self.robot(startup=startup)

    def test_honoured_startup_move_passes(self):
        robot = self.robot(startup="joint2=0.05")
        observation = robot.reset(Scene(id="test", instruction="move"))
        self.assertEqual(robot._session.moves, [([0, 0.05, 0, 0, 0, 0, 0], 2.0)])
        self.assertAlmostEqual(observation.state["joint_pos"][1], 0.05)
        self.assertIsNone(robot._session.stopped)

    def test_startup_move_tolerates_partial_tracking(self):
        self.use_session(SaggingSession)
        robot = self.robot(startup="joint2=0.05,joint3=-0.05")
        observation = robot.reset(Scene(id="test", instruction="move"))
        np.testing.assert_allclose(observation.state["joint_pos"][1:3], [0.03, -0.03])
        self.assertIsNone(robot._session.stopped)

    def test_stalled_startup_move_faults_and_releases_hold(self):
        self.use_session(BlockedSession)
        robot = self.robot(startup="joint2=0.05")
        with self.assertRaises(EmbodimentFault) as caught:
            robot.reset(Scene(id="test", instruction="move"))
        message = str(caught.exception)
        for text in ("joint2", "requested +0.050 rad", "measured +0.000 rad"):
            self.assertIn(text, message)
        self.assertIn("stalled", robot._session.stopped)
        self.assertIn("joint2", robot._trial_fault)
        with self.assertRaises(EmbodimentFault):
            robot.reset(Scene(id="next", instruction="move"))

    def test_ready_stages_walk_from_the_activation_pose(self):
        robot = self.robot(ready="joint3=0.3; joint2=0.2,joint3=0.6")
        observation = robot.reset(Scene(id="ready", instruction="look"))
        self.assertEqual(robot._session.paths, [[0, 0, 0.3, 0, 0, 0, 0], [0, 0.2, 0.6, 0, 0, 0, 0]])
        self.assertAlmostEqual(observation.state["joint_pos"][2], 0.6)
        self.assertIn("ready pose", robot.info.docs)

    def test_ready_rejects_unknown_or_large_offsets(self):
        for ready in ("joint9=0.3", "joint3", "joint3=1.3", "joint3=inf"):
            with self.subTest(ready=ready), self.assertRaises(ValueError):
                self.robot(ready=ready)

    def test_failed_ready_pose_stops_and_faults(self):
        self.use_session(BlockedSession)
        robot = self.robot(ready="joint3=0.3")
        with self.assertRaises(EmbodimentFault) as caught:
            robot.reset(Scene(id="ready", instruction="look"))
        self.assertIn("joint3 stalled", str(caught.exception))
        self.assertIn("ready pose failed", robot._session.stopped)

    def test_step_failure_stops_the_session(self):
        robot = self.robot()
        robot.reset(Scene(id="test", instruction="move"))
        robot._session.go_to = lambda target, timeout=None: (_ for _ in ()).throw(
            RuntimeError("joint2 stalled at +0.0000 rad, path target +0.0100 rad"))
        with self.assertRaises(RuntimeError):
            robot.step(Action(np.array([0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0])))
        self.assertEqual(robot._session.stopped, "Inspect B601 step failed")
        self.assertIn("joint2", robot._trial_fault)

    def test_camera_starts_before_the_motors_are_enabled(self):
        order = []
        camera = FakeCamera()
        camera.start = lambda: order.append("camera")
        session = type("OrderedSession", (FakeSession,), {
            "activate": lambda self: order.append("activate")})
        self.use_session(session)
        with patch("inspect_robots_b601.embodiment.parse_camera", return_value=camera):
            robot = self.robot(camera="realsense")
            robot.reset(Scene(id="camera", instruction="look"))
        self.assertEqual(order, ["camera", "activate"])

    def test_depth_camera_adds_a_depth_image_and_jaw_range(self):
        self.calibrate_wrist_camera()
        camera = DepthCamera(distance=0.15)
        with patch("inspect_robots_b601.embodiment.parse_camera", return_value=camera):
            robot = self.robot(camera="realsense+depth")
            assert_embodiment_conformant(robot.info)
            self.assertIn("state[jaw_range]", robot.info.docs)
            observation = robot.reset(Scene(id="depth", instruction="look"))
        self.assertEqual(observation.extra["front_depth"].shape, (480, 640))
        np.testing.assert_allclose(observation.state["jaw_range"], [0.15, 0.058])

    def test_uncalibrated_depth_camera_has_depth_but_no_jaw_range(self):
        with patch("inspect_robots_b601.embodiment.parse_camera", return_value=DepthCamera(distance=0.15)):
            robot = self.robot(camera="realsense+depth")
            assert_embodiment_conformant(robot.info)
            observation = robot.reset(Scene(id="depth", instruction="look"))
        self.assertEqual(observation.extra["front_depth"].shape, (480, 640))
        self.assertNotIn("jaw_range", observation.state)
        self.assertNotIn("jaw_range", robot.info.docs)

    def test_jaw_range_without_a_reading_is_minus_one(self):
        self.calibrate_wrist_camera()
        camera = DepthCamera(distance=0.0)
        with patch("inspect_robots_b601.embodiment.parse_camera", return_value=camera):
            robot = self.robot(camera="realsense+depth")
            observation = robot.reset(Scene(id="depth", instruction="look"))
        np.testing.assert_array_equal(observation.state["jaw_range"], [-1.0, -1.0])

    def test_colour_only_camera_has_no_depth_fields(self):
        with patch("inspect_robots_b601.embodiment.parse_camera", return_value=FakeCamera()):
            robot = self.robot(camera="realsense")
            observation = robot.reset(Scene(id="colour", instruction="look"))
        self.assertNotIn("jaw_range", observation.state)
        self.assertNotIn("front_depth", observation.extra)
        self.assertNotIn("jaw_range", robot.info.docs)

    def test_camera_strings_select_depth(self):
        from inspect_robots_b601.hardware.camera import parse_camera
        self.assertTrue(parse_camera("realsense+depth:640x480@30").depth)
        self.assertFalse(parse_camera("realsense:640x480@30").depth)
        self.assertIsNone(parse_camera(""))
        with self.assertRaises(ValueError):
            parse_camera("kinect")

    def test_camera_images_are_in_observations(self):
        camera = FakeCamera()
        with patch("inspect_robots_b601.embodiment.parse_camera", return_value=camera):
            robot = self.robot(camera="realsense")
            self.assertIn("camera is wrist-mounted", robot.info.docs)
            observation = robot.reset(Scene(id="camera", instruction="look"))
            self.assertEqual(observation.images["front"].shape, (480, 640, 3))
            self.assertIn("front", observation.image_times)
            robot.close()
        self.assertTrue(camera.started)
        self.assertTrue(camera.closed)


if __name__ == "__main__":
    unittest.main()
