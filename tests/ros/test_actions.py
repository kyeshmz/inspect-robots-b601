"""Exercise actual Humble actions on a dedicated fake-server domain, without CAN."""

import io
import os
import threading
import time
import unittest
import tempfile
from types import SimpleNamespace

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from inspect_robots import Action, Scene
from inspect_robots.errors import EmbodimentFault
from PIL import Image
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Header

from inspect_robots_b601 import B601Embodiment
from inspect_robots_b601._ros import RosSession
from inspect_robots_b601.embodiment import JOINTS, SIM_LOW, SIM_HIGH
from inspect_robots_b601.benchmarks import JointGoalPolicy, hold, joint1_nudge


class ActionTests(unittest.TestCase):
    def setUp(self):
        if os.environ.get("ROS_DOMAIN_ID") != "163" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
            self.fail("Fake-server tests require isolated localhost ROS domain 163")
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("inspect_b601_fake_controller", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.positions = [0., 1., 1., 0., 0., 0.]
        self.gripper_position = 0.2
        self.mode = "success"
        self.publishing = True
        self.cancelled = threading.Event()
        self.group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.node.create_publisher(JointState, "/rebotarm/joint_states", qos_profile_sensor_data)
        self.gripper_pub = self.node.create_publisher(JointMotorState, "/rebotarm/gripper/state", qos_profile_sensor_data)
        self.status_pub = self.node.create_publisher(ArmStatus, "/rebotarm/arm_status", qos)
        self.camera_pub = self.node.create_publisher(CompressedImage, "/inspect_test/camera/compressed", qos_profile_sensor_data)
        image = io.BytesIO()
        Image.new("RGB", (8, 6), color=(255, 0, 0)).save(image, format="JPEG")
        self.image = image.getvalue()
        self.status = ArmStatus(enabled=True, control_loop_active=True, state_machine="IDLE")
        self.status_pub.publish(self.status)
        self.timer = self.node.create_timer(0.02, self.publish, callback_group=self.group)
        self.arm = ActionServer(
            self.node, FollowJointTrajectory, "/rebotarm/follow_joint_trajectory",
            self.execute, goal_callback=self.accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group,
        )
        self.gripper = ActionServer(
            self.node, GripperCommand, "/rebotarm/gripper/command", self.execute_gripper,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group,
        )
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        self.robot = B601Embodiment(
            simulated=False, action_low=SIM_LOW, action_high=SIM_HIGH,
            timeout_s=1.0, state_timeout_s=0.3, control_hz=10,
            gripper_open=0.0, gripper_closed=1.0, gripper_max_effort=0.1,
            cameras="top:/inspect_test/camera/compressed:8x6",
        )
        self.addCleanup(self.cleanup)
        self.robot.reset(Scene(id="fake", instruction="exercise ROS protocol"))

    def cleanup(self):
        try:
            self.robot.close()
        finally:
            self.executor.shutdown(timeout_sec=5)
            self.thread.join(timeout=5)
            self.arm.destroy()
            self.gripper.destroy()
            self.node.destroy_node()
            self.context.try_shutdown()

    def publish(self):
        if self.publishing:
            header = Header(stamp=self.node.get_clock().now().to_msg())
            if self.mode == "old_stamp":
                header.stamp.sec -= 10
            self.state_pub.publish(JointState(header=header, name=list(JOINTS), position=self.positions))
            self.gripper_pub.publish(JointMotorState(header=header, position=self.gripper_position))
            self.camera_pub.publish(CompressedImage(header=header, format="jpeg", data=self.image))

    async def pause(self, seconds):
        future = Future(executor=self.executor)
        timer = self.node.create_timer(seconds, lambda: future.set_result(None), callback_group=self.group)
        try:
            await future
        finally:
            self.node.destroy_timer(timer)

    async def accept(self, _):
        if self.mode == "late":
            await self.pause(0.45)
        return GoalResponse.REJECT if self.mode == "reject" else GoalResponse.ACCEPT

    async def execute(self, handle):
        result = FollowJointTrajectory.Result()
        if self.mode in ("timeout", "stale", "late"):
            if self.mode == "stale":
                self.publishing = False
            deadline = time.monotonic() + 5
            while not handle.is_cancel_requested and time.monotonic() < deadline:
                await self.pause(0.01)
            if handle.is_cancel_requested:
                self.cancelled.set()
                handle.canceled()
            else:
                handle.abort()
            return result
        if self.mode == "abort":
            handle.abort()
            result.error_string = "injected controller fault"
        else:
            if self.mode != "false_success":
                self.positions = list(handle.request.trajectory.points[-1].positions)
            handle.succeed()
            if self.mode == "error_code":
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "injected tolerance fault"
        return result

    def execute_gripper(self, handle):
        self.gripper_position = handle.request.command.position
        handle.succeed()
        return GripperCommand.Result(position=self.gripper_position,
                                     reached_goal=self.mode != "gripper_stall",
                                     stalled=self.mode == "gripper_stall")

    def target(self):
        return Action(np.array([0.02, 1., 1., 0., 0., 0., 0.22]))

    def test_joint_and_gripper_actions_with_rgb_observation(self):
        result = self.robot.step(self.target())
        np.testing.assert_allclose(result.observation.state["joint_pos"], self.target().data)
        self.assertAlmostEqual(result.observation.state["gripper"][0], 0.78)
        self.assertEqual(result.observation.images["top"].shape, (6, 8, 3))
        self.assertGreater(result.observation.images["top"][0, 0, 0], 240)

    def test_rejection_halts(self):
        self.mode = "reject"
        with self.assertRaisesRegex(EmbodimentFault, "rejected"):
            self.robot.step(self.target())

    def test_abort_halts(self):
        self.mode = "abort"
        with self.assertRaisesRegex(EmbodimentFault, "injected controller fault"):
            self.robot.step(self.target())

    def test_error_result_halts(self):
        self.mode = "error_code"
        with self.assertRaisesRegex(EmbodimentFault, "injected tolerance fault"):
            self.robot.step(self.target())

    def test_timeout_cancels_goal(self):
        self.mode = "timeout"
        with self.assertRaisesRegex(EmbodimentFault, "action result"):
            self.robot.step(self.target())
        self.assertTrue(self.cancelled.wait(1))

    def test_stale_feedback_cancels_goal(self):
        self.mode = "stale"
        with self.assertRaisesRegex(EmbodimentFault, "Feedback stopped"):
            self.robot.step(self.target())
        self.assertTrue(self.cancelled.wait(1))

    def test_late_acceptance_is_cancelled(self):
        self.mode = "late"
        self.robot.timeout_s = 0.3
        with self.assertRaisesRegex(EmbodimentFault, "goal acceptance"):
            self.robot.step(self.target())
        self.assertTrue(self.cancelled.wait(1))

    def test_disabled_controller_refuses_commands(self):
        self.status.enabled = False
        self.status_pub.publish(self.status)
        with self.assertRaisesRegex(EmbodimentFault, "disabled"):
            self.robot.step(self.target())
        self.assertEqual(self.positions[0], 0.0)

    def test_controller_fault_reports_the_original_error_code(self):
        self.status.enabled = False
        self.status.error_codes = ["STOP_LATCHED", "tick: command exceeds 0.01 rad/s"]
        self.status_pub.publish(self.status)
        with self.assertRaisesRegex(EmbodimentFault, "tick: command exceeds 0.01 rad/s"):
            self.robot.step(self.target())
        self.assertEqual(self.positions[0], 0.0)

    def test_reset_reports_latched_fault_before_waiting_for_joint_frames(self):
        self.publishing = False
        self.status.enabled = False
        self.status.error_codes = ["STOP_LATCHED", "tick: goal completion deadline expired"]
        self.status_pub.publish(self.status)
        with self.assertRaisesRegex(EmbodimentFault, "goal completion deadline expired"):
            self.robot.reset(Scene(id="stopped", instruction="observe"))

    def test_gripper_stall_is_a_fault(self):
        self.mode = "gripper_stall"
        with self.assertRaisesRegex(EmbodimentFault, "Gripper did not reach"):
            self.robot.step(self.target())

    def test_old_publication_stamps_cannot_become_fresh_on_receipt(self):
        self.mode = "old_stamp"
        with self.assertRaisesRegex(EmbodimentFault, "fresh joint/camera feedback"):
            self.robot.step(self.target())
        self.assertEqual(self.positions[0], 0.0)

    def test_registered_benchmarks_use_actual_ros_feedback(self):
        from inspect_robots import eval
        from inspect_robots.approver import DeltaLimitApprover
        with tempfile.TemporaryDirectory() as directory:
            for factory in (hold, joint1_nudge):
                log, = eval(factory(), JointGoalPolicy(), self.robot, log_dir=directory,
                            approver=DeltaLimitApprover(self.robot.info.action_space, max_delta=0.002))
                self.assertEqual(log.status, "success")
                self.assertEqual(log.results.metrics["b601_measured_goal"], 1.0)

    def test_ros_controller_success_without_motion_scores_failure(self):
        from inspect_robots import eval
        self.mode = "false_success"
        with tempfile.TemporaryDirectory() as directory:
            log, = eval(joint1_nudge(), JointGoalPolicy(), self.robot, log_dir=directory)
            self.assertEqual(log.status, "success")
            self.assertEqual(log.results.metrics["b601_measured_goal"], 0.0)


class CancellationTests(unittest.TestCase):
    def session(self):
        session = object.__new__(RosSession)
        session.config = SimpleNamespace(timeout_s=1.0)
        session.pending = None
        session.result = Future()
        response = Future()
        response.set_result(SimpleNamespace(goals_canceling=[], return_code=3))
        session.handle = SimpleNamespace(accepted=True, cancel_goal_async=lambda: response)
        return session

    def test_already_aborted_goal_can_deliver_its_result_after_empty_cancel_reply(self):
        session = self.session()
        waits = []

        def wait(ready, timeout, label, **kwargs):
            waits.append(label)
            if label.startswith("terminal action result"):
                session.result.set_result(SimpleNamespace(status=6))
            self.assertTrue(ready())

        session._wait = wait
        session._cancel()
        self.assertEqual(len(waits), 2)
        self.assertIsNone(session.handle)
        self.assertIsNone(session.result)

    def test_empty_cancel_reply_without_terminal_result_is_still_unconfirmed(self):
        session = self.session()

        def wait(ready, timeout, label, **kwargs):
            if not ready():
                raise EmbodimentFault("Timed out waiting for " + label)

        session._wait = wait
        with self.assertRaisesRegex(EmbodimentFault, "terminal action result"):
            session._cancel()
        self.assertIsNotNone(session.handle)


if __name__ == "__main__":
    unittest.main()
