"""Own ROS subscriptions and action lifecycles without a background spin thread."""

from __future__ import annotations

import io
import time
import xml.etree.ElementTree as ET
from collections.abc import Callable
from typing import TYPE_CHECKING, Any, cast

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from inspect_robots.errors import EmbodimentFault
from PIL import Image
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from ._types import JointVector, RGBImage, RosObservation
from .embodiment import JOINTS

if TYPE_CHECKING:
    from .embodiment import B601Embodiment


class RosSession:
    def __init__(self, config: B601Embodiment) -> None:
        self.config = config
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("inspect_b601", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.samples: dict[str, tuple[float, JointVector | float | bytes]] = {}
        self.status: Any = None
        self.fault: str | None = None
        self.pending: Any = None
        self.handle: Any = None
        self.result: Any = None
        self.clients: list[tuple[Any, str]] = []
        self.closed = False
        try:
            namespace = "/rebotarm_controller" if config.simulated else "/rebotarm"
            state_topic = "/joint_states" if config.simulated else "/rebotarm/joint_states"
            self.node.create_subscription(
                JointState, state_topic, self._joints, qos_profile_sensor_data
            )
            self.arm = ActionClient(
                self.node, FollowJointTrajectory, namespace + "/follow_joint_trajectory"
            )
            self.clients.append((self.arm, namespace + "/follow_joint_trajectory"))
            if not config.simulated:
                from rebotarm_msgs.msg import ArmStatus, JointMotorState

                status_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self.node.create_subscription(
                    ArmStatus, "/rebotarm/arm_status", self._status, status_qos
                )
                if config.gripper:
                    self.node.create_subscription(
                        JointMotorState,
                        "/rebotarm/gripper/state",
                        self._gripper,
                        qos_profile_sensor_data,
                    )
                    self.gripper = ActionClient(
                        self.node, GripperCommand, "/rebotarm/gripper/command"
                    )
                    self.clients.append((self.gripper, "/rebotarm/gripper/command"))
            for name, (topic, _, _) in config.cameras.items():
                self.node.create_subscription(
                    CompressedImage,
                    topic,
                    lambda msg, key=name: self._store("camera:" + key, bytes(msg.data), msg.header),
                    qos_profile_sensor_data,
                )
            for client, name in self.clients:
                if not client.wait_for_server(timeout_sec=config.timeout_s):
                    raise EmbodimentFault(f"Action server unavailable: {name}")
            if config.simulated:
                self._verify_mock()
        except BaseException:
            self.close()
            raise

    def _store(self, name: str, value: JointVector | float | bytes, header: Any) -> None:
        stamp_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age < -0.1 or age > self.config.state_timeout_s:
            return
        self.samples[name] = (time.monotonic(), value)

    def _joints(self, msg: Any) -> None:
        if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
            self.fault = "Malformed JointState names/positions"
            return
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in JOINTS):
            self.fault = f"JointState is missing B601 joints: received {msg.name}"
            return
        positions = np.asarray([values[name] for name in JOINTS], dtype=np.float64)
        if not np.isfinite(positions).all():
            self.fault = "Non-finite measured joint position"
            return
        self._store("arm", positions, msg.header)

    def _gripper(self, msg: Any) -> None:
        if not np.isfinite(msg.position):
            self.fault = "Non-finite gripper motor position"
        else:
            self._store("gripper", float(msg.position), msg.header)

    def _status(self, msg: Any) -> None:
        self.status = msg

    def _check(self, moving: bool = False) -> None:
        if self.fault:
            raise EmbodimentFault(self.fault)
        if not self.config.simulated and self.status is not None and self.status.error_codes:
            raise EmbodimentFault("B601 controller fault: " + "; ".join(self.status.error_codes))
        if moving:
            now = time.monotonic()
            for name, (stamp, _) in self.samples.items():
                if now - stamp > self.config.state_timeout_s:
                    raise EmbodimentFault(f"Feedback stopped during action: {name}")
            if not self.config.simulated:
                s = self.status
                if s is None:
                    raise EmbodimentFault("B601 status is unavailable")
                if not s.enabled or not s.control_loop_active:
                    raise EmbodimentFault(
                        f"B601 status is disabled or inactive: state={s.state_machine}, "
                        f"enabled={s.enabled}, control_loop_active={s.control_loop_active}"
                    )
                if s.state_machine not in ("IDLE", "TRAJ_RUNNING"):
                    raise EmbodimentFault(f"B601 is unavailable for Inspect: {s.state_machine}")

    def _wait(
        self,
        ready: Callable[[], bool],
        timeout: float,
        label: str,
        *,
        moving: bool = False,
        cleanup: bool = False,
    ) -> None:
        deadline = time.monotonic() + timeout
        while not ready():
            if not cleanup:
                self._check(moving)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise EmbodimentFault(f"Timed out waiting for {label}")
            self.executor.spin_once(timeout_sec=min(0.02, remaining))
        if not cleanup:
            self._check(moving)

    def _verify_mock(self) -> None:
        client = self.node.create_client(GetParameters, "/controller_manager/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=self.config.timeout_s):
                raise EmbodimentFault("Cannot verify the B601 mock controller")
            request = GetParameters.Request()
            request.names = ["robot_description"]
            future = client.call_async(request)
            self._wait(future.done, self.config.timeout_s, "mock robot description")
            robot = ET.fromstring(future.result().values[0].string_value)
            plugins = [
                (p.text or "").strip() for p in robot.findall(".//ros2_control/hardware/plugin")
            ]
            if robot.get("name") != "ReBot_Arm_RS" or plugins != ["mock_components/GenericSystem"]:
                raise EmbodimentFault("Simulation requires the verified B601-RS GenericSystem mock")
        finally:
            self.node.destroy_client(client)

    def observe(self) -> RosObservation:
        started = time.monotonic()
        required = ["arm"] + (["gripper"] if self.config.gripper else [])
        required += ["camera:" + name for name in self.config.cameras]

        def ready() -> bool:
            return all(
                key in self.samples and self.samples[key][0] > started for key in required
            ) and (self.config.simulated or self.status is not None)

        self._wait(ready, self.config.timeout_s, "fresh joint/camera feedback")
        now = time.monotonic()
        if any(now - self.samples[key][0] > self.config.state_timeout_s for key in required):
            raise EmbodimentFault("Stale or unsynchronized observation")
        positions = cast(JointVector, self.samples["arm"][1]).copy()
        stamps = [self.samples["arm"][0]]
        if self.config.gripper:
            stamp, position = self.samples["gripper"]
            positions = np.append(positions, position)
            stamps.append(stamp)
        images: dict[str, RGBImage] = {}
        image_times: dict[str, float] = {}
        for name, (_, width, height) in self.config.cameras.items():
            stamp, data = self.samples["camera:" + name]
            try:
                with Image.open(io.BytesIO(cast(bytes, data))) as image:
                    if image.size != (width, height):
                        raise ValueError(f"Expected {width}x{height}, got {image.size}")
                    images[name] = np.asarray(image.convert("RGB")).copy()
            except Exception as exc:
                raise EmbodimentFault(f"Invalid camera {name}: {exc}") from exc
            image_times[name] = stamp
        return positions, min(stamps), images, image_times

    def _execute(self, client: Any, goal: Any, timeout: float) -> Any:
        self._check(moving=True)
        self.pending = client.send_goal_async(goal)
        try:
            self._wait(self.pending.done, self.config.timeout_s, "goal acceptance", moving=True)
            self.handle = self.pending.result()
            self.pending = None
            if not self.handle.accepted:
                self.handle = None
                raise EmbodimentFault("B601 action goal rejected")
            self.result = self.handle.get_result_async()
            self._wait(self.result.done, timeout, "action result", moving=True)
            response = self.result.result()
            self.handle = self.result = None
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                raise EmbodimentFault(
                    f"B601 action ended with status {response.status}: {response.result}"
                )
            return response.result
        except BaseException as primary:
            try:
                self._cancel()
            except Exception as cleanup:
                raise EmbodimentFault(
                    f"{primary}; additionally cancellation is unconfirmed: {cleanup}"
                ) from primary
            raise

    def move(self, target: JointVector, duration: float) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = target[:6].tolist()
        nanoseconds = round(duration * 1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(
            nanoseconds, 1_000_000_000
        )
        goal.trajectory.points = [point]
        result = self._execute(self.arm, goal, duration + self.config.timeout_s)
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise EmbodimentFault(f"Trajectory failed ({result.error_code}): {result.error_string}")
        if self.config.gripper:
            goal = GripperCommand.Goal()
            goal.command.position = float(target[-1])
            goal.command.max_effort = self.config.gripper_max_effort
            result = self._execute(self.gripper, goal, self.config.timeout_s + 5.0)
            if not result.reached_goal or result.stalled:
                raise EmbodimentFault("Gripper did not reach its target or stalled")

    def _cancel(self) -> None:
        if self.pending is not None:
            self._wait(
                self.pending.done,
                self.config.timeout_s,
                "pending acceptance before cancellation",
                cleanup=True,
            )
            self.handle = self.pending.result()
            self.pending = None
        if self.handle is not None and self.handle.accepted:
            if self.result is None:
                self.result = self.handle.get_result_async()
            if not self.result.done():
                future = self.handle.cancel_goal_async()
                self._wait(
                    future.done, self.config.timeout_s, "cancellation response", cleanup=True
                )
                response = future.result()
                label = "terminal action result after cancellation"
                if not response.goals_canceling:
                    label += f" (no cancellation accepted, code={response.return_code})"
                self._wait(self.result.done, self.config.timeout_s, label, cleanup=True)
            if self.result.result().status not in (
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            ):
                raise EmbodimentFault("Controller did not report a terminal action state")
        self.handle = self.result = None

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self._cancel()
        finally:
            for client, _ in self.clients:
                client.destroy()
            self.executor.remove_node(self.node)
            self.executor.shutdown()
            self.node.destroy_node()
            self.context.try_shutdown()
