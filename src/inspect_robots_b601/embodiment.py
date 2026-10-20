"""Inspect's spaces, measured-state contract and B601 command limits."""

from __future__ import annotations

import math
import time
from collections.abc import Sequence
from typing import TYPE_CHECKING

import numpy as np
from inspect_robots import (
    Action,
    ActionSemantics,
    Box,
    CameraSpec,
    EmbodimentBase,
    EmbodimentInfo,
    Observation,
    ObservationSpace,
    Scene,
    StateField,
    StateSpec,
    StepResult,
)
from inspect_robots.errors import EmbodimentFault, SafetyAbort

from ._types import JointVector

if TYPE_CHECKING:
    from ._ros import RosSession

JOINTS = tuple(f"joint{i}" for i in range(1, 7))
SIM_LOW = (-2.8, 0.0, 0.0, -1.57, -1.57, -3.14)
SIM_HIGH = (2.8, 3.14, 3.14, 1.57, 1.57, 3.14)


def _bounds(
    value: str | Sequence[float] | JointVector | None, default: Sequence[float]
) -> JointVector:
    result = np.asarray(
        default if value is None else (value.split(",") if isinstance(value, str) else value),
        dtype=np.float64,
    )
    if result.shape != (6,) or not np.isfinite(result).all():
        raise ValueError("Arm bounds must contain six finite radians")
    return result


def _cameras(value: str) -> dict[str, tuple[str, int, int]]:
    cameras: dict[str, tuple[str, int, int]] = {}
    if value:
        for entry in value.split(","):
            name, topic, resolution = entry.split(":")
            width, height = map(int, resolution.lower().split("x"))
            if not name.isidentifier() or name in cameras or not topic.startswith("/"):
                raise ValueError("Cameras need unique names and absolute ROS topics")
            if min(width, height) <= 0:
                raise ValueError("Camera dimensions must be positive")
            cameras[name] = (topic, width, height)
    return cameras


class B601Embodiment(EmbodimentBase):
    """Execute one bounded ROS action at a time, with fresh measured feedback.

    Construction and Inspect doctor are hardware-free. ``simulated=True`` uses
    only the verified Seeed mock controller. Hardware bounds are explicit because
    the installed motor zeros need not match the nominal URDF.
    """

    RUNTIME_REQUIREMENTS = {"rclpy": "source /opt/ros/humble/setup.bash; use system Python 3.10"}

    def __init__(
        self,
        *,
        simulated: bool = True,
        action_low: str | Sequence[float] | JointVector | None = None,
        action_high: str | Sequence[float] | JointVector | None = None,
        control_hz: float = 2.0,
        max_velocity: float = 0.15,
        max_step: float = 0.05,
        state_timeout_s: float = 0.5,
        timeout_s: float = 5.0,
        position_tolerance: float = 0.03,
        cameras: str = "",
        gripper_open: float | None = None,
        gripper_closed: float | None = None,
        gripper_max_effort: float | None = None,
    ) -> None:
        if type(simulated) is not bool:
            raise ValueError("simulated must be a boolean")
        if not simulated and (action_low is None or action_high is None):
            raise ValueError("Hardware requires explicit commissioned action_low and action_high")
        low, high = _bounds(action_low, SIM_LOW), _bounds(action_high, SIM_HIGH)
        if np.any(low >= high):
            raise ValueError("Each action_low must be less than action_high")
        for name, value in (
            ("control_hz", control_hz),
            ("max_velocity", max_velocity),
            ("max_step", max_step),
            ("state_timeout_s", state_timeout_s),
            ("timeout_s", timeout_s),
            ("position_tolerance", position_tolerance),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        gripper = (gripper_open, gripper_closed, gripper_max_effort)
        self.gripper = any(v is not None for v in gripper)
        if self.gripper:
            if simulated:
                raise ValueError(
                    "The Seeed mock has trajectory fingers, not a motor GripperCommand"
                )
            if any(v is None or not math.isfinite(v) for v in gripper):
                raise ValueError(
                    "Supply finite gripper_open, gripper_closed and gripper_max_effort"
                )
            assert (
                gripper_open is not None
                and gripper_closed is not None
                and gripper_max_effort is not None
            )
            if gripper_open == gripper_closed or gripper_max_effort <= 0:
                raise ValueError("Gripper endpoints must differ and max effort must be positive")
            low = np.append(low, min(gripper_open, gripper_closed))
            high = np.append(high, max(gripper_open, gripper_closed))
        self.simulated = simulated
        self.control_hz = control_hz
        self.max_velocity, self.max_step = max_velocity, max_step
        self.state_timeout_s, self.timeout_s = state_timeout_s, timeout_s
        self.position_tolerance = position_tolerance
        self.gripper_open, self.gripper_closed = gripper_open, gripper_closed
        self.gripper_max_effort = gripper_max_effort
        self.cameras = _cameras(cameras)
        labels = JOINTS + (("gripper",) if self.gripper else ())
        fields = [StateField("joint_pos", (len(labels),), "rad")]
        if self.gripper:
            fields.append(StateField("gripper", (1,), "normalized"))
        self.info = EmbodimentInfo(
            name="b601_ros_sim" if simulated else "b601_ros",
            action_space=Box(
                shape=(len(labels),),
                low=low,
                high=high,
                semantics=ActionSemantics(
                    control_mode="joint_pos",
                    rotation_repr="none",
                    gripper="continuous" if self.gripper else "none",
                    frame="base",
                    dim_labels=labels,
                ),
            ),
            observation_space=ObservationSpace(
                cameras=tuple(CameraSpec(name, h, w) for name, (_, w, h) in self.cameras.items()),
                state=StateSpec(fields=tuple(fields)),
            ),
            control_hz=control_hz,
            is_simulated=simulated,
            capabilities=frozenset({"self_paced"}),
            docs=(
                "Seeed B601-RS: joint1..joint6 are ordered absolute ROS joint radians. "
                "Each step waits for action completion; the effective rate can be lower "
                "than control_hz. No Cartesian IK or automatic homing/reset is provided. "
                "Only configured ROS cameras are visible. "
                + (
                    f"The final gripper target is motor radians: closed={gripper_closed}, "
                    f"open={gripper_open}. The separate gripper state is normalized 0..1."
                    if self.gripper
                    else "No gripper is configured."
                )
            ),
        )
        self._session: RosSession | None = None
        self._instruction: str | None = None
        self._faulted = False

    def reset(self, scene: Scene, *, seed: int | None = None) -> Observation:
        """Observe the current pose; never enable, home or reset a physical robot."""
        if self._faulted:
            raise EmbodimentFault("This session faulted; inspect the cause and start a new session")
        self._instruction = scene.instruction
        try:
            if self._session is None:
                from ._ros import RosSession

                self._session = RosSession(self)
            return self._observe()
        except Exception as exc:
            self._faulted = True
            raise EmbodimentFault(f"B601 reset failed: {exc}") from exc

    def _observe(self) -> Observation:
        assert self._session is not None
        positions, stamp, images, image_times = self._session.observe()
        state = {"joint_pos": positions}
        if self.gripper:
            assert self.gripper_closed is not None and self.gripper_open is not None
            state["gripper"] = np.asarray(
                [(positions[-1] - self.gripper_closed) / (self.gripper_open - self.gripper_closed)]
            )
        return Observation(
            state=state,
            state_time=stamp,
            images=images,
            image_times=image_times,
            instruction=self._instruction,
        )

    def step(self, action: Action) -> StepResult:
        """Reject invalid targets before sending; faults halt the entire evaluation."""
        if self._session is None or self._faulted:
            raise EmbodimentFault("reset() is required and a faulted session cannot resume")
        try:
            measured = self._observe().state["joint_pos"]
            target = np.asarray(action.data, dtype=np.float64)
            space = self.info.action_space
            assert space.low is not None and space.high is not None
            if target.shape != space.shape or not np.isfinite(target).all():
                raise SafetyAbort("Action must contain one finite radian target per dimension")
            if np.any(target < space.low) or np.any(target > space.high):
                raise SafetyAbort("Action is outside configured joint limits")
            delta = np.abs(target - measured)
            if np.any(delta > self.max_step + 1e-9):
                raise SafetyAbort(f"Action exceeds max_step={self.max_step} rad from measured pose")
            duration = max(1 / self.control_hz, float(delta.max()) / self.max_velocity)
            started = time.monotonic()
            self._session.move(target, duration)
            remaining = duration - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
            observation = self._observe()
            if np.any(np.abs(observation.state["joint_pos"] - target) > self.position_tolerance):
                raise EmbodimentFault("Action succeeded but measured target tolerance was not met")
            return StepResult(observation=observation)
        except BaseException:
            self._faulted = True
            raise

    def close(self) -> None:
        """Cancel this session's goals and release ROS; never change motor power."""
        if self._session is not None:
            session, self._session = self._session, None
            session.close()
