"""Direct Inspect joint1 commissioning adapter; no ROS/action dependency."""

from __future__ import annotations

import hashlib
import json
import math
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
from inspect_robots import (
    Action,
    ActionSemantics,
    Box,
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

if TYPE_CHECKING:
    from b601_hardware.direct import DirectSession


class MotorbridgeB601Embodiment(EmbodimentBase):
    """Direct motor commands with an independently ticking bounded HOLD worker.

    This adapter honestly advertises one action dimension: only joint1 has a
    commissioning transport. Other joints are observed, not enabled or held.
    Constructor/doctor never open CAN. Physical reset activation is opt-in.
    """

    RUNTIME_REQUIREMENTS = {
        "motorbridge": "python -m pip install 'inspect-robots-b601[motorbridge]'",
    }

    def __init__(
        self,
        *,
        profile: str | Path,
        state_dir: str | Path = ".hardware-state",
        allow_motion: bool = False,
        activate_on_reset: bool = False,
        operator_supported_arm: bool = False,
        tracking_report: str = "",
        control_hz: float = 2.0,
        max_step: float = 0.002,
    ) -> None:
        if any(
            type(value) is not bool
            for value in (allow_motion, activate_on_reset, operator_supported_arm)
        ):
            raise ValueError("Motion and operator acknowledgements must be booleans")
        if activate_on_reset and not (allow_motion and operator_supported_arm):
            raise ValueError("Reset activation requires allow_motion and operator_supported_arm")
        if activate_on_reset and not tracking_report:
            raise ValueError(
                "Reset activation also requires a recent successful direct tracking_report"
            )
        if (
            not math.isfinite(control_hz)
            or not 0.2 <= control_hz <= 2.0
            or not math.isfinite(max_step)
            or not 0 < max_step <= 0.002
        ):
            raise ValueError("control_hz in [0.2, 2] and max_step <= 0.002 rad are required")
        path = Path(profile).resolve()
        content = path.read_bytes()
        self.profile: dict[str, Any] = json.loads(content)
        self.profile_sha256 = hashlib.sha256(content).hexdigest()
        self.profile_path, self.tracking_report = path, tracking_report
        joints = self.profile["joints"]
        if len(joints) != 6 or [j["name"] for j in joints] != [f"joint{i}" for i in range(1, 7)]:
            raise ValueError("Expected the reviewed six-joint profile")
        lower, upper = float(joints[0]["lower"]), float(joints[0]["upper"])
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError("Finite increasing joint1 bounds required")
        self.state_dir = str(Path(state_dir).resolve())
        self.allow_motion = allow_motion
        self.activate_on_reset = activate_on_reset
        self.control_hz, self.max_step = control_hz, max_step
        self.info = EmbodimentInfo(
            name="b601_motorbridge_joint1",
            action_space=Box(
                shape=(1,),
                low=np.array([lower]),
                high=np.array([upper]),
                semantics=ActionSemantics(
                    control_mode="joint_pos",
                    rotation_repr="none",
                    gripper="none",
                    frame="base",
                    dim_labels=("joint1",),
                ),
            ),
            observation_space=ObservationSpace(
                cameras=(),
                state=StateSpec(
                    fields=(
                        StateField("joint_pos", (1,), "rad"),
                        StateField("arm_joint_pos", (6,), "rad"),
                    )
                ),
            ),
            control_hz=control_hz,
            is_simulated=False,
            capabilities=frozenset({"self_paced"}),
            docs=(
                "B601-RS direct Motorbridge commissioning: ONLY joint1 is controllable, in absolute raw motor radians. "
                "arm_joint_pos reports all six measured joints; joints2-6 are NOT held. "
                "No gripper, camera, Cartesian IK, home or zeroing. At most 0.002 rad per Inspect step and "
                "0.01 rad per activation envelope, with 0.01 rad/s command limit. "
                "LLM/VLA pauses retain bounded HOLD via an independent worker, not ROS. "
                "Physical reset activates only when all explicit operator flags are configured."
            ),
        )
        self._session: DirectSession | None = None
        self._instruction: str | None = None
        self._faulted = False

    def reset(self, scene: Scene, *, seed: int | None = None) -> Observation:
        if self._faulted:
            raise EmbodimentFault(
                "Direct session faulted; explicit operator recovery/new session required"
            )
        self._instruction = scene.instruction
        try:
            if self._session is None:
                from b601_hardware.direct import DirectSession, validate_tracking_report

                if (
                    hashlib.sha256(self.profile_path.read_bytes()).hexdigest()
                    != self.profile_sha256
                ):
                    raise RuntimeError(
                        "Profile changed after adapter construction; refusing CAN open"
                    )
                if self.activate_on_reset:
                    validate_tracking_report(self.tracking_report, self.profile_path)
                self._session = DirectSession(self.profile, self.state_dir)
                self._session.open()
                if self.activate_on_reset:
                    self._session.activate()
            return self._observe()
        except BaseException as exc:
            self._faulted = True
            raise EmbodimentFault(f"Direct B601 reset failed: {exc}") from exc

    def _observe(self) -> Observation:
        assert self._session is not None
        positions, stamp = self._session.observe()
        return Observation(
            state={
                "joint_pos": np.asarray(positions[:1], dtype=np.float64),
                "arm_joint_pos": np.asarray(positions, dtype=np.float64),
            },
            state_time=stamp,
            images={},
            image_times={},
            instruction=self._instruction,
        )

    def step(self, action: Action) -> StepResult:
        if (
            self._session is None
            or self._faulted
            or not (self.allow_motion and self.activate_on_reset)
        ):
            raise EmbodimentFault("An explicitly activated direct session is required")
        try:
            observation = self._observe()
            target = np.asarray(action.data, dtype=np.float64)
            low, high = self.info.action_space.low, self.info.action_space.high
            assert low is not None and high is not None
            if (
                target.shape != (1,)
                or not np.isfinite(target).all()
                or np.any(target < low)
                or np.any(target > high)
                or abs(target[0] - observation.state["joint_pos"][0]) > self.max_step + 1e-9
            ):
                raise SafetyAbort("Target must be one finite, bounded joint1 value within max_step")
            positions = observation.state["arm_joint_pos"].copy()
            positions[0] = target[0]
            duration = max(
                1 / self.control_hz, abs(target[0] - observation.state["joint_pos"][0]) / 0.01
            )
            started = time.monotonic()
            self._session.move(positions.tolist(), duration)
            remaining = duration - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)  # The independent worker retains bounded HOLD.
            result = self._observe()
            if abs(result.state["joint_pos"][0] - target[0]) > 0.0006:
                raise EmbodimentFault("Direct command did not achieve measured convergence")
            return StepResult(observation=result)
        except BaseException as primary:
            self._faulted = True
            try:
                self._session.stop("Inspect direct step failed or was interrupted")
            except Exception as cleanup:
                raise EmbodimentFault(f"{primary}; stop also failed: {cleanup}") from primary
            raise

    def close(self) -> None:
        if self._session is not None:
            session, self._session = self._session, None
            session.close()
