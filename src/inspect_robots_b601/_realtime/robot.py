"""Robot-local execution: the network can observe or request fixed mock tasks only."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any, Literal

from inspect_robots import ActionChunk, Observation, Scene, eval
from inspect_robots.approver import ChainApprover, ClampApprover, DeltaLimitApprover
from inspect_robots.errors import SafetyAbort

from ..benchmarks import JointGoalPolicy, hold, joint1_nudge
from ..embodiment import JOINTS, B601Embodiment
from ..motorbridge import MotorbridgeB601Embodiment


class _GuardedPolicy(JointGoalPolicy):
    def __init__(self, stopped: threading.Event) -> None:
        super().__init__()
        self._stopped = stopped

    def act(self, observation: Observation) -> ActionChunk:
        if self._stopped.is_set():
            raise SafetyAbort("Realtime operator lease ended or stop was requested")
        return super().act(observation)


class Robot:
    """One local worker owns every backend call; remote code never receives a driver handle."""

    def __init__(
        self,
        backend: Literal["echo", "ros-mock", "motorbridge-read"],
        *,
        profile: Path | None = None,
        state_dir: Path | None = None,
        log_dir: str = "logs/realtime",
    ) -> None:
        if backend not in ("echo", "ros-mock", "motorbridge-read"):
            raise ValueError("Unknown robot-side backend")
        self.backend, self.log_dir = backend, log_dir
        self.stopped = threading.Event()
        self._robot: B601Embodiment | MotorbridgeB601Embodiment | None = None
        self._fault: str | None = None
        if backend == "ros-mock":
            self._robot = B601Embodiment(simulated=True, max_step=0.002, position_tolerance=0.0006)
        elif backend == "motorbridge-read":
            if profile is None or state_dir is None:
                raise ValueError(
                    "Read-only hardware needs explicit profile and shared persistent state paths"
                )
            self._robot = MotorbridgeB601Embodiment(profile=profile, state_dir=state_dir)

    def stop(self) -> None:
        """Latch locally; never expose enable, recovery, homing or raw motor writes."""
        self.stopped.set()

    def status(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "backend": self.backend,
            "endpoint_stopped": self.stopped.is_set(),
            "fault": self._fault,
            "remote_motion": self.backend == "ros-mock",
            "physical_activation_available": False,
        }
        if self._robot is None:
            return {
                **result,
                "robot_connected": False,
                "detail": "Realtime link only; no robot is opened",
            }
        if self.stopped.is_set() or self._fault:
            return result
        try:
            observation = self._robot.reset(Scene(id="realtime-status", instruction="Observe only"))
            return {
                **result,
                "robot_connected": True,
                "simulated": self._robot.info.is_simulated,
                "coordinate_frame": "raw_motor_radians"
                if self.backend == "motorbridge-read"
                else "ros_joint_radians",
                "arm_joint_order": list(JOINTS),
                "state_age_s": max(0.0, time.monotonic() - observation.state_time),
                "state": {key: value.tolist() for key, value in observation.state.items()},
            }
        except Exception as error:
            self._fault = str(error)
            self.stop()
            raise

    def run(self, task: str) -> dict[str, Any]:
        """Only the verified ROS mock may move, in one short measured benchmark."""
        if self.backend != "ros-mock" or not isinstance(self._robot, B601Embodiment):
            raise ValueError(
                "Remote motion is simulation-only; hardware and echo endpoints cannot run tasks"
            )
        if self.stopped.is_set() or self._fault:
            raise RuntimeError("Endpoint stopped/faulted; explicit local restart required")
        if task not in ("hold", "joint1-nudge"):
            raise ValueError("Only hold and joint1-nudge are supported")
        space = self._robot.info.action_space
        guard = ChainApprover(ClampApprover(space), DeltaLimitApprover(space, max_delta=0.002))
        try:
            (log,) = eval(
                (hold if task == "hold" else joint1_nudge)(),
                _GuardedPolicy(self.stopped),
                self._robot,
                approver=guard,
                log_dir=self.log_dir,
            )
            passed = (
                log.status == "success" and log.results.metrics.get("b601_measured_goal") == 1.0
            )
            if not passed:
                self._fault = "Measured benchmark failed"
                self.stop()
            return {"passed": passed, "status": log.status, "metrics": log.results.metrics}
        except Exception as error:
            self._fault = str(error)
            self.stop()
            raise

    def close(self) -> None:
        self.stop()
        if self._robot is not None:
            self._robot.close()
