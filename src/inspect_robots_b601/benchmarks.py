"""B601 joint tasks, deterministic policy and scoring of measured trajectories."""

from __future__ import annotations

import math

import numpy as np
from inspect_robots import Action, ActionChunk, Scene
from inspect_robots.errors import ConfigError, PolicyError
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.scene import Target
from inspect_robots.scorer import Score
from inspect_robots.task import Task


def _goal(target):
    if target is None or target.kind != "b601_joint_delta":
        raise ValueError("A b601_joint_delta target is required")
    delta = float(target.spec["delta"])
    tolerance = float(target.spec["tolerance"])
    if not math.isfinite(delta) or abs(delta) > 0.005:
        raise ValueError("Joint1 benchmark delta must be finite and at most 0.005 rad")
    if not math.isfinite(tolerance) or not 0 < tolerance <= 0.001:
        raise ValueError("Benchmark tolerance must be in (0, 0.001] radians")
    if delta and abs(delta) <= 2 * tolerance:
        raise ValueError("Nudge must exceed twice the tolerance to distinguish motion from holding")
    return delta, tolerance


def _task(name, delta, tolerance, max_steps, epochs):
    target = Target(kind="b601_joint_delta", spec={"delta": delta, "tolerance": tolerance})
    _goal(target)
    instruction = (
        "Keep all joints at their initially observed positions."
        if delta == 0 else
        f"Move joint1 by {delta:+g} radians from its initially observed position, "
        "keep every other joint fixed, then hold the resulting position."
    )
    return Task(
        name=name, scenes=[Scene(id="measured-joints", instruction=instruction, target=target)],
        scorer=MeasuredJointGoal(), max_steps=max_steps, epochs=epochs,
        metadata={"scope": "joint-space commissioning, not manipulation or physical safety qualification"},
    )


def hold(*, tolerance=0.0006, max_steps=3, epochs=1):
    """Keep every measured joint fixed; score drift throughout the trajectory."""
    return _task("b601/hold", 0.0, tolerance, max_steps, epochs)


def joint1_nudge(*, delta=0.0015, tolerance=0.0006, max_steps=3, epochs=1):
    """Request a small relative joint1 goal while observing all other joints."""
    return _task("b601/joint1-nudge", delta, tolerance, max_steps, epochs)


def _measured(observation):
    state = observation.state
    positions = np.asarray(state.get("arm_joint_pos", state.get("joint_pos")), dtype=np.float64)
    if positions.shape not in ((6,), (7,)) or not np.isfinite(positions).all():
        raise ValueError("Scoring requires all six measured arm joints, plus any configured gripper")
    return positions


class MeasuredJointGoal:
    """Require a measured final goal, no non-target drift, and no observed overshoot."""

    name = "b601_measured_goal"

    def __call__(self, record, target):
        try:
            delta, tolerance = _goal(target)
            if record.status != "success" or not record.steps:
                raise ValueError("A nonempty, error-free measured trajectory is required")
            initial = _measured(record.steps[0].observation)
            observations = [observation for step in record.steps
                            for observation in (step.observation, step.result.observation)]
            measured = np.stack([_measured(observation) for observation in observations])
            if measured.shape[1:] != initial.shape:
                raise ValueError("Measured joint layout changed during the trial")
            goal = initial[0] + delta
            target_error = float(abs(measured[-1, 0] - goal))
            other_drift = float(np.max(np.abs(measured[:, 1:] - initial[1:])))
            low, high = sorted((initial[0], goal))
            overshoot = float(max(0.0, low - measured[:, 0].min(), measured[:, 0].max() - high))
            details = {"target_error_rad": target_error, "other_joint_drift_rad": other_drift,
                       "observed_overshoot_rad": overshoot, "tolerance_rad": tolerance}
            passed = max(target_error, other_drift, overshoot) <= tolerance + 1e-9
            return Score(value=passed, explanation="Measured goal and trajectory checks " +
                         ("passed" if passed else "failed"), metadata=details)
        except (KeyError, TypeError, ValueError) as error:
            return Score(value=False, explanation=str(error))


class JointGoalPolicy(PolicyBase):
    """Adapt to the embodiment, then approach one initial-pose-relative joint1 goal."""

    def __init__(self, *, max_step=0.0015):
        if not math.isfinite(max_step) or not 0 < max_step <= 0.002:
            raise ValueError("Scripted max_step must be in (0, 0.002] radians")
        self.max_step = max_step
        self._space = None
        self._target = None

    def bind(self, info):
        space = info.action_space
        semantics = space.semantics
        if (semantics is None or semantics.control_mode != "joint_pos"
                or tuple(semantics.dim_labels) not in (
                    ("joint1",), tuple(f"joint{i}" for i in range(1, 7)),
                    tuple(f"joint{i}" for i in range(1, 7)) + ("gripper",))):
            raise ConfigError("b601_scripted requires the B601 absolute joint-radian contract")
        self._space = space
        self.info = PolicyInfo(name="b601_scripted", action_space=space,
                               observation_space=info.observation_space, control_hz=info.control_hz)

    def reset(self, scene):
        self._delta, _ = _goal(scene.target)
        self._target = None

    def act(self, observation):
        if self._space is None:
            raise PolicyError("Bind the scripted policy before requesting actions")
        measured = np.asarray(observation.state["joint_pos"], dtype=np.float64)
        if measured.shape != self._space.shape or not np.isfinite(measured).all():
            raise PolicyError("Invalid measured joint_pos")
        if self._target is None:
            self._target = measured.copy()
            self._target[0] += self._delta
            if np.any(self._target < self._space.low) or np.any(self._target > self._space.high):
                raise PolicyError("Benchmark goal is outside commissioned limits; no command sent")
        action = measured + np.clip(self._target - measured, -self.max_step, self.max_step)
        return ActionChunk([Action(action)])
