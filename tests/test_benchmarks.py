"""Evaluation results must follow measured trajectories, never dispatched targets."""

from types import SimpleNamespace

import numpy as np
import pytest
from inspect_robots import Observation, Scene, eval
from inspect_robots.approver import DeltaLimitApprover
from inspect_robots.compat import check_compatibility
from inspect_robots.errors import ConfigError, PolicyError
from inspect_robots.registry import resolve

from inspect_robots_b601 import MotorbridgeB601Embodiment
from inspect_robots_b601.benchmarks import JointGoalPolicy, MeasuredJointGoal, hold, joint1_nudge


def observation(values, *, direct=False):
    values = np.asarray(values, dtype=np.float64)
    state = {"joint_pos": values[:1] if direct else values}
    if direct:
        state["arm_joint_pos"] = values
    return Observation(state=state, state_time=1.0, images={}, image_times={})


def record(*positions, status="success", direct=False):
    observations = [observation(value, direct=direct) for value in positions]
    return SimpleNamespace(
        status=status,
        steps=[
            SimpleNamespace(observation=before, result=SimpleNamespace(observation=after))
            for before, after in zip(observations, observations[1:])
        ],
    )


@pytest.mark.parametrize("task_factory", [hold, joint1_nudge])
def test_b601_tasks_execute_and_score_measured_state(fake_ros, task_factory, tmp_path):
    (log,) = eval(
        task_factory(),
        JointGoalPolicy(),
        fake_ros,
        log_dir=str(tmp_path),
        approver=DeltaLimitApprover(fake_ros.info.action_space, max_delta=0.002),
    )
    assert log.status == "success"
    assert log.results.metrics["b601_measured_goal"] == 1.0


def test_successful_dispatch_without_motion_scores_failure(fake_ros, tmp_path):
    fake_ros.reset(Scene(id="prepare", instruction="prepare fake controller"))
    fake_ros._session.frozen = True
    (log,) = eval(joint1_nudge(), JointGoalPolicy(), fake_ros, log_dir=str(tmp_path))
    assert log.status == "success"
    assert log.results.metrics["b601_measured_goal"] == 0.0


def test_scorer_checks_all_six_joints_for_direct_backend():
    before, after = np.zeros(6), np.array([0.0015, 0, 0, 0, 0, 0])
    scorer, target = MeasuredJointGoal(), joint1_nudge().scenes[0].target
    assert scorer(record(before, after, direct=True), target).value
    after[4] = 0.0008
    assert not scorer(record(before, after, direct=True), target).value


def test_scorer_detects_transient_drift_even_if_final_pose_recovers():
    initial = np.zeros(6)
    drift, final = initial.copy(), initial.copy()
    drift[2] = 0.001
    assert not MeasuredJointGoal()(record(initial, drift, final), hold().scenes[0].target).value


def test_scorer_detects_overshoot_even_if_goal_is_eventually_reached():
    initial = np.zeros(6)
    overshoot, final = initial.copy(), initial.copy()
    overshoot[0], final[0] = 0.004, 0.0015
    assert not MeasuredJointGoal()(
        record(initial, overshoot, final), joint1_nudge().scenes[0].target
    ).value


@pytest.mark.parametrize("values", [[0], [float("nan")] * 6, [0] * 5, [0] * 8])
def test_missing_full_arm_or_nonfinite_state_cannot_pass(values):
    assert not MeasuredJointGoal()(record(values, values), hold().scenes[0].target).value


@pytest.mark.parametrize("status", ["error", "cancelled"])
def test_errored_or_cancelled_records_never_score_success(status):
    assert not MeasuredJointGoal()(
        record([0] * 6, [0] * 6, status=status), hold().scenes[0].target
    ).value


def test_empty_record_and_missing_target_fail_closed():
    assert not MeasuredJointGoal()(record(), hold().scenes[0].target).value
    assert not MeasuredJointGoal()(record([0] * 6, [0] * 6), None).value


@pytest.mark.parametrize(
    "options",
    [
        {"delta": float("nan")},
        {"delta": 0.006},
        {"delta": 0.001},
        {"tolerance": 0},
        {"tolerance": float("nan")},
        {"tolerance": 0.01},
    ],
)
def test_invalid_or_unmeasurably_small_goals_are_rejected(options):
    with pytest.raises(ValueError):
        joint1_nudge(**options)


def test_scripted_policy_has_an_initial_pose_relative_goal_not_accumulating_nudges(fake_ros):
    policy = JointGoalPolicy()
    policy.bind(fake_ros.info)
    policy.reset(joint1_nudge().scenes[0])
    initial = np.array([0, 1, 1, 0, 0, 0], dtype=float)
    first = policy.act(observation(initial)).actions[0].data
    second = policy.act(observation(first)).actions[0].data
    np.testing.assert_allclose(first, [0.0015, 1, 1, 0, 0, 0])
    np.testing.assert_allclose(second, first)


def test_goal_outside_bounds_is_rejected_without_a_command(fake_ros):
    policy = JointGoalPolicy()
    policy.bind(fake_ros.info)
    policy.reset(joint1_nudge().scenes[0])
    with pytest.raises(PolicyError, match="outside"):
        policy.act(observation([2.8, 1, 1, 0, 0, 0]))


def test_cubepick_policy_is_incompatible_with_b601(fake_ros):
    assert not check_compatibility(resolve("policy", "scripted"), fake_ros).ok
    with pytest.raises(ConfigError):
        JointGoalPolicy().bind(resolve("embodiment", "cubepick").info)


def test_scripted_policy_adapts_to_one_axis_without_claiming_six_axis_control():
    from pathlib import Path

    profile = Path(__file__).parent / "fixtures/joint1.synthetic.json"
    robot = MotorbridgeB601Embodiment(profile=str(profile))
    policy = JointGoalPolicy()
    policy.bind(robot.info)
    assert policy.info.action_space.shape == (1,)
    assert check_compatibility(policy, robot, joint1_nudge()).ok


def test_upstream_cubepick_pipeline_smoke(tmp_path):
    (log,) = eval("cubepick-reach", "scripted", "cubepick", log_dir=str(tmp_path))
    assert log.status == "success"
    assert log.results.metrics["success_at_end"] == 1.0
