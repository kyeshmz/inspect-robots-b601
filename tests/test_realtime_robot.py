"""Remote requests cannot turn a physical observation endpoint into a motion controller."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from inspect_robots import Observation
from inspect_robots.errors import SafetyAbort

from inspect_robots_b601._realtime.robot import Robot, _GuardedPolicy
from inspect_robots_b601.benchmarks import hold


def test_echo_mode_never_constructs_or_opens_a_robot():
    robot = Robot("echo")
    assert robot.status()["robot_connected"] is False
    assert robot.status()["remote_motion"] is False
    with pytest.raises(ValueError, match="simulation-only"):
        robot.run("hold")
    robot.close()
    assert robot.status()["endpoint_stopped"]


@pytest.mark.parametrize("task", ["hold", "joint1-nudge"])
def test_remote_tasks_run_through_local_mock_and_measured_scorer(fake_ros, tmp_path, task):
    controller = Robot("ros-mock", log_dir=str(tmp_path))
    controller._robot = fake_ros
    assert controller.status()["simulated"] is True
    result = controller.run(task)
    assert result["passed"] is True
    assert result["metrics"]["b601_measured_goal"] == 1.0
    controller.close()


def test_failed_measurement_latches_remote_runs(fake_ros, tmp_path):
    controller = Robot("ros-mock", log_dir=str(tmp_path))
    controller._robot = fake_ros
    controller.status()
    fake_ros._session.frozen = True
    assert controller.run("joint1-nudge")["passed"] is False
    with pytest.raises(RuntimeError, match="restart"):
        controller.run("hold")
    assert controller.status()["fault"]


def test_stop_prevents_the_next_policy_action(fake_ros):
    controller = Robot("ros-mock")
    policy = _GuardedPolicy(controller.stopped)
    policy.bind(fake_ros.info)
    policy.reset(hold().scenes[0])
    controller.stop()
    with pytest.raises(SafetyAbort):
        policy.act(Observation(state={"joint_pos": np.zeros(6)}, state_time=1.0))


def test_physical_backend_is_read_only_and_keeps_profile_state_local(monkeypatch):
    calls = []

    def factory(**options):
        assert set(options) == {"profile", "state_dir"}
        assert options["profile"] == Path("/reviewed/profile.json")
        return SimpleNamespace(
            info=SimpleNamespace(is_simulated=False),
            reset=lambda _: (
                calls.append("read")
                or Observation(
                    state={"joint_pos": np.zeros(1), "arm_joint_pos": np.zeros(6)}, state_time=1.0
                )
            ),
            close=lambda: calls.append("close"),
        )

    monkeypatch.setattr("inspect_robots_b601._realtime.robot.MotorbridgeB601Embodiment", factory)
    controller = Robot(
        "motorbridge-read",
        profile=Path("/reviewed/profile.json"),
        state_dir=Path("/existing/state"),
    )
    assert controller.status()["remote_motion"] is False
    assert controller.status()["state"]["arm_joint_pos"] == [0] * 6
    for task in ("hold", "joint1-nudge", "activate", "recover"):
        with pytest.raises(ValueError, match="simulation-only"):
            controller.run(task)
    controller.stop()
    controller.status()
    assert calls == ["read", "read"]
    controller.close()
    assert calls[-1] == "close"


def test_bad_backend_or_incomplete_physical_config_fails_before_opening():
    with pytest.raises(ValueError):
        Robot("unsupported")
    with pytest.raises(ValueError, match="explicit"):
        Robot("motorbridge-read")
