"""The evaluation runner cannot select hardware or silently spend on inference."""

from types import SimpleNamespace

import pytest

from inspect_robots_b601.evaluate import main


@pytest.mark.parametrize(
    "args",
    [
        ["agent-sim"],
        ["agent-sim", "--model", "openai/test"],
        ["agent-sim", "--allow-paid-inference"],
        ["ros-mock", "--allow-paid-inference"],
        ["ros-mock", "--embodiment", "b601_motorbridge"],
        ["physical"],
    ],
)
def test_invalid_or_unacknowledged_stages_never_construct_a_robot(monkeypatch, args):
    monkeypatch.setattr(
        "inspect_robots_b601.evaluate.B601Embodiment",
        lambda **_: pytest.fail("Must reject before constructing a robot"),
    )
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2


@pytest.mark.parametrize("domain,local", [("62", "1"), ("61", "0"), ("", "")])
def test_wrong_ros_environment_is_rejected(monkeypatch, domain, local):
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", local)
    monkeypatch.setattr(
        "inspect_robots_b601.evaluate.B601Embodiment",
        lambda **_: pytest.fail("No connection outside isolated mock domain"),
    )
    with pytest.raises(SystemExit):
        main(["ros-mock"])


@pytest.mark.parametrize("variable", ["ROS_DISCOVERY_SERVER", "ROS_STATIC_PEERS"])
def test_remote_discovery_is_rejected(monkeypatch, variable):
    monkeypatch.setenv("ROS_DOMAIN_ID", "61")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv(variable, "remote-host")
    with pytest.raises(SystemExit):
        main(["ros-mock"])


@pytest.mark.parametrize("metric,expected", [(1.0, 0), (0.0, 1)])
def test_runner_checks_the_score_not_just_execution_status(monkeypatch, metric, expected):
    from inspect_robots_b601 import B601Embodiment

    monkeypatch.setenv("ROS_DOMAIN_ID", "61")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.delenv("ROS_DISCOVERY_SERVER", raising=False)
    monkeypatch.delenv("ROS_STATIC_PEERS", raising=False)
    calls = []
    robot = B601Embodiment()
    monkeypatch.setattr(robot, "close", lambda: calls.append("close"))

    def construct(**options):
        assert options["simulated"] is True
        return robot

    monkeypatch.setattr("inspect_robots_b601.evaluate.B601Embodiment", construct)
    monkeypatch.setattr(
        "inspect_robots_b601.evaluate.eval",
        lambda *_, **__: [
            SimpleNamespace(
                status="success", results=SimpleNamespace(metrics={"b601_measured_goal": metric})
            )
        ],
    )
    assert main(["ros-mock"]) == expected
    assert calls == ["close"]
