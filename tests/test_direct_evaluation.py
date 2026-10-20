"""Run the actual independent HOLD worker and Inspect loop with injected fake motors."""

import hashlib
import json
import time
from dataclasses import replace
from pathlib import Path

import pytest
from inspect_robots import eval
from inspect_robots.approver import DeltaLimitApprover

from b601_hardware.direct import DirectSession, main
from inspect_robots_b601 import MotorbridgeB601Embodiment
from inspect_robots_b601.benchmarks import JointGoalPolicy, hold, joint1_nudge
from tests.test_direct_core import FakeLedger, FakeMotor, FakeOwner

PROFILE = Path(__file__).parent / "fixtures/joint1.synthetic.json"


@pytest.mark.parametrize("task_factory", [hold, joint1_nudge])
def test_inspect_evaluation_through_real_direct_core(monkeypatch, tmp_path, task_factory):
    motor, ledger, owner = FakeMotor(), FakeLedger(), FakeOwner()

    def factory(profile, state_dir):
        return DirectSession(
            profile,
            state_dir,
            transport=motor,
            ledger=ledger,
            owner_factory=lambda: owner,
            period=0.005,
        )

    report = tmp_path / "synthetic-report.json"
    report.write_text(
        json.dumps(
            {
                "schema": "b601-direct-joint1-test-v1",
                "success": True,
                "cleanup": [],
                "profile_sha256": hashlib.sha256(PROFILE.read_bytes()).hexdigest(),
                "recorded_at_unix_s": time.time(),
                "requested_delta_rad": 0.005,
                "duration_s": 2.0,
                "disable_acknowledged": True,
                "measured_delta_rad": 0.005,
            }
        )
    )
    robot = MotorbridgeB601Embodiment(
        profile=str(PROFILE),
        state_dir=str(tmp_path),
        allow_motion=True,
        activate_on_reset=True,
        operator_supported_arm=True,
        tracking_report=str(report),
    )
    robot.info = replace(robot.info, name="fake-b601-motorbridge", is_simulated=True)
    monkeypatch.setattr("b601_hardware.direct.DirectSession", factory)
    try:
        (log,) = eval(
            task_factory(),
            JointGoalPolicy(),
            robot,
            log_dir=str(tmp_path / "logs"),
            approver=DeltaLimitApprover(robot.info.action_space, max_delta=0.002),
        )
        assert log.status == "success"
        assert log.results.metrics["b601_measured_goal"] == 1.0
    finally:
        robot.close()
    assert not motor.enabled
    assert owner.closed
    assert all(event[1] == "joint1" for event in motor.writes)


def test_synthetic_profile_cannot_open_a_physical_session():
    profile = json.loads(PROFILE.read_text())
    with pytest.raises(ValueError, match="physical CAN is forbidden"):
        DirectSession(profile, "unused")


@pytest.mark.parametrize("command", ["recover", "test-joint1"])
def test_operator_actions_require_acknowledgement_before_session_creation(
    monkeypatch, command, tmp_path
):
    monkeypatch.setattr(
        "b601_hardware.direct.DirectSession", lambda *_: pytest.fail("No physical session")
    )
    with pytest.raises(SystemExit) as error:
        main([command, "--profile", str(PROFILE), "--state-dir", str(tmp_path)])
    assert error.value.code == 2


def test_commissioning_cli_requires_explicit_profile_and_state_dir():
    for args in (["read"], ["read", "--profile", str(PROFILE)]):
        with pytest.raises(SystemExit) as error:
            main(args)
        assert error.value.code == 2
