"""Software-only physical runner checks. Every motor and provider boundary is fake."""

import argparse
import base64
import hashlib
import json
import subprocess
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from inspect_robots import Action, Scene
from inspect_robots.errors import SafetyAbort
from inspect_robots.log import read_eval_log

from b601_hardware.direct import DirectSession
from inspect_robots_b601 import _physical_http
from inspect_robots_b601 import physical_agent as physical
from tests.test_direct_core import FakeMotor, FakeOwner
from tests.test_physical_owners import existing_state


def configuration(tmp_path, monkeypatch):
    state, legacy = existing_state(tmp_path)
    profile = tmp_path / "profile.json"
    value = json.loads((Path(__file__).parent / "fixtures/joint1.synthetic.json").read_text())
    value.update(test_only=False, legacy_stop_latch=str(legacy))
    profile.write_text(json.dumps(value))
    report = tmp_path / "report.json"
    report.write_text(
        json.dumps(
            {
                "schema": "b601-direct-joint1-test-v1",
                "success": True,
                "cleanup": [],
                "profile_sha256": hashlib.sha256(profile.read_bytes()).hexdigest(),
                "recorded_at_unix_s": time.time(),
                "requested_delta_rad": 0.005,
                "duration_s": 2.0,
                "disable_acknowledged": True,
                "measured_delta_rad": 0.005,
            }
        )
    )
    monkeypatch.setenv("B601_FAKE_KEY", "fake-only-never-a-real-credential")
    args = argparse.Namespace(
        command="run",
        profile=profile,
        state_dir=state,
        tracking_report=report,
        log_dir=tmp_path / "logs",
        task="joint1-nudge",
        provider_url="https://physical.invalid/v1",
        model="fake-model",
        api_key_env="B601_FAKE_KEY",
        allow_physical_motion=True,
        confirm_supported_arm=True,
        confirm_independent_cutoff=True,
        allow_paid_inference=True,
    )
    return args


def argv(args):
    return [args.command] + [
        item
        for name, value in vars(args).items()
        if name != "command"
        for item in (
            [f"--{name.replace('_', '-')}"]
            if value is True
            else [f"--{name.replace('_', '-')}", str(value)]
            if value is not False
            else []
        )
    ]


def reply(name, arguments):
    body = json.dumps(
        {
            "status": "completed",
            "output": [
                {
                    "type": "function_call",
                    "call_id": "fake-call",
                    "name": name,
                    "arguments": json.dumps(arguments),
                }
            ],
        }
    ).encode()
    return json.dumps({"status": 200, "body": base64.b64encode(body).decode()}).encode()


DONE = ("done", {"summary": "Measured outcome", "hindsight": "Software fixture only"})
MOVE = ("move_joints", {"targets": {"joint1": 0.2015}, "note": "Bounded fake nudge"})


class QuantizedMotor(FakeMotor):
    def __init__(self):
        super().__init__()
        self.positions = [0.2, 0.1, -0.1, 0, 0, 0]

    def send_axis(self, axis, position):
        self.writes.append(("position", axis, position))
        self.positions[0] = round(0.2 + round((position - 0.2) / 0.00075) * 0.00075, 8)


def fake_boundaries(monkeypatch, calls, *, on_open=None, on_request=None, motor=None):
    motor, owner = motor or QuantizedMotor(), FakeOwner()
    sessions, requests = [], []
    real_run = subprocess.run
    if on_open:
        original_open = motor.open

        def open_motor():
            result = original_open()
            on_open()
            return result

        motor.open = open_motor

    def session(profile, state_dir, **kwargs):
        result = DirectSession(
            profile, state_dir, transport=motor, owner_factory=lambda: owner, period=0.005, **kwargs
        )
        sessions.append(result)
        return result

    def serve(command, **kwargs):
        if command[0] == "git":
            return real_run(command, **kwargs)
        assert command[1:] == ["-I", "-m", "inspect_robots_b601._physical_http"]
        assert kwargs["timeout"] == 5.0
        request = json.loads(kwargs["input"])
        assert request["url"] == "https://physical.invalid/v1/responses"
        assert request["headers"]["authorization"] == "Bearer fake-only-never-a-real-credential"
        payload = json.loads(base64.b64decode(request["body"]))
        assert payload["model"] == "fake-model"
        requests.append(payload)
        if on_request:
            on_request(len(requests), motor, sessions[-1])
        assert len(requests) <= len(calls), "No corrective provider retry is allowed"
        return subprocess.CompletedProcess(command, 0, reply(*calls[len(requests) - 1]), b"")

    monkeypatch.setattr(physical, "DirectSession", session)
    monkeypatch.setattr(_physical_http.subprocess, "run", serve)
    return motor, owner, sessions, requests


def test_check_is_write_free_sdk_lazy_and_provider_free(tmp_path, monkeypatch, capsys):
    args = configuration(tmp_path, monkeypatch)
    args.command = "check"
    monkeypatch.setattr(physical, "DirectSession", lambda *a, **k: pytest.fail("No session"))
    monkeypatch.setattr("b601_hardware.state.StopLedger", lambda *a, **k: pytest.fail("No ledger"))
    monkeypatch.setattr(
        _physical_http.subprocess, "run", lambda *a, **k: pytest.fail("No subprocess")
    )
    monkeypatch.setattr("socket.socket", lambda *a, **k: pytest.fail("No sockets"))
    before = {p: p.read_bytes() for p in tmp_path.rglob("*") if p.is_file()}
    assert physical.main(argv(args)) == 0
    assert "not readiness" in capsys.readouterr().out
    assert {p: p.read_bytes() for p in tmp_path.rglob("*") if p.is_file()} == before
    assert not args.log_dir.exists()
    assert "motorbridge" not in sys.modules
    assert "rclpy" not in sys.modules


@pytest.mark.parametrize(
    "field,value",
    [
        ("allow_physical_motion", False),
        ("confirm_supported_arm", False),
        ("confirm_independent_cutoff", False),
        ("allow_paid_inference", False),
        ("provider_url", "http://physical.invalid"),
        ("provider_url", "https://key@physical.invalid"),
        ("model", " "),
        ("api_key_env", "MISSING_FAKE_KEY"),
        ("api_key_env", "bad-name"),
    ],
)
def test_configuration_rejection_has_no_effects(tmp_path, monkeypatch, field, value):
    args = configuration(tmp_path, monkeypatch)
    setattr(args, field, value)
    monkeypatch.setattr(physical, "DirectSession", lambda *a, **k: pytest.fail("No session"))
    with pytest.raises(SystemExit) as error:
        physical.main(argv(args))
    assert error.value.code == 2
    assert not args.log_dir.exists()


@pytest.mark.parametrize(
    "mutation",
    [
        "stale",
        "future",
        "mismatch",
        "schema",
        "malformed",
        "failed",
        "test_only",
        "state_missing",
        "profile_missing",
        "report_missing",
        "legacy_missing",
        "state_invalid",
        "retained_stop",
        "profile_symlink",
        "state_symlink",
        "report_symlink",
        "legacy_symlink",
    ],
)
def test_operational_evidence_rejected_without_session(tmp_path, monkeypatch, mutation):
    args = configuration(tmp_path, monkeypatch)
    report = json.loads(args.tracking_report.read_text())
    profile = json.loads(args.profile.read_text())
    if mutation in ("stale", "future"):
        report["recorded_at_unix_s"] = time.time() + (-601 if mutation == "stale" else 10)
    elif mutation == "mismatch":
        report["profile_sha256"] = "wrong"
    elif mutation == "schema":
        report["schema"] = "b601-physical-agent-summary-v1"
    elif mutation == "failed":
        report["success"] = False
    args.tracking_report.write_text(json.dumps(report))
    if mutation == "malformed":
        args.tracking_report.write_text("{")
    if mutation == "test_only":
        profile["test_only"] = True
        args.profile.write_text(json.dumps(profile))
    if mutation == "state_invalid":
        (args.state_dir / "stop-state.json").write_text("{}")
    if mutation == "retained_stop":
        path = args.state_dir / "stop-state.json"
        value = json.loads(path.read_text())
        value["stopped"] = True
        path.write_text(json.dumps(value))
    paths = {
        "profile": args.profile,
        "report": args.tracking_report,
        "legacy": Path(profile["legacy_stop_latch"]),
        "state": args.state_dir / "stop-state.json",
    }
    if mutation.endswith("_missing"):
        paths[mutation.removesuffix("_missing")].unlink()
    if mutation.endswith("_symlink"):
        path = paths[mutation.removesuffix("_symlink")]
        backup = path.with_suffix(".backup")
        path.rename(backup)
        path.symlink_to(backup)
    monkeypatch.setattr(physical, "DirectSession", lambda *a, **k: pytest.fail("No session"))
    with pytest.raises(SystemExit) as error:
        physical.main(argv(args))
    assert error.value.code == 2
    assert not args.log_dir.exists()


@pytest.mark.parametrize(
    "calls,expected_score,success", [((MOVE, DONE), 1.0, True), ((DONE,), 0.0, False)]
)
def test_real_agent_core_worker_scoring_and_logs(
    tmp_path, monkeypatch, calls, expected_score, success
):
    args = configuration(tmp_path, monkeypatch)
    motor, owner, sessions, requests = fake_boundaries(monkeypatch, calls)
    monkeypatch.setenv("INSPECT_ROBOTS_MODEL", "hostile-global-model")
    monkeypatch.setenv("INSPECT_ROBOTS_BASE_URL", "https://hostile.invalid")
    monkeypatch.setenv("OPENROUTER_API_KEY", "must-not-inherit")
    result = physical._run(physical._configuration(args))
    assert result["success"] is success
    assert result["score"] == expected_score
    assert motor.positions == (
        [0.2015, 0.1, -0.1, 0, 0, 0] if success else [0.2, 0.1, -0.1, 0, 0, 0]
    )
    assert result["torque_off_confirmed"] is False
    assert result["hardware"]["disable_acknowledged"] is True
    assert sessions[0].closed and owner.closed and motor.closed and not motor.enabled
    assert all(event[1] == "joint1" for event in motor.writes)
    assert len(requests) == len(calls)
    summary = json.loads((args.log_dir / "physical-agent-summary.json").read_text())
    assert summary == result
    assert summary["schema"] == "b601-physical-agent-summary-v1"
    (log_path,) = [
        p for p in args.log_dir.glob("*.json") if p.name != "physical-agent-summary.json"
    ]
    log = read_eval_log(str(log_path))
    assert log.results.metrics == {"b601_measured_goal": expected_score}
    assert log.results.total_trials == 1
    assert log.eval.embodiment == "b601_motorbridge_joint1"
    assert "fake-only-never-a-real-credential" not in log_path.read_text()


@pytest.mark.parametrize(
    "mutation",
    ["expire", "change_profile", "matching_changed_profile", "delete_state", "invalid_state"],
)
def test_open_cannot_authorize_expired_or_changed_evidence(tmp_path, monkeypatch, mutation):
    args = configuration(tmp_path, monkeypatch)

    def mutate():
        if mutation == "expire":
            value = json.loads(args.tracking_report.read_text())
            value["recorded_at_unix_s"] -= 601
            args.tracking_report.write_text(json.dumps(value))
        if mutation in ("change_profile", "matching_changed_profile"):
            args.profile.write_text(args.profile.read_text() + " ")
        if mutation == "matching_changed_profile":
            value = json.loads(args.tracking_report.read_text())
            value["profile_sha256"] = hashlib.sha256(args.profile.read_bytes()).hexdigest()
            args.tracking_report.write_text(json.dumps(value))
        if mutation == "delete_state":
            (args.state_dir / "stop-state.json").unlink()
        if mutation == "invalid_state":
            (args.state_dir / "stop-state.json").write_text("{}")

    motor, owner, sessions, requests = fake_boundaries(monkeypatch, (), on_open=mutate)
    result = physical._run(physical._configuration(args))
    assert result["success"] is False
    assert requests == []
    assert not any(row[0] in ("enable", "prepare", "position") for row in motor.writes)
    if mutation == "delete_state":
        assert not (args.state_dir / "stop-state.json").exists()


@pytest.mark.parametrize(
    "calls",
    [
        (("move_joints", {"targets": {"joint1": 0.204}, "note": "Known invalid chunk"}),),
        (
            MOVE,
            (
                "move_joints",
                {
                    "targets": {"joint1": 0.2025},
                    "note": "Individually legal but cumulative violation",
                },
            ),
        ),
        (("move_joints", {"targets": {"joint2": 0.1}, "note": "No other axes"}),),
    ],
)
def test_provider_violation_is_terminal_without_corrective_retry(tmp_path, monkeypatch, calls):
    args = configuration(tmp_path, monkeypatch)
    motor, owner, sessions, requests = fake_boundaries(monkeypatch, calls)
    result = physical._run(physical._configuration(args))
    assert not result["success"]
    assert len(requests) == len(calls)
    assert motor.positions[0] in (0.2, 0.2015)
    assert all(0.1994 <= row[2] <= 0.2021 for row in motor.writes if row[0] == "position")
    assert json.loads((args.state_dir / "stop-state.json").read_text())["stopped"] is True
    assert owner.closed and not motor.enabled


def test_full_chunk_and_dispatch_gate_use_one_immutable_corridor(tmp_path, monkeypatch):
    args = configuration(tmp_path, monkeypatch)
    motor, owner, sessions, _ = fake_boundaries(monkeypatch, ())
    robot = physical._Trial(physical._configuration(args))
    try:
        robot.reset(Scene(id="one", instruction="Fixed fake trial"))
        assert robot.info.action_space.semantics.max_step == (0.002,)
        assert robot.corridor.initial == 0.2
        count = len(motor.writes)
        with pytest.raises(SafetyAbort, match="Complete motion chunk"):
            robot.pre_check(np.array([[0.2005], [0.2015], [0.2025]]))
        assert not any(row[0] == "position" and row[2] != 0.2 for row in motor.writes[count:])
        with pytest.raises(SafetyAbort):
            robot.step(Action(np.array([0.2005])))
        with pytest.raises(SafetyAbort, match="reset twice"):
            robot.reset(Scene(id="two", instruction="Forbidden"))
    finally:
        robot.close()
    assert owner.closed


@pytest.mark.parametrize(
    "fault",
    [
        "provider",
        "interrupt",
        "stale",
        "legacy_stop",
        "retained_stop",
        "deadline",
        "late_reply",
        "disable",
        "transport_close",
        "owner_close",
        "summary_write",
        "provider_close",
        "budget",
    ],
)
def test_failure_paths_never_report_success(tmp_path, monkeypatch, fault):
    args = configuration(tmp_path, monkeypatch)
    calls = (MOVE, DONE)
    if fault == "budget":
        calls = (MOVE, MOVE, MOVE)

    def on_request(count, motor, session):
        if fault == "provider":
            raise RuntimeError("Fake provider failure")
        if fault == "interrupt":
            raise KeyboardInterrupt("Fake interrupt")
        if fault == "stale":
            motor.old = True
        if fault == "legacy_stop":
            path = Path(json.loads(args.profile.read_text())["legacy_stop_latch"])
            path.write_text('{"schema_version":1,"latched":true,"last_stop_unix_s":3}')
        if fault == "retained_stop":
            path = args.state_dir / "stop-state.json"
            value = json.loads(path.read_text())
            value["stopped"] = True
            path.write_text(json.dumps(value))
        if fault in ("deadline", "late_reply"):
            before = len(motor.writes)
            time.sleep(0.1)
            assert len(motor.writes) > before
            session._active_deadline = time.monotonic() - 1
            time.sleep(0.03)
            assert not motor.enabled
            if fault == "deadline":
                raise subprocess.TimeoutExpired("fake", 5)
        if fault == "disable" and count == 2:
            motor.fail_disable = True

    motor, owner, sessions, requests = fake_boundaries(monkeypatch, calls, on_request=on_request)
    if fault == "transport_close":
        motor.close = lambda: (_ for _ in ()).throw(RuntimeError("Fake transport close failure"))
    if fault == "owner_close":
        owner.close = lambda: (_ for _ in ()).throw(RuntimeError("Fake owner close failure"))
    if fault == "summary_write":
        monkeypatch.setattr(
            physical,
            "private_write",
            lambda *a: (_ for _ in ()).throw(OSError("Fake result persistence failure")),
        )
    if fault == "provider_close":
        monkeypatch.setattr(
            _physical_http._BoundedTransport,
            "close",
            lambda self: (_ for _ in ()).throw(RuntimeError("Fake provider close failure")),
        )
    result = physical._run(physical._configuration(args))
    assert not result["success"]
    assert result["failures"]
    assert len(requests) <= 3
    if fault in ("disable", "transport_close", "owner_close", "summary_write", "provider_close"):
        assert result["score"] == 1.0
    assert result["torque_off_confirmed"] is False
    if fault == "transport_close":
        assert not owner.closed
    if fault in ("deadline", "late_reply"):
        assert "deadline" in str(result["hardware"])


def test_infeasible_goal_does_not_activate_or_infer(tmp_path, monkeypatch):
    args = configuration(tmp_path, monkeypatch)
    motor = QuantizedMotor()
    motor.positions[0] = 0.999
    _, _, _, requests = fake_boundaries(monkeypatch, (), motor=motor)
    result = physical._run(physical._configuration(args))
    assert not result["success"] and requests == []
    assert not any(row[0] == "enable" for row in motor.writes)


def test_main_reports_verified_result_and_setup_failures(tmp_path, monkeypatch, capsys):
    args = configuration(tmp_path, monkeypatch)
    fake_boundaries(monkeypatch, (DONE,))
    assert physical.main(argv(args)) == 1
    assert '"score": 0.0' in capsys.readouterr().out
    other = tmp_path / "other"
    other.mkdir()
    args = configuration(other, monkeypatch)
    monkeypatch.setattr(
        physical, "_run", lambda config: (_ for _ in ()).throw(ValueError("Fake setup rejection"))
    )
    assert physical.main(argv(args)) == 1
    assert "setup failed" in capsys.readouterr().err


def test_config_cannot_bypass_consent(tmp_path, monkeypatch):
    config = physical._configuration(configuration(tmp_path, monkeypatch))
    config = replace(config, consent=physical._Consent(False, True, True, True))
    with pytest.raises(ValueError, match="consent"):
        physical._run(config)


@pytest.mark.parametrize(
    "failure", ["busy_owner", "worker_close", "retained_primary", "missing_sidecar"]
)
def test_owner_and_persistence_failure_after_measured_success(tmp_path, monkeypatch, failure):
    args = configuration(tmp_path, monkeypatch)
    motor, owner, sessions, requests = fake_boundaries(monkeypatch, (MOVE, DONE))
    original_create = physical.DirectSession

    def create(*args, **kwargs):
        session = original_create(*args, **kwargs)
        if failure == "busy_owner":
            session.owner_factory = lambda: (_ for _ in ()).throw(RuntimeError("Busy CAN owner"))
        original_close = session.close

        def close():
            if failure == "retained_primary":
                session.stop("Authentic terminal fault after the measured score")
            if failure == "worker_close":
                session._stop.set()
                session._thread.join(timeout=2)
                session._worker_close_failed = True
                raise RuntimeError("Fake unconfirmed worker termination")
            original_close()

        session.close = close
        return session

    monkeypatch.setattr(physical, "DirectSession", create)
    if failure == "missing_sidecar":
        original_eval = physical.eval

        def evaluate(*args, **kwargs):
            logs = original_eval(*args, **kwargs)
            for path in Path(kwargs["log_dir"]).rglob("*.jsonl"):
                path.unlink()
            return logs

        monkeypatch.setattr(physical, "eval", evaluate)
    result = physical._run(physical._configuration(args))
    assert not result["success"]
    if failure == "busy_owner":
        assert requests == [] and not any(row[0] == "enable" for row in motor.writes)
    else:
        assert result["score"] == 1.0
    if failure == "worker_close":
        assert result["hardware"] is None
        assert any(item["operation"] == "receipt.read" for item in result["failures"])
        assert not owner.closed
        sessions[0]._worker_close_failed = False
        DirectSession.close(sessions[0])
    if failure == "retained_primary":
        assert "Authentic terminal fault" in str(result["hardware"])


def test_adapter_retains_session_after_failed_close(tmp_path, monkeypatch):
    args = configuration(tmp_path, monkeypatch)
    motor, owner, sessions, _ = fake_boundaries(monkeypatch, ())
    robot = physical._Trial(physical._configuration(args))
    robot.reset(Scene(id="one", instruction="Software fixture"))
    original_close = motor.close
    motor.close = lambda: (_ for _ in ()).throw(RuntimeError("Fake close failure"))
    with pytest.raises(RuntimeError):
        robot.close()
    assert robot._session is sessions[0]
    assert robot.final_report.primary.operation == "close"
    assert not robot.cleanup_complete and not owner.closed
    motor.close = original_close
    robot.close()
    assert robot.cleanup_complete
    assert robot.final_report.primary.operation == "close"


def test_raw_out_of_profile_target_is_vetoed_before_agent_clipping(tmp_path, monkeypatch):
    args = configuration(tmp_path, monkeypatch)
    motor, owner, sessions, requests = fake_boundaries(
        monkeypatch, (("move_joints", {"targets": {"joint1": 100}, "note": "Malicious"}),)
    )
    result = physical._run(physical._configuration(args))
    assert not result["success"] and len(requests) == 1
    assert motor.positions[0] == 0.2
    assert not any(row[0] == "position" and row[2] != 0.2 for row in motor.writes)
