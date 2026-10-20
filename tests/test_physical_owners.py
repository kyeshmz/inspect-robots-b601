import hashlib
import json
import time
from pathlib import Path

import pytest

from b601_hardware.direct import DirectSession
from b601_hardware.state import StopLedger, inspect_existing_stop_state
from tests.test_direct_core import FakeMotor, FakeOwner


def existing_state(tmp_path):
    legacy = tmp_path / "legacy.json"
    raw = b'{"schema_version":1,"latched":true,"last_stop_unix_s":1}'
    legacy.write_bytes(raw)
    state = tmp_path / "state"
    state.mkdir()
    (state / "stop-state.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "stopped": False,
                "acknowledged_legacy_sha256": hashlib.sha256(raw).hexdigest(),
            }
        )
    )
    return state, legacy


def test_inspection_and_existing_acquisition_never_create_or_archive(tmp_path):
    state, legacy = existing_state(tmp_path)
    before = {p: p.read_bytes() for p in tmp_path.rglob("*") if p.is_file()}
    assert inspect_existing_stop_state(state, legacy)["stopped"] is False
    ledger = StopLedger(state, legacy, existing_only=True)
    assert ledger.stopped is False
    assert {p: p.read_bytes() for p in tmp_path.rglob("*") if p.is_file()} == before
    (state / "stop-state.json").unlink()
    with pytest.raises(FileNotFoundError):
        StopLedger(state, legacy, existing_only=True)
    with pytest.raises(FileNotFoundError):
        _ = ledger.stopped
    assert not (state / "stop-state.json").exists()


def test_existing_owner_observes_local_and_legacy_stops(tmp_path):
    state, legacy = existing_state(tmp_path)
    ledger = StopLedger(state, legacy, existing_only=True)
    value = json.loads((state / "stop-state.json").read_text())
    value["stopped"] = True
    (state / "stop-state.json").write_text(json.dumps(value))
    assert ledger.stopped is True
    value["stopped"] = False
    (state / "stop-state.json").write_text(json.dumps(value))
    assert ledger.stopped is True
    ledger = StopLedger(state, legacy, existing_only=True)
    legacy.write_text('{"schema_version":1,"latched":true,"last_stop_unix_s":2}')
    assert ledger.stopped is True


def test_symlinked_ancestor_is_rejected(tmp_path):
    state, legacy = existing_state(tmp_path)
    alias = tmp_path / "alias"
    alias.symlink_to(state, target_is_directory=True)
    with pytest.raises(RuntimeError, match="symlink"):
        inspect_existing_stop_state(alias, legacy)


def test_deadline_fault_is_retained_after_acknowledged_disable(tmp_path):
    from tests.test_direct_core import FakeLedger

    profile = json.loads(
        (Path(__file__).parent / "fixtures/joint1.synthetic.json").read_text()
    )
    motor, ledger, owner = FakeMotor(), FakeLedger(), FakeOwner()
    session = DirectSession(
        profile,
        tmp_path,
        transport=motor,
        ledger=ledger,
        owner_factory=lambda: owner,
        active_timeout_s=0.04,
        period=0.005,
    )
    session.open()
    session.activate()
    time.sleep(0.08)
    session.close()
    assert session.closed
    assert session.report.primary.operation == "stop"
    assert "deadline" in str(session.report.primary.error)
    assert session.report.disable_acknowledged is True
    assert not motor.enabled
    assert ledger.stopped
    assert owner.closed
