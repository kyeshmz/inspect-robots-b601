import hashlib
import json
import tempfile
import time
import unittest
from pathlib import Path

from b601_hardware.core import PositionSample, State
from b601_hardware.direct import DirectSession, validate_tracking_report

ROOT = Path(__file__).resolve().parents[1]


class FakeMotor:
    def __init__(self):
        self.positions = [0.0] * 6
        self.writes = []
        self.enabled = False
        self.closed = False
        self.old = False
        self.fail_disable = False

    def open(self):
        return self

    def read_positions(self):
        stamp = time.monotonic() - (1.0 if self.old else 0)
        return PositionSample(tuple(self.positions), started_at=stamp, finished_at=stamp)

    def prepare_axis(self, axis, hold):
        self.writes.append(("prepare", axis, hold))

    def enable_axis(self, axis):
        self.enabled = True
        self.writes.append(("enable", axis))

    def send_axis(self, axis, position):
        self.positions[0] = position
        self.writes.append(("position", axis, position))

    def disable_axis(self, axis):
        if self.fail_disable:
            raise RuntimeError("disable not acknowledged")
        self.enabled = False
        self.writes.append(("disable", axis))

    def close(self):
        self.closed = True


class FakeLedger:
    def __init__(self, stopped=False):
        self.stopped = stopped
        self.events = []

    def snapshot(self):
        return {"stopped": self.stopped, "hash": "unchanged"}

    def stop(self, reason, details=None):
        self.stopped = True
        self.events.append(reason)

    def acknowledge(self, details=None):
        self.stopped = False


class FakeOwner:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class DirectTests(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((ROOT / "tests/fixtures/joint1.synthetic.json").read_text())
        self.motor, self.ledger, self.owner = FakeMotor(), FakeLedger(), FakeOwner()
        self.session = DirectSession(self.profile, "unused", transport=self.motor,
                                     ledger=self.ledger, owner_factory=lambda: self.owner, period=0.005)
        self.addCleanup(self.session.close)

    def test_open_observe_and_close_do_not_enable(self):
        self.session.open()
        self.assertEqual(self.session.observe()[0], (0.0,) * 6)
        self.assertEqual(self.motor.writes, [])
        self.session.close()
        self.assertEqual(self.motor.writes, [])
        self.assertTrue(self.owner.closed)

    def test_direct_goal_and_hold_continue_during_policy_pause(self):
        self.session.open()
        self.session.activate()
        self.session.move((0.0015, 0, 0, 0, 0, 0), 0.3)
        self.assertLessEqual(abs(self.motor.positions[0] - 0.0015), 0.0006)
        count = len(self.motor.writes)
        time.sleep(0.05)  # Simulates time spent outside step() awaiting inference.
        self.assertGreater(len(self.motor.writes), count + 1)
        self.session.close()
        self.assertFalse(self.motor.enabled)
        self.assertTrue(all(row[1] == "joint1" for row in self.motor.writes))

    def test_ledger_stop_is_not_automatically_cleared(self):
        self.ledger.stopped = True
        self.session.open()
        with self.assertRaises(RuntimeError):
            self.session.activate()
        self.assertEqual(self.motor.writes, [])
        self.session.recover()
        self.assertEqual(self.session.core.state, State.INACTIVE)
        self.assertFalse(self.motor.enabled)

    def test_external_stop_disables_and_blocks_new_goals(self):
        self.session.open()
        self.session.activate()
        self.ledger.stopped = True
        deadline = time.monotonic() + 0.5
        while self.motor.enabled and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertFalse(self.motor.enabled)
        with self.assertRaises(RuntimeError):
            self.session.move((0.001, 0, 0, 0, 0, 0), 0.3)

    def test_stale_feedback_stops_worker_and_never_reconnects(self):
        self.session.open()
        self.session.activate()
        self.motor.old = True
        deadline = time.monotonic() + 0.5
        while self.motor.enabled and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertFalse(self.motor.enabled)
        self.assertTrue(self.ledger.stopped)
        with self.assertRaises(RuntimeError):
            self.session.observe()
        # Keep this test's close report visible instead of treating failure as torque-off.
        self.motor.old = False
        with self.assertRaisesRegex(RuntimeError, "cleanup has failures"):
            self.session.close()

    def test_tracking_report_must_be_successful_recent_and_same_profile(self):
        profile_path = ROOT / "tests/fixtures/joint1.synthetic.json"
        report = {"schema": "b601-direct-joint1-test-v1", "success": True, "cleanup": [],
                  "profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
                  "requested_delta_rad": 0.005, "duration_s": 2.0,
                  "measured_delta_rad": 0.005,
                  "disable_acknowledged": True, "recorded_at_unix_s": 1000}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trial.json"
            path.write_text(json.dumps(report))
            validate_tracking_report(path, profile_path, now=lambda: 1100)
            for changes in ({"success": False}, {"recorded_at_unix_s": 0},
                            {"profile_sha256": "different"}, {"disable_acknowledged": False},
                            {"measured_delta_rad": 0.000213}):
                path.write_text(json.dumps({**report, **changes}))
                with self.assertRaises(ValueError):
                    validate_tracking_report(path, profile_path, now=lambda: 1100)

    def test_cleanup_disable_failure_is_durably_latched(self):
        self.session.open()
        self.session.activate()
        self.motor.fail_disable = True
        with self.assertRaisesRegex(RuntimeError, "cleanup has failures"):
            self.session.close()
        self.assertTrue(self.ledger.stopped)
        self.assertTrue(self.ledger.events)
        self.assertIs(self.session.core.report.disable_acknowledged, False)


if __name__ == "__main__":
    unittest.main()
