import json
from pathlib import Path
import tempfile
import unittest

from inspect_robots_b601.hardware.state import StopLedger


class StopLedgerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.legacy = self.root / "legacy.json"
        self.original = json.dumps({"schema_version": 1, "latched": True, "last_stop_unix_s": 1.0})
        self.legacy.write_text(self.original)
        self.ledger = StopLedger(self.root / "state", self.legacy)

    def test_recovery_is_persistent_and_leaves_legacy_evidence_intact(self):
        self.assertTrue(self.ledger.stopped)
        self.ledger.acknowledge({"fresh_feedback": True})
        self.assertFalse(StopLedger(self.root / "state", self.legacy).stopped)
        self.assertEqual(self.legacy.read_text(), self.original)
        archived, = (self.root / "state").glob("legacy-*.json")
        self.assertEqual(archived.read_bytes(), self.legacy.read_bytes())

    def test_new_legacy_stop_invalidates_acknowledgement(self):
        self.ledger.acknowledge()
        self.legacy.write_text(json.dumps({"schema_version": 1, "latched": True, "last_stop_unix_s": 2.0}))
        self.assertTrue(self.ledger.stopped)

    def test_change_during_recovery_is_not_silently_acknowledged(self):
        self.legacy.write_text(json.dumps({"schema_version": 1, "latched": True, "last_stop_unix_s": 2.0}))
        with self.assertRaisesRegex(RuntimeError, "changed during recovery"):
            self.ledger.acknowledge()

    def test_local_stop_survives_restart(self):
        self.ledger.acknowledge()
        self.ledger.stop("feedback lost", {"primary": "timeout", "cleanup": ["disable unconfirmed"]})
        restored = StopLedger(self.root / "state", self.legacy)
        self.assertTrue(restored.stopped)
        self.assertEqual(restored.snapshot()["details"]["primary"], "timeout")

    def test_missing_or_invalid_legacy_file_is_fail_closed(self):
        for value in ("{}", "not json", '{"schema_version":1,"latched":false,"last_stop_unix_s":0}'):
            self.legacy.write_text(value)
            with self.assertRaises((ValueError, RuntimeError)):
                _ = self.ledger.stopped
        self.legacy.unlink()
        with self.assertRaises(FileNotFoundError):
            _ = self.ledger.stopped

    def test_without_another_controller_the_ledger_stands_alone(self):
        ledger = StopLedger(self.root / "standalone")
        self.assertTrue(ledger.stopped)  # A new arm still starts latched.
        ledger.acknowledge({"fresh_feedback": True})
        restored = StopLedger(self.root / "standalone")
        self.assertFalse(restored.stopped)
        self.assertIsNone(restored.snapshot()["legacy_stop_latch"])
        self.assertEqual(list((self.root / "standalone").glob("legacy-*.json")), [])
        restored.stop("feedback lost")
        self.assertTrue(StopLedger(self.root / "standalone").stopped)

    def test_adding_another_controller_later_requires_a_new_recovery(self):
        ledger = StopLedger(self.root / "standalone")
        ledger.acknowledge()
        self.assertTrue(StopLedger(self.root / "standalone", self.legacy).stopped)


if __name__ == "__main__":
    unittest.main()
