import json
import tempfile
import unittest
from pathlib import Path

from b601_hardware.state import StopLedger


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
        (archived,) = (self.root / "state").glob("legacy-*.json")
        self.assertEqual(archived.read_bytes(), self.legacy.read_bytes())

    def test_new_legacy_stop_invalidates_acknowledgement(self):
        self.ledger.acknowledge()
        self.legacy.write_text(
            json.dumps({"schema_version": 1, "latched": True, "last_stop_unix_s": 2.0})
        )
        self.assertTrue(self.ledger.stopped)

    def test_change_during_recovery_is_not_silently_acknowledged(self):
        self.legacy.write_text(
            json.dumps({"schema_version": 1, "latched": True, "last_stop_unix_s": 2.0})
        )
        with self.assertRaisesRegex(RuntimeError, "changed during recovery"):
            self.ledger.acknowledge()

    def test_local_stop_survives_restart(self):
        self.ledger.acknowledge()
        self.ledger.stop(
            "feedback lost", {"primary": "timeout", "cleanup": ["disable unconfirmed"]}
        )
        restored = StopLedger(self.root / "state", self.legacy)
        self.assertTrue(restored.stopped)
        self.assertEqual(restored.snapshot()["details"]["primary"], "timeout")

    def test_missing_or_invalid_legacy_file_is_fail_closed(self):
        for value in (
            "{}",
            "not json",
            '{"schema_version":1,"latched":false,"last_stop_unix_s":0}',
        ):
            self.legacy.write_text(value)
            with self.assertRaises((ValueError, RuntimeError)):
                _ = self.ledger.stopped
        self.legacy.unlink()
        with self.assertRaises(FileNotFoundError):
            _ = self.ledger.stopped


if __name__ == "__main__":
    unittest.main()
