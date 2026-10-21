"""CAN ownership locks and interface checks; real lock files in a temporary directory, no CAN."""

import fcntl
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from inspect_robots_b601.hardware import bus

READY = {
    "channel": "can0",
    "type": 280,
    "flags": ["UP"],
    "bitrate": 1_000_000,
    "ctrlmode": [],
    "can_state": "ERROR-ACTIVE",
    "driver": "gs_usb",
    "usb_vendor": "1d50",
    "usb_product": "606f",
}


class ChannelTests(unittest.TestCase):
    def test_channel_must_be_a_plain_interface_name(self):
        self.assertEqual(bus.channel_name({"channel": "can0"}), "can0")
        for value in (None, "", "../can0", "can 0", "a" * 16, ".", 0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                bus.channel_name({"channel": value})

    def test_default_lock_is_per_channel_and_can_be_pinned(self):
        self.assertEqual(bus.shared_lock_path({"channel": "can2"}).name, "b601-can2.lock")
        with patch.object(bus.os, "access", return_value=False):
            self.assertEqual(bus.shared_lock_path({"channel": "can2"}), Path("/tmp/b601-can2.lock"))
        self.assertEqual(
            bus.shared_lock_path({"channel": "can2", "can_lock": "/run/lock/site.lock"}),
            Path("/run/lock/site.lock"),
        )


class InterfaceTests(unittest.TestCase):
    def test_any_ready_adapter_passes_an_unpinned_profile(self):
        self.assertEqual(bus.interface_problems(READY, {"channel": "can0"}), [])
        self.assertEqual(
            bus.interface_problems(
                {**READY, "usb_vendor": None, "usb_product": None}, {"channel": "can0"}
            ),
            [],
        )

    def test_pins_are_enforced_only_when_present(self):
        pinned = {"channel": "can0", "driver": "peak_usb", "usb_vendor": "0c72"}
        problems = bus.interface_problems(READY, pinned)
        self.assertEqual(len(problems), 2)
        self.assertTrue(all("the profile pins" in problem for problem in problems))

    def test_missing_interface_is_named(self):
        with self.assertRaisesRegex(FileNotFoundError, "can-missing does not exist"):
            bus.interface_metadata("can-missing")


class LockTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.profile = {
            "channel": "can0",
            "can_lock": str(self.root / "shared.lock"),
            "native_owner_lock": str(self.root / "native.lock"),
        }

    def held_elsewhere(self, path):
        fd = os.open(path, os.O_CREAT | os.O_RDWR)
        self.addCleanup(os.close, fd)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def claim(self, profile=None, info=READY):
        return bus.BusClaim(profile or self.profile, metadata_reader=lambda channel: info)

    def test_claim_holds_both_locks_until_closed(self):
        claim = self.claim()
        with self.assertRaisesRegex(BlockingIOError, "another B601 session holds"):
            bus.ownership_lock(self.root / "shared.lock")
        with self.assertRaisesRegex(RuntimeError, "owned by another controller"):
            bus.claim_native_bus(self.root / "native.lock")
        claim.close()
        self.claim().close()

    def test_busy_shared_lock_releases_the_native_lock(self):
        self.held_elsewhere(self.root / "shared.lock")
        with self.assertRaises(BlockingIOError):
            self.claim()
        bus.claim_native_bus(self.root / "native.lock").close()

    def test_without_another_controller_only_the_shared_lock_is_taken(self):
        profile = {"channel": "can0", "can_lock": str(self.root / "shared.lock")}
        claim = self.claim(profile)
        self.assertEqual(sorted(path.name for path in self.root.iterdir()), ["shared.lock"])
        claim.close()

    def test_unready_interface_is_refused_and_releases_both_locks(self):
        with self.assertRaisesRegex(RuntimeError, "can0 is not ready: interface is down"):
            self.claim(info={**READY, "flags": []})
        pinned = {**self.profile, "driver": "peak_usb"}
        with self.assertRaisesRegex(
            RuntimeError, "driver is 'gs_usb' but the profile pins 'peak_usb'"
        ):
            self.claim(pinned)
        self.claim().close()


if __name__ == "__main__":
    unittest.main()
