"""Run with `python -I` after installing a wheel, without any checkout imports."""

import importlib
import subprocess
import sys
import tempfile
import unittest
from importlib.metadata import distribution
from pathlib import Path
from unittest.mock import patch


class FakeSession:
    def __init__(self, profile, state_dir):
        self.activated = False
        self.closed = False
        self._home = None

    def open(self):
        pass

    def observe(self):
        return (0.0,) * 7

    def close(self):
        self.closed = True


class InstalledPackageTests(unittest.TestCase):
    def test_extras_are_motorbridge_agent_and_dev(self):
        dist = distribution("inspect-robots-b601")
        self.assertEqual(
            sorted(dist.metadata.get_all("Provides-Extra")), ["agent", "dev", "motorbridge"]
        )
        self.assertEqual(dist.metadata["License-Expression"], "MIT")

    def test_wheel_holds_one_package_with_its_defaults(self):
        dist = distribution("inspect-robots-b601")
        files = {str(path) for path in dist.files}
        expected = {
            "inspect_robots_b601/__init__.py",
            "inspect_robots_b601/embodiment.py",
            "inspect_robots_b601/preflight.py",
            "inspect_robots_b601/cli.py",
            "inspect_robots_b601/py.typed",
            "inspect_robots_b601/defaults/b601-rs.json",
            "inspect_robots_b601/defaults/agent.ini",
            "inspect_robots_b601/hardware/__init__.py",
            "inspect_robots_b601/hardware/session.py",
            "inspect_robots_b601/hardware/bus.py",
            "inspect_robots_b601/hardware/state.py",
        }
        self.assertTrue(expected <= files, expected - files)
        scripts = {"../../../bin/inspect-robots-b601", "../../../bin/inspect-robots-b601-preflight"}
        for path in files - scripts:
            top = path.split("/")[0]
            self.assertTrue(top == "inspect_robots_b601" or top.endswith(".dist-info"), path)
        for name in (
            "inspect_robots_b601",
            "inspect_robots_b601.embodiment",
            "inspect_robots_b601.hardware.session",
        ):
            module = importlib.import_module(name)
            relative = name.replace(".", "/") + (
                "/__init__.py" if name == "inspect_robots_b601" else ".py"
            )
            self.assertEqual(
                Path(module.__file__).resolve(), Path(dist.locate_file(relative)).resolve()
            )

    def test_commands_are_installed_and_hardware_free(self):
        scripts = {
            entry.name: entry.value
            for entry in distribution("inspect-robots-b601").entry_points
            if entry.group == "console_scripts"
        }
        self.assertEqual(
            scripts,
            {
                "inspect-robots-b601": "inspect_robots_b601.cli:main",
                "inspect-robots-b601-preflight": "inspect_robots_b601.preflight:main",
            },
        )
        help_text = subprocess.run(
            [str(Path(sys.executable).with_name("inspect-robots-b601")), "--help"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        self.assertIn("--confirm-supported-arm", help_text)
        preflight = subprocess.run(
            [str(Path(sys.executable).with_name("inspect-robots-b601-preflight")), "--dry-run"],
            capture_output=True,
            text=True,
        )
        self.assertIn("b601: conformant (no issues)", preflight.stdout)

    def test_registry_discovery_is_hardware_free(self):
        code = """
import sys
from inspect_robots.registry import registered
from inspect_robots_b601 import B601Embodiment, __version__
assert registered("embodiment")["b601"] is B601Embodiment
assert B601Embodiment().info.name == "b601"
assert __version__ != "0.0.0+unknown"
assert "motorbridge" not in sys.modules
"""
        subprocess.run([sys.executable, "-I", "-c", code], check=True)

    def test_inspect_cli_lists_and_doctors_b601(self):
        inspect_robots = str(Path(sys.executable).with_name("inspect-robots"))
        listed = subprocess.run(
            [sys.executable, "-I", inspect_robots, "list", "embodiments"],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertIn("b601", listed.stdout)
        self.assertNotIn("failed to load", listed.stderr)
        doctor = subprocess.run(
            [sys.executable, "-I", inspect_robots, "doctor", "--embodiment", "b601"],
            capture_output=True,
            text=True,
        )
        self.assertIn("b601: conformant (no issues)", doctor.stdout + doctor.stderr)

    def test_readonly_reset_uses_the_installed_session(self):
        from inspect_robots import Scene

        from inspect_robots_b601 import B601Embodiment

        with tempfile.TemporaryDirectory() as directory:
            robot = B601Embodiment(state_dir=str(Path(directory) / "state"))
            with patch("inspect_robots_b601.hardware.session.ArmSession", FakeSession), robot:
                observation = robot.reset(Scene(id="fake", instruction="Observe only"))
                session = robot._session
                self.assertFalse(session.activated)
                self.assertEqual(observation.state["joint_pos"].shape, (7,))
            self.assertTrue(session.closed)
        self.assertNotIn("motorbridge", sys.modules)


if __name__ == "__main__":
    unittest.main(verbosity=2)
