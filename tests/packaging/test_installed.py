"""Run with `python -I` after installing a wheel, without any checkout imports."""

import importlib
import json
import subprocess
import sys
import tempfile
import time
import unittest
from importlib.metadata import distribution
from pathlib import Path
from unittest.mock import patch


class FakeSession:
    def __init__(self, profile, state_dir):
        self.activated = False
        self.closed = False

    def open(self):
        pass

    def observe(self):
        return (0.0,) * 6, time.monotonic()

    def close(self):
        self.closed = True


class InstalledPackageTests(unittest.TestCase):
    def test_distribution_contains_both_adapters_and_shared_core(self):
        dist = distribution("inspect-robots-b601")
        files = {str(path) for path in dist.files}
        expected = {
            "inspect_robots_b601/py.typed",
            "b601_hardware/py.typed",
            "inspect_robots_b601/__init__.py",
            "inspect_robots_b601/embodiment.py",
            "inspect_robots_b601/motorbridge.py",
            "inspect_robots_b601/_ros.py",
            "b601_hardware/__init__.py",
            "b601_hardware/core.py",
            "b601_hardware/direct.py",
            "b601_hardware/state.py",
            "b601_hardware/transport.py",
        }
        self.assertTrue(expected <= files, expected - files)
        script_paths = {
            Path(sys.executable).with_name(point.name).resolve()
            for point in dist.entry_points
            if point.group == "console_scripts"
        }
        for path in files:
            if Path(dist.locate_file(path)).resolve() in script_paths:
                continue
            top = path.split("/")[0]
            self.assertTrue(
                top in ("inspect_robots_b601", "b601_hardware") or top.endswith(".dist-info"), path
            )
        for name in (
            "inspect_robots_b601",
            "b601_hardware.core",
            "b601_hardware.direct",
            "b601_hardware.state",
            "b601_hardware.transport",
        ):
            module = importlib.import_module(name)
            relative = name.replace(".", "/") + (
                "/__init__.py" if name == "inspect_robots_b601" else ".py"
            )
            self.assertEqual(
                Path(module.__file__).resolve(), Path(dist.locate_file(relative)).resolve()
            )

    def test_public_import_and_registry_discovery_are_hardware_free(self):
        code = """
import sys
from inspect_robots_b601 import B601Embodiment, MotorbridgeB601Embodiment
from inspect_robots.registry import registered
factories = registered("embodiment")
assert factories["b601_ros"] is B601Embodiment
assert factories["b601_motorbridge"] is MotorbridgeB601Embodiment
assert not {"rclpy", "motorbridge", "b601_hardware.direct"} & sys.modules.keys()
"""
        subprocess.run([sys.executable, "-I", "-c", code], check=True)

    def test_inspect_cli_discovers_both_plugins(self):
        result = subprocess.run(
            [
                sys.executable,
                "-I",
                str(Path(sys.executable).with_name("inspect-robots")),
                "list",
                "embodiments",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertIn("b601_ros", result.stdout)
        self.assertIn("b601_motorbridge", result.stdout)
        self.assertNotIn("failed to load", result.stderr)

    def test_removed_cloud_transport_is_not_in_the_installed_distribution(self):
        dist = distribution("inspect-robots-b601")
        self.assertNotIn("realtime", dist.metadata.get_all("Provides-Extra", []))
        self.assertFalse(any(point.name == "b601-realtime" for point in dist.entry_points))
        self.assertFalse(any("aiortc" in requirement.lower() for requirement in dist.requires))
        self.assertFalse(
            any(
                "_realtime/" in str(path) or str(path).endswith("/realtime.py")
                for path in dist.files
            )
        )

    def test_ros_conformance_without_ros(self):
        from inspect_robots.conformance import assert_embodiment_conformant

        from inspect_robots_b601 import B601Embodiment

        robot = B601Embodiment()
        self.addCleanup(robot.close)
        assert_embodiment_conformant(robot.info)
        self.assertTrue(robot.info.is_simulated)
        self.assertIsNone(robot._session)
        self.assertNotIn("rclpy", sys.modules)

    def test_registered_tasks_policy_and_scorer_are_available(self):
        from inspect_robots.registry import resolve

        from inspect_robots_b601 import B601Embodiment
        from inspect_robots_b601.benchmarks import JointGoalPolicy, MeasuredJointGoal

        for name in ("b601/hold", "b601/joint1-nudge"):
            task = resolve("task", name)
            self.assertEqual(task.name, name)
            self.assertEqual(task.scorers[0].name, "b601_measured_goal")
        policy = resolve("policy", "b601_scripted")
        self.assertIsInstance(policy, JointGoalPolicy)
        policy.bind(B601Embodiment().info)
        self.assertEqual(policy.info.action_space.shape, (6,))
        self.assertIsInstance(resolve("scorer", "b601_measured_goal"), MeasuredJointGoal)

    def test_motorbridge_requires_explicit_profile(self):
        from inspect_robots_b601 import MotorbridgeB601Embodiment

        with self.assertRaises(TypeError):
            MotorbridgeB601Embodiment()
        requirement = MotorbridgeB601Embodiment.RUNTIME_REQUIREMENTS["motorbridge"]
        self.assertIn("inspect-robots-b601[motorbridge]", requirement)
        self.assertNotIn("scripts/", requirement)

    def test_motorbridge_readonly_reset_uses_installed_core(self):
        from inspect_robots import Scene
        from inspect_robots.conformance import assert_embodiment_conformant

        from inspect_robots_b601 import MotorbridgeB601Embodiment

        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory) / "fake-profile.json"
            profile.write_text(
                json.dumps(
                    {
                        "joints": [
                            {"name": f"joint{i}", "lower": -1.0, "upper": 1.0} for i in range(1, 7)
                        ]
                    }
                )
            )
            robot = MotorbridgeB601Embodiment(profile=str(profile), state_dir=directory)
            assert_embodiment_conformant(robot.info)
            with patch("b601_hardware.direct.DirectSession", FakeSession):
                try:
                    observation = robot.reset(Scene(id="fake", instruction="Observe only"))
                    session = robot._session
                    self.assertFalse(session.activated)
                    self.assertEqual(observation.state["joint_pos"].shape, (1,))
                    self.assertEqual(observation.state["arm_joint_pos"].shape, (6,))
                finally:
                    robot.close()
            self.assertTrue(session.closed)
        self.assertNotIn("motorbridge", sys.modules)
        self.assertNotIn("rclpy", sys.modules)


if __name__ == "__main__":
    unittest.main(verbosity=2)
