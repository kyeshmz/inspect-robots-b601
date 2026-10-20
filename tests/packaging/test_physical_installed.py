"""Installed 0.5.0 CLI acceptance. All evidence is synthetic, never operational."""

import argparse
import contextlib
import hashlib
import importlib
import io
import json
import os
import runpy
import subprocess
import sys
import tempfile
import time
import unittest
from importlib.metadata import distribution, version
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]


def require_installed():
    for package, expected, module_name in (
        ("inspect-robots-b601", "0.5.0", "inspect_robots_b601"),
        ("inspect-robots", "0.60.0", "inspect_robots"),
        ("inspect-robots-agent", "0.28.0", "inspect_robots_agent"),
    ):
        assert version(package) == expected
        dist = distribution(package)
        origin = Path(importlib.import_module(module_name).__file__).resolve()
        assert origin == Path(dist.locate_file(f"{module_name}/__init__.py")).resolve()
        assert origin.is_relative_to(Path(sys.prefix).resolve())
        assert (
            not json.loads(dist.read_text("direct_url.json") or "{}")
            .get("dir_info", {})
            .get("editable")
        )
        print(f"{package}=={expected} from {origin}", flush=True)
    assert (3, 10) <= sys.version_info[:2] < (3, 13)
    assert int(version("numpy").split(".")[0]) < 2
    assert version("motorbridge") == "0.5.6"


class PhysicalInstalledTests(unittest.TestCase):
    artifacts_dir = None

    def test_help_does_not_import_optional_agent_or_sdk(self):
        code = """
import importlib.abc
import runpy
import sys
class RejectOptional(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in {"httpx", "inspect_robots_agent", "motorbridge", "rclpy"}:
            raise AssertionError("Optional import attempted: " + fullname)
sys.meta_path.insert(0, RejectOptional())
sys.argv = [sys.argv[1], "--help"]
runpy.run_path(sys.argv[0], run_name="__main__")
"""
        result = subprocess.run(
            [
                sys.executable,
                "-I",
                "-c",
                code,
                str(Path(sys.executable).with_name("b601-physical-agent")),
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        (self.artifacts_dir / "optional-free-help.txt").write_text(result.stdout + result.stderr)

    def test_installed_help_check_and_rejections_have_zero_external_effects(self):

        entry = str(Path(sys.executable).with_name("b601-physical-agent"))
        with tempfile.TemporaryDirectory(prefix="fake-physical-") as temporary:
            root = Path(temporary)
            profile = json.loads((ROOT / "tests/fixtures/joint1.synthetic.json").read_text())
            legacy = root / "synthetic-legacy.json"
            legacy.write_text('{"schema_version":1,"latched":true,"last_stop_unix_s":1}')
            profile.update(test_only=False, legacy_stop_latch=str(legacy))
            profile_path = root / "synthetic-profile.json"
            profile_path.write_text(json.dumps(profile))
            state = root / "synthetic-state"
            state.mkdir()
            (state / "stop-state.json").write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "stopped": False,
                        "acknowledged_legacy_sha256": hashlib.sha256(
                            legacy.read_bytes()
                        ).hexdigest(),
                    }
                )
            )
            report = root / "synthetic-report.json"
            report.write_text(
                json.dumps(
                    {
                        "schema": "b601-direct-joint1-test-v1",
                        "success": True,
                        "cleanup": [],
                        "profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
                        "recorded_at_unix_s": time.time(),
                        "requested_delta_rad": 0.005,
                        "duration_s": 2.0,
                        "disable_acknowledged": True,
                        "measured_delta_rad": 0.005,
                    }
                )
            )
            common = [
                "--profile",
                str(profile_path),
                "--state-dir",
                str(state),
                "--tracking-report",
                str(report),
                "--log-dir",
                str(root / "must-not-exist"),
                "--task",
                "joint1-nudge",
                "--provider-url",
                "https://physical.invalid/v1",
                "--model",
                "fake-explicit-model",
                "--api-key-env",
                "EXPLICIT_FAKE_KEY",
            ]
            before = {
                str(p.relative_to(root)): p.read_bytes() for p in root.rglob("*") if p.is_file()
            }
            output = io.StringIO()
            with (
                patch("socket.socket", side_effect=AssertionError("Sockets forbidden")),
                patch(
                    "b601_hardware.direct.DirectSession",
                    side_effect=AssertionError("Session forbidden"),
                ),
                patch(
                    "inspect_robots_b601.physical_agent.DirectSession",
                    side_effect=AssertionError("Session forbidden"),
                ),
                patch(
                    "b601_hardware.state.StopLedger",
                    side_effect=AssertionError("Ledger construction forbidden"),
                ),
                patch(
                    "subprocess.run", side_effect=AssertionError("Provider subprocess forbidden")
                ),
                patch.dict(
                    os.environ,
                    {
                        "EXPLICIT_FAKE_KEY": "synthetic-only",
                        "INSPECT_ROBOTS_MODEL": "hostile-default",
                        "OPENROUTER_API_KEY": "must-not-inherit",
                    },
                    clear=True,
                ),
                contextlib.redirect_stdout(output),
                contextlib.redirect_stderr(output),
            ):
                for args, expected in (
                    (["--help"], 0),
                    (["check"] + common, 0),
                    (["run"] + common, 2),
                    (["run"], 2),
                ):
                    with (
                        patch.object(sys, "argv", [entry, *args]),
                        self.assertRaises(SystemExit) as caught,
                    ):
                        runpy.run_path(entry, run_name="__main__")
                    self.assertEqual(caught.exception.code, expected)
                os.environ.pop("EXPLICIT_FAKE_KEY")
                with (
                    patch.object(sys, "argv", [entry, "check", *common]),
                    self.assertRaises(SystemExit) as caught,
                ):
                    runpy.run_path(entry, run_name="__main__")
                self.assertEqual(caught.exception.code, 2)
            self.assertEqual(
                before,
                {str(p.relative_to(root)): p.read_bytes() for p in root.rglob("*") if p.is_file()},
            )
            self.assertFalse((root / "must-not-exist").exists())
            self.assertNotIn("motorbridge", sys.modules)
            self.assertNotIn("rclpy", sys.modules)
            self.assertIn("not readiness", output.getvalue())
            (self.artifacts_dir / "installed-cli.txt").write_text(output.getvalue())

    def test_real_installed_agent_direct_core_and_failure_regressions(self):
        command = [
            sys.executable,
            "-I",
            "-m",
            "pytest",
            str(ROOT / "tests/test_physical_agent.py"),
            str(ROOT / "tests/test_physical_http.py"),
            str(ROOT / "tests/test_physical_joined.py"),
            str(ROOT / "tests/test_physical_owners.py"),
            "-q",
            f"--basetemp={self.artifacts_dir / 'fake-tests'}",
        ]
        result = subprocess.run(command, capture_output=True, text=True, timeout=180)
        (self.artifacts_dir / "installed-runtime.txt").write_text(result.stdout + result.stderr)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        print(result.stdout, flush=True)
        successes = [
            json.loads(path.read_text())
            for path in (self.artifacts_dir / "fake-tests").rglob("physical-agent-summary.json")
            if json.loads(path.read_text())["success"]
        ]
        self.assertTrue(successes)
        summary = successes[0]
        self.assertEqual(summary["score"], 1.0)
        report = self.artifacts_dir / "physical-agent-report.html"
        viewed = subprocess.run(
            [
                sys.executable,
                "-I",
                str(Path(sys.executable).with_name("inspect-robots")),
                "view",
                summary["eval_log"],
                "--no-video",
                "-o",
                str(report),
            ],
            capture_output=True,
            text=True,
            timeout=60,
        )
        (self.artifacts_dir / "physical-view.txt").write_text(viewed.stdout + viewed.stderr)
        self.assertEqual(viewed.returncode, 0, viewed.stdout + viewed.stderr)
        for expected in (
            "b601_motorbridge_joint1",
            "b601_measured_goal=1",
            "Bounded fake nudge",
            "move_joints",
        ):
            self.assertIn(expected, report.read_text())

    def test_simulation_cli_still_refuses_physical_selection(self):
        result = subprocess.run(
            [str(Path(sys.executable).with_name("b601-evaluate")), "physical"],
            capture_output=True,
            text=True,
            timeout=10,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("invalid choice", result.stderr)
        (self.artifacts_dir / "simulation-rejection.txt").write_text(result.stdout + result.stderr)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifacts-dir", required=True, type=Path)
    args = parser.parse_args()
    require_installed()
    PhysicalInstalledTests.artifacts_dir = args.artifacts_dir.resolve()
    PhysicalInstalledTests.artifacts_dir.mkdir(parents=True, exist_ok=True)
    unittest.main(argv=[sys.argv[0]], verbosity=2)
