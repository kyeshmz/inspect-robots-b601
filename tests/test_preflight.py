"""inspect-robots-b601-preflight: conformance and runtime checks with no hardware."""

import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

from inspect_robots_b601 import preflight
from inspect_robots_b601.embodiment import DEFAULT_PROFILE


def run(*argv):
    with redirect_stdout(io.StringIO()) as output:
        code = preflight.main(list(argv))
    return code, output.getvalue()


class PreflightTests(unittest.TestCase):
    def test_stock_profile_is_ready(self):
        with patch.object(preflight, "missing_runtime_requirements", return_value={}):
            code, output = run("--dry-run")
        self.assertEqual(code, 0, output)
        self.assertIn("b601: conformant (no issues)", output)
        self.assertIn("OK: ready for Inspect Robots.", output)
        self.assertIn("(dry-run) No motion will be commanded.", output)

    def test_missing_motorbridge_fails_with_the_install_command(self):
        missing = {"motorbridge": "python -m pip install 'inspect-robots-b601[motorbridge]'"}
        with patch.object(preflight, "missing_runtime_requirements", return_value=missing):
            code, output = run()
        self.assertEqual(code, 1)
        self.assertIn("missing motorbridge: python -m pip install 'inspect-robots-b601[motorbridge]'", output)

    def test_json_report_includes_the_camera_declaration(self):
        with patch.object(preflight, "missing_runtime_requirements", return_value={}):
            code, output = run("--json", "--camera", "realsense+depth")
        report = json.loads(output)
        self.assertEqual(code, 0)
        self.assertEqual((report["ok"], report["embodiment"], report["issues"]), (True, "b601", []))

    def test_close_range_depth_profile_needs_the_enhanced_depth_package(self):
        with tempfile.TemporaryDirectory() as directory:
            profile = json.loads(DEFAULT_PROFILE.read_text())
            profile["wrist_camera"] = {"jaw_window_px": [300, 272, 345, 296], "jaw_tip_depth_m": 0.092,
                                       "depth_backend": "close-range"}
            path = Path(directory) / "arm.json"
            path.write_text(json.dumps(profile))
            with patch.object(preflight, "missing_runtime_requirements", return_value={}), \
                    patch.object(preflight, "close_range_problem", return_value="No module named 'rs_depth'"):
                code, output = run("--profile", str(path), "--camera", "realsense+depth")
                self.assertEqual(code, 1)
                self.assertIn("missing rs_depth: install the RealSense Enhanced Depth package", output)
                # Without a depth camera there is no depth to need.
                self.assertEqual(run("--profile", str(path), "--camera", "realsense")[0], 0)
            with patch.object(preflight, "missing_runtime_requirements", return_value={}), \
                    patch.object(preflight, "close_range_problem", return_value=None):
                self.assertEqual(run("--profile", str(path), "--camera", "realsense+depth")[0], 0)

    def test_run_preflight_defaults_to_the_stock_profile(self):
        report = preflight.run_preflight()
        self.assertTrue(report.conformance.ok)
        self.assertEqual(report.conformance.embodiment, "b601")


if __name__ == "__main__":
    unittest.main()
