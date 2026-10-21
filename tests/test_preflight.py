"""inspect-robots-b601-preflight: conformance and runtime checks with no hardware."""

import io
import json
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

from inspect_robots_b601 import preflight


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

    def test_run_preflight_defaults_to_the_stock_profile(self):
        report = preflight.run_preflight()
        self.assertTrue(report.conformance.ok)
        self.assertEqual(report.conformance.embodiment, "b601")


if __name__ == "__main__":
    unittest.main()
