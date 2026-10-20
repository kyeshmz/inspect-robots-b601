"""The shipped annotations must reject invalid calls, not merely silence mypy."""

import subprocess
import sys
from pathlib import Path

import pytest


def test_consumer_types_reject_wrong_parameter_and_action_objects(tmp_path):
    pytest.importorskip("mypy")
    consumer = tmp_path / "consumer.py"
    consumer.write_text(
        "from inspect_robots_b601 import B601Embodiment\n"
        "robot = B601Embodiment(control_hz='fast')\n"
        "robot.step('not-an-Action')\n"
    )
    config = Path(__file__).resolve().parents[1] / "pyproject.toml"
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "mypy",
            "--config-file",
            str(config),
            "--follow-imports=silent",
            str(consumer),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 1, result.stdout + result.stderr
    assert result.stdout.count("[arg-type]") == 2, result.stdout
