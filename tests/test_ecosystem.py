"""The b601 embodiment through the real Inspect Robots CLI and tools; fake motors, camera and model.

Covers `inspect-robots run` / `eval-set` with the LLM agent policy, registered tasks from any
installed suite, the log tools (`inspect`, `summarize`, `view`, `video`), the setup wizard's
device and option slots, and the CLI's device claim. Nothing opens CAN, a camera or a real model.
"""

import importlib.util
import io
import json
import os
import shutil
import tempfile
import threading
import unittest
from contextlib import redirect_stdout
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from unittest.mock import patch

import numpy as np
from inspect_robots import cli
from inspect_robots.compat import check_compatibility
from inspect_robots.registry import registered, resolve

from inspect_robots_b601 import B601Embodiment
from inspect_robots_b601.hardware.session import ArmSession
from inspect_robots_b601.hardware.state import StopLedger
from test_session import Dummy, FakeController

AGENT = importlib.util.find_spec("inspect_robots_agent") is not None
# The fake model moves joint3 and the gripper once, then reports done.
CALLS = (
    ("move_joints", {"targets": {"joint3": 0.04, "gripper": 0.3}, "note": "raise and open"}),
    ("done", {"summary": "raised joint3 and opened the gripper", "hindsight": "none"}),
)


class FakeDepthCamera:
    depth, width, height = True, 640, 480

    def start(self):
        return self

    def read(self):
        return np.full((self.height, self.width, 3), 90, np.uint8)

    def read_with_depth(self):
        return self.read(), np.full((self.height, self.width), 0.2)

    def close(self):
        pass


class FakeModel(BaseHTTPRequestHandler):
    """An OpenAI Responses endpoint that replays CALLS, then keeps answering done."""

    requests: list = []

    def log_message(self, *args):
        pass

    def do_POST(self):
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        self.requests.append(body)
        name, arguments = CALLS[min(len(self.requests), len(CALLS)) - 1]
        data = json.dumps({
            "status": "completed",
            "output": [{"type": "function_call", "call_id": f"call-{len(self.requests)}",
                        "name": name, "arguments": json.dumps(arguments)}],
            "usage": {"input_tokens": 1, "output_tokens": 1},
        }).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


@unittest.skipUnless(AGENT, "install the agent extra")
class CliTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        state = self.root / "state"
        StopLedger(state).acknowledge(details={"test": "fresh arm"})
        self.controller = FakeController()
        FakeModel.requests = []
        server = HTTPServer(("127.0.0.1", 0), FakeModel)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        config = self.root / "config.ini"
        config.write_text("")

        def session(profile, state_dir):
            return ArmSession(profile, state_dir, controller_factory=lambda channel: self.controller,
                              bus_factory=lambda profile: Dummy(), period=0.005, home_wait=0.0)

        for patcher in (
            patch("inspect_robots_b601.hardware.session.ArmSession", session),
            patch("inspect_robots_b601.embodiment.parse_camera", lambda value: FakeDepthCamera() if value else None),
            patch.dict(os.environ, {"FAKE_MODEL_KEY": "not-a-key", "XDG_RUNTIME_DIR": str(self.root)}),
        ):
            patcher.start()
            self.addCleanup(patcher.stop)
        self.common = [
            "--config", str(config), "--policy", "agent", "-P", "model=test/model",
            "-P", f"base_url=http://127.0.0.1:{server.server_port}/v1", "-P", "wire=responses",
            "-P", "api_key_env=FAKE_MODEL_KEY", "-P", "max_llm_calls=3", "--embodiment", "b601",
            "-E", "channel=can9", "-E", "allow_motion=true", "-E", "activate_on_reset=true",
            "-E", "operator_supported_arm=true", "-E", f"state_dir={state}", "-E", "camera=realsense+depth",
            "--no-prompt", "--grader", "none", "--store-frames", "--log-dir", str(self.root / "logs"),
        ]

    def cli(self, *argv):
        with redirect_stdout(io.StringIO()) as output:
            code = cli.main(list(argv))
        return code, output.getvalue()

    def assert_arm_home_and_disabled(self):
        for motor in self.controller.motors:
            self.assertFalse(motor.enabled)
            self.assertAlmostEqual(motor.disabled_at, 0.0, places=6)

    def test_run_with_the_agent_and_every_log_tool(self):
        code, output = self.cli("run", *self.common, "--no-rerun", "--max-steps", "5",
                                "--instruction", "Raise joint3 a little and open the gripper, then finish")
        self.assertEqual(code, 0, output)
        self.assertIn("run status: completed", output)
        tools = {tool["name"]: tool for tool in FakeModel.requests[0]["tools"]}
        self.assertEqual(set(tools), {"move_joints", "done", "give_up"})
        self.assertIn("gripper", json.dumps(tools["move_joints"]))
        self.assert_arm_home_and_disabled()

        (log,) = (self.root / "logs").glob("*.json")
        self.assertEqual(json.loads(log.read_text())["eval"]["embodiment"], "b601")
        code, output = self.cli("inspect", str(log), "--transcript")
        self.assertEqual(code, 0, output)
        self.assertIn("state[tip]", output)
        self.assertEqual(self.cli("summarize", str(log), "-o", str(self.root / "summary.md"))[0], 0)
        self.assertIn("Embodiment: b601", (self.root / "summary.md").read_text())
        self.assertEqual(self.cli("view", str(log), "-o", str(self.root / "report.html"))[0], 0)
        self.assertGreater((self.root / "report.html").stat().st_size, 0)
        if shutil.which("ffmpeg"):
            self.assertEqual(self.cli("video", str(log))[0], 0)
            self.assertTrue(list((self.root / "logs").rglob("*_front.mp4")))

    def test_registered_tasks_and_eval_set(self):
        tasks = sorted(registered("task"))
        code, output = self.cli("eval-set", *self.common, *tasks[:2])
        self.assertEqual(code, 0, output)
        self.assertIn("run status: completed", output)
        self.assert_arm_home_and_disabled()

    def test_every_registered_task_accepts_the_agent_on_b601(self):
        robot = B601Embodiment(camera="realsense+depth")
        for name in sorted(registered("task")):
            with self.subTest(task=name):
                policy = resolve("policy", "agent", model="test/model", base_url="https://x.invalid/v1",
                                 env={"OPENAI_API_KEY": "not-a-key"})
                policy.bind(robot.info)
                report = check_compatibility(policy, robot, resolve("task", name))
                self.assertTrue(report.ok, [issue.message for issue in report.errors])


class SetupWizardTests(unittest.TestCase):
    def test_wizard_offers_the_can_channel_and_the_motion_options(self):
        try:
            from inspect_robots._setup import run_setup
        except ImportError:  # pragma: no cover - the wizard's module is private to the core
            self.skipTest("this inspect-robots has no setup wizard module")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, kind in (("can0", "280"), ("can1", "280"), ("eth0", "1")):
                (root / "net" / name).mkdir(parents=True)
                (root / "net" / name / "type").write_text(kind + "\n")
            prompts = []

            def answer(prompt):
                prompts.append(prompt)
                if prompt.lower().startswith("embodiment"):
                    return "b601"
                if "CAN channel" in prompt:
                    return "can1"
                if any(arg in prompt for arg in ("allow_motion", "operator_supported_arm", "activate_on_reset")):
                    return "y"
                return ""

            missing = root / "missing"
            env = {"XDG_CONFIG_HOME": str(root / "config"), "HOME": str(root)}
            code = run_setup(env, input_fn=answer, out=io.StringIO(), interactive=True, sysfs_net=root / "net",
                             by_id_dir=missing, by_path_dir=missing, sysfs_video=missing,
                             serial_by_id_dir=missing)
            config = next((root / "config").rglob("config.ini")).read_text()
        self.assertEqual(code, 0)
        self.assertTrue(any("B601-RS arm CAN channel" in prompt for prompt in prompts), prompts)
        self.assertIn("embodiment = b601", config)
        self.assertIn("channel = can1", config)
        for option in ("allow_motion = true", "operator_supported_arm = true", "activate_on_reset = true"):
            self.assertIn(option, config)


if __name__ == "__main__":
    unittest.main()
