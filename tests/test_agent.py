"""Run with pytest, or with isolated Python after installing the wheel and agent."""

import argparse
import importlib
import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
from dataclasses import dataclass
from importlib.metadata import distribution, version
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import httpx
import numpy as np
from inspect_robots import eval
from inspect_robots.approver import ChainApprover, ClampApprover, DeltaLimitApprover
from inspect_robots.log import read_eval_log
from inspect_robots.registry import registered, resolve
from inspect_robots_agent import LLMAgentPolicy, agent_policy

from inspect_robots_b601 import B601Embodiment
from inspect_robots_b601.benchmarks import joint1_nudge


@dataclass(frozen=True)
class AgentScenario:
    name: str
    calls: tuple[tuple[str, dict], ...]
    expected_joint1_commands: tuple[float, ...]
    expected_final_pose: tuple[float, ...]
    expected_score: float


SCENARIOS = (
    AgentScenario(
        name="measured-nudge",
        calls=(
            ("move_joints", {"targets": {"joint1": 0.0015}, "note": "Hold other joints"}),
            ("done", {"summary": "Measured nudge complete", "hindsight": "Check measured scoring"}),
        ),
        expected_joint1_commands=(0.00075, 0.0015, 0.0015),
        expected_final_pose=(0.0015, 1, 1, 0, 0, 0),
        expected_score=1.0,
    ),
    AgentScenario(
        name="done-without-motion",
        calls=(("done", {"summary": "Claimed success without moving", "hindsight": "No motion"}),),
        expected_joint1_commands=(0.0,),
        expected_final_pose=(0, 1, 1, 0, 0, 0),
        expected_score=0.0,
    ),
)


class FakeRosSession:
    def __init__(self):
        self.positions = np.array([0.0, 1.0, 1.0, 0.0, 0.0, 0.0])
        self.commands = []
        self.closed = False

    def observe(self):
        return self.positions.copy(), time.monotonic(), {}, {}

    def move(self, target, duration):
        self.commands.append(target.copy())
        self.positions = target.copy()

    def close(self):
        self.closed = True


def require_installed_packages():
    if not (3, 10) <= sys.version_info[:2] < (3, 13):
        raise AssertionError("Acceptance requires Python 3.10 through 3.12")
    for package, expected_version, module_name in (
        ("inspect-robots-b601", "0.4.0", "inspect_robots_b601"),
        ("inspect-robots", "0.60.0", "inspect_robots"),
        ("inspect-robots-agent", "0.28.0", "inspect_robots_agent"),
    ):
        if version(package) != expected_version:
            raise AssertionError(f"{package} must be {expected_version}")
        dist = distribution(package)
        origin = Path(importlib.import_module(module_name).__file__).resolve()
        expected = Path(dist.locate_file(f"{module_name}/__init__.py")).resolve()
        if origin != expected or not origin.is_relative_to(Path(sys.prefix).resolve()):
            raise AssertionError(f"{package} did not load from the installed environment: {origin}")
        direct_url = json.loads(dist.read_text("direct_url.json") or "{}")
        if direct_url.get("dir_info", {}).get("editable"):
            raise AssertionError(f"{package} is editable, not an installed wheel")
        print(f"{package}=={expected_version} at {origin}", flush=True)
    if int(version("numpy").split(".")[0]) >= 2:
        raise AssertionError("Acceptance requires NumPy below 2")


class AgentAcceptanceTests(unittest.TestCase):
    artifacts_dir = None

    def test_agent_evaluation_logs_and_report(self):
        self.assertIs(registered("embodiment")["b601_ros"], B601Embodiment)
        self.assertIs(registered("task")["b601/joint1-nudge"], joint1_nudge)
        self.assertIs(registered("policy")["agent"], agent_policy)
        with tempfile.TemporaryDirectory() as temporary:
            root = self.artifacts_dir or Path(temporary)
            for scenario in SCENARIOS:
                with self.subTest(scenario=scenario.name):
                    self.check_scenario(scenario, root / scenario.name)

    def check_scenario(self, scenario, directory):
        directory.mkdir(parents=True, exist_ok=True)
        log_dir = Path(tempfile.mkdtemp(prefix="eval-", dir=directory))
        requests = []
        session = FakeRosSession()

        def serve(request):
            self.assertEqual(str(request.url), "https://b601-agent.invalid/v1/responses")
            self.assertNotIn("authorization", request.headers)
            payload = json.loads(request.content)
            requests.append(payload)
            self.assertEqual(payload["model"], "b601-test-model")
            tools = {tool["name"]: tool for tool in payload["tools"]}
            self.assertEqual(set(tools), {"move_joints", "done", "give_up"})
            self.assertIn(
                "joint1, joint2, joint3, joint4, joint5, joint6",
                tools["move_joints"]["parameters"]["properties"]["targets"]["description"],
            )
            self.assertLessEqual(len(requests), len(scenario.calls))
            name, arguments = scenario.calls[len(requests) - 1]
            return httpx.Response(
                200,
                json={
                    "status": "completed",
                    "output": [
                        {
                            "type": "function_call",
                            "call_id": f"call-{len(requests)}",
                            "name": name,
                            "arguments": json.dumps(arguments),
                        }
                    ],
                },
            )

        with (
            httpx.MockTransport(serve) as transport,
            patch.dict(
                sys.modules,
                {"inspect_robots_b601._ros": SimpleNamespace(RosSession=lambda _: session)},
            ),
            patch("socket.socket", side_effect=AssertionError("Real sockets are forbidden")),
            patch.dict(os.environ, {}, clear=True),
        ):
            robot = resolve("embodiment", "b601_ros", max_step=0.002, position_tolerance=0.0006)
            policy = resolve(
                "policy",
                "agent",
                model="b601-test-model",
                base_url="https://b601-agent.invalid/v1",
                wire="responses",
                transport=transport,
                env={},
                max_speed_frac=0.0004,
                max_llm_calls=3,
                max_retries=1,
            )
            self.assertIsInstance(policy, LLMAgentPolicy)
            try:
                (log,) = eval(
                    resolve("task", "b601/joint1-nudge", max_steps=6),
                    policy,
                    robot,
                    log_dir=str(log_dir),
                    approver=ChainApprover(
                        ClampApprover(robot.info.action_space),
                        DeltaLimitApprover(robot.info.action_space, max_delta=0.002),
                    ),
                )
            finally:
                robot.close()

        commands = np.asarray(session.commands)
        (directory / "provider-requests.json").write_text(json.dumps(requests, indent=2))
        (directory / "commands.json").write_text(json.dumps(commands.tolist(), indent=2))
        self.assertTrue(session.closed)
        self.assertEqual(log.status, "success", log.error)
        self.assertEqual(len(requests), len(scenario.calls))
        self.assertEqual(commands.shape, (len(scenario.expected_joint1_commands), 6))
        self.assertTrue(np.isfinite(commands).all())
        np.testing.assert_allclose(commands[:, 0], scenario.expected_joint1_commands, atol=1e-12)
        np.testing.assert_array_equal(commands[:, 1:], [[1, 1, 0, 0, 0]] * len(commands))
        self.assertLessEqual(np.abs(np.diff(commands, axis=0)).max(initial=0), 0.002)
        np.testing.assert_allclose(session.positions, scenario.expected_final_pose, atol=1e-12)
        self.assertEqual(log.results.metrics, {"b601_measured_goal": scenario.expected_score})
        self.assertEqual(log.stats.total_steps, len(scenario.expected_joint1_commands))
        self.assertEqual(log.results.total_trials, 1)
        self.assertEqual(log.results.errored_trials, 0)
        self.assertEqual(log.eval.task, "b601/joint1-nudge")
        self.assertEqual(log.eval.policy, "agent")
        self.assertEqual(log.eval.embodiment, "b601_ros_sim")
        self.assertEqual(log.eval.inspect_robots_version, "0.60.0")
        (sample,) = log.samples
        self.assertEqual(sample.epochs, ({"b601_measured_goal": scenario.expected_score},))
        self.assertEqual(sample.termination_reasons, ("done",))
        self.assertEqual(sample.policy_transcripts, (policy.transcript(),))
        recorded_calls = [
            (call["function"]["name"], json.loads(call["function"]["arguments"]))
            for message in sample.policy_transcripts[0]
            for call in message.get("tool_calls", [])
        ]
        self.assertEqual(recorded_calls, list(scenario.calls))
        transcript_path = log_dir / sample.trial_metadata[0]["transcript"]
        self.assertEqual(
            [json.loads(line) for line in transcript_path.read_text().splitlines()],
            policy.transcript(),
        )
        (log_path,) = log_dir.glob("*.json")
        self.assertEqual(read_eval_log(str(log_path)), log)

        report = log_dir / "report.html"
        result = subprocess.run(
            [
                sys.executable,
                "-I",
                str(Path(sys.executable).with_name("inspect-robots")),
                "view",
                str(log_path),
                "--no-video",
                "-o",
                str(report),
            ],
            check=True,
            capture_output=True,
            text=True,
            env={"PATH": str(Path(sys.executable).parent), "HOME": str(directory)},
            timeout=60,
        )
        (log_dir / "view.txt").write_text(result.stdout + result.stderr)
        html = report.read_text()
        for expected in (
            "b601/joint1-nudge",
            "b601_ros_sim",
            f"b601_measured_goal={scenario.expected_score:g}",
            "agent",
            scenario.calls[-1][1]["summary"],
        ):
            self.assertIn(expected, html)
        for name, arguments in scenario.calls:
            self.assertIn(name, html)
            self.assertIn(arguments.get("note", arguments.get("summary")), html)
        print(f"{scenario.name}: score={scenario.expected_score}, log={log_path}, report={report}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifacts-dir", type=Path, required=True)
    args = parser.parse_args()
    require_installed_packages()
    AgentAcceptanceTests.artifacts_dir = args.artifacts_dir.resolve()
    unittest.main(argv=[sys.argv[0]], verbosity=2)
