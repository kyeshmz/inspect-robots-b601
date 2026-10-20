"""Real provider child, agent, worker, evaluator and viewer with loopback-only HTTP."""

import base64
import json
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

from inspect_robots_b601 import physical_agent as physical
from tests.test_physical_agent import DONE, MOVE, argv, configuration, fake_boundaries, reply


@pytest.mark.parametrize(
    "scenario,status,calls,expected_exit,expected_pose,expected_score",
    [
        ("nudge", 200, (MOVE, DONE), 0, 0.2015, 1.0),
        ("done-without-motion", 200, (DONE,), 1, 0.2, 0.0),
        *[("http-error", status, (DONE,), 1, 0.2, None) for status in (302, 401, 403, 429, 500)],
        ("failed-responses", 200, (DONE,), 1, 0.2, None),
    ],
)
def test_joined_cli_provider_child_and_measured_outcome(
    tmp_path,
    monkeypatch,
    capsys,
    scenario,
    status,
    calls,
    expected_exit,
    expected_pose,
    expected_score,
):
    args = configuration(tmp_path, monkeypatch)
    credential = "fake-only-never-a-real-credential"
    real_run = subprocess.run
    received, server_failures = [], []

    class Provider(BaseHTTPRequestHandler):
        def do_POST(self):
            try:
                assert self.path == "/v1/responses"
                assert self.headers["Authorization"] == f"Bearer {credential}"
                payload = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                assert payload["model"] == "fake-model"
                assert {tool["name"] for tool in payload["tools"]} == {
                    "move_joints",
                    "done",
                    "give_up",
                }
                index = len(received)
                received.append(payload)
                assert index < len(calls), "No HTTP retry is allowed"
                if scenario == "http-error":
                    body = ("Invalid API key: " + credential).encode()
                elif scenario == "failed-responses":
                    body = json.dumps(
                        {"status": "failed", "error": {"message": "Invalid API key: " + credential}}
                    ).encode()
                else:
                    body = base64.b64decode(json.loads(reply(*calls[index]))["body"])
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except BaseException as error:
                server_failures.append(type(error).__name__)
                raise

        def log_message(self, *arguments):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Provider)
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.start()

    def redirect_only(command, **kwargs):
        assert command == [sys.executable, "-I", "-m", "inspect_robots_b601._physical_http"]
        request = json.loads(kwargs["input"])
        assert request["url"] == "https://physical.invalid/v1/responses"
        request["url"] = f"http://127.0.0.1:{server.server_port}/v1/responses"
        kwargs["input"] = json.dumps(request).encode()
        return real_run(command, **kwargs)

    motor, owner, sessions, requests = fake_boundaries(
        monkeypatch, calls, provider_run=redirect_only
    )
    try:
        assert physical.main(argv(args)) == expected_exit
        captured = capsys.readouterr()
        (tmp_path / "stdout.txt").write_text(captured.out)
        (tmp_path / "stderr.txt").write_text(captured.err)
        summary = json.loads((args.log_dir / "physical-agent-summary.json").read_text())
        assert summary["success"] is (expected_exit == 0)
        assert summary["score"] == expected_score
        assert summary["torque_off_confirmed"] is False
        assert motor.positions[0] == expected_pose
        assert len(received) == len(requests) == len(calls)
        assert not server_failures
        assert sessions[0].closed and not sessions[0]._thread.is_alive()
        assert owner.closed and motor.closed and not motor.enabled
        assert all(event[1] == "joint1" for event in motor.writes)
        assert all(
            0.1994 <= event[2] <= 0.2021
            for event in motor.writes
            if event[0] in ("prepare", "position")
        )
        if scenario in ("http-error", "failed-responses"):
            assert json.loads((args.state_dir / "stop-state.json").read_text())["stopped"] is True
            assert motor.positions == [0.2, 0.1, -0.1, 0, 0, 0]
            assert summary["requests"] == 1
        (log,) = [p for p in args.log_dir.glob("*.json") if p.name != "physical-agent-summary.json"]
        if scenario in ("http-error", "failed-responses"):
            diagnostic = f"HTTP {status}" if scenario == "http-error" else "Responses status failed"
            assert (
                f"SafetyAbort: Physical provider {diagnostic}; no retry or recovery"
                in log.read_text()
            )
        report = tmp_path / "report.html"
        viewed = real_run(
            [
                str(Path(sys.executable).with_name("inspect-robots")),
                "view",
                str(log),
                "--no-video",
                "-o",
                str(report),
            ],
            capture_output=True,
            text=True,
            timeout=60,
        )
        (tmp_path / "view.txt").write_text(viewed.stdout + viewed.stderr)
        assert viewed.returncode == 0, viewed.stdout + viewed.stderr
        assert "b601_motorbridge_joint1" in report.read_text()
        if expected_score is not None:
            assert f"b601_measured_goal={expected_score:g}" in report.read_text()
        leaked = [
            str(path.relative_to(tmp_path))
            for path in tmp_path.rglob("*")
            if path.is_file() and credential.encode() in path.read_bytes()
        ]
        assert leaked == []
    finally:
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=2)
        assert not server_thread.is_alive()
