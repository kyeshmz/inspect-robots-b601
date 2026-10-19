"""The official-mock wrapper owns only its own launch and never selects hardware."""

import importlib.util
import signal
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize("failed_task", [None, "hold", "joint1-nudge"])
def test_launch_is_fixed_to_mock_and_its_own_process_is_cleaned_up(monkeypatch, tmp_path, failed_task):
    path = Path(__file__).resolve().parents[1] / "scripts/smoke-official-mock.py"
    spec = importlib.util.spec_from_file_location("official_mock_smoke", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("ROS_DOMAIN_ID", "61")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.delenv("ROS_DISCOVERY_SERVER", raising=False)
    monkeypatch.delenv("ROS_STATIC_PEERS", raising=False)
    events = []

    def robot(**options):
        assert options["simulated"] is True
        return SimpleNamespace(reset=lambda _: events.append("verify"),
                               close=lambda: events.append("close"))

    process = SimpleNamespace(pid=12345, poll=lambda: None,
                              wait=lambda **_: events.append("wait"))

    def launch(command, **options):
        assert command == ["ros2", "launch", "rebotarm_moveit_config", "demo.launch.py",
                           "model:=rs", "use_rviz:=false"]
        assert options["start_new_session"] is True
        events.append("launch")
        return process

    def evaluate(args):
        assert "verify" in events
        assert args[0] == "ros-mock"
        task = args[args.index("--task") + 1]
        events.append(task)
        return int(task == failed_task)

    monkeypatch.setattr(module, "B601Embodiment", robot)
    monkeypatch.setattr(module.subprocess, "Popen", launch)
    monkeypatch.setattr(module.os, "killpg", lambda pid, sig: events.append((pid, sig)))
    monkeypatch.setattr(module, "evaluate", evaluate)
    if failed_task:
        with pytest.raises(RuntimeError, match=failed_task):
            module.main()
    else:
        module.main()
    assert events.count("close") == 2
    assert (12345, signal.SIGINT) in events
    assert events[-1] == "wait"


def test_wrapper_rejects_nonisolated_environment_before_launch(monkeypatch):
    path = Path(__file__).resolve().parents[1] / "scripts/smoke-official-mock.py"
    spec = importlib.util.spec_from_file_location("official_mock_smoke_bad_environment", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("ROS_DOMAIN_ID", "62")
    monkeypatch.setattr(module.subprocess, "Popen", lambda *_: pytest.fail("No launch permitted"))
    with pytest.raises(RuntimeError, match="domain 61"):
        module.main()
