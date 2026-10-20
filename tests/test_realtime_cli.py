"""CLI roles remain separate, validate locally, and fail closed without credentials."""

import asyncio
from types import SimpleNamespace

import pytest

from inspect_robots_b601._realtime.protocol import Codec, Invite
from inspect_robots_b601._realtime.sfu import Config
from inspect_robots_b601.realtime import _operator, _robot, main

CONFIG = Config(
    "test-app", "test-only-app-secret-long-enough", "test-only-pairing-key-32-characters"
)


@pytest.mark.parametrize(
    "args",
    [
        ["robot", "--backend", "physical"],
        ["robot", "--backend", "motorbridge-read"],
        ["robot", "--backend", "echo", "--profile", "/fake/profile"],
        ["operator", "--invite", "unused", "--backend", "motorbridge-read"],
        ["operator", "--invite", "unused", "--request", "activate"],
        ["operator", "--invite", "unused", "--request", "hold", "--watch"],
    ],
)
def test_invalid_role_options_never_connect(monkeypatch, args):
    monkeypatch.setattr(
        "inspect_robots_b601.realtime.Link", lambda *_: pytest.fail("No network permitted")
    )
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2


def test_ros_backend_rejects_remote_discovery_before_connecting(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "62")
    monkeypatch.setattr(
        "inspect_robots_b601.realtime.Link", lambda *_: pytest.fail("No network permitted")
    )
    with pytest.raises(SystemExit):
        main(["robot", "--backend", "ros-mock"])


def test_missing_configuration_is_an_actionable_local_error(monkeypatch, capsys):
    monkeypatch.delenv("CLOUDFLARE_REALTIME_APP_ID", raising=False)
    assert main(["robot"]) == 1
    assert "stopped" in capsys.readouterr().err


def test_physical_role_rejects_new_or_relative_state_before_connecting(monkeypatch, tmp_path):
    monkeypatch.setattr(
        "inspect_robots_b601.realtime.Link", lambda *_: pytest.fail("No connection permitted")
    )
    with pytest.raises(SystemExit) as failure:
        main(
            [
                "robot",
                "--backend",
                "motorbridge-read",
                "--profile",
                "/reviewed/profile.json",
                "--state-dir",
                str(tmp_path / "does-not-exist"),
            ]
        )
    assert failure.value.code == 2


@pytest.mark.parametrize(
    "event,passed,expected",
    [
        ("status", True, True),
        ("result", True, True),
        ("result", False, False),
        ("error", False, False),
        ("busy", False, False),
    ],
)
def test_operator_correlates_a_signed_result_not_a_claim_of_acceptance(
    monkeypatch, event, passed, expected
):
    invite = Invite(CONFIG.app_id, "robot-session", "channel", "a" * 32)
    instances = []

    class FakeLink:
        def __init__(self, config):
            self.codec = Codec(config.pairing_key, invite.nonce, "robot")
            self.request = None
            self.closed = False
            instances.append(self)

        async def connect(self, channel, remote):
            assert remote == invite

        def send(self, message):
            self.request = self.codec.decode(message)

        async def receive(self, timeout):
            return self.codec.encode({"id": self.request["id"], "event": event, "passed": passed})

        async def close(self):
            self.closed = True

    monkeypatch.setattr("inspect_robots_b601.realtime.Link", FakeLink)
    args = SimpleNamespace(
        invite=invite.encode(), request="hold" if event == "result" else "status"
    )
    assert asyncio.run(_operator(args, CONFIG)) is expected
    assert instances[0].closed
    if event == "result":
        assert instances[0].request["task"] == "hold"


@pytest.mark.parametrize("invalid_message", [False, True])
def test_robot_owns_local_calls_and_latches_when_the_link_ends(
    monkeypatch, capsys, invalid_message
):
    import threading

    calls, instances = [], []

    class Controller:
        def __init__(self, *_, **__):
            instances.append(self)

        def stop(self):
            calls.append(("stop", threading.get_ident()))

        def status(self):
            calls.append(("status", threading.get_ident()))
            return {"backend": "echo"}

        def run(self, task):
            calls.append((task, threading.get_ident()))
            return {"passed": True}

        def close(self):
            calls.append(("close", threading.get_ident()))

    class FakeLink:
        def __init__(self, config):
            self.session_id, self.closed = "robot-session", False
            self.command = 0
            self.responses = []

        async def connect(self, channel):
            self.codec = Codec(CONFIG.pairing_key, channel.removeprefix("b601-"), "operator")

        def send(self, message):
            self.responses.append(self.codec.decode(message))

        async def receive(self, timeout):
            await asyncio.sleep(0.02)
            if invalid_message:
                return "{}"
            self.command += 1
            if self.command == 1:
                return self.codec.encode({"id": "b" * 32, "command": "status"})
            if self.command == 2:
                return self.codec.encode({"id": "c" * 32, "command": "run", "task": "hold"})
            if self.command == 3:
                return self.codec.encode({"id": "c" * 32, "command": "run", "task": "hold"})
            if self.command == 4:
                return self.codec.encode({"id": "d" * 32, "command": "stop"})
            raise RuntimeError("test connection ended")

        async def close(self):
            self.closed = True

    links = []

    def link_factory(config):
        link = FakeLink(config)
        links.append(link)
        return link

    monkeypatch.setattr("inspect_robots_b601.realtime.Link", link_factory)
    monkeypatch.setattr("inspect_robots_b601.realtime.Robot", Controller)
    args = SimpleNamespace(backend="echo", profile=None, state_dir=None, log_dir="unused")
    with pytest.raises((RuntimeError, ValueError)):
        asyncio.run(_robot(args, CONFIG))
    assert links[0].closed
    assert calls[-1][0] == "close"
    local_call_threads = {thread for name, thread in calls if name != "stop"}
    assert len(local_call_threads) == 1
    assert threading.get_ident() not in local_call_threads
    assert CONFIG.app_secret not in capsys.readouterr().out
    if invalid_message:
        assert not any(name in ("status", "hold") for name, _ in calls)
    else:
        assert sum(name == "hold" for name, _ in calls) == 1
        assert any(reply["event"] == "duplicate" for reply in links[0].responses)


def test_watch_reuses_one_connection_for_fresh_observation_requests(monkeypatch):
    invite = Invite(CONFIG.app_id, "robot-session", "channel", "a" * 32)
    instances = []

    class WatchingLink:
        def __init__(self, config):
            self.codec = Codec(config.pairing_key, invite.nonce, "robot")
            self.sent, self.closed = [], False
            instances.append(self)

        async def connect(self, *_, **__):
            pass

        def send(self, message):
            self.sent.append(self.codec.decode(message))

        async def receive(self, timeout):
            if len(self.sent) > 1:
                raise RuntimeError("test-watch-ended")
            return self.codec.encode({"id": self.sent[-1]["id"], "event": "status"})

        async def close(self):
            self.closed = True

    async def no_sleep(_):
        pass

    monkeypatch.setattr("inspect_robots_b601.realtime.Link", WatchingLink)
    monkeypatch.setattr("inspect_robots_b601.realtime.asyncio.sleep", no_sleep)
    with pytest.raises(RuntimeError, match="test-watch-ended"):
        asyncio.run(
            _operator(SimpleNamespace(invite=invite.encode(), request="status", watch=True), CONFIG)
        )
    assert len(instances) == 1 and instances[0].closed
    assert len(instances[0].sent) == 2
    assert instances[0].sent[0]["id"] != instances[0].sent[1]["id"]
    assert all(request["command"] == "status" for request in instances[0].sent)
