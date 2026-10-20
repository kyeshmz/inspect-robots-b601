"""Realtime accepts fresh authenticated requests, not raw joint targets or replays."""

import hashlib
import hmac
import json

import pytest

from inspect_robots_b601._realtime.protocol import Codec, Command, Invite, canonical, load_json

KEY = "test-only-pairing-key-32-characters"
NONCE = "a" * 32
REQUEST = {"id": "b" * 32, "command": "status"}


def signed(body, key=KEY):
    body = {key: value for key, value in body.items() if key != "mac"}
    mac = hmac.new(key.encode(), canonical(body).encode(), hashlib.sha256).hexdigest()
    return canonical({**body, "mac": mac})


def test_bidirectional_authentication_and_new_operator_processes():
    robot, operator = Codec(KEY, NONCE, "robot"), Codec(KEY, NONCE, "operator")
    assert robot.decode(operator.encode(REQUEST)) == REQUEST
    assert operator.decode(robot.encode({"event": "status"})) == {"event": "status"}
    assert robot.decode(Codec(KEY, NONCE, "operator").encode(REQUEST)) == REQUEST


def test_replay_bad_key_and_reflected_direction_fail():
    robot, operator = Codec(KEY, NONCE, "robot"), Codec(KEY, NONCE, "operator")
    message = operator.encode(REQUEST)
    robot.decode(message)
    with pytest.raises(ValueError, match="sequence"):
        robot.decode(message)
    with pytest.raises(ValueError, match="authentication"):
        Codec("wrong-but-long-enough-pairing-secret", NONCE, "robot").decode(message)
    with pytest.raises(ValueError, match="direction"):
        operator.decode(message)


@pytest.mark.parametrize(
    "changes",
    [
        {"v": 2},
        {"v": True},
        {"nonce": "c" * 32},
        {"seq": 0},
        {"seq": True},
        {"sender": "../invalid"},
        {"sent_ms": "1000"},
        {"payload": []},
    ],
)
def test_even_authenticated_malformed_envelopes_are_rejected(changes):
    body = json.loads(Codec(KEY, NONCE, "operator", now=lambda: 1000).encode(REQUEST))
    with pytest.raises(ValueError):
        Codec(KEY, NONCE, "robot", now=lambda: 1000).decode(signed({**body, **changes}))


@pytest.mark.parametrize("remote_time", [990, 1010])
def test_stale_and_future_packets_fail(remote_time):
    message = Codec(KEY, NONCE, "operator", now=lambda: remote_time).encode(REQUEST)
    with pytest.raises(ValueError, match="Stale"):
        Codec(KEY, NONCE, "robot", now=lambda: 1000).decode(message)


def test_invite_is_a_public_locator_without_credentials():
    invite = Invite("test-app", "robot-session", "b601-test", NONCE)
    assert Invite.parse(invite.encode()) == invite
    assert "secret" not in repr(invite)
    assert "pairing" not in repr(invite)


@pytest.mark.parametrize("value", ["", "x" * 2049, "../bad", "a", "e30"])
def test_invalid_invites_are_rejected(value):
    with pytest.raises(ValueError):
        Invite.parse(value)


@pytest.mark.parametrize("message", ['{"v":1,"v":2}', '{"x":NaN}', "[]", '"str"', "x" * 17000])
def test_ambiguous_or_unbounded_json_is_rejected(message):
    with pytest.raises(ValueError):
        load_json(message)


def test_size_and_operator_history_are_bounded():
    with pytest.raises(ValueError, match="size"):
        Codec(KEY, NONCE, "robot").encode({"data": "x" * 17000})
    robot = Codec(KEY, NONCE, "robot")
    for _ in range(128):
        robot.decode(Codec(KEY, NONCE, "operator").encode(REQUEST))
    with pytest.raises(ValueError, match="limit"):
        robot.decode(Codec(KEY, NONCE, "operator").encode(REQUEST))


@pytest.mark.parametrize(
    "payload",
    [
        {"command": "activate", "id": "b" * 32},
        {"command": "run", "task": "home", "id": "b" * 32},
        {"command": "status", "positions": [0] * 6, "id": "b" * 32},
        {"command": "status"},
        {"command": "run", "task": "hold", "id": "bad"},
    ],
)
def test_remote_activation_recovery_arbitrary_tasks_and_raw_targets_are_forbidden(payload):
    with pytest.raises(ValueError):
        Command.parse(payload)


@pytest.mark.parametrize("name", ["status", "stop", "run"])
def test_only_named_commands_parse(name):
    body = {**REQUEST, "command": name}
    if name == "run":
        body["task"] = "hold"
    assert Command.parse(body).name == name


@pytest.mark.parametrize("key", ["short", "x" * 257])
def test_invalid_pairing_keys_fail_before_connecting(key):
    with pytest.raises(ValueError):
        Codec(key, NONCE, "robot")
