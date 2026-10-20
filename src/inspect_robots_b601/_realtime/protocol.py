"""Bounded, authenticated, fresh messages; never deserialize motor commands."""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import re
import secrets
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal, cast

MAX_MESSAGE_BYTES = 16_384
IDENTIFIER = re.compile(r"[A-Za-z0-9_-]{1,128}")
NONCE = re.compile(r"[a-f0-9]{32}")
TASKS = ("hold", "joint1-nudge")


def _unique(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON keys are forbidden")
        result[key] = value
    return result


def _invalid_number(value: str) -> None:
    raise ValueError("Non-finite JSON numbers are forbidden")


def load_json(data: str | bytes, *, limit: int = MAX_MESSAGE_BYTES) -> dict[str, Any]:
    """Parse a small JSON object without duplicate keys or non-finite constants."""
    raw = data.encode() if isinstance(data, str) else data
    if len(raw) > limit:
        raise ValueError("Message exceeds the size limit")
    result = json.loads(raw, object_pairs_hook=_unique, parse_constant=_invalid_number)
    if not isinstance(result, dict):
        raise ValueError("A JSON object is required")
    return cast(dict[str, Any], result)


def canonical(value: dict[str, Any]) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def identifier(value: object) -> str:
    if not isinstance(value, str) or IDENTIFIER.fullmatch(value) is None:
        raise ValueError("Invalid Realtime resource identifier")
    return value


@dataclass(frozen=True)
class Invite:
    """Publication locator only: no API secret, pairing key, SDP or motor profile."""

    app_id: str
    session_id: str
    channel: str
    nonce: str

    def __post_init__(self) -> None:
        for value in (self.app_id, self.session_id, self.channel):
            identifier(value)
        if NONCE.fullmatch(self.nonce) is None:
            raise ValueError("Invalid pairing nonce")

    def encode(self) -> str:
        data = canonical(
            {
                "v": 1,
                "app": self.app_id,
                "session": self.session_id,
                "channel": self.channel,
                "nonce": self.nonce,
            }
        )
        return base64.urlsafe_b64encode(data.encode()).decode().rstrip("=")

    @classmethod
    def parse(cls, token: str) -> Invite:
        if len(token) > 2048 or re.fullmatch(r"[A-Za-z0-9_-]+", token) is None:
            raise ValueError("Invalid pairing invite")
        data = load_json(base64.urlsafe_b64decode(token + "=" * (-len(token) % 4)))
        if (
            set(data) != {"v", "app", "session", "channel", "nonce"}
            or type(data["v"]) is not int
            or data["v"] != 1
        ):
            raise ValueError("Unsupported pairing invite")
        return cls(
            identifier(data["app"]),
            identifier(data["session"]),
            identifier(data["channel"]),
            identifier(data["nonce"]),
        )


class Codec:
    """One process/session: authenticate direction and nonce, reject replay and stale data."""

    def __init__(
        self,
        key: str,
        nonce: str,
        role: Literal["robot", "operator"],
        *,
        now: Callable[[], float] = time.time,
    ) -> None:
        if (
            role not in ("robot", "operator")
            or not 32 <= len(key.encode()) <= 256
            or NONCE.fullmatch(nonce) is None
        ):
            raise ValueError("Use a 32–256 byte pairing key and a fresh valid invite")
        self._key = key.encode()
        self._nonce, self._role, self._now = nonce, role, now
        self._sender = secrets.token_hex(16)
        self._sent = 0
        self._received: dict[str, int] = {}

    def encode(self, payload: dict[str, Any]) -> str:
        self._sent += 1
        body = {
            "v": 1,
            "nonce": self._nonce,
            "role": self._role,
            "sender": self._sender,
            "seq": self._sent,
            "sent_ms": round(self._now() * 1000),
            "payload": payload,
        }
        mac = hmac.new(self._key, canonical(body).encode(), hashlib.sha256).hexdigest()
        result = canonical({**body, "mac": mac})
        if len(result.encode()) > MAX_MESSAGE_BYTES:
            raise ValueError("Message exceeds the size limit")
        return result

    def decode(self, message: str | bytes) -> dict[str, Any]:
        body = load_json(message)
        if set(body) != {"v", "nonce", "role", "sender", "seq", "sent_ms", "payload", "mac"}:
            raise ValueError("Invalid message envelope")
        mac = body.pop("mac")
        expected = hmac.new(self._key, canonical(body).encode(), hashlib.sha256).hexdigest()
        if (
            not isinstance(mac, str)
            or re.fullmatch(r"[a-f0-9]{64}", mac) is None
            or not hmac.compare_digest(mac, expected)
        ):
            raise ValueError("Message authentication failed")
        remote_role = "operator" if self._role == "robot" else "robot"
        if (
            type(body["v"]) is not int
            or body["v"] != 1
            or body["nonce"] != self._nonce
            or body["role"] != remote_role
        ):
            raise ValueError("Message belongs to a different session or direction")
        sender = body["sender"]
        if not isinstance(sender, str) or NONCE.fullmatch(sender) is None:
            raise ValueError("Invalid message sender")
        if sender not in self._received and len(self._received) >= 128:
            raise ValueError(
                "Pairing has reached its operator-session limit; restart the robot endpoint"
            )
        sequence, stamp = body["seq"], body["sent_ms"]
        if (
            type(sequence) is not int
            or sequence <= self._received.get(sender, 0)
            or type(stamp) is not int
        ):
            raise ValueError("Repeated or malformed message sequence")
        if abs(self._now() * 1000 - stamp) > 5000:
            raise ValueError("Stale message or unsynchronized clocks")
        if not isinstance(body["payload"], dict):
            raise ValueError("Message payload must be an object")
        self._received[sender] = sequence
        return cast(dict[str, Any], body["payload"])


@dataclass(frozen=True)
class Command:
    """Small allowlist: status, stop, and fixed simulation tasks; no raw joint writes."""

    name: Literal["status", "stop", "run"]
    request_id: str
    task: str | None = None

    @classmethod
    def parse(cls, payload: dict[str, Any]) -> Command:
        command = payload.get("command")
        request_id = payload.get("id")
        if not isinstance(request_id, str) or NONCE.fullmatch(request_id) is None:
            raise ValueError("A fresh request ID is required")
        if command in ("status", "stop") and set(payload) == {"command", "id"}:
            return cls(cast(Literal["status", "stop"], command), request_id)
        if (
            command == "run"
            and set(payload) == {"command", "task", "id"}
            and payload["task"] in TASKS
        ):
            return cls("run", request_id, payload["task"])
        raise ValueError("Only status, stop, and the fixed hold/joint1-nudge tasks are accepted")
