"""One Cloudflare Realtime SFU session, negotiated channel and bounded teardown."""

from __future__ import annotations

import asyncio
import os
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, cast

from .protocol import MAX_MESSAGE_BYTES, Invite, identifier

if TYPE_CHECKING:
    import httpx


@dataclass(frozen=True)
class Config:
    """Privileged backend credentials; never include secrets in an invite or report."""

    app_id: str
    app_secret: str = field(repr=False)
    pairing_key: str = field(repr=False)
    turn_url: str = ""
    turn_username: str = field(default="", repr=False)
    turn_credential: str = field(default="", repr=False)

    def __post_init__(self) -> None:
        identifier(self.app_id)
        if (
            len(self.app_secret) < 16
            or not self.app_secret.isascii()
            or any(char.isspace() for char in self.app_secret)
            or not 32 <= len(self.pairing_key.encode()) <= 256
        ):
            raise ValueError(
                "Set CLOUDFLARE_REALTIME_APP_SECRET and a 32–256 byte B601_REALTIME_KEY"
            )
        turn = (self.turn_url, self.turn_username, self.turn_credential)
        if any(turn) and not all(turn):
            raise ValueError("TURN URL, username and credential must be configured together")
        if self.turn_url and not self.turn_url.startswith(("turn:", "turns:")):
            raise ValueError("TURN URL must use turn: or turns:")
        if "@" in self.turn_url:
            raise ValueError(
                "Keep TURN credentials in their separate private variables, not in the URL"
            )

    @classmethod
    def from_env(cls, env: Mapping[str, str] | None = None) -> Config:
        values = os.environ if env is None else env
        required = (
            "CLOUDFLARE_REALTIME_APP_ID",
            "CLOUDFLARE_REALTIME_APP_SECRET",
            "B601_REALTIME_KEY",
        )
        missing = [name for name in required if not values.get(name)]
        if missing:
            raise ValueError(
                "Load the private Realtime environment on this computer; missing: "
                + ", ".join(missing)
            )
        app = values.get("CLOUDFLARE_REALTIME_APP_ID", "")
        secret, key = (
            values.get("CLOUDFLARE_REALTIME_APP_SECRET", ""),
            values.get("B601_REALTIME_KEY", ""),
        )
        turn = tuple(
            values.get(name, "")
            for name in (
                "B601_REALTIME_TURN_URL",
                "B601_REALTIME_TURN_USERNAME",
                "B601_REALTIME_TURN_CREDENTIAL",
            )
        )
        return cls(app, secret, key, *turn)


def _peer(config: Config) -> Any:
    from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection

    servers = [RTCIceServer(urls="stun:stun.cloudflare.com:3478")]
    if config.turn_url:
        servers.append(
            RTCIceServer(
                urls=config.turn_url,
                username=config.turn_username,
                credential=config.turn_credential,
            )
        )
    return RTCPeerConnection(RTCConfiguration(iceServers=servers))


class Link:
    """An outbound-only native endpoint; no tunnel, listener, retry or automatic reconnect."""

    def __init__(
        self,
        config: Config,
        *,
        transport: httpx.AsyncBaseTransport | None = None,
        peer_factory: Callable[[Config], Any] = _peer,
        timeout: float = 20.0,
    ) -> None:
        import httpx

        self.config, self.timeout = config, timeout
        self.peer = peer_factory(config)
        self._http = httpx.AsyncClient(
            base_url=f"https://rtc.live.cloudflare.com/v1/apps/{config.app_id}",
            headers={"Authorization": f"Bearer {config.app_secret}"},
            timeout=15.0,
            transport=transport,
            trust_env=False,
        )
        self.session_id = ""
        self._ids: set[int] = set()
        self._channels: list[Any] = []
        self._channel: Any = None
        self._messages: asyncio.Queue[str | bytes | None] = asyncio.Queue(maxsize=16)
        self._fault: str | None = None
        self._closed = False
        self._used = False
        self.peer.on("datachannel", self._channels.append)
        self.peer.on("connectionstatechange", self._connection_changed)

    def _fail(self, message: str) -> None:
        if not self._closed and self._fault is None:
            self._fault = message
            if not self._messages.full():
                self._messages.put_nowait(None)

    def _connection_changed(self) -> None:
        if self.peer.connectionState in ("failed", "disconnected", "closed"):
            self._fail("Realtime connection ended; start a fresh local session")

    def _message(self, message: object) -> None:
        if (
            not isinstance(message, (str, bytes))
            or len(message.encode() if isinstance(message, str) else message) > MAX_MESSAGE_BYTES
        ):
            self._fail("Invalid or oversized Realtime message")
        elif self._messages.full():
            self._fail("Realtime receive buffer overflow; no queued commands will be replayed")
        else:
            self._messages.put_nowait(message)

    async def _wait(self, ready: Callable[[], bool]) -> None:
        deadline = asyncio.get_running_loop().time() + self.timeout
        while not ready():
            if self._fault or asyncio.get_running_loop().time() >= deadline:
                raise RuntimeError(self._fault or "Realtime negotiation timed out")
            await asyncio.sleep(0.02)
        if self._fault:
            raise RuntimeError(self._fault)

    async def _api(
        self,
        method: str,
        path: str,
        payload: dict[str, Any] | None = None,
        *,
        closing: bool = False,
    ) -> dict[str, Any]:
        try:
            if payload is None:
                response = await self._http.request(method, path)
            else:
                response = await self._http.request(method, path, json=payload)
        except Exception:
            raise RuntimeError("Cloudflare Realtime request failed; no automatic retry") from None
        if not response.is_success:
            raise RuntimeError(
                f"Cloudflare Realtime HTTP {response.status_code}; response body omitted"
            )
        data = response.json()
        if not isinstance(data, dict):
            raise RuntimeError("Malformed Cloudflare Realtime response")
        resources = data.get("dataChannels", [data["dataChannel"]] if "dataChannel" in data else [])
        if not isinstance(resources, list) or any(not isinstance(item, dict) for item in resources):
            raise RuntimeError("Malformed DataChannel allocation response")
        if path.endswith(("/establish", "/new")):
            for item in resources:
                value = item.get("id")
                if type(value) is int and 0 <= value < 65535:
                    self._ids.add(value)
        if data.get("errorCode") or any(
            item.get("errorCode") and not (closing and item["errorCode"] == "close_track_error")
            for item in resources
        ):
            raise RuntimeError("Cloudflare Realtime reported an operation/resource failure")
        if closing and {item.get("id") for item in resources} != self._ids:
            raise RuntimeError("Cloudflare did not confirm every allocated channel's cleanup")
        return cast(dict[str, Any], data)

    async def _answer(self, response: dict[str, Any]) -> None:
        from aiortc import RTCSessionDescription

        offer = response.get("sessionDescription")
        if (
            not isinstance(offer, dict)
            or offer.get("type") != "offer"
            or not isinstance(offer.get("sdp"), str)
        ):
            raise RuntimeError("Expected a Cloudflare DataChannel transport offer")
        await asyncio.wait_for(
            self.peer.setRemoteDescription(RTCSessionDescription(sdp=offer["sdp"], type="offer")),
            self.timeout,
        )
        answer = await asyncio.wait_for(self.peer.createAnswer(), self.timeout)
        await asyncio.wait_for(self.peer.setLocalDescription(answer), self.timeout)
        await self._wait(lambda: self.peer.iceGatheringState == "complete")
        answer = self.peer.localDescription
        if answer is None or answer.type != "answer":
            raise RuntimeError("No complete local WebRTC answer")
        await self._api(
            "PUT",
            f"/sessions/{self.session_id}/renegotiate",
            {"sessionDescription": {"type": "answer", "sdp": answer.sdp}},
        )

    async def connect(self, channel: str, *, remote: Invite | None = None) -> None:
        if self._used or self._closed:
            raise RuntimeError("Realtime links are single-use")
        self._used = True
        identifier(channel)
        if remote is not None and remote.app_id != self.config.app_id:
            raise ValueError("Invite belongs to another Cloudflare application")
        created = await self._api("POST", "/sessions/new")
        self.session_id = identifier(created.get("sessionId"))
        response = await self._api(
            "POST",
            f"/sessions/{self.session_id}/datachannels/establish",
            {"dataChannel": {"location": "remote", "dataChannelName": "server-events"}},
        )
        reserved = response.get("dataChannel")
        if (
            not isinstance(reserved, dict)
            or type(reserved.get("id")) is not int
            or not 0 <= reserved["id"] < 65535
        ):
            raise RuntimeError("Invalid reserved DataChannel allocation")
        await self._answer(response)
        await self._wait(lambda: self.peer.connectionState == "connected")
        request: dict[str, Any] = {"location": "local", "dataChannelName": channel}
        if remote is not None:
            request = {
                "location": "remote",
                "sessionId": remote.session_id,
                "dataChannelName": remote.channel,
                "canReply": True,
                "waitForAck": True,
            }
        existing_ids = self._ids.copy()
        allocated = await self._api(
            "POST", f"/sessions/{self.session_id}/datachannels/new", {"dataChannels": [request]}
        )
        entries = allocated.get("dataChannels")
        if not isinstance(entries, list) or len(entries) != 1:
            raise RuntimeError("Expected exactly one application DataChannel")
        channel_id = entries[0].get("id")
        if type(channel_id) is not int or not 0 <= channel_id < 65535 or channel_id in existing_ids:
            raise RuntimeError("Invalid negotiated DataChannel ID")
        if "sessionDescription" in allocated:
            await self._answer(allocated)
        self._channel = self.peer.createDataChannel(
            channel, negotiated=True, id=channel_id, ordered=True
        )
        self._channels.append(self._channel)
        self._channel.on("message", self._message)
        self._channel.on("close", lambda: self._fail("Realtime data channel closed"))
        await self._wait(lambda: self._channel.readyState == "open")
        if remote is not None:
            self.send(
                "ready"
            )  # SFU readiness gate consumes this before signed application messages.

    def send(self, message: str) -> None:
        if (
            self._fault
            or self._closed
            or self._channel is None
            or self._channel.readyState != "open"
        ):
            raise RuntimeError(self._fault or "Realtime data channel is not open")
        if len(message.encode()) > MAX_MESSAGE_BYTES or self._channel.bufferedAmount > 65_536:
            self._fail("Realtime send buffer exceeded its bound")
            raise RuntimeError("Realtime send backpressure; session halted")
        self._channel.send(message)

    async def receive(self, timeout: float = 15.0) -> str | bytes:
        if self._fault or self._closed:
            raise RuntimeError(self._fault or "Realtime link is closed")
        message = await asyncio.wait_for(self._messages.get(), timeout)
        if message is None or self._fault:
            raise RuntimeError(self._fault or "Realtime link ended")
        return message

    async def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if self.session_id and self._ids:
                await self._api(
                    "PUT",
                    f"/sessions/{self.session_id}/datachannels/close",
                    {"dataChannels": [{"id": value} for value in sorted(self._ids)]},
                    closing=True,
                )
                self._ids.clear()
        finally:
            try:
                for channel in self._channels:
                    channel.close()
                await self.peer.close()
            finally:
                await self._http.aclose()
