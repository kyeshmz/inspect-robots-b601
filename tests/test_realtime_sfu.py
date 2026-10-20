"""The native endpoint follows Cloudflare's allocation/SDP/cleanup contract."""

import asyncio
import json
import sys
from types import SimpleNamespace

import httpx
import pytest

from inspect_robots_b601._realtime.protocol import Invite
from inspect_robots_b601._realtime.sfu import Config, Link

CONFIG = Config(
    "test-app", "test-only-app-secret-long-enough", "test-only-pairing-key-32-characters"
)


class Channel:
    def __init__(self, label, channel_id):
        self.label, self.id = label, channel_id
        self.readyState, self.bufferedAmount = "open", 0
        self.handlers, self.sent = {}, []

    def on(self, name, handler):
        self.handlers[name] = handler

    def send(self, message):
        self.sent.append(message)

    def close(self):
        self.readyState = "closed"


class Peer:
    def __init__(self, config):
        self.connectionState, self.iceGatheringState = "connected", "complete"
        self.localDescription = None
        self.handlers, self.channels, self.events = {}, [], []

    def on(self, name, handler):
        self.handlers[name] = handler

    async def setRemoteDescription(self, value):
        assert value.type == "offer"
        self.events.append("offer")

    async def createAnswer(self):
        return SimpleNamespace(type="answer", sdp="fake-answer")

    async def setLocalDescription(self, value):
        self.localDescription = value
        self.events.append("answer")

    def createDataChannel(self, label, **options):
        assert options["negotiated"] is True and options["ordered"] is True
        channel = Channel(label, options["id"])
        self.channels.append(channel)
        return channel

    async def close(self):
        self.connectionState = "closed"


@pytest.fixture
def fake_aiortc(monkeypatch):
    monkeypatch.setitem(
        sys.modules,
        "aiortc",
        SimpleNamespace(RTCSessionDescription=lambda **values: SimpleNamespace(**values)),
    )


def api(requests, *, failed=None, malformed=False, closing_error=False):
    def respond(request):
        assert request.url.host == "rtc.live.cloudflare.com"
        assert request.headers["Authorization"] == "Bearer " + CONFIG.app_secret
        body = json.loads(request.content) if request.content else None
        requests.append((request.method, request.url.path, body))
        path = request.url.path
        if failed == "http":
            return httpx.Response(403, text=CONFIG.app_secret)
        if path.endswith("/sessions/new"):
            return httpx.Response(200, json={"sessionId": "local-session"})
        if path.endswith("/establish"):
            return httpx.Response(
                200,
                json={
                    "dataChannel": {"id": 0},
                    "sessionDescription": {"type": "offer", "sdp": "fake-offer"},
                },
            )
        if path.endswith("/renegotiate"):
            return httpx.Response(200, json={})
        if path.endswith("/datachannels/new"):
            if failed == "partial":
                return httpx.Response(
                    200, json={"dataChannels": [{"id": 2, "errorCode": "injected"}]}
                )
            return httpx.Response(200, json={"dataChannels": [] if malformed else [{"id": 2}]})
        if path.endswith("/close"):
            return httpx.Response(
                200, json={"dataChannels": [] if closing_error else body["dataChannels"]}
            )
        raise AssertionError(path)

    return httpx.MockTransport(respond)


def test_publisher_allocates_before_opening_and_cleans_only_owned_ids(fake_aiortc):
    async def run():
        requests = []
        link = Link(CONFIG, transport=api(requests), peer_factory=Peer)
        await link.connect("b601-channel")
        assert link.peer.events == ["offer", "answer"]
        assert requests[2][0] == "PUT" and requests[2][1].endswith("/renegotiate")
        assert requests[3][2] == {
            "dataChannels": [{"location": "local", "dataChannelName": "b601-channel"}]
        }
        link.peer.channels[0].handlers["message"]("from-subscriber")
        assert await link.receive() == "from-subscriber"
        link.send("to-subscriber")
        assert link.peer.channels[0].sent == ["to-subscriber"]
        await link.close()
        assert requests[-1][2] == {"dataChannels": [{"id": 0}, {"id": 2}]}
        assert link.peer.connectionState == "closed"
        await link.close()

    asyncio.run(run())


def test_subscriber_requests_reply_and_sends_sfu_readiness_before_application_messages(fake_aiortc):
    async def run():
        requests = []
        link = Link(CONFIG, transport=api(requests), peer_factory=Peer)
        invite = Invite(CONFIG.app_id, "publisher-session", "robot-channel", "a" * 32)
        await link.connect(invite.channel, remote=invite)
        assert requests[3][2]["dataChannels"][0] == {
            "location": "remote",
            "sessionId": "publisher-session",
            "dataChannelName": "robot-channel",
            "canReply": True,
            "waitForAck": True,
        }
        assert link.peer.channels[0].sent == ["ready"]
        await link.close()

    asyncio.run(run())


@pytest.mark.parametrize("failure", ["http", "partial", "malformed"])
def test_provider_failures_are_not_retried_and_never_print_credentials(fake_aiortc, failure):
    async def run():
        requests = []
        link = Link(
            CONFIG,
            transport=api(requests, failed=failure, malformed=failure == "malformed"),
            peer_factory=Peer,
        )
        with pytest.raises(RuntimeError) as error:
            await link.connect("b601-channel")
        assert CONFIG.app_secret not in str(error.value)
        await link.close()
        assert sum(path.endswith("/sessions/new") for _, path, _ in requests) == 1
        assert link.peer.connectionState == "closed"
        if failure == "partial":
            assert requests[-1][2] == {"dataChannels": [{"id": 0}, {"id": 2}]}

    asyncio.run(run())


def test_cleanup_uncertainty_still_closes_local_peer(fake_aiortc):
    async def run():
        link = Link(CONFIG, transport=api([], closing_error=True), peer_factory=Peer)
        await link.connect("b601-channel")
        with pytest.raises(RuntimeError, match="cleanup"):
            await link.close()
        assert link.peer.connectionState == "closed"

    asyncio.run(run())


@pytest.mark.parametrize("failure", ["large", "overflow", "send", "closed"])
def test_message_buffers_and_connection_failures_fail_closed(fake_aiortc, failure):
    async def run():
        link = Link(CONFIG, transport=api([]), peer_factory=Peer)
        await link.connect("b601-channel")
        if failure == "large":
            link._message("x" * 17000)
        elif failure == "overflow":
            for _ in range(17):
                link._message("queued")
        elif failure == "send":
            link.peer.channels[0].bufferedAmount = 70_000
            with pytest.raises(RuntimeError, match="backpressure"):
                link.send("message")
        else:
            link.peer.connectionState = "failed"
            link.peer.handlers["connectionstatechange"]()
        with pytest.raises(RuntimeError):
            await link.receive()
        with pytest.raises(RuntimeError):
            link.send("message")
        await link.close()

    asyncio.run(run())


def test_invite_for_another_app_fails_before_an_api_call(fake_aiortc):
    async def run():
        requests = []
        link = Link(CONFIG, transport=api(requests), peer_factory=Peer)
        with pytest.raises(ValueError, match="another"):
            await link.connect("name", remote=Invite("other-app", "session", "name", "a" * 32))
        assert requests == []
        await link.close()

    asyncio.run(run())


def test_config_does_not_repr_secrets_and_requires_complete_configuration():
    env = {
        "CLOUDFLARE_REALTIME_APP_ID": CONFIG.app_id,
        "CLOUDFLARE_REALTIME_APP_SECRET": CONFIG.app_secret,
        "B601_REALTIME_KEY": CONFIG.pairing_key,
    }
    assert Config.from_env(env) == CONFIG
    assert CONFIG.app_secret not in repr(CONFIG) and CONFIG.pairing_key not in repr(CONFIG)
    for missing in env:
        with pytest.raises(ValueError):
            Config.from_env({key: value for key, value in env.items() if key != missing})
    with pytest.raises(ValueError, match="together"):
        Config.from_env({**env, "B601_REALTIME_TURN_URL": "turn:test"})


def test_transport_failure_is_single_use_and_secrets_are_not_reflected(fake_aiortc):
    async def run():
        calls = []

        def unavailable(request):
            calls.append(request)
            raise httpx.ConnectError(CONFIG.app_secret)

        link = Link(CONFIG, transport=httpx.MockTransport(unavailable), peer_factory=Peer)
        with pytest.raises(RuntimeError) as failure:
            await link.connect("name")
        assert CONFIG.app_secret not in str(failure.value)
        with pytest.raises(RuntimeError, match="single-use"):
            await link.connect("name")
        assert len(calls) == 1
        await link.close()

    asyncio.run(run())
