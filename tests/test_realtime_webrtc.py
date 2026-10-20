"""Exercise real aiortc/SCTP peers against an in-process SFU contract fake, without cloud credentials."""

import asyncio
import json

import httpx
import pytest

from inspect_robots_b601._realtime.protocol import Codec, Invite
from inspect_robots_b601._realtime.sfu import Config, Link


def test_native_webrtc_message_and_reply_with_distinct_endpoint_channel_ids():
    rtc = pytest.importorskip("aiortc")

    async def run():
        peers, channels, publications = {}, {}, {}

        async def server(request):
            body = json.loads(request.content) if request.content else {}
            parts = request.url.path.split("/")
            if parts[-2:] == ["sessions", "new"]:
                session = f"session-{len(peers)}"
                peers[session] = rtc.RTCPeerConnection(rtc.RTCConfiguration(iceServers=[]))
                channels[session] = {}
                return httpx.Response(200, json={"sessionId": session})
            session = parts[parts.index("sessions") + 1]
            peer = peers[session]
            if parts[-1] == "establish":
                channels[session][0] = peer.createDataChannel(
                    "server-events", negotiated=True, id=0
                )
                await peer.setLocalDescription(await peer.createOffer())
                return httpx.Response(
                    200,
                    json={
                        "dataChannel": {"id": 0},
                        "sessionDescription": {"type": "offer", "sdp": peer.localDescription.sdp},
                    },
                )
            if parts[-1] == "renegotiate":
                await peer.setRemoteDescription(
                    rtc.RTCSessionDescription(**body["sessionDescription"])
                )
                return httpx.Response(200, json={})
            if parts[-1] == "new":
                resource = body["dataChannels"][0]
                channel_id = 2 if resource["location"] == "local" else 4
                channel = peer.createDataChannel(
                    resource["dataChannelName"], negotiated=True, id=channel_id
                )
                channels[session][channel_id] = channel
                if resource["location"] == "local":
                    targets = []
                    publications[(session, resource["dataChannelName"])] = (channel, targets)

                    def publish(message):
                        for subscriber, gate in targets:
                            if gate[0] and subscriber.readyState == "open":
                                subscriber.send(message)

                    channel.on("message", publish)
                else:
                    publisher, targets = publications[
                        (resource["sessionId"], resource["dataChannelName"])
                    ]
                    gate = [False]
                    targets.append((channel, gate))

                    def reply(message):
                        if not gate[0]:
                            assert message == "ready"
                            gate[0] = True
                        elif resource["canReply"]:
                            publisher.send(message)

                    channel.on("message", reply)
                return httpx.Response(200, json={"dataChannels": [{"id": channel_id}]})
            if parts[-1] == "close":
                for item in body["dataChannels"]:
                    channels[session][item["id"]].close()
                return httpx.Response(200, json=body)
            raise AssertionError(request.url.path)

        config = Config(
            "test-app", "test-only-app-secret-long-enough", "test-only-pairing-key-32-characters"
        )

        def factory(_):
            return rtc.RTCPeerConnection(rtc.RTCConfiguration(iceServers=[]))

        publisher = Link(
            config, transport=httpx.MockTransport(server), peer_factory=factory, timeout=5
        )
        subscriber = Link(
            config, transport=httpx.MockTransport(server), peer_factory=factory, timeout=5
        )
        try:
            await publisher.connect("robot-channel")
            invite = Invite(config.app_id, publisher.session_id, "robot-channel", "a" * 32)
            await subscriber.connect(invite.channel, remote=invite)
            robot_codec = Codec(config.pairing_key, invite.nonce, "robot")
            operator_codec = Codec(config.pairing_key, invite.nonce, "operator")
            request = {"id": "b" * 32, "command": "status"}
            subscriber.send(operator_codec.encode(request))
            assert robot_codec.decode(await publisher.receive(timeout=5)) == request
            result = {"id": request["id"], "event": "status", "state": [0.0] * 6}
            publisher.send(robot_codec.encode(result))
            assert operator_codec.decode(await subscriber.receive(timeout=5)) == result
        finally:
            try:
                await subscriber.close()
                await publisher.close()
            finally:
                for peer in peers.values():
                    await peer.close()

    asyncio.run(run())
