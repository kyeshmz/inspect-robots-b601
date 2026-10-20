"""Trusted operator/robot CLIs over Cloudflare Realtime SFU, without a tunnel."""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import secrets
import sys
from collections.abc import Sequence
from concurrent.futures import ThreadPoolExecutor
from importlib.metadata import version
from pathlib import Path
from typing import Any, Literal, cast

from ._realtime.protocol import Codec, Command, Invite
from ._realtime.robot import Robot
from ._realtime.sfu import Config, Link


async def _robot(args: argparse.Namespace, config: Config) -> None:
    nonce = secrets.token_hex(16)
    codec = Codec(config.pairing_key, nonce, "robot")
    controller = Robot(
        cast(Literal["echo", "ros-mock", "motorbridge-read"], args.backend),
        profile=args.profile,
        state_dir=args.state_dir,
        log_dir=args.log_dir,
    )
    link = Link(config)
    executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="b601-local-owner")
    active: asyncio.Future[dict[str, Any]] | None = None
    active_id = ""
    last_message = 0.0
    seen: set[str] = set()
    primary: BaseException | None = None
    loop = asyncio.get_running_loop()
    try:
        await link.connect("b601-" + nonce)
        invite = Invite(config.app_id, link.session_id, "b601-" + nonce, nonce)
        print("Operator command (invite contains no credentials):", flush=True)
        print(f"b601-realtime operator --invite {invite.encode()}", flush=True)
        while True:
            if active is not None and active.done():
                try:
                    result = active.result()
                    reply = {"id": active_id, "event": "result", **result}
                except Exception as error:
                    reply = {"id": active_id, "event": "error", "error": str(error)}
                active = None
                link.send(codec.encode(reply))
            if active is not None and loop.time() - last_message > 3.0:
                controller.stop()
            try:
                message = await link.receive(timeout=0.5)
            except asyncio.TimeoutError:
                continue
            command = Command.parse(codec.decode(message))
            last_message = loop.time()
            if command.name != "status" and command.request_id in seen:
                reply = {
                    "id": command.request_id,
                    "event": "duplicate",
                    "detail": "Request already received; check status rather than replaying it",
                }
            elif command.name != "status" and len(seen) >= 1024:
                raise RuntimeError(
                    "Request history reached its bound; restart locally with a fresh invite"
                )
            elif command.name == "stop":
                controller.stop()
                reply = {
                    "id": command.request_id,
                    "event": "stopped",
                    "detail": "Local stop latched; this is not a physical emergency-stop guarantee",
                }
            elif active is not None:
                reply = {"id": command.request_id, "event": "busy"}
            elif command.name == "status":
                try:
                    status = await loop.run_in_executor(executor, controller.status)
                    reply = {"id": command.request_id, "event": "status", **status}
                except Exception as error:
                    reply = {"id": command.request_id, "event": "error", "error": str(error)}
            else:
                assert command.task is not None
                active_id = command.request_id
                active = loop.run_in_executor(executor, controller.run, command.task)
                reply = {"id": command.request_id, "event": "accepted"}
            if command.name != "status":
                seen.add(command.request_id)
            link.send(codec.encode(reply))
    except BaseException as error:
        primary = error
        raise
    finally:
        controller.stop()
        if active is not None:
            await asyncio.gather(active, return_exceptions=True)
        cleanup = []
        try:
            await loop.run_in_executor(executor, controller.close)
        except Exception as error:
            cleanup.append(f"robot cleanup: {error}")
        finally:
            executor.shutdown(wait=True)
            try:
                await link.close()
            except Exception as error:
                cleanup.append(f"Realtime cleanup: {error}")
        if cleanup:
            prefix = "" if primary is None else f"{type(primary).__name__}: {primary}; "
            raise RuntimeError(prefix + "; ".join(cleanup)) from primary


async def _operator(args: argparse.Namespace, config: Config) -> bool:
    invite = Invite.parse(args.invite)
    codec = Codec(config.pairing_key, invite.nonce, "operator")
    link = Link(config)
    request_id = secrets.token_hex(16)
    payload: dict[str, Any] = {"id": request_id, "command": args.request}
    if args.request in ("hold", "joint1-nudge"):
        payload.update(command="run", task=args.request)
    primary: BaseException | None = None
    try:
        await link.connect(invite.channel, remote=invite)
        link.send(codec.encode(payload))
        deadline = asyncio.get_running_loop().time() + 20.0
        while True:
            remaining = deadline - asyncio.get_running_loop().time()
            if remaining <= 0:
                raise RuntimeError("Command outcome unconfirmed; do not automatically repeat it")
            try:
                reply = codec.decode(await link.receive(timeout=min(1.0, remaining)))
            except asyncio.TimeoutError:
                link.send(codec.encode({"id": secrets.token_hex(16), "command": "status"}))
                continue
            if reply.get("id") != request_id:
                continue
            print(json.dumps(reply, allow_nan=False), flush=True)
            if getattr(args, "watch", False) and reply.get("event") == "status":
                await asyncio.sleep(1.0)
                request_id = secrets.token_hex(16)
                payload["id"] = request_id
                link.send(codec.encode(payload))
                deadline = asyncio.get_running_loop().time() + 20.0
                continue
            if reply.get("event") in ("status", "stopped", "result", "error", "busy", "duplicate"):
                return (
                    reply.get("event") not in ("error", "busy", "duplicate")
                    and reply.get("passed", True) is True
                )
    except BaseException as error:
        primary = error
        raise
    finally:
        try:
            await link.close()
        except Exception as cleanup:
            prefix = "" if primary is None else f"{type(primary).__name__}: {primary}; "
            raise RuntimeError(prefix + f"Realtime cleanup unconfirmed: {cleanup}") from primary


def main(argv: Sequence[str] | None = None) -> int:
    """Connect two trusted backend computers; physical motion is never a remote capability."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--version",
        action="version",
        version=f"inspect-robots-b601 {version('inspect-robots-b601')}",
    )
    roles = parser.add_subparsers(dest="role", required=True)
    robot = roles.add_parser("robot", help="Run beside the robot; all robot calls stay local")
    robot.add_argument(
        "--backend", choices=("echo", "ros-mock", "motorbridge-read"), default="echo"
    )
    robot.add_argument("--profile", type=Path)
    robot.add_argument("--state-dir", type=Path)
    robot.add_argument("--log-dir", default="logs/realtime")
    operator = roles.add_parser("operator", help="Send one request from the operator computer")
    operator.add_argument("--invite", required=True)
    operator.add_argument(
        "--watch",
        action="store_true",
        help="Keep one connection open and print status about once per second",
    )
    operator.add_argument(
        "--request", choices=("status", "hold", "joint1-nudge", "stop"), default="status"
    )
    args = parser.parse_args(argv)
    if args.role == "operator" and args.watch and args.request != "status":
        parser.error(
            "--watch is observation-only; use a separate explicit task request after closing it"
        )
    if args.role == "robot":
        if args.backend == "ros-mock" and (
            os.environ.get("ROS_DOMAIN_ID") != "61"
            or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
            or os.environ.get("ROS_DISCOVERY_SERVER")
            or os.environ.get("ROS_STATIC_PEERS")
        ):
            parser.error(
                "Mock backend requires isolated localhost ROS domain 61; remote discovery is forbidden"
            )
        if args.backend == "motorbridge-read" and (args.profile is None or args.state_dir is None):
            parser.error("motorbridge-read needs --profile and the existing shared --state-dir")
        if args.backend == "motorbridge-read" and (
            not args.profile.is_absolute()
            or not args.state_dir.is_absolute()
            or not args.state_dir.is_dir()
        ):
            parser.error(
                "Use an absolute local profile and the existing absolute persistent state directory"
            )
        if args.backend != "motorbridge-read" and (
            args.profile is not None or args.state_dir is not None
        ):
            parser.error("Profile/state paths belong only to the read-only physical backend")
    try:
        config = Config.from_env()
        if args.role == "robot":
            print(
                f"Robot-side backend: {args.backend}. Physical remote activation is disabled.",
                flush=True,
            )
            asyncio.run(_robot(args, config))
            return 0
        return 0 if asyncio.run(_operator(args, config)) else 1
    except KeyboardInterrupt:
        return 130
    except ModuleNotFoundError:
        print(
            "Install the realtime extra on this computer: inspect-robots-b601[realtime]",
            file=sys.stderr,
        )
        return 1
    except Exception as error:
        print(f"Realtime endpoint stopped: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
