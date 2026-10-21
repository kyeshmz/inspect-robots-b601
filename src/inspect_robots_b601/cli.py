"""inspect-robots-b601: operator commands for a B601-RS on a direct Motorbridge/SocketCAN bus.

Run one command at a time. Commands that move the arm require
--confirm-supported-arm: the operator states that the arm and load are
supported and an independent power cutoff is within reach. check never opens
CAN; read and recover open it but never enable a motor. recover also needs the
confirmation, because it re-arms motion.

Defaults can be set once per shell (or in the checkout's .env):

    B601_PROFILE    robot profile (default: the bundled stock B601-RS on can0)
    B601_STATE_DIR  stop latch, beside logs/ (default: .hardware-state)
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

from .embodiment import DEFAULT_PROFILE
from .hardware import Motor, MotorController, PathLike, Profile

if TYPE_CHECKING:
    from .hardware.session import ArmSession

Handler = Callable[[argparse.Namespace], int]

AGENT_CONFIG = Path(__file__).with_name("defaults") / "agent.ini"
AXES = [f"joint{i}" for i in range(1, 7)] + ["gripper"]
CONFIRM_HELP = "the arm and load are supported and an independent cutoff is within reach"


def _stamp() -> str:
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())


def _load(path: PathLike) -> Profile:
    profile: Profile = json.loads(Path(path).read_text())
    return profile


def _print(report: dict[str, Any]) -> int:
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["success"] else 1


def _error(error: BaseException) -> dict[str, str]:
    return {"type": type(error).__name__, "message": str(error)}


def _interrupt_on_signals() -> None:
    # The first SIGINT/SIGTERM ends the motion so the session walks home before disabling.
    # Explicit because a background process starts with SIGINT ignored.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)


def _inspect(command: list[str]) -> int:
    from inspect_robots.cli import main as inspect_main

    return inspect_main(command)


# --- no motion -----------------------------------------------------------------------------------


def check(args: argparse.Namespace) -> int:
    """Check the profile's CAN interface and the Motorbridge install without opening either."""
    from importlib.metadata import PackageNotFoundError, version

    from .hardware.bus import channel_name, interface_metadata, interface_problems, shared_lock_path

    profile = _load(args.profile)
    channel = channel_name(profile)
    ready = True
    try:
        info = interface_metadata(channel)
    except Exception as error:
        print(f"{channel}: {error}")
        ready = False
    else:
        print(json.dumps(info, indent=2))
        for problem in interface_problems(info, profile):
            print(f"{channel}: {problem}")
            ready = False
    try:
        print(f"motorbridge: {version('motorbridge')} installed")
    except PackageNotFoundError:
        print(
            "motorbridge: not installed; python -m pip install 'inspect-robots-b601[motorbridge]'"
        )
        ready = False
    if ready:
        print(f"{channel}: ready; sessions will lock {shared_lock_path(profile)}")
    return 0 if ready else 1


def read(args: argparse.Namespace) -> int:
    """Read every motor's position and fault report, and the stop latch; never enables a motor."""
    from .hardware.session import read as read_arm

    report: dict[str, Any] = {"success": False}
    try:
        report.update(read_arm(_load(args.profile), args.state_dir), success=True)
    except BaseException as error:
        report["primary"] = _error(error)
    return _print(report)


def recover(args: argparse.Namespace) -> int:
    """Acknowledge a latched stop after fresh reads of every motor; every motor stays disabled."""
    from .hardware.session import recover as recover_arm

    report: dict[str, Any] = {"success": False}
    try:
        report.update(recover_arm(_load(args.profile), args.state_dir), success=True)
    except BaseException as error:
        report["primary"] = _error(error)
    return _print(report)


# --- motion (enables every configured motor) -----------------------------------------------------


def _session(args: argparse.Namespace, profile: Profile | None = None, **kwargs: Any) -> ArmSession:
    from .hardware.session import ArmSession

    return ArmSession(
        profile if profile is not None else _load(args.profile), args.state_dir, **kwargs
    )


def hold(args: argparse.Namespace) -> int:
    """Enable every configured motor at its measured pose, hold, walk home, then disable."""
    session = _session(args)
    report: dict[str, Any] = {"success": False}
    try:
        session.open()
        session.activate()
        start = session.observe()
        report["initial_positions"] = list(start)
        current, deviation = start, 0.0
        deadline = time.monotonic() + args.hold_seconds
        while time.monotonic() < deadline:
            time.sleep(0.25)
            current = session.observe()
            deviation = max(
                deviation, max(abs(a - b) for a, b in zip(current, start, strict=False))
            )
            if deviation > args.max_deviation:
                raise RuntimeError(
                    f"hold deviation {deviation:.6f} rad exceeds {args.max_deviation} rad"
                )
        report.update(success=True, positions=list(current), max_hold_deviation=deviation)
    except BaseException as error:
        report["primary"] = _error(error)
    finally:
        session.close()
    return _print(report)


def probe(args: argparse.Namespace) -> int:
    """Move one axis a small amount and report the measured change."""
    session = _session(args)
    report: dict[str, Any] = {"success": False}
    try:
        session.open()
        session.activate()
        before = session.observe()
        target = list(before)
        target[args.joint] += args.delta
        after = session.move(target, args.duration)
        report.update(
            success=True,
            joint=session.joints[args.joint]["name"],
            before=before,
            target=target,
            after=after,
            measured_delta=after[args.joint] - before[args.joint],
        )
    except BaseException as error:
        report["primary"] = _error(error)
    finally:
        session.close()
    return _print(report)


def _stage(text: str) -> dict[str, float]:
    offsets: dict[str, float] = {}
    for item in text.split(","):
        name, separator, value = item.strip().partition("=")
        if not separator or name not in AXES:
            raise argparse.ArgumentTypeError(f"bad stage item {item!r}")
        offsets[name] = float(value)
        if abs(offsets[name]) > 1.2:
            raise argparse.ArgumentTypeError("stage offsets must be at most 1.2 rad")
    return offsets


def pose(args: argparse.Namespace) -> int:
    """Walk staged poses from the activation pose, snap the wrist camera at each, walk home."""
    _interrupt_on_signals()
    args.out.mkdir(parents=True, exist_ok=True)
    camera = None
    if not args.no_camera:
        from .hardware.camera import RealSenseCamera

        camera = RealSenseCamera()
    profile = _load(args.profile)
    if args.gravity != "profile":
        profile["gravity_feedforward"] = args.gravity == "on"
    session = _session(args, profile, home_wait=args.home_wait)
    report: dict[str, Any] = {"success": False, "stages": []}
    try:
        if camera is not None:
            camera.start()  # Before torque: nothing may stall the hold loop once enabled.
        session.open()
        session.activate()
        if session._home is None:
            raise RuntimeError("the arm has no activation pose")
        home = list(session._home)
        report["home"] = home
        report["gravity_feedforward"] = session._gravity is not None
        for number, offsets in enumerate(args.stage, 1):
            target = list(home)
            for name, offset in offsets.items():
                target[AXES.index(name)] = home[AXES.index(name)] + offset
            print(f"stage {number}: {offsets}", flush=True)
            measured = session.go_to(
                target,
                timeout=max(abs(a - b) for a, b in zip(target, home, strict=False)) / 0.05 + 15.0,
            )
            entry: dict[str, Any] = {
                "offsets": offsets,
                "pose": target,
                "measured": list(measured),
                "lag": [round(a - b, 4) for a, b in zip(target, measured, strict=False)],
                "max_send_gap_s": round(session.max_send_gap, 4),
            }
            if camera is not None:
                from PIL import Image

                image = args.out / f"stage-{number}.jpg"
                Image.fromarray(camera.read()).save(image, quality=90)
                entry["image"] = str(image)
            report["stages"].append(entry)
            print(json.dumps(entry), flush=True)
        if args.hold_seconds:
            time.sleep(args.hold_seconds)
        report["success"] = True
    except BaseException as error:
        report["primary"] = _error(error)
    finally:
        print("walking home before disabling", flush=True)
        session.close()
        if camera is not None:
            camera.close()
        report["max_send_gap_s"] = round(session.max_send_gap, 4)
    (args.out / "report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k != "stages"}, indent=2))
    return 0 if report["success"] else 1


def gripper(args: argparse.Namespace) -> int:
    """Low-torque probe of the gripper motor alone; the arm motors stay disabled."""
    from .hardware.bus import BusClaim, channel_name
    from .hardware.session import LIMIT_TORQUE, MECH_POS, WATCHDOG, _default_controller

    profile = _load(args.profile)
    config = profile.get("gripper")
    if not config:
        raise SystemExit("the profile has no gripper")
    claim = BusClaim(profile)
    controller: MotorController | None = None
    motor: Motor | None = None
    enabled = False
    report: dict[str, Any] = {"success": False}
    try:
        controller = _default_controller(channel_name(profile))
        from motorbridge import Mode

        motor = controller.add_robstride_motor(config["motor_id"], 0xFD, config["model"])
        initial = motor.robstride_get_param_f32(MECH_POS)
        report["initial"] = initial
        motor.robstride_write_param_f32(LIMIT_TORQUE, args.torque_cap)
        motor.robstride_write_param_u32(WATCHDOG, 5000)
        if abs(motor.robstride_get_param_f32(LIMIT_TORQUE) - args.torque_cap) > 1e-6:
            raise RuntimeError("gripper torque cap readback mismatch")
        motor.ensure_mode(Mode.MIT, 1000)
        motor.send_mit(initial, 0.0, args.kp, args.kd, 0.0)
        motor.enable()
        enabled = True
        target = initial + args.delta
        started = time.monotonic()
        while (elapsed := time.monotonic() - started) < args.duration:
            motor.send_mit(
                initial + (target - initial) * (elapsed / args.duration), 0.0, args.kp, args.kd, 0.0
            )
            time.sleep(0.02)
        motor.send_mit(target, 0.0, args.kp, args.kd, 0.0)
        time.sleep(0.2)
        final = motor.robstride_get_param_f32(MECH_POS)
        report.update(success=True, target=target, final=final, measured_delta=final - initial)
    except BaseException as error:
        report["primary"] = _error(error)
    finally:
        if motor is not None:
            try:
                motor.send_mit(motor.robstride_get_param_f32(MECH_POS), 0.0, args.kp, args.kd, 0.0)
                if enabled:
                    motor.disable()
            except Exception:
                pass
        if controller is not None:
            controller.close()
        claim.close()
    return _print(report)


def agent(args: argparse.Namespace) -> int:
    """Run Inspect's agent policy on every axis through the b601 embodiment."""
    _interrupt_on_signals()
    command = [
        "run",
        "--config",
        str(args.inspect_config.resolve()),
        "--embodiment",
        "b601",
        "--policy",
        "agent",
        "-E",
        f"profile={args.profile.resolve()}",
        "-E",
        f"state_dir={args.state_dir.resolve()}",
        "-E",
        "allow_motion=true",
        "-E",
        "activate_on_reset=true",
        "-E",
        "operator_supported_arm=true",
        "-E",
        f"ready={args.ready}",
        "-P",
        f"max_llm_calls={args.max_llm_calls}",
        "--instruction",
        args.instruction,
        "--max-steps",
        str(args.max_steps),
        # The real arm never reports a success termination, so a person judges each trial.
        # The delta limiter uses the embodiment's declared per-step max_step.
        "--scorer",
        "operator",
        "--log-dir",
        str(args.log_dir or Path("logs") / f"b601-agent-{_stamp()}"),
    ]
    if args.hold_seconds > 0:
        command.extend(("-E", f"hold_seconds={args.hold_seconds}"))
    return _inspect(command)


# --- argument parsing ----------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    """The inspect-robots-b601 parser, with defaults from B601_PROFILE and B601_STATE_DIR."""
    profile = Path(os.environ.get("B601_PROFILE") or DEFAULT_PROFILE)
    state_dir = Path(os.environ.get("B601_STATE_DIR") or ".hardware-state")

    parser = argparse.ArgumentParser(
        prog="inspect-robots-b601",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    commands = parser.add_subparsers(dest="command", required=True, metavar="COMMAND")

    def command(
        name: str, handler: Handler, *, confirm: bool = False, state: bool = True
    ) -> argparse.ArgumentParser:
        sub = commands.add_parser(name, help=handler.__doc__, description=handler.__doc__)
        sub.set_defaults(handler=handler, confirm=confirm, error=sub.error)
        sub.add_argument(
            "--profile", type=Path, default=profile, help="robot profile (default: %(default)s)"
        )
        if state:
            sub.add_argument(
                "--state-dir",
                type=Path,
                default=state_dir,
                help="stop latch; share it between every command (default: %(default)s)",
            )
        if confirm:
            sub.add_argument("--confirm-supported-arm", action="store_true", help=CONFIRM_HELP)
        return sub

    command("check", check, state=False)
    command("read", read)
    command("recover", recover, confirm=True)
    sub = command("hold", hold, confirm=True)
    sub.add_argument("--hold-seconds", type=float, default=3.0)
    sub.add_argument("--max-deviation", type=float, default=0.1, help="rad (default: %(default)s)")
    sub = command("probe", probe, confirm=True)
    sub.add_argument("--joint", type=int, required=True, help="0=joint1 ... 5=joint6, 6=gripper")
    sub.add_argument("--delta", type=float, default=0.05)
    sub.add_argument("--duration", type=float, default=2.0)
    sub = command("pose", pose, confirm=True)
    sub.add_argument(
        "--stage",
        type=_stage,
        action="append",
        required=True,
        help="'name=offset,...' in raw radians from the activation pose; repeat for more stages",
    )
    sub.add_argument(
        "--hold-seconds", type=float, default=0.0, help="hold the last stage before walking home"
    )
    sub.add_argument("--out", type=Path, default=Path("logs") / f"pose-{_stamp()}")
    sub.add_argument("--no-camera", action="store_true")
    sub.add_argument(
        "--gravity",
        choices=("profile", "on", "off"),
        default="profile",
        help="gravity feed-forward: follow the profile flag, or force it on/off",
    )
    sub.add_argument(
        "--home-wait",
        type=float,
        default=None,
        help=(
            "seconds to keep retrying a blocked home before disabling (default: until interrupted)"
        ),
    )
    sub = command("gripper", gripper, confirm=True)
    sub.add_argument("--delta", type=float, default=0.1)
    sub.add_argument("--duration", type=float, default=1.0)
    sub.add_argument("--torque-cap", type=float, default=0.5)
    sub.add_argument("--kp", type=float, default=50.0)
    sub.add_argument("--kd", type=float, default=4.0)
    sub = command("agent", agent, confirm=True)
    sub.add_argument(
        "--inspect-config",
        type=Path,
        default=AGENT_CONFIG,
        help="Inspect run defaults: model, camera, rates (default: the bundled agent.ini)",
    )
    sub.add_argument(
        "--instruction",
        default=(
            "Move joint2 up by 0.05 radians and move the gripper by 0.04 radians "
            "from their observed positions, keep the other joints fixed, then finish."
        ),
    )
    sub.add_argument("--max-steps", type=int, default=300)
    sub.add_argument("--max-llm-calls", type=int, default=30)
    sub.add_argument(
        "--ready",
        default="joint3=0.3;joint2=0.2,joint3=0.6;joint2=0.3,joint3=0.8,joint4=-0.5",
        help="scripted stages walked before the agent starts ('' keeps the rest pose)",
    )
    sub.add_argument(
        "--hold-seconds",
        type=float,
        default=0.0,
        help="keep the final pose energized after the agent finishes",
    )
    sub.add_argument("--log-dir", type=Path)
    return parser


def _validate(args: argparse.Namespace) -> None:
    """Bounds from the original operator scripts; errors exit 2 before anything opens."""
    if args.confirm and not args.confirm_supported_arm:
        args.error(f"{args.command} needs --confirm-supported-arm ({CONFIRM_HELP})")
    name = args.command
    if name == "hold" and not (0 < args.hold_seconds <= 10 and 0 < args.max_deviation <= 0.5):
        args.error("hold-seconds must be in (0, 10] and max-deviation in (0, 0.5]")
    if name == "probe" and not (
        0 <= args.joint <= 6 and 0 < abs(args.delta) <= 0.1 and 0 < args.duration <= 5
    ):
        args.error("joint must be 0..6, delta magnitude at most 0.1 rad and duration in (0, 5] s")
    if name == "pose" and not 0 <= args.hold_seconds <= 60:
        args.error("hold-seconds must be in [0, 60]")
    if name == "gripper" and not (
        0 < abs(args.delta) <= 0.3 and 0 < args.duration <= 3.0 and 0 < args.torque_cap <= 1.0
    ):
        args.error("delta magnitude at most 0.3 rad, duration at most 3 s, torque cap in (0, 1] Nm")
    if name == "agent" and not (1 <= args.max_steps <= 300 and 1 <= args.max_llm_calls <= 30):
        args.error("max-steps must be in [1, 300] and max-llm-calls in [1, 30]")


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point for ``inspect-robots-b601``; returns the process exit code."""
    args = build_parser().parse_args(argv)
    _validate(args)
    handler: Handler = args.handler
    return handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
