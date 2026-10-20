"""One attended direct joint1 agent trial. Check is read-only, not qualification."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
import time
from collections.abc import Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

import numpy as np
import numpy.typing as npt
from inspect_robots import Action, Observation, Scene, StepResult, eval
from inspect_robots.errors import SafetyAbort
from inspect_robots.log import read_eval_log

from b601_hardware.direct import DirectSession, error_details, validate_tracking_report
from b601_hardware.state import existing_path, inspect_existing_stop_state, private_write
from b601_hardware.transport import Joint1Transport

from .benchmarks import hold, joint1_nudge
from .motorbridge import MotorbridgeB601Embodiment

_MAX_REQUESTS = 3
_MAX_STEPS = 6
_ACTIVE_SECONDS = 30.0
_TOLERANCE = 0.0006


@dataclass(frozen=True)
class _Consent:
    physical_motion: bool
    supported_arm: bool
    independent_cutoff: bool
    paid_inference: bool

    @property
    def complete(self) -> bool:
        return all(
            (self.physical_motion, self.supported_arm, self.independent_cutoff, self.paid_inference)
        )


@dataclass(frozen=True)
class _RunConfig:
    profile: Path
    profile_bytes: bytes
    state_dir: Path
    tracking_report: Path
    log_dir: Path
    task: str
    provider_url: str
    model: str
    api_key_env: str
    consent: _Consent


def _configuration(args: argparse.Namespace) -> _RunConfig:
    consent = _Consent(
        args.allow_physical_motion,
        args.confirm_supported_arm,
        args.confirm_independent_cutoff,
        args.allow_paid_inference,
    )
    if args.command == "run" and not consent.complete:
        raise ValueError(
            "run requires all four physical, support, cutoff and paid acknowledgements"
        )
    endpoint = urlsplit(args.provider_url)
    if (
        endpoint.scheme != "https"
        or not endpoint.hostname
        or endpoint.username is not None
        or endpoint.password is not None
        or endpoint.query
        or endpoint.fragment
    ):
        raise ValueError("provider-url must be an explicit HTTPS API base URL without credentials")
    if not args.model.strip() or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", args.api_key_env):
        raise ValueError(
            "An explicit nonempty model and credential environment-variable name are required"
        )
    if not os.environ.get(args.api_key_env, "").strip():
        raise ValueError("The explicitly selected credential environment variable must be nonempty")
    profile = existing_path(args.profile)
    content = profile.read_bytes()
    value = json.loads(content)
    if value.get("test_only", False) is not False:
        raise ValueError(
            "A reviewed operational profile is required; test-only profiles are forbidden"
        )
    Joint1Transport(value)
    state_dir = existing_path(args.state_dir, directory=True)
    if inspect_existing_stop_state(state_dir, value["legacy_stop_latch"])["stopped"]:
        raise ValueError("Stop is retained; this command never recovers or acknowledges it")
    report = existing_path(args.tracking_report)
    validate_tracking_report(report, profile)
    log_dir = Path(args.log_dir).absolute()
    existing_path(log_dir.parent, directory=True)
    if log_dir.exists() or log_dir.is_symlink():
        raise ValueError("log-dir must be a new path in an existing real directory")
    return _RunConfig(
        profile,
        content,
        state_dir,
        report,
        log_dir,
        args.task,
        args.provider_url.rstrip("/"),
        args.model,
        args.api_key_env,
        consent,
    )


@dataclass(frozen=True)
class _Corridor:
    initial: float
    goal: float
    lower: float
    upper: float

    def contains(self, values: npt.NDArray[np.float64]) -> bool:
        return bool(
            np.isfinite(values).all()
            and np.all(values >= self.lower)
            and np.all(values <= self.upper)
        )


class _Trial(MotorbridgeB601Embodiment):
    def __init__(self, config: _RunConfig) -> None:
        super().__init__(
            profile=config.profile,
            state_dir=config.state_dir,
            tracking_report=str(config.tracking_report),
            allow_motion=True,
            activate_on_reset=True,
            operator_supported_arm=config.consent.supported_arm,
        )
        if self.profile_sha256 != hashlib.sha256(config.profile_bytes).hexdigest():
            raise ValueError("Profile changed after configuration check")
        self._config = config
        self.corridor: _Corridor | None = None
        self._used = False
        self.aborted = False

    def _create_session(self) -> DirectSession:
        return DirectSession(
            self.profile,
            self.state_dir,
            existing_state_only=True,
            active_timeout_s=_ACTIVE_SECONDS,
            activation_guard=self._guard_activation,
        )

    def _guard_activation(self, positions: tuple[float, ...]) -> None:
        corridor = self.corridor
        if corridor is None:
            raise SafetyAbort("The fixed initial-pose trial corridor is required before activation")
        initial = positions[0]
        goal = initial + (corridor.goal - corridor.initial)
        if (
            abs(initial - corridor.initial) > _TOLERANCE
            or not corridor.lower <= initial <= corridor.upper
            or not corridor.lower <= goal <= corridor.upper
        ):
            raise SafetyAbort(
                "Activation feedback drifted or makes the fixed trial goal infeasible"
            )

    def _before_activate(self) -> None:
        assert self._session is not None
        positions, _ = self._session.observe()
        initial = positions[0]
        goal = initial + (0.0015 if self._config.task == "joint1-nudge" else 0.0)
        lower, upper = self.profile["joints"][0]["lower"], self.profile["joints"][0]["upper"]
        if not lower <= initial <= upper or not lower <= goal <= upper:
            self._veto("Initial pose or task goal is outside the reviewed profile")
        self.corridor = _Corridor(
            initial,
            goal,
            max(lower, min(initial, goal) - _TOLERANCE),
            min(upper, max(initial, goal) + _TOLERANCE),
        )
        self._revalidate()
        super()._before_activate()

    def _revalidate(self) -> None:
        config = self._config
        existing_path(config.profile)
        existing_path(config.tracking_report)
        if config.profile.read_bytes() != config.profile_bytes:
            raise ValueError("Captured profile changed; activation refused")
        if inspect_existing_stop_state(config.state_dir, self.profile["legacy_stop_latch"])[
            "stopped"
        ]:
            raise ValueError("Existing stop state inhibits activation")
        validate_tracking_report(config.tracking_report, config.profile)

    def reset(self, scene: Scene, *, seed: int | None = None) -> Observation:
        if self._used:
            self._veto("A physical trial cannot reset twice")
        self._used = True
        self._revalidate()
        return super().reset(scene, seed=seed)

    def _veto(self, reason: str) -> None:
        self.aborted = True
        try:
            self.abort(reason)
        except BaseException as error:
            raise SafetyAbort(f"{reason}; terminal stop also failed: {error}") from error
        raise SafetyAbort(reason)

    def ensure_active(self) -> None:
        if self.aborted or self._faulted or self._session is None:
            self._veto("Physical trial is not active")
        assert self._session is not None
        self._session.observe()
        report = self.final_report
        if report is None or report.primary is not None or report.cleanup:
            self._veto("Physical session has a retained failure")

    def pre_check(self, waypoints: npt.NDArray[np.float64]) -> str | None:
        self.ensure_active()
        if (
            self.corridor is None
            or waypoints.ndim != 2
            or waypoints.shape[1:] != (1,)
            or not len(waypoints)
            or not self.corridor.contains(waypoints)
        ):
            self._veto("Complete motion chunk exceeds the fixed initial-pose trial corridor")
        return None

    def step(self, action: Action) -> StepResult:
        self.pre_check(np.asarray(action.data, dtype=np.float64).reshape(1, -1))
        return super().step(action)

    def check_reply(self, body: bytes) -> None:
        self.ensure_active()
        payload = json.loads(body)
        for item in payload.get("output", []):
            if item.get("type") == "function_call" and item.get("name") == "move_joints":
                arguments = json.loads(item["arguments"])
                targets = arguments.get("targets", {})
                if set(targets) != {"joint1"}:
                    self._veto("Only an explicit joint1 target is allowed")
                self.pre_check(np.asarray([[targets["joint1"]]], dtype=np.float64))


def _run(config: _RunConfig) -> dict[str, Any]:
    from inspect_robots_agent import LLMAgentPolicy

    from ._physical_http import _BoundedTransport

    if not config.consent.complete:
        raise ValueError("Physical run lacks explicit consent")
    config.log_dir.mkdir(mode=0o700)
    robot = _Trial(config)
    transport = _BoundedTransport(
        robot.ensure_active, robot.check_reply, max_requests=_MAX_REQUESTS
    )
    failures: list[dict[str, str]] = []
    result: dict[str, Any] = {
        "schema": "b601-physical-agent-summary-v1",
        "success": False,
        "torque_off_confirmed": False,
        "task": config.task,
        "profile_sha256": hashlib.sha256(config.profile_bytes).hexdigest(),
        "score": None,
        "failures": failures,
    }
    try:
        policy = LLMAgentPolicy(
            model=config.model,
            base_url=config.provider_url,
            api_key_env=config.api_key_env,
            env={config.api_key_env: os.environ[config.api_key_env]},
            wire="responses",
            wire_capture=False,
            max_llm_calls=_MAX_REQUESTS,
            max_retries=1,
            backoff_s=0,
            max_speed_frac=1.0,
            effort="low",
            prior_learnings=None,
            depth="off",
            transport=transport,
            pre_check=robot.pre_check,
        )
        task = (joint1_nudge if config.task == "joint1-nudge" else hold)(max_steps=_MAX_STEPS)
        instruction = (
            "Move only joint1 by +0.0015 radians from its initial measured pose, then hold joint1. "
            if config.task == "joint1-nudge"
            else "Hold only joint1 at its initial measured pose. "
        )
        task.scenes = [
            replace(
                task.scenes[0],
                instruction=instruction
                + "Joints2-6 are observed, NOT held, and must remain mechanically supported. "
                "No gripper or other-axis commands. A corridor violation ends this trial; it is not correctable.",
            )
        ]
        (log,) = eval(task, policy, robot, log_dir=str(config.log_dir), fail_on_error=True)
        result.update(status=log.status, score=log.results.metrics.get("b601_measured_goal"))
        paths = list(config.log_dir.glob("*.json"))
        if len(paths) != 1 or read_eval_log(str(paths[0])) != log:
            raise RuntimeError("Required evaluation log persistence is unconfirmed")
        result["eval_log"] = str(paths[0])
        for sample in log.samples:
            for metadata in sample.trial_metadata:
                for name in ("actions", "transcript"):
                    path = metadata.get(name)
                    if (
                        not isinstance(path, str)
                        or not existing_path(config.log_dir / path).read_bytes()
                    ):
                        raise RuntimeError(f"Required {name} sidecar persistence is unconfirmed")
        reasons = [reason for sample in log.samples for reason in sample.termination_reasons]
        if log.status != "success" or result["score"] != 1.0 or reasons != ["done"]:
            raise RuntimeError("Trial did not finish with a measured score of 1 and agent done")
    except BaseException as error:
        failures.append(
            {"operation": "evaluation", "type": type(error).__name__, "message": str(error)}
        )
    finally:
        if failures:
            try:
                robot.abort("Physical agent evaluation failed or was interrupted")
            except BaseException as error:
                failures.append(
                    {"operation": "stop", "type": type(error).__name__, "message": str(error)}
                )
        for operation, close in (("robot.close", robot.close), ("provider.close", transport.close)):
            try:
                close()
            except BaseException as error:
                failures.append(
                    {"operation": operation, "type": type(error).__name__, "message": str(error)}
                )
        try:
            receipt = robot.final_report
        except BaseException as error:
            receipt = None
            failures.append(
                {"operation": "receipt.read", "type": type(error).__name__, "message": str(error)}
            )
        result["hardware"] = None if receipt is None else error_details(receipt)
        if (
            receipt is None
            or receipt.primary is not None
            or receipt.cleanup
            or receipt.disable_acknowledged is not True
            or not robot.cleanup_complete
            or robot.aborted
        ):
            failures.append(
                {
                    "operation": "final_receipt",
                    "type": "UnconfirmedCleanup",
                    "message": "Hardware receipt is absent, faulted, or cleanup is incomplete",
                }
            )
        result.update(requests=transport.requests, recorded_at_unix_s=time.time())
        result["success"] = not failures
        try:
            private_write(config.log_dir / "physical-agent-summary.json", result)
        except BaseException as error:
            result["success"] = False
            failures.append(
                {"operation": "summary.write", "type": type(error).__name__, "message": str(error)}
            )
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("check", "run"))
    for name in ("profile", "state-dir", "tracking-report", "log-dir"):
        parser.add_argument(f"--{name}", required=True, type=Path)
    parser.add_argument("--task", choices=("hold", "joint1-nudge"), required=True)
    parser.add_argument(
        "--provider-url", required=True, help="Explicit HTTPS Responses API base URL"
    )
    parser.add_argument("--model", required=True)
    parser.add_argument(
        "--api-key-env", required=True, help="Name of the deliberately selected credential variable"
    )
    for name in (
        "allow-physical-motion",
        "confirm-supported-arm",
        "confirm-independent-cutoff",
        "allow-paid-inference",
    ):
        parser.add_argument(f"--{name}", action="store_true")
    args = parser.parse_args(argv)
    try:
        config = _configuration(args)
    except (OSError, RuntimeError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    if args.command == "check":
        print(
            "Configuration check passed. No CAN, ownership, SDK, provider or state writes occurred. This is not readiness, motor-disable confirmation or physical qualification."
        )
        return 0
    try:
        result = _run(config)
    except Exception as error:
        print(f"Physical agent setup failed: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
