"""``b601`` embodiment: a Seeed reBot Arm B601-RS on a direct Motorbridge/SocketCAN bus.

Construction reads the robot profile and declares the spaces; it never opens CAN, a camera
or a motor. ``reset()`` opens an :class:`~inspect_robots_b601.hardware.session.ArmSession`
and, only with every operator flag set, enables and holds all configured motors.
"""

from __future__ import annotations

import contextlib
import json
import math
import time
from collections.abc import Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any, ClassVar, Protocol

import numpy as np
import numpy.typing as npt
from inspect_robots import (
    Action,
    ActionSemantics,
    Box,
    CameraSpec,
    EmbodimentBase,
    EmbodimentInfo,
    Observation,
    ObservationSpace,
    Scene,
    StateField,
    StateSpec,
    StepResult,
)
from inspect_robots.conformance import DeviceSlot, OptionSlot
from inspect_robots.errors import EmbodimentFault, SafetyAbort

from .hardware import Profile
from .hardware.bus import channel_name
from .hardware.camera import DEPTH_BACKENDS, Depth, Image, parse_camera
from .hardware.gravity import gripper_tip
from .hardware.session import MoveError

if TYPE_CHECKING:
    from .hardware.session import ArmSession

Vector = npt.NDArray[np.float64]


class OperatorConsole(Protocol):
    """The part of Inspect's operator session this embodiment uses once connected."""

    def gate(self, prompt: str, *, hint: str | None = None) -> None:
        """Block on prompt until the operator answers; hint explains a failure to read input."""

    def write_line(self, text: str) -> None:
        """Write one line of operator-facing text to the terminal."""


# A stock B601-RS on can0: limits, motor IDs, gains and torque caps, with no unit calibration.
DEFAULT_PROFILE = Path(__file__).with_name("defaults") / "b601-rs.json"

BOUNDS_TOLERANCE = 0.01
# A startup axis must cover at least this fraction of its delta. A blocked joint
# moves ~0; P-only gravity sag (~0.045 rad) is a near-constant offset, not a stall.
STARTUP_MIN_FRACTION = 0.5
MAX_READY_OFFSET = 1.2
# A joint further than this from its target after a stalled walk is marked blocked.
BLOCKED_MARGIN = 0.03
# A blocked joint is released once the agent asks for this much travel the other way.
UNBLOCK_MARGIN = 0.01
GRIPPER_DECLARED_STEP = 0.3


class B601Embodiment(EmbodimentBase):
    """Six arm joints plus the gripper in absolute raw motor radians, with an optional wrist camera.

    Every configured motor is enabled and held while active; there is no collision checking.
    """

    RUNTIME_REQUIREMENTS: ClassVar[dict[str, str]] = {
        "motorbridge": "python -m pip install 'inspect-robots-b601[motorbridge]'",
    }

    # The setup wizard probes SocketCAN interfaces for this slot and writes the answer to
    # [embodiment.args] channel, which replaces the profile's channel.
    DEVICE_SLOTS = (DeviceSlot(arg="channel", kind="can", label="B601-RS arm CAN channel"),)

    # The wizard asks these as yes/no questions and writes the answers to [embodiment.args].
    # All three must be yes before a run enables the motors; each defaults to no.
    OPTION_SLOTS = (
        OptionSlot(arg="allow_motion", label="Let runs move this arm (allow_motion)"),
        OptionSlot(
            arg="operator_supported_arm",
            label=(
                "The arm and load are supported and a cutoff is in reach (operator_supported_arm)"
            ),
        ),
        OptionSlot(
            arg="activate_on_reset",
            label="Enable the motors when each trial starts (activate_on_reset; needs both above)",
        ),
    )

    def __init__(
        self,
        *,
        profile: str = "",
        channel: str = "",
        state_dir: str = ".hardware-state",
        allow_motion: bool = False,
        activate_on_reset: bool = False,
        operator_supported_arm: bool = False,
        control_hz: float = 1.0,
        max_step: float = 0.5,
        step_size: float = 0.1,
        camera: str = "",
        startup: str = "",
        ready: str = "",
        hold_seconds: float = 0.0,
    ) -> None:
        if any(
            type(value) is not bool
            for value in (allow_motion, activate_on_reset, operator_supported_arm)
        ):
            raise ValueError("Motion and operator acknowledgements must be booleans")
        if activate_on_reset and not (allow_motion and operator_supported_arm):
            raise ValueError("Reset activation requires allow_motion and operator_supported_arm")
        if not math.isfinite(control_hz) or not 0.2 <= control_hz <= 2.0:
            raise ValueError("control_hz must be in [0.2, 2]")
        if not math.isfinite(max_step) or not 0 < max_step <= 1.0:
            raise ValueError("max_step must be in (0, 1] rad")
        if not math.isfinite(step_size) or not 0 < step_size <= min(0.2, max_step):
            raise ValueError("step_size must be in (0, min(0.2, max_step)] rad")
        if not math.isfinite(hold_seconds) or not 0 <= hold_seconds <= 300:
            raise ValueError("hold_seconds must be in [0, 300]")
        path = Path(profile or DEFAULT_PROFILE).resolve()
        self.profile: Profile = json.loads(path.read_text())
        self.profile_path = path
        if channel:
            self.profile["channel"] = channel_name({"channel": channel})
        joints = self.profile["joints"]
        if len(joints) != 6 or [j["name"] for j in joints] != [f"joint{i}" for i in range(1, 7)]:
            raise ValueError("Expected the reviewed six-joint profile")
        gripper = self.profile.get("gripper")
        if gripper is not None and (
            gripper.get("name") != "gripper"
            or type(gripper.get("motor_id")) is not int
            or gripper.get("model") != "rs-00"
        ):
            raise ValueError("Gripper must be an rs-00 motor named 'gripper'")
        axes = list(joints) + ([gripper] if gripper is not None else [])
        low = np.asarray([float(j["lower"]) for j in axes], dtype=np.float64) - BOUNDS_TOLERANCE
        high = np.asarray([float(j["upper"]) for j in axes], dtype=np.float64) + BOUNDS_TOLERANCE
        if not np.isfinite(low).all() or not np.isfinite(high).all() or np.any(low >= high):
            raise ValueError("Finite increasing joint bounds required")
        self.state_dir = str(Path(state_dir).resolve())
        self.allow_motion = allow_motion
        self.activate_on_reset = activate_on_reset
        self.operator_supported_arm = operator_supported_arm
        self.control_hz, self.max_step, self.step_size = control_hz, max_step, step_size
        self.hold_seconds = float(hold_seconds)
        self.camera_arg = camera
        self.startup_arg = startup
        self.trial_log = Path(self.state_dir).parent / "logs" / "motorbridge-trials.jsonl"
        self.trial_images = Path(self.state_dir).parent / "logs" / "motorbridge-trials"
        self.low, self.high = low, high
        self._trial_start: tuple[float, ...] | None = None
        self._trial_started_at: str | None = None
        self._trial_fault: str | None = None
        self.camera = parse_camera(camera)
        camera_specs = (
            ()
            if self.camera is None
            else (CameraSpec("front", self.camera.height, self.camera.width),)
        )
        # Depth comes from the wrist D405 (close-range improved where installed).
        # The jaws themselves are too dark to measure, so the profile's wrist_camera calibration
        # records where their tips sit in the depth image; without it there is no jaw_range.
        # Its depth_backend names the depth the calibration was measured with,
        # and a run refuses any other.
        self.depth = bool(self.camera is not None and getattr(self.camera, "depth", False))
        wrist = self.profile.get("wrist_camera") or {}
        self.depth_backend: str | None = wrist.get("depth_backend") if self.depth else None
        if self.depth_backend not in (None, *DEPTH_BACKENDS):
            raise ValueError(
                f"wrist_camera.depth_backend must be one of {', '.join(DEPTH_BACKENDS)}"
            )
        self.jaw_window: tuple[int, int, int, int] | None = None
        self.jaw_tip_depth: float | None = None
        if self.camera is not None and self.depth and wrist:
            x0, y0, x1, y1 = (int(v) for v in wrist["jaw_window_px"])
            if not (0 <= x0 < x1 <= self.camera.width and 0 <= y0 < y1 <= self.camera.height):
                raise ValueError("wrist_camera.jaw_window_px must lie inside the camera image")
            self.jaw_window, self.jaw_tip_depth = (x0, y0, x1, y1), float(wrist["jaw_tip_depth_m"])
        camera_docs = (
            ""
            if self.camera is None
            else (
                " The 'front' RealSense color camera is wrist-mounted on the gripper, looking "
                "forward along the jaws (visible at the bottom of the frame). After every move, "
                "judge position and contact from the image before the next move: joint "
                "positions cannot show contact or the target, and a joint can stop short "
                "without an error. To grasp, open first, centre the object between the jaws "
                "in the image, then close toward 0. Verify the joint1 pan direction on camera."
            )
        )
        if self.depth:
            camera_docs += (
                " Each observation also has a metric depth image of the same view (near = bright)."
            )
        if self.jaw_window:
            camera_docs += (
                " state[jaw_range] = [distance from the camera to the first surface just past the "
                "jaw tips, how far that surface is beyond the jaw tips] in metres, measured along "
                "the jaws; -1 means no reading. Second value above 0: the surface is that far "
                "beyond the tips; about 0: the tips are at it; below 0: it is already between the "
                "jaws. To grip something, close only once the second value is about -0.01 to "
                "-0.02 (the jaws are around it); closing at 0 or above closes on nothing. "
                "Images alone cannot show depth; use jaw_range and state[tip] z."
            )
        labels: tuple[str, ...] = tuple(j["name"] for j in axes)
        self.labels = labels
        self.startup: dict[str, float] = {}
        if startup:
            for item in startup.split(","):
                name, separator, value = item.partition("=")
                if not separator or name not in labels:
                    raise ValueError("startup deltas must be name=radian pairs")
                delta = float(value)
                if not math.isfinite(delta) or not 0 < abs(delta) <= 0.1:
                    raise ValueError("startup deltas must be finite, non-zero and at most 0.1 rad")
                self.startup[name] = delta
        # Ready stages: ';'-separated, each 'name=offset,...' from the activation pose.
        self.ready: list[dict[str, float]] = []
        for stage in filter(None, (part.strip() for part in str(ready).split(";"))):
            offsets: dict[str, float] = {}
            for item in stage.split(","):
                name, separator, value = item.strip().partition("=")
                if not separator or name not in labels:
                    raise ValueError("ready stages must be name=radian pairs")
                offset = float(value)
                if not math.isfinite(offset) or abs(offset) > MAX_READY_OFFSET:
                    raise ValueError(
                        f"ready offsets must be finite and at most {MAX_READY_OFFSET} rad"
                    )
                offsets[name] = offset
            self.ready.append(offsets)
        self.ready_arg = ready
        # Arm joints only; the conformance check wants joint_pos to be the one action-sized field.
        self._blocked_shape: tuple[int, ...] = (6,) if gripper is not None else (1, 6)
        ready_docs = (
            ""
            if not self.ready
            else (
                " Each trial starts from a raised ready pose; the arm walks back home afterwards."
            )
        )
        self.info = EmbodimentInfo(
            name="b601",
            action_space=Box(
                shape=(len(axes),),
                low=low,
                high=high,
                semantics=ActionSemantics(
                    control_mode="joint_pos",
                    rotation_repr="none",
                    gripper="continuous" if gripper is not None else "none",
                    frame="base",
                    dim_labels=labels,
                    # The agent plans steps from this; the delta limiter enforces it.
                    max_step=(step_size,) * 6
                    + ((GRIPPER_DECLARED_STEP,) if gripper is not None else ()),
                ),
            ),
            observation_space=ObservationSpace(
                cameras=camera_specs,
                state=StateSpec(
                    fields=(
                        StateField("joint_pos", (len(axes),), "rad"),
                        StateField("blocked", self._blocked_shape, "direction"),
                        StateField("tip", (4,), "m,m,m,deg"),
                    )
                    + ((StateField("jaw_range", (2,), "m"),) if self.jaw_window else ())
                ),
            ),
            control_hz=control_hz,
            is_simulated=False,
            capabilities=frozenset({"self_paced"}),
            docs=(
                "B601-RS six-joint Motorbridge demo: ordered absolute raw motor radians; "
                "all six joints are enabled and held. The gripper, when configured, is the "
                "final dimension: about 0 = closed, positive = open (0.8 opens moderately; "
                "the range runs to about 4.7, so open further for wide objects), "
                "holding force capped at about 1 Nm. After closing, a gripper reading near 0 "
                "means the jaws closed on nothing; one that stops well above 0 is holding "
                "something. At the folded rest pose (joint2 and "
                "joint3 near 0), +joint2 tips the gripper DOWN: raise joint3 (+) first to "
                "lift it clear (+0.3 rad lifts it about 15 cm). Negative joint1 pans left; "
                "negative joint4 pitches the wrist (and camera) down. Moves are walked at "
                f"about 0.05 rad/s; arm targets more than {max_step:g} rad from the current "
                "pose or outside the joint limits are shortened. If a joint stops short "
                "against something it holds where it stopped, and state[blocked] (one value "
                "per arm joint, joint1..joint6) shows +1 or -1 for the direction it is "
                "blocked in; it will not be pushed further that way until you move it back. "
                "state[tip] is the gripper tip [x forward, y left, z up] in metres from the "
                "arm's base, then the jaws' (and camera's) pitch in degrees, negative = "
                "looking down. Use z to check whether a move really raised or lowered the "
                "gripper: the camera's perspective alone is misleading. "
                "No Cartesian IK or collision checking." + ready_docs + camera_docs
            ),
        )
        self._session: ArmSession | None = None
        self._operator: OperatorConsole | None = None
        self._instruction: str | None = None
        self._faulted = False
        self._blocked: Vector = np.zeros(6, dtype=np.float64)
        self._blocked_at: Vector = np.zeros(6, dtype=np.float64)

    def __enter__(self) -> B601Embodiment:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def _arm(self) -> ArmSession:
        if self._session is None:
            raise EmbodimentFault("reset() has not opened the arm")
        return self._session

    def defer_operator_end(self) -> None:
        """Let Inspect own terminal input, so typed operator feedback reaches the policy."""

    def connect_operator_session(self, session: OperatorConsole) -> None:
        """Hand terminal ownership to Inspect's operator session for the rest of the run.

        From here on status lines go through the session, and each trial waits for the
        operator to confirm the arm is supported before the motors enable or move.
        """
        self._operator = session
        if self.camera is not None:
            self.camera.notify = self._say
        if self._session is not None:
            self._session.notify = self._say

    def _say(self, text: str) -> None:
        if self._operator is not None:
            self._operator.write_line(text)
        else:
            print(text, flush=True)

    def _gate(self, action: str) -> None:
        if self._operator is not None:
            self._operator.gate(
                f"B601: arm and load supported, cutoff within reach? Press Enter to {action}.",
                hint="the operator confirms each B601 trial at this terminal; --no-prompt skips it",
            )

    def reset(self, scene: Scene, *, seed: int | None = None) -> Observation:
        """Record the previous trial and start the next; returns its first observation."""
        if self._faulted:
            raise EmbodimentFault("B601 session faulted; explicit operator recovery required")
        try:
            self._record_trial()  # The previous trial, if any, ends here.
            self._instruction = scene.instruction
            self._blocked[:] = 0.0
            self._trial_start = self._trial_started_at = self._trial_fault = None
            if self._session is None:
                from .hardware.session import ArmSession

                if self.camera is not None:
                    # Start streaming before torque is on; nothing else may stall the hold loop.
                    self.camera.start()
                    self._check_depth_backend()
                session = ArmSession(self.profile, self.state_dir)
                session.notify = self._say
                self._session = session
                session.open()
                if self.activate_on_reset:
                    self._gate("enable the motors")
                    session.activate()
                    self._start_trial()
            elif self.activate_on_reset:
                # Every later trial starts like the first: back at the activation pose, confirmed.
                self._arm().go_home()
                self._gate("start the next trial")
                self._start_trial()
            observation = self._observe()
            if self._trial_start is None:
                self._trial_start = tuple(float(v) for v in observation.state["joint_pos"])
                self._trial_started_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            return observation
        except BaseException as exc:
            self._faulted = True
            raise EmbodimentFault(f"B601 reset failed: {exc}") from exc

    def _check_depth_backend(self) -> None:
        """Refuse, before the bus opens, a depth camera that started without the profile's depth."""
        started = getattr(self.camera, "depth_backend", None)
        if self.depth_backend is not None and started != self.depth_backend:
            fix = (
                " Install the RealSense Enhanced Depth package (librealsense2-enhanced-depth)."
                if self.depth_backend == "close-range"
                else ""
            )
            raise RuntimeError(
                f"the profile's wrist_camera calibration needs {self.depth_backend} depth, "
                f"but the camera started with {started} depth.{fix}"
            )

    def _start_trial(self) -> None:
        """Record the activation pose as the trial start, then walk the ready and startup moves."""
        self._trial_start = tuple(float(v) for v in self._arm().observe())
        self._trial_started_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        if self.ready:
            self._ready_move()
        if self.startup:
            self._startup_move()

    def _ready_move(self) -> None:
        """Walk the configured stages from the activation pose; a failure walks home and stops."""
        session = self._arm()
        try:
            if session._home is None:
                raise RuntimeError("the arm has no activation pose")
            home = np.asarray(session._home, dtype=np.float64)
            for offsets in self.ready:
                pose = home.copy()
                for name, offset in offsets.items():
                    pose[self.labels.index(name)] = home[self.labels.index(name)] + offset
                session.go_to(
                    pose.tolist(), timeout=float(np.max(np.abs(pose - home))) / 0.05 + 15.0
                )
        except BaseException as exc:
            self._trial_fault = f"Ready pose failed: {exc}"
            session.stop("Inspect B601 ready pose failed")
            raise EmbodimentFault(self._trial_fault) from exc

    def _startup_move(self) -> None:
        """Apply the opt-in startup deltas; a stalled axis must not stay commanded."""
        session = self._arm()
        before = np.asarray(session.observe(), dtype=np.float64)
        target = before.copy()
        for name, delta in self.startup.items():
            target[self.labels.index(name)] += delta
        # The session's convergence tolerance exceeds any startup delta, so check here.
        after = np.asarray(session.move(target.tolist(), 2.0), dtype=np.float64)
        for name, delta in self.startup.items():
            index = self.labels.index(name)
            moved = float(after[index] - before[index])
            if moved * math.copysign(1.0, delta) < STARTUP_MIN_FRACTION * abs(delta):
                self._trial_fault = (
                    f"Startup move stalled: {name} requested {delta:+.3f} rad, "
                    f"measured {moved:+.3f} rad; check for contact"
                )
                # Release the hold instead of pushing on the blockage until close().
                session.stop("Inspect B601 startup move stalled")
                raise EmbodimentFault(self._trial_fault)

    def _observation(self, positions: Sequence[float] | Vector) -> Observation:
        images: dict[str, Image] = {}
        image_times: dict[str, float] = {}
        extra: dict[str, Any] = {}
        state: dict[str, Vector] = {
            "joint_pos": np.asarray(positions, dtype=np.float64),
            "blocked": self._blocked.reshape(self._blocked_shape).copy(),
            "tip": np.asarray(gripper_tip(positions[:6]), dtype=np.float64).round(3),
        }
        if self.camera is not None:
            if self.depth:
                images["front"], depth = self.camera.read_with_depth()
                extra["front_depth"] = depth
                if self.jaw_window:
                    state["jaw_range"] = self._jaw_range(depth)
            else:
                images["front"] = self.camera.read()
            image_times["front"] = time.monotonic()
        return Observation(
            state=state,
            state_time=time.monotonic(),
            images=images,
            image_times=image_times,
            instruction=self._instruction,
            extra=extra,
        )

    def _jaw_range(self, depth: Depth) -> Vector:
        """[camera-to-surface, tips-to-surface] in metres along the jaws,
        or -1s without a reading.
        """
        if self.jaw_window is None or self.jaw_tip_depth is None:
            return np.array([-1.0, -1.0])
        x0, y0, x1, y1 = self.jaw_window
        window = np.asarray(depth[y0:y1, x0:x1], dtype=np.float64)
        valid = window[np.isfinite(window) & (window > 0)]
        if valid.size < 0.2 * window.size:
            return np.array([-1.0, -1.0])
        distance = float(np.median(valid))
        reading: Vector = np.array(
            [distance, distance - self.jaw_tip_depth], dtype=np.float64
        ).round(3)
        return reading

    def _observe(self) -> Observation:
        return self._observation(self._arm().observe())

    def step(self, action: Action) -> StepResult:
        """Walk to the requested pose, shortened to the limits; a stall marks the joint blocked."""
        if (
            self._session is None
            or self._faulted
            or not (self.allow_motion and self.activate_on_reset)
        ):
            raise EmbodimentFault("An explicitly activated B601 session is required")
        session = self._session
        try:
            measured = np.asarray(session.observe(), dtype=np.float64)
            requested = np.asarray(action.data, dtype=np.float64)
            if requested.shape != self.info.action_space.shape or not np.isfinite(requested).all():
                raise SafetyAbort("Action must contain one finite radian target per axis")
            # Shorten out-of-range or oversize requests instead of ending the trial; the
            # agent sees where the arm really went in the next observation.
            target = np.clip(requested, self.low, self.high)
            reach = np.full(target.shape, self.max_step, dtype=np.float64)
            if target.shape[0] > 6:
                reach[6] = np.inf  # the gripper is force-limited; its bounds suffice
            target = np.clip(target, measured - reach, measured + reach)
            target = self._respect_blocks(target)
            limited = [
                f"{self.labels[i]} {requested[i]:+.3f} -> {target[i]:+.3f}"
                for i in range(target.shape[0])
                if abs(requested[i] - target[i]) > 1e-9
            ]
            # The session walks at its velocity limit, never leads the measured pose by
            # more than a small reach, and names any joint that stalls.
            travel = np.abs(target - measured)
            seconds = max(
                float(travel[:6].max()) / 0.05,
                float(travel[6]) / 0.5 if travel.shape[0] > 6 else 0.0,
            )
            info: dict[str, Any] = {"limited": limited} if limited else {}
            try:
                session.go_to(target.tolist(), timeout=seconds + 20.0)
            except MoveError as stall:
                # Contact is part of the task, not a fault:
                # hold, mark the joint, let the agent replan.
                positions = np.asarray(session.observe(), dtype=np.float64)
                for index in range(6):
                    gap = target[index] - positions[index]
                    if abs(gap) > BLOCKED_MARGIN:
                        self._blocked[index] = math.copysign(1.0, gap)
                        self._blocked_at[index] = positions[index]
                info["stalled"] = str(stall)
                return StepResult(observation=self._observation(positions), info=info)
            positions = self._settle(target)
            return StepResult(observation=self._observation(positions), info=info)
        except BaseException as primary:
            self._faulted = True
            self._trial_fault = str(primary)
            try:
                session.stop("Inspect B601 step failed")
            except Exception as cleanup:
                raise EmbodimentFault(f"{primary}; stop also failed: {cleanup}") from primary
            raise

    def _respect_blocks(self, target: Vector) -> Vector:
        """Hold blocked joints at their stop unless the agent moves them back the other way."""
        target = target.copy()
        for index, direction in enumerate(self._blocked):
            if direction == 0:
                continue
            stop = self._blocked_at[index]
            if (target[index] - stop) * direction < -UNBLOCK_MARGIN:
                self._blocked[index] = 0.0
            elif (target[index] - stop) * direction > 0:
                target[index] = stop
        return target

    def _settle(self, target: Vector, seconds: float = 1.0, tolerance: float = 0.02) -> Vector:
        """Give the arm a moment to finish converging so the observation shows where it stopped."""
        session = self._arm()
        deadline = time.monotonic() + seconds
        while True:
            positions = np.asarray(session.observe(), dtype=np.float64)
            if (
                np.max(np.abs(positions[:6] - target[:6])) <= tolerance
                or time.monotonic() >= deadline
            ):
                return positions

    def _record_trial(self) -> None:
        session = self._session
        if self._trial_start is None or session is None or not hasattr(session, "_home"):
            return
        final = None
        with contextlib.suppress(Exception):
            final = tuple(float(v) for v in session.observe())
        image_path = None
        if self.camera is not None:
            try:
                from PIL import Image

                self.trial_images.mkdir(parents=True, exist_ok=True)
                stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
                image_path = self.trial_images / f"{stamp}.jpg"
                Image.fromarray(self.camera.read()).save(image_path, quality=90)
            except Exception:
                image_path = None
        record = {
            "started_at": self._trial_started_at,
            "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "instruction": self._instruction,
            "profile": str(self.profile_path),
            "labels": list(self.labels),
            "start": list(self._trial_start),
            "final": list(final) if final is not None else None,
            "fault": self._trial_fault,
            "hardware_ok": self._trial_fault is None and final is not None,
            "returns_to_start": True,
            "hold_seconds": self.hold_seconds,
            "startup": self.startup_arg,
            "ready": self.ready_arg,
            "camera": self.camera_arg,
            "depth_backend": getattr(self.camera, "depth_backend", None),
            "profile_home_pose": self.profile.get("home_pose"),
            "torque_caps": [j.get("torque_cap_nm") for j in self.profile["joints"]]
            + (
                [self.profile["gripper"].get("torque_cap_nm")]
                if self.profile.get("gripper")
                else []
            ),
            "final_image": str(image_path) if image_path is not None else None,
        }
        self.trial_log.parent.mkdir(parents=True, exist_ok=True)
        with self.trial_log.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record) + "\n")

    def close(self) -> None:
        """Record the trial, optionally hold the final pose, then close the camera and the arm."""
        with contextlib.suppress(Exception):
            self._record_trial()
        if self._session is not None and self.hold_seconds > 0:
            session = self._session
            if getattr(session, "_enabled", False) and getattr(session, "_failure", None) is None:
                self._say(f"holding final pose for {self.hold_seconds:g} s")
                time.sleep(self.hold_seconds)
        if self.camera is not None:
            self.camera.close()
        if self._session is not None:
            session, self._session = self._session, None
            session.close()
