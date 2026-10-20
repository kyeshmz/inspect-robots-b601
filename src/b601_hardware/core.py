"""Hardware-independent, single-axis commissioning policy (raw motor radians).

Caller contract::

    core = CommissioningCore(motor, bounds, max_feedback_age=reviewed_age,
                             persisted_stopped=durable_stop, clock=monotonic)
    try:
        core.activate(monotonic())   # explicit joint1 preparation and enable
        target = list(core.bounded_positions)
        target[0] += 0.005
        core.start_goal(target, 2.0, monotonic(), joint_names=JOINT_NAMES)
        core.tick(monotonic())       # owner calls regularly; no worker/queue
    finally:
        report = core.close()        # disable attempt, then resource close

UNSUPPORTED physical guarantees: software inactivity is not torque-off; a
successful disable call is only an acknowledgement. The transport must provide
real acquisition timestamps in this clock's domain, bounded synchronous calls,
verified joint mapping, and a commissioned mode with an appropriate physical
speed limit. This core limits *commanded* speed, not actual motor speed. It
cannot stop an already in-flight call or enforce deadlines while not being ticked.

prepare_axis must stage the supplied bounded HOLD without enabling, homing,
zeroing, changing mode, or replaying a previous target. Whether this can be done
safely on an already powered rig is a transport/operator prerequisite. Only
joint1 is written. Inactive joints are observed, not actively held by this core.

The wrapper owns durable stop storage. It must persist stop_latched transitions,
provide that state on construction, and call stop(reason) for external stops.
recover() is an explicit operator action;
it clears only the in-memory latch, returns the prior error report, and leaves the
core inactive. No saved Home, filesystem access, vendor imports, or auto-recovery.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import Enum
from numbers import Real
from typing import Protocol

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
ACTIVE_AXIS = JOINT_NAMES[0]
JOINT1_KP_CHOICES = (50, 100)
MEASURED_TOLERANCE = math.radians(0.03)
MAX_EXCURSION = 0.01
MAX_VELOCITY = 0.01
MAX_COMMAND_STEP = 0.002
MAX_DURATION = 5.0
GOAL_TOLERANCE = 0.0006
_ROUNDING = 1e-12  # Computed rate/step arithmetic only; never expands joint bounds.


class State(str, Enum):
    INACTIVE = "inactive"
    ACTIVE = "active"
    STOPPED = "stopped"
    CLOSED = "closed"


class GoalState(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    CANCELED = "canceled"
    ABORTED = "aborted"


class FeedbackError(RuntimeError):
    pass


class GoalRejected(ValueError):
    pass


class MotionError(RuntimeError):
    pass


@dataclass(frozen=True)
class PositionSample:
    """Use either a whole-read interval OR six acquisition timestamps.

    An interval conservatively ages every position from started_at, not from
    the time the last position was returned. Cached reads must retain their
    original acquisition timestamps. Validation occurs at the core boundary.
    """

    positions: Sequence[float]
    started_at: float | None = None
    finished_at: float | None = None
    joint_timestamps: Sequence[float] | None = None


class MotorInterface(Protocol):
    """Synchronous transport: return None on writes, raise on any failure.

    No method may queue/retry a command after returning, suppress a send error,
    or implicitly enable/recover/change mode. Successful disable return means
    only that the transport's disable operation completed, not torque-off proof.
    """

    def read_positions(self) -> PositionSample: ...
    def prepare_axis(self, axis: str, hold_position: float) -> None: ...
    def send_axis(self, axis: str, position: float) -> None: ...
    def enable_axis(self, axis: str) -> None: ...
    def disable_axis(self, axis: str) -> None: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class Failure:
    operation: str
    error: BaseException


@dataclass(frozen=True)
class ErrorReport:
    primary: Failure | None
    cleanup: tuple[Failure, ...]
    disable_acknowledged: bool | None
    recovery_failures: tuple[Failure, ...] = ()
    torque_off_confirmed: bool = False


@dataclass(frozen=True)
class _Feedback:
    raw: tuple[float, ...]
    bounded: tuple[float, ...]
    oldest: float
    checked_at: float


@dataclass(frozen=True)
class _Goal:
    start: _Feedback
    command_start: float
    target: tuple[float, ...]
    duration: float


def _number(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("expected a finite real number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("expected a finite real number")
    return result


def _six(values: Sequence[float]) -> tuple[float, ...]:
    result = tuple(_number(value) for value in values)
    if len(result) != len(JOINT_NAMES):
        raise ValueError("expected exactly six values in joint1..joint6 order")
    return result


class CommissioningCore:
    """One owner for activation, one bounded goal, and fail-latched cleanup.

    Methods are serialized; no motion is queued. Feedback gates every prepare,
    enable, and position write. Disable checks feedback but is attempted even
    when that check fails. close() always attempts transport close afterwards.
    Operational faults are re-raised unchanged; report retains the original
    exception separately from subsequent cleanup failures. Cleanup methods return
    the report rather than masking the original exception with a cleanup error.

    Goal duration is also its hard completion deadline (no implicit settle
    extension). Measured convergence may finish a goal early. Tick regularly
    before that deadline. Excursion is bounded both from the
    goal's measured start and from this activation's measured pose, preventing
    successive goals from ratcheting outside the first-test envelope.
    Each goal interpolates from the last issued command. Measurements remain
    the reference for physical bounds and convergence, not a new command origin.
    """

    def __init__(
        self,
        motor: MotorInterface,
        bounds: Sequence[Sequence[float]],
        *,
        max_feedback_age: float,
        persisted_stopped: bool,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        limits = tuple(tuple(_number(v) for v in pair) for pair in bounds)
        if len(limits) != 6 or any(len(p) != 2 or p[0] >= p[1] for p in limits):
            raise ValueError("supply six finite, increasing (lower, upper) bounds")
        age = _number(max_feedback_age)
        if age <= 0:
            raise ValueError("max_feedback_age must be positive")
        if not isinstance(persisted_stopped, bool):
            raise ValueError("persisted_stopped must be explicitly true or false")
        self._motor = motor
        self._bounds = limits
        self._max_age = age
        self._clock = clock
        self._lock = threading.RLock()
        self._state = State.STOPPED if persisted_stopped else State.INACTIVE
        self._stopped = persisted_stopped
        self._goal: _Goal | None = None
        self._goal_state = GoalState.IDLE
        self._feedback: _Feedback | None = None
        self._activation_pose: tuple[float, ...] | None = None
        self._hold: float | None = None
        self._last_command: float | None = None
        self._last_write_at: float | None = None
        self._last_time: float | None = None
        # Set BEFORE prepare, so partially failed activation still gets disable.
        self._axis_attempted = False
        self._disable_ack: bool | None = None
        self._primary: Failure | None = None
        self._cleanup: list[Failure] = []
        self._recovery_failures: list[Failure] = []

    @property
    def state(self) -> State:
        return self._state

    @property
    def stop_latched(self) -> bool:
        return self._stopped

    @property
    def goal_state(self) -> GoalState:
        return self._goal_state

    @property
    def measured_positions(self) -> tuple[float, ...] | None:
        """Last accepted raw sample; this property is NOT a freshness guarantee."""
        return None if self._feedback is None else self._feedback.raw

    @property
    def bounded_positions(self) -> tuple[float, ...] | None:
        """Last measured-derived legal HOLD vector; not a freshness guarantee."""
        return None if self._feedback is None else self._feedback.bounded

    @property
    def feedback_timestamp(self) -> float | None:
        """Oldest source acquisition time, never the time a cache was reread."""
        return None if self._feedback is None else self._feedback.oldest

    @property
    def report(self) -> ErrorReport:
        with self._lock:
            return ErrorReport(
                self._primary,
                tuple(self._cleanup),
                self._disable_ack,
                tuple(self._recovery_failures),
            )

    def _time(self, now: float | None = None) -> float:
        value = _number(self._clock() if now is None else now)
        if self._last_time is not None and value < self._last_time:
            raise FeedbackError("monotonic clock moved backwards")
        self._last_time = value
        return value

    def _read(self, now: float | None = None) -> _Feedback:
        self._time(now)
        sample = self._motor.read_positions()
        checked_at = self._time()  # Include transport read latency in age checks.
        try:
            if not isinstance(sample, PositionSample):
                raise ValueError("transport did not return PositionSample")
            raw = _six(sample.positions)
            if sample.joint_timestamps is not None:
                if sample.started_at is not None or sample.finished_at is not None:
                    raise ValueError("ambiguous sample timing")
                timestamps = _six(sample.joint_timestamps)
                oldest, newest = min(timestamps), max(timestamps)
            else:
                oldest = _number(sample.started_at)
                newest = _number(sample.finished_at)
                if oldest > newest:
                    raise ValueError("sample interval is reversed")
            if newest > checked_at or checked_at - oldest > self._max_age:
                raise ValueError("feedback is stale or future-dated")
            bounded = []
            for name, value, (lower, upper) in zip(JOINT_NAMES, raw, self._bounds):
                if value < lower - MEASURED_TOLERANCE or value > upper + MEASURED_TOLERANCE:
                    raise ValueError(f"{name} measured outside supplied bounds")
                bounded.append(min(upper, max(lower, value)))
        except (TypeError, ValueError, OverflowError) as exc:
            raise FeedbackError(str(exc)) from exc
        feedback = _Feedback(raw, tuple(bounded), oldest, checked_at)
        self._feedback = feedback
        return feedback

    def _before_write(self, feedback: _Feedback) -> float:
        now = self._time()
        if now - feedback.oldest > self._max_age:
            raise FeedbackError("feedback expired before write")
        return now

    def _record(self, operation: str, error: BaseException) -> None:
        failure = Failure(operation, error)
        if self._primary is None:
            self._primary = failure
        elif operation == "recover.feedback":
            self._recovery_failures.append(failure)
        else:
            self._cleanup.append(failure)

    def _latch(self, operation: str, error: BaseException) -> None:
        self._record(operation, error)
        self._stopped = True
        self._state = State.STOPPED
        if self._goal is not None:
            self._goal_state = GoalState.ABORTED
        self._goal = None
        self._hold = None

    def _disable(self) -> None:
        if not self._axis_attempted:
            return
        try:
            self._before_write(self._read())
        except BaseException as exc:
            self._latch("disable.feedback", exc)
        try:
            self._motor.disable_axis(ACTIVE_AXIS)
        except BaseException as exc:
            self._disable_ack = False
            self._latch("disable", exc)
        else:
            self._disable_ack = True
            self._axis_attempted = False

    def _fault(self, operation: str, error: BaseException) -> None:
        self._latch(operation, error)
        self._disable()

    def _require_active(self) -> None:
        if self._state != State.ACTIVE:
            raise RuntimeError("explicit activation required; stopped state needs recovery")

    def observe(self, now: float | None = None) -> tuple[float, ...]:
        """Read actual positions while inactive/stopped, without clearing a stop.

        A failed read latches a fault. If this instance previously attempted
        activation, ordinary fault cleanup may attempt disable; observation
        itself never prepares, enables, or sends a position target.
        """
        with self._lock:
            if self._state == State.CLOSED:
                raise RuntimeError("cannot observe a closed transport")
            try:
                return self._read(now).raw
            except BaseException as exc:
                if not self._stopped or self._primary is None:
                    self._fault("observe", exc)
                raise

    def activate(
        self,
        now: float | None = None,
        *,
        guard: Callable[[tuple[float, ...]], None] | None = None,
    ) -> None:
        with self._lock:
            if self._state != State.INACTIVE or self._stopped:
                raise RuntimeError("activation requires inactive, recovered state")
            try:
                feedback = self._read(now)
                if guard is not None:
                    guard(feedback.raw)
                self._before_write(feedback)
                self._axis_attempted = True
                self._disable_ack = None
                self._motor.prepare_axis(ACTIVE_AXIS, feedback.bounded[0])
                self._last_command = feedback.bounded[0]
                self._last_write_at = self._time()
                prepared = feedback
                feedback = self._read()
                if guard is not None:
                    guard(feedback.raw)
                if any(abs(a - b) > MEASURED_TOLERANCE for a, b in zip(prepared.raw, feedback.raw)):
                    raise FeedbackError("pose changed during activation preparation")
                self._before_write(feedback)
                self._motor.enable_axis(ACTIVE_AXIS)
                self._activation_pose = feedback.raw
                self._hold = prepared.bounded[0]
                self._goal_state = GoalState.IDLE
                self._state = State.ACTIVE
            except BaseException as exc:
                self._fault("activate", exc)
                raise

    def start_goal(
        self,
        target: Sequence[float],
        duration: float,
        now: float | None = None,
        *,
        joint_names: Sequence[str] = JOINT_NAMES,
    ) -> None:
        with self._lock:
            self._require_active()
            if self._goal is not None:
                raise GoalRejected("one goal is already running; cancel it explicitly")
            try:
                if tuple(joint_names) != JOINT_NAMES:
                    raise ValueError("joint_names must be exactly joint1..joint6 in order")
                targets = _six(target)
                seconds = _number(duration)
                if not 0 < seconds <= MAX_DURATION:
                    raise ValueError("duration must be positive and at most 5 seconds")
                for value, (lower, upper) in zip(targets, self._bounds):
                    if not lower <= value <= upper:
                        raise ValueError("explicit targets must be strictly within supplied bounds")
            except (TypeError, ValueError, OverflowError) as exc:
                raise GoalRejected(str(exc)) from exc
            try:
                feedback = self._read(now)
            except BaseException as exc:
                self._fault("start_goal.feedback", exc)
                raise
            if any(abs(a - b) > MEASURED_TOLERANCE for a, b in zip(targets[1:], feedback.raw[1:])):
                raise GoalRejected(
                    "inactive-axis targets must equal measured positions within tolerance"
                )
            assert self._activation_pose is not None
            if any(
                abs(targets[0] - origin) > MAX_EXCURSION + _ROUNDING
                for origin in (feedback.raw[0], self._activation_pose[0])
            ):
                raise GoalRejected("joint1 target exceeds the 0.01 rad excursion envelope")
            assert self._last_command is not None
            travel = max(
                abs(targets[0] - feedback.bounded[0]), abs(targets[0] - self._last_command)
            )
            if travel / seconds > MAX_VELOCITY + _ROUNDING:
                raise GoalRejected("goal exceeds the 0.01 rad/s commanded velocity limit")
            self._goal = _Goal(feedback, self._last_command, targets, seconds)
            self._goal_state = GoalState.RUNNING

    def _send(self, position: float, feedback: _Feedback) -> None:
        now = self._before_write(feedback)
        lower, upper = self._bounds[0]
        if not lower <= position <= upper:
            raise MotionError("command outside strict joint1 bounds")
        assert self._last_command is not None and self._last_write_at is not None
        assert self._activation_pose is not None
        if abs(position - self._activation_pose[0]) > MAX_EXCURSION + _ROUNDING:
            raise MotionError("command exceeds activation excursion envelope")
        step = abs(position - self._last_command)
        if step > MAX_COMMAND_STEP + _ROUNDING:
            raise MotionError("command step exceeds 0.002 rad; delayed targets are not replayed")
        if step > MAX_VELOCITY * (now - self._last_write_at) + _ROUNDING:
            raise MotionError(
                "command exceeds 0.01 rad/s; delayed targets are not replayed: "
                f"step={step:.9g}, elapsed={now - self._last_write_at:.9g}, "
                f"previous={self._last_command:.9g}, requested={position:.9g}, "
                f"measured={feedback.raw[0]:.9g}"
            )
        self._motor.send_axis(ACTIVE_AXIS, position)
        self._last_command = position
        self._last_write_at = self._time()

    def tick(self, now: float | None = None) -> tuple[float, ...] | None:
        with self._lock:
            if self._state != State.ACTIVE:
                return None
            try:
                feedback = self._read(now)
                assert self._activation_pose is not None and self._hold is not None
                if abs(feedback.raw[0] - self._activation_pose[0]) > MAX_EXCURSION + _ROUNDING:
                    raise MotionError("measured joint1 left activation excursion envelope")
                goal = self._goal
                position = self._hold
                succeeded = False
                if goal is not None:
                    elapsed = feedback.checked_at - goal.start.checked_at
                    tracking = (
                        f"elapsed_s={elapsed:.9g}, duration_s={goal.duration:.9g}, "
                        f"target_rad={goal.target[0]:.9g}, last_command_rad={self._last_command:.9g}, "
                        f"measured_rad={feedback.raw[0]:.9g}, "
                        f"error_rad={goal.target[0] - feedback.raw[0]:.9g}, "
                        f"tolerance_rad={GOAL_TOLERANCE:.9g}"
                    )
                    if elapsed > goal.duration:
                        raise MotionError("goal completion deadline expired: " + tracking)
                    if abs(feedback.raw[0] - goal.start.raw[0]) > MAX_EXCURSION + _ROUNDING:
                        raise MotionError("measured joint1 left goal excursion envelope")
                    if any(
                        abs(a - b) > MEASURED_TOLERANCE
                        for a, b in zip(feedback.raw[1:], goal.start.raw[1:])
                    ):
                        raise MotionError("inactive axis moved during goal")
                    start = goal.command_start
                    position = start + (goal.target[0] - start) * (elapsed / goal.duration)
                    if abs(position - goal.target[0]) <= GOAL_TOLERANCE and all(
                        abs(a - b) <= GOAL_TOLERANCE for a, b in zip(feedback.raw, goal.target)
                    ):
                        succeeded = True
                    elif elapsed == goal.duration:
                        raise MotionError(
                            "goal deadline reached without measured convergence: " + tracking
                        )
                self._send(position, feedback)
                if succeeded:
                    self._hold = position
                    self._goal = None
                    self._goal_state = GoalState.SUCCEEDED
                return feedback.raw
            except BaseException as exc:
                self._fault("tick", exc)
                raise

    def cancel(self, now: float | None = None) -> None:
        with self._lock:
            if self._state != State.ACTIVE:
                return
            if self._goal is not None:
                self._goal_state = GoalState.CANCELED
            self._goal = None  # Drop it BEFORE any read or write can fail.
            try:
                feedback = self._read(now)
                position = feedback.bounded[0]
                # A pre-first-tick cancel need not write the already prepared HOLD.
                if position != self._last_command:
                    self._send(position, feedback)
                self._hold = position
            except BaseException as exc:
                self._fault("cancel", exc)
                raise

    def deactivate(self) -> ErrorReport:
        with self._lock:
            if self._state == State.CLOSED:
                return self.report
            if self._goal is not None:
                self._goal_state = GoalState.CANCELED
            self._goal = None
            self._hold = None
            self._state = State.STOPPED if self._stopped else State.INACTIVE
            self._disable()
            return self.report

    def stop(self, reason: str) -> ErrorReport:
        """Latch an external/operator stop; never interpret it as a HOLD request."""
        with self._lock:
            if not isinstance(reason, str) or not reason.strip():
                raise ValueError("an explicit stop reason is required")
            if self._state == State.CLOSED:
                return self.report
            if not self._stopped:
                self._fault("stop", MotionError(reason))
            return self.report

    def recover(self, now: float | None = None) -> ErrorReport:
        with self._lock:
            if self._state != State.STOPPED or self._axis_attempted:
                raise RuntimeError(
                    "recovery requires stopped, inactive state with disable acknowledged"
                )
            try:
                self._read(now)
            except BaseException as exc:
                self._latch("recover.feedback", exc)
                raise
            prior = self.report
            self._primary = None
            self._cleanup.clear()
            self._recovery_failures.clear()
            self._stopped = False
            self._state = State.INACTIVE
            self._goal_state = GoalState.IDLE
            self._activation_pose = None
            self._last_command = None
            self._last_write_at = None
            return prior

    def close(self) -> ErrorReport:
        with self._lock:
            if self._state == State.CLOSED:
                return self.report
            self.deactivate()
            try:
                self._motor.close()
            except BaseException as exc:
                self._latch("close", exc)
            finally:
                self._state = State.CLOSED
            return self.report
