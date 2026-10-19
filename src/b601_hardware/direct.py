"""ROS-free owner of the existing joint1 core, transport and durable stop policy."""

from __future__ import annotations

import fcntl
import hashlib
import json
import math
import os
import threading
import time
from pathlib import Path

from .core import CommissioningCore, GOAL_TOLERANCE, GoalState, JOINT_NAMES, State
from .state import StopLedger
from .transport import Joint1Transport

CAN_OWNER_LOCK = Path("/home/jetson/kye/froghollow-demo/b601/.runtime/can-can1.lock")


def validate_tracking_report(path, profile_path, *, now=time.time):
    """Accidental-use gate, not a cryptographic proof of physical qualification."""
    report = json.loads(Path(path).read_text())
    age = now() - report["recorded_at_unix_s"]
    measured = report.get("measured_delta_rad")
    expected = hashlib.sha256(Path(profile_path).read_bytes()).hexdigest()
    if (report.get("schema") != "b601-direct-joint1-test-v1" or report.get("success") is not True
            or report.get("profile_sha256") != expected or report.get("cleanup") != []
            or type(age) not in (int, float) or not 0 <= age <= 600
            or report.get("requested_delta_rad") != 0.005 or report.get("duration_s") != 2.0
            or type(measured) not in (int, float) or not math.isfinite(measured)
            or abs(measured - 0.005) > GOAL_TOLERANCE
            or report.get("disable_acknowledged") is not True):
        raise ValueError("A successful direct joint1 test with this profile from the last 10 minutes is required")
    return report


def claim_native_bus():
    fd = os.open(CAN_OWNER_LOCK, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    stream = os.fdopen(fd, "a+")
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BaseException:
        stream.close()
        raise RuntimeError("CAN is owned by another controller; no handoff is performed")
    return stream


def error_details(report):
    return {
        "primary": None if report.primary is None else {
            "operation": report.primary.operation, "error": str(report.primary.error)},
        "cleanup": [{"operation": failure.operation, "error": str(failure.error)}
                    for failure in report.cleanup],
        "disable_acknowledged": report.disable_acknowledged,
        "torque_off_confirmed": False,
    }


class DirectSession:
    """One worker maintains bounded HOLD while Inspect waits for an LLM.

    Opening only reads. Activation and recovery are separate explicit calls.
    There is no command queue, reconnect, mode change, home or other-axis write.
    The same two CAN locks and stop ledger are shared with the ROS controller.
    """

    def __init__(self, profile, state_dir, *, transport=None, ledger=None,
                  owner_factory=claim_native_bus, clock=time.monotonic, period=0.02):
        if profile.get("test_only") and transport is None:
            raise ValueError("Test-only profiles require an injected fake transport; physical CAN is forbidden")
        self.profile, self.state_dir = profile, Path(state_dir)
        self.transport = transport if transport is not None else Joint1Transport(profile)
        self.ledger = ledger
        self.owner_factory, self.clock, self.period = owner_factory, clock, period
        self.core = None
        self._owner = self._thread = None
        self._lock = threading.RLock()
        self._changed = threading.Condition(self._lock)
        self._stop = threading.Event()
        self._failure = None
        self._persisted = None
        self._opened = self._closed = False

    def open(self):
        if self._opened or self._closed:
            raise RuntimeError("Direct sessions are single-use")
        try:
            self._owner = self.owner_factory()
            self.transport.open()
            if self.ledger is None:
                self.ledger = StopLedger(self.state_dir, self.profile["legacy_stop_latch"])
            self.core = CommissioningCore(
                self.transport, [(joint["lower"], joint["upper"]) for joint in self.profile["joints"]],
                max_feedback_age=self.profile["feedback_timeout_s"],
                persisted_stopped=bool(self.ledger.stopped), clock=self.clock,
            )
            self.core.observe()
            self._opened = True
            self._thread = threading.Thread(target=self._loop, name="b601-motorbridge-hold", daemon=True)
            self._thread.start()
            return self
        except BaseException as primary:
            try:
                self.close()
            except Exception as cleanup:
                raise RuntimeError(f"{primary}; additionally close failed: {cleanup}") from primary
            raise

    def _persist(self):
        if self.core.stop_latched and self.core.report.primary is not None:
            details = error_details(self.core.report)
            if details != self._persisted:
                self.ledger.stop("direct Motorbridge core stopped", details=details)
                self._persisted = details

    def _check(self):
        if self._failure is not None:
            raise RuntimeError("Direct session faulted; start a new explicitly recovered session") from self._failure
        if self.ledger.stopped and not self.core.stop_latched:
            self.core.stop("durable or external stop latch is set")
            self._persist()

    def _persist_after(self, primary):
        try:
            self._persist()
        except Exception as storage:
            raise RuntimeError(f"{primary}; stop evidence persistence also failed: {storage}") from primary

    def _loop(self):
        while not self._stop.is_set():
            with self._changed:
                try:
                    self._check()
                    if self.core.state == State.ACTIVE:
                        self.core.tick()
                    self._persist()
                except BaseException as error:
                    self._failure = error
                    self.core.stop("direct control loop failed: " + str(error))
                    try:
                        self._persist()
                    except Exception as storage:
                        self._failure = RuntimeError(f"{error}; stop persistence also failed: {storage}")
                    self._stop.set()
                self._changed.notify_all()
            self._stop.wait(self.period)

    def observe(self):
        with self._lock:
            self._check()
            try:
                values = self.core.observe()
                return tuple(values), self.core.feedback_timestamp
            except BaseException as error:
                self._persist_after(error)
                raise

    def recover(self):
        with self._lock:
            self._check()
            before = self.ledger.snapshot()
            prior = self.core.recover()
            if self.ledger.snapshot() != before:
                self.core.stop("stop evidence changed during direct recovery")
                self._persist()
                raise RuntimeError("New stop evidence appeared; acknowledgement refused")
            self.ledger.acknowledge(details={"fresh_inactive_recovery": True, "backend": "direct-motorbridge"})
            self._persisted = None
            self._check()
            return error_details(prior)

    def activate(self):
        with self._lock:
            self._check()
            if self.ledger.stopped or self.core.stop_latched:
                raise RuntimeError("Stop is latched; explicit operator recovery required")
            try:
                self.core.activate()
                self._check()
                if self.core.state != State.ACTIVE:
                    raise RuntimeError("Activation did not remain active")
            except BaseException as error:
                if self.core.state == State.ACTIVE:
                    self.core.stop("direct activation verification failed")
                self._persist_after(error)
                raise

    def stop(self, reason):
        with self._lock:
            self.core.stop(reason)
            self._persist()

    def move(self, target, duration):
        with self._changed:
            self._check()
            self.core.start_goal(target, duration, joint_names=JOINT_NAMES)
            deadline = self.clock() + duration + 1.0
            try:
                while self.core.goal_state == GoalState.RUNNING:
                    self._check()
                    remaining = deadline - self.clock()
                    if remaining <= 0:
                        raise TimeoutError("Direct motion worker did not complete")
                    self._changed.wait(timeout=min(0.02, remaining))
                self._check()
                if self.core.goal_state != GoalState.SUCCEEDED:
                    raise RuntimeError(f"Direct goal ended as {self.core.goal_state.value}: {error_details(self.core.report)}")
            except BaseException as error:
                self.core.stop("direct evaluation interrupted or failed")
                self._persist_after(error)
                raise

    def close(self):
        if self._closed:
            return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                raise RuntimeError("Motor worker did not exit; cancellation and ownership release unconfirmed")
        with self._lock:
            report = None if self.core is None else self.core.close()
            if report is not None:
                self._persist()  # Cleanup faults must inhibit the next owner too.
            # Also allows explicit retry if a prior transport close failed.
            self.transport.close()
            if self._owner is not None:
                self._owner.close()
                self._owner = None
            self._closed = True
            if report is not None and (report.disable_acknowledged is False or report.cleanup):
                raise RuntimeError("Direct cleanup has failures: " + str(error_details(report)))


def main(argv=None):
    import argparse
    import sys
    from .state import private_write

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("read", "recover", "test-joint1"))
    parser.add_argument("--profile", type=Path, required=True,
                        help="Explicit reviewed robot profile; never use test fixtures")
    parser.add_argument("--state-dir", type=Path, required=True,
                        help="Existing persistent commissioning state shared by all owners")
    parser.add_argument("--confirm-supported-arm", action="store_true")
    args = parser.parse_args(argv)
    if args.command in ("recover", "test-joint1") and not args.confirm_supported_arm:
        parser.error("This operator action requires --confirm-supported-arm and an accessible physical cutoff")
    content = args.profile.read_bytes()
    profile = json.loads(content)
    session = DirectSession(profile, args.state_dir)
    report = {"schema": "b601-direct-joint1-test-v1", "success": False, "cleanup": [],
              "primary": None, "torque_off_confirmed": False,
              "profile_sha256": hashlib.sha256(content).hexdigest(),
              "requested_delta_rad": 0.005, "duration_s": 2.0}
    try:
        session.open()
        if args.command == "recover":
            report["prior_errors"] = session.recover()
        positions, stamp = session.observe()
        report.update(positions=positions, measured_at_monotonic=stamp,
                      stop_latched=session.core.stop_latched, state=session.core.state.value)
        if args.command == "test-joint1":
            print("OPERATOR TRIAL: enable joint1 only, +0.005 rad / 2 s, then request disable; other axes are NOT held",
                  file=sys.stderr, flush=True)
            session.activate()
            before, _ = session.observe()
            target = list(before)
            target[0] += 0.005
            session.move(target, 2.0)
            after, _ = session.observe()
            report.update(baseline_positions=before, final_positions=after,
                          measured_delta_rad=after[0] - before[0])
            if any(abs(value - desired) > GOAL_TOLERANCE for value, desired in zip(after, target)):
                raise RuntimeError("Direct trial lacks measured convergence")
        report["success"] = True
    except BaseException as error:
        report["primary"] = {"type": type(error).__name__, "message": str(error)}
    finally:
        try:
            session.close()
        except Exception as cleanup:
            report["cleanup"].append({"type": type(cleanup).__name__, "message": str(cleanup)})
            report["success"] = False
        if session.core is not None:
            report["disable_acknowledged"] = session.core.report.disable_acknowledged
        report["recorded_at_unix_s"] = time.time()
        if args.command == "test-joint1":
            path = args.state_dir / f"direct-joint1-test-{time.time_ns()}.json"
            report["report_path"] = str(path.resolve())
            try:
                private_write(path, report)
            except Exception as error:
                report["success"] = False
                report["report_write_error"] = str(error)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
