"""Hardware-free commissioning policy tests; no ROS or vendor imports."""

import math
import unittest

from b601_hardware.core import (
    ACTIVE_AXIS,
    GOAL_TOLERANCE,
    JOINT_NAMES,
    MAX_COMMAND_STEP,
    MEASURED_TOLERANCE,
    CommissioningCore,
    FeedbackError,
    GoalRejected,
    GoalState,
    MotionError,
    PositionSample,
    State,
)


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class Motor:
    def __init__(self, clock):
        self.clock = clock
        self.positions = [0.0] * 6
        self.calls = []
        self.sample = None
        self.errors = {}
        self.after_prepare = None
        self.last_target = None

    def _call(self, name, *args):
        self.calls.append((name, *args))
        if name in self.errors:
            raise self.errors[name]

    def read_positions(self):
        self._call("read")
        if self.sample is not None:
            return self.sample
        return PositionSample(tuple(self.positions), self.clock.now, self.clock.now)

    def prepare_axis(self, axis, hold_position):
        self._call("prepare", axis, hold_position)
        self.last_target = hold_position
        if self.after_prepare:
            self.after_prepare()

    def send_axis(self, axis, position):
        self._call("send", axis, position)
        self.last_target = position

    def enable_axis(self, axis):
        self._call("enable", axis)

    def disable_axis(self, axis):
        self._call("disable", axis)

    def close(self):
        self._call("close")

    @property
    def writes(self):
        return [call for call in self.calls if call[0] != "read"]


class CommissioningTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.motor = Motor(self.clock)
        self.bounds = [(-1.0, 1.0)] * 6
        self.core = self.make_core()

    def make_core(self, **kwargs):
        return CommissioningCore(
            self.motor, self.bounds, max_feedback_age=0.1,
            persisted_stopped=kwargs.get("persisted_stopped", False), clock=self.clock,
        )

    def begin(self, delta=0.005, duration=2.0):
        self.core.activate(self.clock.now)
        target = list(self.core.measured_positions)
        target[0] += delta
        self.core.start_goal(target, duration, self.clock.now)

    def tick_at(self, elapsed):
        self.clock.now = 100.0 + elapsed
        return self.core.tick(self.clock.now)

    def test_constructor_and_inactive_tick_do_not_touch_motor(self):
        self.assertEqual(self.core.state, State.INACTIVE)
        self.assertIsNone(self.core.tick(self.clock.now))
        self.assertEqual(self.motor.calls, [])
        with self.assertRaises(RuntimeError):
            self.core.start_goal([0.0] * 6, 2.0, self.clock.now)
        self.assertEqual(self.motor.calls, [])

    def test_observe_inactive_reads_only_and_keeps_oldest_source_timestamp(self):
        self.motor.positions = [0.1] * 6
        self.motor.sample = PositionSample(self.motor.positions, 99.95, 99.99)
        self.assertEqual(self.core.observe(100.0), (0.1,) * 6)
        self.assertEqual(self.core.feedback_timestamp, 99.95)
        self.assertEqual(self.core.state, State.INACTIVE)
        self.assertEqual(self.motor.writes, [])
        self.clock.now = 100.02
        self.core.observe(self.clock.now)
        self.assertEqual(self.core.feedback_timestamp, 99.95)

    def test_observation_failure_latches_and_good_observation_does_not_recover(self):
        self.motor.sample = PositionSample([0.0] * 6, 99.0, 100.0)
        with self.assertRaises(FeedbackError):
            self.core.observe(100.0)
        self.assertTrue(self.core.stop_latched)
        self.motor.sample = None
        self.core.observe(100.0)
        self.assertEqual(self.core.state, State.STOPPED)
        self.assertEqual(self.motor.writes, [])

    def test_observation_fault_is_recorded_even_when_persisted_stop_was_already_set(self):
        self.core = self.make_core(persisted_stopped=True)
        error = OSError("read fault while already stopped")
        self.motor.errors["read"] = error
        with self.assertRaises(OSError):
            self.core.observe(100.0)
        self.assertIs(self.core.report.primary.error, error)
        self.assertEqual(self.motor.writes, [])

    def test_invalid_constructor_inputs_are_hardware_free(self):
        for bounds in ([], [(1.0, -1.0)] * 6, [(0.0, math.nan)] * 6):
            with self.subTest(bounds=bounds), self.assertRaises(ValueError):
                CommissioningCore(self.motor, bounds, max_feedback_age=0.1,
                                  persisted_stopped=False, clock=self.clock)
        self.assertEqual(self.motor.calls, [])

    def test_activation_only_prepares_and_enables_joint1_with_fresh_read_each(self):
        self.motor.positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        self.core.activate(self.clock.now)
        self.assertEqual(self.motor.calls, [
            ("read",), ("prepare", "joint1", 0.1), ("read",), ("enable", "joint1"),
        ])
        self.assertEqual(self.core.state, State.ACTIVE)

    def test_measured_tolerance_projects_hold_but_not_explicit_target(self):
        self.motor.positions[0] = 1.0 + MEASURED_TOLERANCE / 2
        self.motor.positions[5] = -1.0 - MEASURED_TOLERANCE / 2
        self.core.activate(self.clock.now)
        self.assertIn(("prepare", ACTIVE_AXIS, 1.0), self.motor.writes)
        self.assertGreater(self.core.measured_positions[0], 1.0)
        self.assertEqual(self.core.bounded_positions, (1.0, 0.0, 0.0, 0.0, 0.0, -1.0))
        with self.assertRaises(GoalRejected):
            self.core.start_goal(self.motor.positions, 2.0, self.clock.now)
        target = [1.0, 0.0, 0.0, 0.0, 0.0, -1.0]
        self.core.start_goal(target, 2.0, self.clock.now)
        self.core.cancel(self.clock.now)
        self.assertEqual(self.core.state, State.ACTIVE)
        self.assertEqual(self.core.goal_state, GoalState.CANCELED)
        self.assertFalse(any(c[0] == "send" for c in self.motor.writes))

    def test_every_joint_is_checked_before_activation(self):
        for axis in range(6):
            with self.subTest(axis=axis):
                self.motor.calls.clear()
                self.motor.positions = [0.0] * 6
                self.motor.positions[axis] = 1.0 + MEASURED_TOLERANCE + 0.00001
                core = self.make_core()
                with self.assertRaises(FeedbackError):
                    core.activate(self.clock.now)
                self.assertTrue(core.stop_latched)
                self.assertEqual(self.motor.writes, [])

    def test_malformed_missing_stale_future_and_ambiguous_samples_latch(self):
        samples = [
            object(),
            PositionSample([0.0] * 5, 100.0, 100.0),
            PositionSample([0.0] * 5 + [math.nan], 100.0, 100.0),
            PositionSample([0.0] * 5 + [math.inf], 100.0, 100.0),
            PositionSample([0.0] * 5 + [None], 100.0, 100.0),
            PositionSample([0.0] * 5 + [True], 100.0, 100.0),
            PositionSample([0.0] * 6),
            PositionSample([0.0] * 6, 99.0, 100.0),
            PositionSample([0.0] * 6, 100.0, 100.1),
            PositionSample([0.0] * 6, 100.0, 99.9),
            PositionSample([0.0] * 6, joint_timestamps=[100.0] * 5 + [99.0]),
            PositionSample([0.0] * 6, joint_timestamps=[100.0] * 5),
            PositionSample([0.0] * 6, 100.0, 100.0, [100.0] * 6),
        ]
        for sample in samples:
            with self.subTest(sample=sample):
                self.motor.calls.clear()
                self.motor.sample = sample
                core = self.make_core()
                with self.assertRaises(FeedbackError):
                    core.activate(self.clock.now)
                self.assertEqual(core.state, State.STOPPED)
                self.assertEqual(self.motor.writes, [])

    def test_per_joint_acquisition_times_are_supported(self):
        self.motor.sample = PositionSample([0.0] * 6, joint_timestamps=[99.95] * 6)
        self.core.activate(self.clock.now)
        self.assertEqual(self.core.state, State.ACTIVE)

    def test_read_latency_is_included_in_freshness(self):
        original_read = self.motor.read_positions

        def slow_read():
            sample = original_read()
            self.clock.now += 0.2
            return sample

        self.motor.read_positions = slow_read
        with self.assertRaises(FeedbackError):
            self.core.activate(100.0)
        self.assertEqual(self.motor.writes, [])

    def test_partial_activation_failure_disables_only_attempted_axis(self):
        for operation in ("prepare", "enable"):
            with self.subTest(operation=operation):
                self.motor.calls.clear()
                error = RuntimeError(f"{operation} failed after partial write")
                self.motor.errors = {operation: error}
                core = self.make_core()
                with self.assertRaises(RuntimeError) as raised:
                    core.activate(self.clock.now)
                self.assertIs(raised.exception, error)
                self.assertIs(core.report.primary.error, error)
                self.assertEqual(self.motor.writes[-1], ("disable", "joint1"))
                self.assertTrue(core.stop_latched)
                core.close()
                self.assertEqual(self.motor.writes[-1], ("close",))

    def test_pose_change_after_prepare_prevents_enable(self):
        self.motor.after_prepare = lambda: self.motor.positions.__setitem__(2, 0.001)
        with self.assertRaises(FeedbackError):
            self.core.activate(self.clock.now)
        self.assertFalse(any(c[0] == "enable" for c in self.motor.writes))
        self.assertEqual(self.motor.writes[-1], ("disable", "joint1"))

    def test_interruption_during_enable_still_latches_and_disables(self):
        interruption = KeyboardInterrupt()
        self.motor.errors["enable"] = interruption
        with self.assertRaises(KeyboardInterrupt) as raised:
            self.core.activate(self.clock.now)
        self.assertIs(raised.exception, interruption)
        self.assertIs(self.core.report.primary.error, interruption)
        self.assertEqual(self.core.state, State.STOPPED)
        self.assertEqual(self.motor.writes[-1], ("disable", "joint1"))
        with self.assertRaises(RuntimeError):
            self.core.activate(self.clock.now)

    def test_stale_feedback_after_prepare_never_enables(self):
        self.motor.after_prepare = lambda: setattr(
            self.motor, "sample", PositionSample([0.0] * 6, 99.0, 100.0),
        )
        with self.assertRaises(FeedbackError):
            self.core.activate(self.clock.now)
        self.assertFalse(any(c[0] == "enable" for c in self.motor.writes))
        self.assertEqual(self.motor.writes[-1], ("disable", "joint1"))

    def test_bad_names_targets_durations_velocity_and_inactive_changes_rejected(self):
        self.core.activate(self.clock.now)
        cases = [
            ([0.005] + [0.0] * 5, 2.0, JOINT_NAMES[::-1]),
            ([0.005] + [0.0] * 5, 2.0, ("joint1",) * 6),
            ([0.0] * 5, 2.0, JOINT_NAMES),
            ([math.nan] + [0.0] * 5, 2.0, JOINT_NAMES),
            ([0.02] + [0.0] * 5, 2.0, JOINT_NAMES),
            ([0.005] + [0.0] * 5, 0.1, JOINT_NAMES),
            ([0.005, 0.001] + [0.0] * 4, 2.0, JOINT_NAMES),
        ]
        cases.extend(([0.005] + [0.0] * 5, duration, JOINT_NAMES)
                     for duration in (0, -1, 5.001, math.inf, math.nan))
        before = list(self.motor.writes)
        for target, duration, names in cases:
            with self.subTest(target=target, duration=duration), self.assertRaises(GoalRejected):
                self.core.start_goal(target, duration, self.clock.now, joint_names=names)
        self.assertEqual(self.motor.writes, before)
        self.assertFalse(self.core.stop_latched)

    def test_cancel_before_first_tick_never_sends_goal_and_does_not_replay(self):
        self.begin()
        self.core.cancel(self.clock.now)
        self.assertEqual(self.core.goal_state, GoalState.CANCELED)
        self.assertFalse(any(c[0] == "send" for c in self.motor.writes))
        self.tick_at(1.0)
        self.tick_at(3.0)
        self.assertEqual([c[2] for c in self.motor.writes if c[0] == "send"], [0.0, 0.0])

    def test_cancel_holds_measured_pose_and_drops_old_target(self):
        self.begin()
        self.tick_at(0.4)
        self.motor.positions[0] = 0.0008
        self.clock.now = 100.5
        self.core.cancel(self.clock.now)
        self.assertEqual(self.motor.last_target, 0.0008)
        self.tick_at(3.0)
        self.assertEqual(self.motor.last_target, 0.0008)
        self.assertEqual(self.core.goal_state, GoalState.CANCELED)

    def test_cancel_does_not_bypass_step_limit_to_snap_to_measured_pose(self):
        self.begin()
        self.motor.positions[0] = 0.005
        self.clock.now = 101.0
        with self.assertRaises(MotionError):
            self.core.cancel(self.clock.now)
        self.assertEqual(self.core.state, State.STOPPED)
        self.assertFalse(any(c[0] == "send" for c in self.motor.writes))

    def test_only_one_goal_and_no_implicit_preemption(self):
        self.begin()
        with self.assertRaises(GoalRejected):
            self.core.start_goal([-0.005] + [0.0] * 5, 2.0, self.clock.now)
        self.tick_at(0.4)
        self.assertAlmostEqual(self.motor.last_target, 0.001)

    def test_new_waypoint_continues_held_command_despite_tracking_offset(self):
        self.motor.positions[0] = 0.0539
        self.core.activate(self.clock.now)
        self.tick_at(0.02)
        self.motor.positions[0] = 0.0525
        self.tick_at(0.04)
        self.core.start_goal([0.05375] + [0.0] * 5, 0.5, self.clock.now)
        self.tick_at(0.06)
        self.assertAlmostEqual(self.motor.last_target, 0.053894)
        self.assertEqual(self.core.goal_state, GoalState.RUNNING)
        self.assertFalse(self.core.stop_latched)

    def test_goal_rate_is_checked_from_the_command_as_well_as_measurement(self):
        self.core.activate(self.clock.now)
        self.motor.positions[0] = -0.0015
        self.tick_at(0.02)
        with self.assertRaisesRegex(GoalRejected, "velocity"):
            self.core.start_goal([-0.0014] + [0.0] * 5, 0.1, self.clock.now)

    def test_convergence_does_not_snap_command_to_measured_position(self):
        self.begin(delta=0.00125, duration=0.5)
        self.tick_at(0.24)
        previous = self.motor.last_target
        self.motor.positions[0] = 0.0011
        self.tick_at(0.27)
        self.assertEqual(self.core.goal_state, GoalState.SUCCEEDED)
        self.assertAlmostEqual(self.motor.last_target, 0.000675)
        self.assertLessEqual(abs(self.motor.last_target - previous), 0.01 * 0.03)
        self.tick_at(0.3)
        self.assertAlmostEqual(self.motor.last_target, 0.000675)

    def test_inspect_sized_waypoints_with_lag_preserve_every_command_limit(self):
        self.motor.positions[0] = 0.0539
        self.core.activate(self.clock.now)
        self.motor.positions[0] = 0.0525
        self.tick_at(0.02)
        previous_command, previous_time = self.motor.last_target, self.clock.now
        for target in (0.05375, 0.055, 0.05625, 0.0575):
            self.core.start_goal([target] + [0.0] * 5, 0.5, self.clock.now)
            for _ in range(24):
                self.clock.now += 0.02
                self.motor.positions[0] += 0.5 * (self.motor.last_target - self.motor.positions[0])
                self.core.tick(self.clock.now)
                delta = abs(self.motor.last_target - previous_command)
                self.assertLessEqual(delta, MAX_COMMAND_STEP)
                self.assertLessEqual(delta, 0.01 * (self.clock.now - previous_time) + 1e-12)
                previous_command, previous_time = self.motor.last_target, self.clock.now
                if self.core.goal_state == GoalState.SUCCEEDED:
                    break
            self.assertEqual(self.core.goal_state, GoalState.SUCCEEDED)
            self.assertLessEqual(abs(self.motor.positions[0] - target), GOAL_TOLERANCE)
        self.assertFalse(self.core.stop_latched)

    def test_bounded_goal_succeeds_only_with_measured_convergence(self):
        self.begin()
        last = 0.0
        for elapsed in (0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8):
            self.motor.positions[0] = last
            self.tick_at(elapsed)
            self.assertLessEqual(abs(self.motor.last_target - last), MAX_COMMAND_STEP)
            last = self.motor.last_target
            self.assertEqual(self.core.goal_state, GoalState.RUNNING)
        self.motor.positions[0] = 0.005 - GOAL_TOLERANCE / 2
        self.tick_at(2.0)
        self.assertEqual(self.core.goal_state, GoalState.SUCCEEDED)
        self.assertEqual(self.motor.last_target, 0.005)
        self.assertTrue(all(c[1] == ACTIVE_AXIS for c in self.motor.writes if c[0] != "close"))

    def test_dispatch_without_physical_motion_never_counts_as_success(self):
        self.begin()
        for elapsed in (0.4, 0.8, 1.2, 1.6):
            self.tick_at(elapsed)
        self.assertGreater(self.motor.last_target, 0.0)
        self.assertEqual(self.motor.positions[0], 0.0)
        with self.assertRaisesRegex(MotionError, "measured convergence"):
            self.tick_at(2.0)
        self.assertEqual(self.core.goal_state, GoalState.ABORTED)
        self.assertTrue(self.core.stop_latched)

    def test_measured_convergence_can_finish_before_exact_deadline(self):
        self.begin()
        for elapsed in (0.4, 0.8, 1.2, 1.6):
            self.tick_at(elapsed)
        self.motor.positions[0] = 0.0047
        self.tick_at(1.95)
        self.assertEqual(self.core.goal_state, GoalState.SUCCEEDED)
        self.assertAlmostEqual(self.motor.last_target, 0.004875)
        self.tick_at(2.1)
        self.assertEqual(self.core.goal_state, GoalState.SUCCEEDED)
        self.assertAlmostEqual(self.motor.last_target, 0.004875)

    def test_timeout_drops_goal_even_if_feedback_later_looks_converged(self):
        self.begin()
        self.motor.positions[0] = 0.005
        with self.assertRaisesRegex(MotionError, "deadline expired"):
            self.tick_at(2.01)
        self.assertEqual(self.core.goal_state, GoalState.ABORTED)
        writes = list(self.motor.writes)
        self.tick_at(3.0)
        self.assertEqual(self.motor.writes, writes)

    def test_timeout_records_target_command_and_actual_tracking_error(self):
        self.begin()
        for elapsed in (0.4, 0.8, 1.2, 1.6):
            self.tick_at(elapsed)
        self.motor.positions[0] = 0.0002
        with self.assertRaises(MotionError) as raised:
            self.tick_at(2.01)
        message = str(raised.exception)
        for detail in ("target_rad=0.005", "last_command_rad=0.004", "measured_rad=0.0002",
                       "error_rad=0.0048", "tolerance_rad=0.0006"):
            self.assertIn(detail, message)
        self.assertTrue(self.core.stop_latched)

    def test_delayed_tick_cannot_skip_command_step_limit(self):
        self.begin()
        with self.assertRaisesRegex(MotionError, "command step"):
            self.tick_at(1.0)
        self.assertFalse(any(c[0] == "send" for c in self.motor.writes))

    def test_stale_feedback_prevents_next_target_and_latches(self):
        self.begin()
        self.motor.sample = PositionSample([0.0] * 6, 99.0, 100.0)
        with self.assertRaises(FeedbackError):
            self.tick_at(0.2)
        self.assertFalse(any(c[0] == "send" for c in self.motor.writes))
        self.assertEqual(self.motor.writes[-1], ("disable", "joint1"))
        self.assertEqual(self.core.goal_state, GoalState.ABORTED)
        self.assertFalse(self.core.report.torque_off_confirmed)

    def test_transport_fault_is_primary_despite_disable_and_close_failures(self):
        self.begin()
        primary = OSError("CAN send failed")
        disable = OSError("disable not acknowledged")
        close = OSError("socket close failed")
        self.motor.errors = {"send": primary, "disable": disable, "close": close}
        with self.assertRaises(OSError) as raised:
            self.tick_at(0.2)
        self.assertIs(raised.exception, primary)
        self.assertIs(self.core.report.primary.error, primary)
        self.assertIn(disable, [failure.error for failure in self.core.report.cleanup])
        report = self.core.close()
        self.assertIs(report.primary.error, primary)
        self.assertIn(close, [failure.error for failure in report.cleanup])
        self.assertFalse(report.disable_acknowledged)
        self.assertFalse(report.torque_off_confirmed)
        self.assertEqual(self.motor.writes[-1], ("close",))

    def test_read_fault_preserves_primary_even_when_disable_feedback_also_fails(self):
        self.begin()
        original = OSError("read failed")
        disable = OSError("disable failed")
        self.motor.errors = {"read": original, "disable": disable}
        with self.assertRaises(OSError) as raised:
            self.tick_at(0.2)
        self.assertIs(raised.exception, original)
        self.assertIs(self.core.report.primary.error, original)
        self.assertEqual([f.operation for f in self.core.report.cleanup],
                         ["disable.feedback", "disable"])
        self.core.close()
        self.assertEqual(self.motor.writes[-1], ("close",))

    def test_cancel_with_invalid_feedback_sends_no_target(self):
        self.begin()
        self.motor.sample = PositionSample([math.nan] + [0.0] * 5, 100.0, 100.0)
        with self.assertRaises(FeedbackError):
            self.core.cancel(self.clock.now)
        self.assertTrue(self.core.stop_latched)
        self.assertFalse(any(c[0] == "send" for c in self.motor.writes))
        self.assertEqual(self.motor.writes[-1], ("disable", "joint1"))

    def test_deactivate_cancels_before_disable_close_never_homes(self):
        self.begin()
        report = self.core.close()
        self.assertEqual(self.core.state, State.CLOSED)
        self.assertEqual(self.core.goal_state, GoalState.CANCELED)
        self.assertEqual(self.motor.writes, [
            ("prepare", "joint1", 0.0), ("enable", "joint1"),
            ("disable", "joint1"), ("close",),
        ])
        self.assertTrue(report.disable_acknowledged)
        self.assertFalse(report.torque_off_confirmed)
        self.assertIsNone(report.primary)
        before = list(self.motor.calls)
        self.core.close()
        self.assertEqual(self.motor.calls, before)

    def test_persisted_stop_needs_fresh_explicit_recovery_and_fresh_activation(self):
        self.core = self.make_core(persisted_stopped=True)
        with self.assertRaises(RuntimeError):
            self.core.activate(self.clock.now)
        self.assertEqual(self.motor.calls, [])
        self.motor.sample = PositionSample([0.0] * 6, 99.0, 100.0)
        with self.assertRaises(FeedbackError):
            self.core.recover(self.clock.now)
        self.assertTrue(self.core.stop_latched)
        self.motor.sample = None
        prior = self.core.recover(self.clock.now)
        self.assertIsInstance(prior.primary.error, FeedbackError)
        self.assertFalse(self.core.stop_latched)
        self.assertEqual(self.core.state, State.INACTIVE)
        self.assertEqual(self.motor.writes, [])
        self.core.tick(self.clock.now)
        self.assertEqual(self.motor.writes, [])
        self.core.activate(self.clock.now)
        self.assertEqual(self.core.goal_state, GoalState.IDLE)

    def test_failed_disable_blocks_recovery_until_explicit_deactivation_retry(self):
        self.begin()
        self.motor.errors["disable"] = OSError("disable failed")
        self.core.deactivate()
        with self.assertRaises(RuntimeError):
            self.core.recover(self.clock.now)
        self.motor.errors.clear()
        self.core.deactivate()
        self.core.recover(self.clock.now)
        self.assertEqual(self.core.state, State.INACTIVE)
        self.assertEqual(self.core.goal_state, GoalState.IDLE)

    def test_failed_recovery_is_distinct_from_original_fault_and_cleanup(self):
        self.begin()
        original = OSError("send failed")
        self.motor.errors["send"] = original
        with self.assertRaises(OSError):
            self.tick_at(0.2)
        self.motor.sample = PositionSample([0.0] * 6, 99.0, 100.0)
        with self.assertRaises(FeedbackError):
            self.core.recover(self.clock.now)
        self.assertIs(self.core.report.primary.error, original)
        self.assertEqual(self.core.report.cleanup, ())
        self.assertEqual(len(self.core.report.recovery_failures), 1)
        self.assertTrue(self.core.stop_latched)

    def test_external_stop_latches_drops_goal_and_requires_deliberate_reactivation(self):
        self.begin()
        self.core.stop("operator stop")
        self.assertTrue(self.core.stop_latched)
        self.assertEqual(self.core.goal_state, GoalState.ABORTED)
        self.assertEqual(self.motor.writes[-1], ("disable", "joint1"))
        self.core.recover(self.clock.now)
        self.assertEqual(self.core.state, State.INACTIVE)
        before = list(self.motor.writes)
        self.core.tick(self.clock.now)
        self.assertEqual(self.motor.writes, before)
        self.core.activate(self.clock.now)
        self.clock.now = 100.2
        self.core.tick(self.clock.now)
        self.assertEqual(self.motor.last_target, 0.0)
        self.assertEqual(self.core.goal_state, GoalState.IDLE)

    def test_inactive_axis_drift_uses_raw_start_even_near_boundaries(self):
        self.motor.positions[5] = -1.0 - MEASURED_TOLERANCE / 2
        self.core.activate(self.clock.now)
        target = list(self.core.bounded_positions)
        target[0] = 0.005
        self.core.start_goal(target, 2.0, self.clock.now)
        self.motor.positions[5] = -1.0 + MEASURED_TOLERANCE * 0.75
        with self.assertRaisesRegex(MotionError, "inactive axis moved"):
            self.tick_at(0.2)
        self.assertFalse(any(c[0] == "send" for c in self.motor.writes))

    def test_successive_goals_cannot_ratchet_outside_activation_envelope(self):
        self.begin()
        for elapsed in (0.4, 0.8, 1.2, 1.6):
            self.tick_at(elapsed)
        self.motor.positions[0] = 0.005
        self.tick_at(2.0)
        with self.assertRaisesRegex(GoalRejected, "excursion"):
            self.core.start_goal([0.014] + [0.0] * 5, 2.0, self.clock.now)

    def test_clock_regression_latches_without_target_write(self):
        self.begin()
        self.clock.now = 99.9
        with self.assertRaises(FeedbackError):
            self.core.tick(self.clock.now)
        self.assertFalse(any(c[0] == "send" for c in self.motor.writes))
        self.assertTrue(self.core.stop_latched)


if __name__ == "__main__":
    unittest.main()
