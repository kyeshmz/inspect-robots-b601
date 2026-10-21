"""Hardware-free tests for the B601 Motorbridge session, read and recover."""

import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from inspect_robots_b601.embodiment import DEFAULT_PROFILE
from inspect_robots_b601.hardware import session as session_module
from inspect_robots_b601.hardware.session import ArmSession, MoveError, read, recover
from inspect_robots_b601.hardware.state import StopLedger


class FakeMotor:
    def __init__(self, position=0.0):
        self.position = position
        self.enabled = False
        self.caps = {}
        self.disabled_at = None  # position when torque was removed
        self.ceiling = None      # blocked: the joint cannot go above this
        self.forced = None       # an external push: reads this regardless of command
        self.broken = False      # CAN I/O failure
        self.slow_reads = False  # replies never arrive; each read waits out its timeout
        self.broken_reads = False  # position reads fail instantly, commands still work
        self.taus = []
        self.fault = 0           # fault word from the motor's fault report

    def robstride_get_param_f32(self, parameter, timeout_ms=1000):
        if self.slow_reads:
            time.sleep(timeout_ms / 1000)
            raise OSError("reply timed out")
        if self.broken or (self.broken_reads and parameter == 0x7019):
            raise OSError("no reply")
        if parameter == 0x7019:
            return self.position if self.forced is None else self.forced
        return self.caps.get(parameter, 0.0)

    def robstride_write_param_f32(self, parameter, value):
        self.caps[parameter] = value

    def robstride_write_param_u32(self, parameter, value):
        self.caps[parameter] = value

    def ensure_mode(self, mode, timeout_ms=1000):
        pass

    def send_mit(self, position, vel, kp, kd, tau):
        if self.broken:
            raise OSError("bus down")
        self.taus.append(tau)
        self.position = position if self.ceiling is None else min(position, self.ceiling)

    def get_state(self):
        return SimpleNamespace(status_code=0, pos=self.position, vel=0.0, torq=0.4, t_mos=36.0, t_rotor=0.0)

    def robstride_get_fault_report(self):
        return self.fault, 0

    def enable(self):
        self.enabled = True

    def disable(self):
        if self.enabled and self.disabled_at is None:
            self.disabled_at = self.position
        self.enabled = False


class FakeController:
    def __init__(self, positions=None):
        self.motors = [FakeMotor(p) for p in (positions or [0.0] * 7)]

    def add_robstride_motor(self, motor_id, feedback_id, model):
        return self.motors[motor_id - 1]

    def disable_all(self):
        for motor in self.motors:
            motor.disable()

    def close(self):
        pass


class Dummy:
    closed = False

    def close(self):
        self.closed = True


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.legacy = Path(self.directory.name) / "legacy.json"
        self.legacy.write_text(json.dumps({
            "schema_version": 1, "latched": True, "last_stop_unix_s": 0,
        }))
        self.state_dir = Path(self.directory.name) / "state"
        self.state_dir.mkdir()
        (self.state_dir / "stop-state.json").write_text(json.dumps({
            "schema_version": 1, "stopped": False,
            "acknowledged_legacy_sha256":
                __import__("hashlib").sha256(self.legacy.read_bytes()).hexdigest(),
            "reason": "test recovery",
        }))
        profile = json.loads(DEFAULT_PROFILE.read_text())
        profile["legacy_stop_latch"] = str(self.legacy)
        profile["gravity_feedforward"] = False  # fakes ignore torque; FF has its own tests
        self.profile = profile
        self.controller = FakeController()
        self.claims = []

    def claim(self, profile):
        self.claims.append(Dummy())
        return self.claims[-1]

    def session(self, **kwargs):
        kwargs.setdefault("home_wait", 0.0)
        kwargs.setdefault("home_retry_s", 0.01)
        session = ArmSession(self.profile, self.state_dir,
                                   controller_factory=lambda channel: self.controller,
                                   bus_factory=self.claim, period=0.005, **kwargs)
        self.addCleanup(session.close)
        return session

    def active(self, **kwargs):
        session = self.session(**kwargs)
        session.open()
        session.activate()
        return session

    def stop_state(self):
        return json.loads((self.state_dir / "stop-state.json").read_text())

    def assert_disabled_at_home(self):
        for motor in self.controller.motors:
            self.assertFalse(motor.enabled)
            self.assertIsNotNone(motor.disabled_at)
            self.assertAlmostEqual(motor.disabled_at, 0.0, places=6)

    def test_open_activate_hold_and_move_all_joints(self):
        session = self.session()
        session.open()
        self.assertEqual(session.observe(), (0.0,) * 7)
        session.activate()
        self.assertTrue(all(m.enabled for m in self.controller.motors))
        session.move([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01], 2.0)
        for motor in self.controller.motors:
            self.assertAlmostEqual(motor.position, 0.01)
        session.close()
        self.assertFalse(any(m.enabled for m in self.controller.motors))

    def test_bus_is_claimed_for_the_profile_channel_and_released_on_close(self):
        channels = []
        self.profile["channel"] = "can7"
        session = ArmSession(self.profile, self.state_dir, bus_factory=self.claim,
                                   controller_factory=lambda channel: channels.append(channel) or self.controller)
        self.addCleanup(session.close)
        session.open()
        self.assertEqual(channels, ["can7"])
        self.assertEqual(len(self.claims), 1)
        session.close()
        self.assertTrue(self.claims[0].closed)

    def test_without_another_controller_a_new_state_dir_needs_only_its_own_recovery(self):
        del self.profile["legacy_stop_latch"]
        state_dir = Path(self.directory.name) / "fresh"

        def session():
            created = ArmSession(self.profile, state_dir, bus_factory=self.claim,
                                       controller_factory=lambda channel: self.controller, period=0.005)
            self.addCleanup(created.close)
            return created

        with self.assertRaisesRegex(RuntimeError, "stop is latched"):
            session().open()
        self.assertEqual(self.claims, [])
        StopLedger(state_dir).acknowledge(details={"test": True})
        opened = session().open()
        self.assertEqual(opened.observe(), (0.0,) * 7)

    def fresh_state(self):
        del self.profile["legacy_stop_latch"]
        return Path(self.directory.name) / "fresh"

    def test_read_reports_every_motor_and_the_latch_without_enabling(self):
        self.controller.motors[6].fault = 0x08
        report = read(self.profile, self.state_dir, controller_factory=lambda channel: self.controller,
                      bus_factory=self.claim)
        self.assertEqual(list(report["positions_rad"]), [f"joint{i}" for i in range(1, 7)] + ["gripper"])
        self.assertEqual(report["faults"]["gripper"], {"fault": 0x08, "warning": 0})
        self.assertFalse(report["stop"]["stopped"])
        self.assertFalse(any(motor.enabled for motor in self.controller.motors))
        self.assertTrue(self.claims[0].closed)

    def test_recover_acknowledges_a_new_arm_after_fresh_reads(self):
        state_dir = self.fresh_state()
        self.assertTrue(StopLedger(state_dir).stopped)
        result = recover(self.profile, state_dir, controller_factory=lambda channel: self.controller,
                         bus_factory=self.claim)
        self.assertTrue(result["prior"]["stopped"])
        self.assertFalse(StopLedger(state_dir).stopped)
        self.assertEqual(StopLedger(state_dir).snapshot()["details"]["fresh_positions_rad"]["joint1"], 0.0)
        self.assertFalse(any(motor.enabled for motor in self.controller.motors))
        self.assertTrue(self.claims[0].closed)

    def test_recover_refuses_a_faulted_motor_or_failed_read(self):
        state_dir = self.fresh_state()
        self.controller.motors[3].fault = 0x04
        with self.assertRaisesRegex(RuntimeError, "motor faults on joint4"):
            recover(self.profile, state_dir, controller_factory=lambda channel: self.controller,
                    bus_factory=self.claim)
        self.controller.motors[3].fault = 0
        self.controller.motors[2].broken = True
        with self.assertRaises(OSError):
            recover(self.profile, state_dir, controller_factory=lambda channel: self.controller,
                    bus_factory=self.claim)
        self.assertTrue(StopLedger(state_dir).stopped)
        self.assertTrue(all(claim.closed for claim in self.claims))

    def test_every_axis_needs_a_torque_cap(self):
        del self.profile["gripper"]["torque_cap_nm"]
        with self.assertRaisesRegex(ValueError, "torque_cap_nm"):
            ArmSession(self.profile, self.state_dir)

    def test_missing_motorbridge_is_explained(self):
        with patch.object(session_module, "Controller", None):
            with self.assertRaisesRegex(RuntimeError, r"inspect-robots-b601\[motorbridge\]"):
                session_module._default_controller("can0")

    def test_target_bounds_and_step_are_rejected(self):
        session = self.session()
        session.open()
        session.activate()
        with self.assertRaises(ValueError):
            session.move([0.12, 0, 0, 0, 0, 0, 0], 2.0)
        with self.assertRaises(ValueError):
            session.move([0.01, 0, 0, 0, 0, 0, 0], 0.1)

    def test_stop_disables_all(self):
        session = self.session()
        session.open()
        session.activate()
        session.stop("test stop")
        self.assertFalse(any(m.enabled for m in self.controller.motors))

    def test_close_walks_home_before_disabling(self):
        session = self.active()
        session.move([0.05, 0.05, 0.05, 0.0, 0.0, 0.0, 0.2], 2.0)
        self.assertAlmostEqual(self.controller.motors[2].position, 0.05)
        session.close()
        self.assert_disabled_at_home()

    def test_stop_latches_then_walks_home_before_disabling(self):
        session = self.active()
        session.move([0.0, 0.0, 0.08, 0.0, 0.0, 0.0, 0.0], 2.0)
        session.stop("operator stop")
        self.assert_disabled_at_home()
        self.assertTrue(self.stop_state()["stopped"])

    def test_blocked_joint_reports_the_joint_and_releases_its_push(self):
        session = self.active()
        self.controller.motors[1].ceiling = 0.0
        with self.assertRaises(MoveError) as raised:
            session.move([0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0], 2.0)
        self.assertIn("joint2 requested +0.050 rad, moved +0.000 rad", str(raised.exception))
        self.assertAlmostEqual(session._command[1], 0.0)
        self.assertTrue(self.controller.motors[1].enabled)
        session.close()
        self.assert_disabled_at_home()

    def test_partial_progress_inside_tolerance_still_converges(self):
        session = self.active()
        self.controller.motors[2].ceiling = 0.035  # 70% of the request, 0.015 short
        session.move([0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0], 2.0)
        session.close()
        self.assert_disabled_at_home()

    def test_envelope_fault_holds_names_the_joint_and_homes_before_disabling(self):
        session = self.active()
        session.move([0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0], 2.0)
        self.controller.motors[3].forced = 0.7
        deadline = time.monotonic() + 2.0
        while session._failure is None and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertIsNotNone(session._failure)
        self.assertTrue(all(m.enabled for m in self.controller.motors))
        details = self.stop_state()["details"]["error"]
        self.assertIn("joint4 measured +0.7000 rad", details)
        with self.assertRaises(RuntimeError):
            session.move([0.0] * 7, 2.0)
        self.controller.motors[3].forced = None
        session.close()
        self.assert_disabled_at_home()

    def test_blocked_home_holds_until_the_wait_expires(self):
        session = self.active(home_wait=0.05)
        session.move([0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0], 2.0)
        self.controller.motors[2].forced = 0.05  # cannot leave the raised pose
        with patch("inspect_robots_b601.hardware.session.PATH_STALL_S", 0.2):
            started = time.monotonic()
            session.close()
        self.assertGreater(time.monotonic() - started, 0.2)
        self.assertFalse(any(m.enabled for m in self.controller.motors))

    def test_lost_can_control_disables_and_latches(self):
        session = self.active()
        self.controller.motors[0].broken = True
        deadline = time.monotonic() + 2.0
        while session._enabled and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertFalse(session._enabled)
        self.assertTrue(session._lost_control)
        self.assertEqual(self.stop_state()["reason"], "arm control lost")

    def test_missed_replies_never_stall_the_command_stream(self):
        session = self.active()
        self.controller.motors[2].slow_reads = True
        time.sleep(0.3)
        self.assertLess(session.max_send_gap, 0.06)
        self.assertIsNone(session._failure)
        self.controller.motors[2].slow_reads = False
        session.close()
        self.assert_disabled_at_home()

    def test_persistent_missed_replies_fault_then_walk_home(self):
        session = self.active()
        session.move([0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0], 2.0)
        self.controller.motors[4].broken_reads = True
        deadline = time.monotonic() + 5.0
        while session._failure is None and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertIn("joint5 position reads failed", str(session._failure))
        self.assertTrue(all(m.enabled for m in self.controller.motors))
        self.controller.motors[4].broken_reads = False
        session.close()
        self.assert_disabled_at_home()

    def test_gravity_feedforward_ramps_in_reaches_arm_only_and_ramps_out(self):
        with patch("inspect_robots_b601.hardware.session.FF_RAMP_S", 0.1):
            session = self.active(gravity=lambda q: (0.0, 1.5, -2.5, 0.25, 0.0, 0.0))
            time.sleep(0.01)
            self.assertLess(self.controller.motors[1].taus[-1], 1.5)
            time.sleep(0.2)
            self.assertEqual(self.controller.motors[1].taus[-1], 1.5)
            self.assertEqual(self.controller.motors[2].taus[-1], -2.5)
            self.assertEqual(self.controller.motors[6].taus[-1], 0.0)
            session.close()
        self.assertEqual(self.controller.motors[1].taus[-1], 0.0)
        self.assert_disabled_at_home()

    def test_profile_flag_selects_the_urdf_gravity_model(self):
        from inspect_robots_b601.hardware.gravity import compensation_torques
        self.profile["gravity_feedforward"] = True
        self.assertIs(self.session()._gravity, compensation_torques)
        self.profile["gravity_feedforward"] = False
        self.assertIsNone(self.session()._gravity)

    def test_every_session_writes_a_motor_log_from_open_to_disable_at_home(self):
        session = self.active()
        session.move([0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0], 2.0)
        time.sleep(0.3)
        session.close()
        logs = list((Path(self.directory.name) / "logs" / "motor").glob("*.jsonl"))
        self.assertEqual(len(logs), 1)
        records = [json.loads(line) for line in logs[0].read_text().splitlines()]
        statuses = [record["status"] for record in records]
        for status in ("motor-log-start", "session-open", "motors-enabled", "actuator-output",
                       "path-start", "motors-disabled", "motor-log-end"):
            self.assertIn(status, statuses)
        self.assertLess(statuses.index("path-start"), statuses.index("motors-disabled"))
        disabled = next(record for record in records if record["status"] == "motors-disabled")
        self.assertLess(disabled["max_distance_from_home_rad"], 0.01)
        output = next(record for record in records if record["status"] == "actuator-output")
        self.assertEqual(output["thermal"]["joint3"]["measured_c"], 36.0)
        self.assertEqual(output["thermal"]["joint3"]["cap_nm"], 36.0)

    def test_go_to_waits_for_a_gripper_only_move(self):
        session = self.active()
        measured = session.go_to([0.0] * 6 + [0.8], timeout=10.0)
        self.assertAlmostEqual(measured[6], 0.8, places=3)

    def test_go_to_finishes_when_the_gripper_closes_on_an_object(self):
        session = self.active()
        self.controller.motors[6].ceiling = 0.3
        with patch("inspect_robots_b601.hardware.session.GRIPPER_SETTLE_S", 0.2):
            measured = session.go_to([0.0] * 6 + [0.8], timeout=10.0)
        self.assertAlmostEqual(measured[6], 0.3, places=3)
        self.assertGreater(session._command[6], 0.3)  # still squeezing the object
        self.controller.motors[6].ceiling = None
        session.close()
        self.assert_disabled_at_home()

    def test_go_to_walks_to_a_ready_pose_and_close_returns_home(self):
        session = self.active()
        with patch("inspect_robots_b601.hardware.session.MAX_VELOCITY", 1.0):
            measured = session.go_to([0.0, 0.2, 0.3, 0.0, 0.0, 0.0, 0.0], timeout=5.0)
            self.assertLessEqual(abs(measured[2] - 0.3), session._tolerance)
            self.assertAlmostEqual(session.observe()[2], 0.3, places=3)
            session.close()
        self.assert_disabled_at_home()


if __name__ == "__main__":
    unittest.main()
