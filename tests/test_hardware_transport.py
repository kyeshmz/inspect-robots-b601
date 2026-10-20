"""Software-only wire/ownership tests: all CAN and MotorBridge I/O is fake."""

import copy
import hashlib
import json
import math
import os
import socket
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from b601_hardware import transport as transport_module
from b601_hardware.core import CommissioningCore, GoalState, PositionSample, State
from b601_hardware.transport import (
    CleanupError,
    Joint1Transport,
    OpenError,
    SessionFailed,
    StrictParameterReader,
    TransportError,
    UnconfirmedOperation,
    decode_parameter_reply,
    decode_status_reply,
)

FRAME = struct.Struct("=IB3x8s")
EFF = 0x80000000
FORMATS = {0x7005: "<b", 0x700B: "<f", 0x7019: "<f", 0x7028: "<I"}
PROFILE = json.loads((Path(__file__).parent / "fixtures/joint1.synthetic.json").read_text())


def reply(parameter=0x7019, value=0.125, motor=1, host=0xFD, error=0):
    data = bytearray(8)
    struct.pack_into("<H", data, 0, parameter)
    struct.pack_into(FORMATS[parameter], data, 4, value)
    return FRAME.pack(EFF | (17 << 24) | (error << 16) | (motor << 8) | host, 8, data)


def status(mode=2, faults=0, motor=1, host=0xFD):
    return FRAME.pack(
        EFF | (2 << 24) | (mode << 22) | (faults << 16) | (motor << 8) | host, 8, bytes(8)
    )


class Clock:
    def __init__(self):
        self.now, self.wall_shift = 100.0, 0

    def __call__(self):
        return self.now

    def wall_ns(self):
        return 1_700_000_000_000_000_000 + round(self.now * 1e9) + self.wall_shift


class Resource:
    def __init__(self, events, name):
        self.events, self.name, self.close_error = events, name, None

    def close(self):
        self.events.append((self.name + ".close",))
        if self.close_error:
            raise self.close_error


class Bus(Resource):
    def __init__(self, clock, events):
        super().__init__(events, "bus")
        self.clock, self.queue = clock, []
        self.timeout = 0
        self.sent = []
        self.response_hook = None
        self.control_hook = None
        self.errors = {}
        self.writes_take_effect = True
        self.mode, self.fault_bits = 0, 0
        self.values = {0x7005: 0, 0x700B: 1.0, 0x7028: 0}
        self.positions = [0.125, 0.2, 0.3, 0.1, 0.1, 0.1]
        self.short_send = False
        self.option_error = None
        self.unsupported = set()

    def packet(self, frame, *, flags=0, timestamp=None):
        stamp = self.clock.wall_ns() if timestamp is None else timestamp
        seconds, nanos = divmod(stamp, 1_000_000_000)
        return (
            frame,
            [(socket.SOL_SOCKET, 64, struct.pack("=qq", seconds, nanos))],
            flags,
            ("can1",),
        )

    def setsockopt(self, *args):
        self.events.append(("setsockopt", *args))
        if self.option_error:
            raise self.option_error

    def bind(self, address):
        self.events.append(("bind", address))

    def setblocking(self, value):
        self.events.append(("blocking", value))

    def settimeout(self, value):
        self.timeout = value

    def send(self, frame):
        self.sent.append(frame)
        identifier, _, data = FRAME.unpack(frame)
        motor, parameter = identifier & 0xFF, struct.unpack_from("<H", data)[0]
        kind = (identifier >> 24) & 31
        if kind in (3, 4):
            operation = "enable" if kind == 3 else "disable"
            self.events.append((operation, motor))
            if operation in self.errors:
                raise self.errors[operation]
            self.mode = 2 if kind == 3 else 0
            if self.control_hook is not None:
                self.queue.extend(self.control_hook(kind))
            else:
                self.queue.append(self.packet(status(self.mode, self.fault_bits)))
            return len(frame) - int(self.short_send)
        if kind == 18:
            value = struct.unpack_from(FORMATS[parameter], data, 4)[0]
            operation = "write_f32" if parameter == 0x700B else "write_u32"
            self.events.append((operation, parameter, value))
            if operation in self.errors:
                raise self.errors[operation]
            if self.writes_take_effect:
                self.values[parameter] = value
            return len(frame) - int(self.short_send)
        if kind != 17:
            raise AssertionError(f"unexpected raw command type {kind}")
        self.events.append(("query", motor, parameter))
        if self.response_hook is not None:
            self.queue.extend(self.response_hook(motor, parameter))
        else:
            value = self.positions[motor - 1] if parameter == 0x7019 else self.values[parameter]
            self.queue.append(
                self.packet(
                    reply(parameter, value, motor, error=int(parameter in self.unsupported))
                )
            )
        return len(frame) - int(self.short_send)

    def recvmsg(self, size, ancsize, flags=0):
        if flags & socket.MSG_DONTWAIT:
            if not self.queue:
                raise BlockingIOError()
            return self.queue.pop(0)
        if not self.queue:
            self.clock.now += self.timeout
            raise TimeoutError("injected timeout")
        self.clock.now += 0.001
        item = self.queue.pop(0)
        return item() if callable(item) else item


class Motor(Resource):
    def __init__(self, bus, events):
        super().__init__(events, "motor")
        self.bus, self.errors = bus, {}
        self.follow_target = False

    def _call(self, name, *args):
        self.events.append((name, *args))
        if name in self.errors:
            raise self.errors[name]

    def robstride_write_param_f32(self, parameter, value):
        raise AssertionError("vendor parameter writes bypass bounded raw submission")

    def robstride_write_param_u32(self, parameter, value):
        raise AssertionError("vendor parameter writes bypass bounded raw submission")

    def send_mit(self, *args):
        self._call("mit", *args)
        if self.follow_target:
            self.bus.positions[0] = args[0]

    def enable(self):
        raise AssertionError("vendor enable has weak ACK semantics and is forbidden")

    def disable(self):
        raise AssertionError("vendor disable has weak ACK semantics and is forbidden")

    def get_state(self):
        raise AssertionError("cached feedback is forbidden")

    def robstride_get_param_f32(self, *args):
        raise AssertionError("vendor parameter reads are forbidden")


class Controller(Resource):
    def __init__(self, motor, events):
        super().__init__(events, "controller")
        self.motor, self.add_error = motor, None

    def add_robstride_motor(self, *args):
        self.events.append(("add", *args))
        if self.add_error:
            raise self.add_error
        return self.motor

    def shutdown(self):
        raise AssertionError("automatic shutdown/disable is forbidden")

    def __enter__(self):
        raise AssertionError("vendor context manager is forbidden")


class DecoderTests(unittest.TestCase):
    def test_status_mode_fault_fields_and_identity(self):
        for mode in range(4):
            for faults in (0, 1, 32, 63):
                self.assertEqual(decode_status_reply(status(mode, faults)), (mode, faults))
        identifier, dlc, data = FRAME.unpack(status())
        for frame in (
            status(motor=2),
            status(host=0xFE),
            reply(),
            FRAME.pack(identifier & ~EFF, dlc, data),
            FRAME.pack(identifier | 0x40000000, dlc, data),
            FRAME.pack(identifier | 0x20000000, dlc, data),
            FRAME.pack(identifier, 7, data),
            status()[:-1],
            FRAME.pack(identifier, 8, b"\x00\xc4\x56" + bytes(5)),
        ):
            self.assertIsNone(decode_status_reply(frame))
        for flags in (socket.MSG_DONTROUTE, socket.MSG_TRUNC, socket.MSG_CTRUNC):
            self.assertIsNone(decode_status_reply(status(), flags=flags))

    def test_dtype_values(self):
        for parameter, dtype, value in [
            (0x7005, "i8", -1),
            (0x700B, "f32", 0.125),
            (0x7019, "f32", -0.125),
            (0x7028, "u32", 5000),
        ]:
            with self.subTest(dtype=dtype):
                self.assertEqual(
                    decode_parameter_reply(reply(parameter, value), 1, parameter, dtype), value
                )

    def test_mismatches_and_local_frames_are_rejected(self):
        original = reply()
        identifier, dlc, data = FRAME.unpack(original)
        frames = [
            original[:-1],
            original + b"x",
            FRAME.pack(identifier & ~EFF, dlc, data),
            FRAME.pack(identifier | 0x40000000, dlc, data),
            FRAME.pack(identifier | 0x20000000, dlc, data),
            FRAME.pack(identifier, 7, data),
            FRAME.pack(identifier ^ (3 << 24), dlc, data),
            reply(host=0xFE),
            reply(motor=2),
            reply(0x700B),
        ]
        for frame in frames:
            with self.subTest(frame=frame):
                self.assertIsNone(decode_parameter_reply(frame, 1, 0x7019, "f32"))
        for flags in (socket.MSG_DONTROUTE, socket.MSG_TRUNC, socket.MSG_CTRUNC):
            self.assertIsNone(decode_parameter_reply(original, 1, 0x7019, "f32", flags=flags))

    def test_motor_errors_and_nonfinite_values_raise(self):
        for frame in (
            reply(error=1),
            reply(error=255),
            reply(value=math.nan),
            reply(value=math.inf),
            reply(value=-math.inf),
        ):
            with self.assertRaises(TransportError):
                decode_parameter_reply(frame, 1, 0x7019, "f32")

    def test_parameter_dtype_and_motor_are_validated(self):
        for args in (
            (1, 0x7005, "u32"),
            (1, 0x7005, "u8"),
            (1, 0x1234, "f32"),
            (0, 0x7019, "f32"),
            (7, 0x7019, "f32"),
            (True, 0x7019, "f32"),
        ):
            with self.assertRaises(ValueError):
                decode_parameter_reply(reply(), *args)


class ReaderTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.events = Clock(), []
        self.bus = Bus(self.clock, self.events)
        self.reader = StrictParameterReader(self.bus, clock=self.clock, wall_ns=self.clock.wall_ns)

    def test_live_hardware_fault_is_not_discarded_by_parameter_read_drain(self):
        self.reader.set_enabled(True)
        sent = len(self.bus.sent)
        self.bus.queue.append(self.bus.packet(status(faults=2)))
        with self.assertRaisesRegex(TransportError, "fault bits=0x02"):
            self.reader.read(1, 0x7019, "f32")
        self.assertEqual(len(self.bus.sent), sent)
        original = self.reader.failure
        self.reader.set_enabled(False)
        self.assertTrue(self.reader.last_outcome.mode_acknowledged)
        self.assertIs(self.reader.failure, original)

    def test_running_motor_reset_is_detected_while_waiting_for_a_position(self):
        self.reader.set_enabled(True)
        self.bus.response_hook = lambda *args: [
            self.bus.packet(status(mode=0)),
            self.bus.packet(reply()),
        ]
        with self.assertRaisesRegex(TransportError, "mode=0"):
            self.reader.read(1, 0x7019, "f32")

    def test_old_or_local_status_cannot_create_a_new_running_fault(self):
        old = self.clock.wall_ns() - 1
        self.reader.set_enabled(True)
        self.bus.queue.extend(
            [
                self.bus.packet(status(faults=2), timestamp=old),
                self.bus.packet(status(faults=2), flags=socket.MSG_DONTROUTE),
            ]
        )
        self.assertEqual(self.reader.read(1, 0x7019, "f32")[0], 0.125)

    def test_drain_preexisting_data_and_stamp_request_start(self):
        self.bus.queue.append(self.bus.packet(reply(value=99)))
        value, started = self.reader.read(1, 0x7019, "f32")
        self.assertEqual(value, 0.125)
        self.assertEqual(started, 100)
        self.assertGreater(self.clock.now, started)
        identifier, dlc, payload = FRAME.unpack(self.bus.sent[0])
        self.assertEqual(identifier, EFF | (17 << 24) | (0xFD << 8) | 1)
        self.assertEqual((dlc, payload), (8, b"\x19\x70" + bytes(6)))

    def test_unrelated_local_and_pre_request_timestamps_do_not_satisfy_read(self):
        def responses(*args):
            return [
                self.bus.packet(reply(motor=2)),
                self.bus.packet(reply(value=99), flags=socket.MSG_DONTROUTE),
                self.bus.packet(reply(value=98), timestamp=self.clock.wall_ns() - 1),
                self.bus.packet(reply(value=0.25)),
            ]

        self.bus.response_hook = responses
        self.assertEqual(self.reader.read(1, 0x7019, "f32")[0], 0.25)

    def test_timeout_is_50ms_and_poisons_session_without_host_fallback(self):
        self.bus.response_hook = lambda *args: []
        with self.assertRaises(TimeoutError):
            self.reader.read(1, 0x7019, "f32")
        self.assertAlmostEqual(self.clock.now, 100.05)
        self.bus.queue.append(self.bus.packet(reply()))  # A delayed reply cannot rescue/retry.
        with self.assertRaises(SessionFailed):
            self.reader.read(1, 0x7019, "f32")
        self.assertEqual(len(self.bus.sent), 1)

    def test_reply_delivered_after_deadline_is_rejected(self):
        def late():
            self.clock.now += 0.05
            return self.bus.packet(reply())

        self.bus.response_hook = lambda *args: [late]
        with self.assertRaises(TimeoutError):
            self.reader.read(1, 0x7019, "f32")

    def test_drain_flood_is_bounded_before_any_request_is_sent(self):
        original = self.bus.recvmsg

        def flood(size, ancsize, flags=0):
            if flags & socket.MSG_DONTWAIT:
                self.clock.now += 0.01
                return self.bus.packet(reply(motor=2))
            return original(size, ancsize, flags)

        self.bus.recvmsg = flood
        with self.assertRaises(TimeoutError):
            self.reader.read(1, 0x7019, "f32")
        self.assertEqual(self.bus.sent, [])
        with self.assertRaises(SessionFailed):
            self.reader.read(1, 0x7019, "f32")

    def test_missing_future_or_corrupt_receive_timestamp_fails_session(self):
        packets = [
            (reply(), [], 0, ()),
            self.bus.packet(reply(), timestamp=self.clock.wall_ns() + 1_000_000_000),
            (reply(), [(socket.SOL_SOCKET, 64, b"bad")], 0, ()),
        ]
        for packet in packets:
            reader = StrictParameterReader(self.bus, clock=self.clock, wall_ns=self.clock.wall_ns)
            self.bus.response_hook = lambda *args, packet=packet: [packet]
            with self.assertRaises(TransportError):
                reader.read(1, 0x7019, "f32")
            with self.assertRaises(SessionFailed):
                reader.read(1, 0x7019, "f32")

    def test_realtime_clock_step_is_not_accepted_as_freshness(self):
        def stepped():
            self.clock.wall_shift = 1_000_000_000
            return self.bus.packet(reply())

        self.bus.response_hook = lambda *args: [stepped]
        with self.assertRaisesRegex(TransportError, "clocks diverged"):
            self.reader.read(1, 0x7019, "f32")

    def test_short_send_and_parameter_error_poison_session(self):
        self.bus.short_send = True
        with self.assertRaisesRegex(TransportError, "incomplete"):
            self.reader.read(1, 0x7019, "f32")
        with self.assertRaises(SessionFailed):
            self.reader.read(1, 0x7019, "f32")
        self.bus.short_send = False
        self.bus.unsupported.add(0x7028)
        reader = StrictParameterReader(self.bus, clock=self.clock, wall_ns=self.clock.wall_ns)
        with self.assertRaisesRegex(TransportError, "parameter-read error"):
            reader.read(1, 0x7028, "u32")

    def test_mode_ack_ignores_wrong_identity_loopback_version_and_stale_status(self):
        def responses(kind):
            identifier, _, _ = FRAME.unpack(status())
            return [
                self.bus.packet(status(host=0xFE)),
                self.bus.packet(status(motor=2)),
                self.bus.packet(reply()),
                self.bus.packet(status(), flags=socket.MSG_DONTROUTE),
                self.bus.packet(FRAME.pack(identifier, 8, b"\x00\xc4\x56" + bytes(5))),
                self.bus.packet(status(0, 63), timestamp=self.clock.wall_ns() - 1),
                self.bus.packet(status(2)),
            ]

        self.bus.control_hook = responses
        self.reader.set_enabled(True)
        self.assertTrue(self.reader.last_outcome.mode_acknowledged)
        self.assertEqual(self.reader.last_outcome.mode_observed, 2)
        self.assertEqual(len(self.bus.sent), 1)

    def test_mode_ack_timeout_is_bounded_and_disable_cleanup_cannot_unpoison(self):
        self.bus.control_hook = lambda kind: []
        with self.assertRaises(TimeoutError) as caught:
            self.reader.set_enabled(True)
        primary = caught.exception
        self.assertAlmostEqual(self.clock.now, 100.05)
        self.assertTrue(self.reader.last_outcome.transport_submitted)
        self.assertFalse(self.reader.last_outcome.mode_acknowledged)
        with self.assertRaises(SessionFailed):
            self.reader.set_enabled(True)
        self.bus.queue.append(self.bus.packet(status(2)))  # Late enable reply is drained.
        self.bus.control_hook = lambda kind: [self.bus.packet(status(0))]
        self.reader.set_enabled(False)
        self.assertIs(self.reader.failure, primary)
        self.assertTrue(self.reader.last_outcome.mode_acknowledged)
        with self.assertRaises(SessionFailed):
            self.reader.read(1, 0x7019, "f32")
        self.assertEqual([(FRAME.unpack(f)[0] >> 24) & 31 for f in self.bus.sent], [3, 4])

    def test_stale_only_mode_ack_times_out_without_observing_a_mode(self):
        self.bus.control_hook = lambda kind: [
            self.bus.packet(status(2), timestamp=self.clock.wall_ns() - 1)
        ]
        with self.assertRaises(TimeoutError):
            self.reader.set_enabled(True)
        self.assertIsNone(self.reader.last_outcome.mode_observed)
        self.assertFalse(self.reader.last_outcome.mode_acknowledged)
        self.assertAlmostEqual(self.clock.now, 100.05)

    def test_fresh_wrong_mode_or_enable_fault_fails_immediately(self):
        for mode, faults in ((0, 0), (1, 0), (3, 0), (2, 1), (2, 32), (2, 63)):
            with self.subTest(mode=mode, faults=faults):
                reader = StrictParameterReader(
                    self.bus, clock=self.clock, wall_ns=self.clock.wall_ns
                )
                self.bus.control_hook = lambda kind: [
                    self.bus.packet(status(mode, faults)),
                    self.bus.packet(status(2)),
                ]
                with self.assertRaises(UnconfirmedOperation) as caught:
                    reader.set_enabled(True)
                self.assertEqual(
                    (caught.exception.outcome.mode_observed, caught.exception.outcome.fault_bits),
                    (mode, faults),
                )
                self.assertFalse(reader.last_outcome.mode_acknowledged)
                with self.assertRaises(SessionFailed):
                    reader.set_enabled(True)

    def test_status_needs_intact_fresh_kernel_timestamp(self):
        for packet in (
            (status(), [], 0, ()),
            self.bus.packet(status(), timestamp=self.clock.wall_ns() + 1_000_000_000),
        ):
            reader = StrictParameterReader(self.bus, clock=self.clock, wall_ns=self.clock.wall_ns)
            self.bus.control_hook = lambda kind: [packet]
            with self.assertRaises(TransportError):
                reader.set_enabled(True)
            self.assertFalse(reader.last_outcome.mode_acknowledged)

    def test_guard_writer_refuses_mode_zero_faultclear_and_arbitrary_watchdog(self):
        for parameter, value in (
            (0x7005, 0),
            (0x7029, 1),
            (0x700B, 0.31),
            (0x700B, math.nan),
            (0x7028, 0),
            (0x7028, 5000.0),
        ):
            with self.assertRaises(ValueError):
                self.reader.write_guard(parameter, value)
        self.assertEqual(self.bus.sent, [])


class VendorVerificationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.package = Path(self.directory.name) / "motorbridge"
        (self.package / "lib").mkdir(parents=True)
        (self.package / "__init__.py").write_text("")
        self.native = self.package / "lib/libmotor_abi.so"
        self.native.write_bytes(b"fake reviewed native binary, never loaded")
        self.wrappers = {}
        for name in ("core.py", "abi.py"):
            (self.package / name).write_text("# fake reviewed wrapper\n")
            self.wrappers[name] = hashlib.sha256((self.package / name).read_bytes()).hexdigest()
        self.distribution = SimpleNamespace(version="0.5.6", locate_file=lambda name: self.package)
        self.spec = SimpleNamespace(origin=str(self.package / "__init__.py"))
        self.addCleanup(patch.stopall)
        patch.dict(os.environ, {}, clear=True).start()
        patch.object(
            transport_module.importlib.metadata, "distribution", return_value=self.distribution
        ).start()
        patch.object(transport_module.importlib.util, "find_spec", return_value=self.spec).start()
        patch.object(
            transport_module, "_NATIVE_SHA256", hashlib.sha256(self.native.read_bytes()).hexdigest()
        ).start()
        patch.object(transport_module, "_WRAPPER_SHA256", self.wrappers).start()

    def test_verified_package_and_exact_override_are_accepted_without_loading_sdk(self):
        self.assertEqual(transport_module._verify_motorbridge(), self.native)
        os.environ["MOTORBRIDGE_LIB"] = str(self.native)
        self.assertEqual(transport_module._verify_motorbridge(), self.native)

    def test_wrong_version_or_unreviewed_override_is_rejected(self):
        self.distribution.version = "0.5.7"
        with self.assertRaisesRegex(TransportError, "0.5.6"):
            transport_module._verify_motorbridge()
        self.distribution.version = "0.5.6"
        for override in ("", "/unreviewed/libmotor_abi.so"):
            os.environ["MOTORBRIDGE_LIB"] = override
            with self.assertRaisesRegex(TransportError, "override"):
                transport_module._verify_motorbridge()

    def test_tampered_native_wrapper_or_import_shadow_is_rejected(self):
        for path in (self.native, self.package / "core.py", self.package / "abi.py"):
            original = path.read_bytes()
            path.write_bytes(b"different implementation")
            with self.assertRaisesRegex(TransportError, "content"):
                transport_module._verify_motorbridge()
            path.write_bytes(original)
        self.spec.origin = "/another/motorbridge/__init__.py"
        with self.assertRaisesRegex(TransportError, "import path"):
            transport_module._verify_motorbridge()

    def test_previously_loaded_different_abi_cannot_construct_controller(self):
        constructed = []
        fake_module = SimpleNamespace(Controller=lambda **kw: constructed.append(kw))
        fake_abi = SimpleNamespace(
            get_abi=lambda: SimpleNamespace(lib=SimpleNamespace(_name="/other/native.so"))
        )
        with patch.dict(sys.modules, {"motorbridge": fake_module, "motorbridge.abi": fake_abi}):
            with self.assertRaisesRegex(TransportError, "loaded MotorBridge ABI"):
                transport_module._controller("can1")
        self.assertEqual(constructed, [])


class TransportTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.events = Clock(), []
        self.bus = Bus(self.clock, self.events)
        self.motor = Motor(self.bus, self.events)
        self.controller = Controller(self.motor, self.events)
        self.lock = Resource(self.events, "lock")
        self.metadata = {
            "channel": "can1",
            "type": 280,
            "driver": "peak_usb",
            "usb_vendor": "0c72",
            "usb_product": "000c",
            "flags": ["UP"],
            "bitrate": 1_000_000,
            "ctrlmode": [],
            "can_state": "ERROR-ACTIVE",
        }
        self.transport = self.make_transport()

    def make_transport(self, profile=None):
        def factory(name, result):
            def create(*args):
                self.events.append((name, *args))
                return result

            return create

        return Joint1Transport(
            PROFILE if profile is None else profile,
            socket_factory=factory("socket", self.bus),
            controller_factory=factory("controller", self.controller),
            lock_factory=factory("lock", self.lock),
            metadata_reader=factory("metadata", self.metadata),
            clock=self.clock,
            wall_ns=self.clock.wall_ns,
        )

    def tearDown(self):
        self.motor.close_error = self.controller.close_error = self.bus.close_error = None
        self.transport.close()

    def test_constructor_is_network_free_and_read_before_open_refused(self):
        self.assertEqual(self.events, [])
        with self.assertRaises(SessionFailed):
            self.transport.read_positions()
        self.assertEqual(self.events, [])

    def test_invalid_profile_never_opens_hardware(self):
        for key, value in (
            ("channel", "can0"),
            ("driver", "mttcan"),
            ("host_id", 0),
            ("torque_cap_nm", 0.31),
            ("watchdog_ticks", 0),
        ):
            profile = copy.deepcopy(PROFILE)
            profile[key] = value
            with self.assertRaises(ValueError):
                self.make_transport(profile)
        self.assertEqual(self.events, [])

    def test_open_orders_lock_metadata_socket_then_single_motor_registration(self):
        self.transport.open()
        names = [event[0] for event in self.events]
        self.assertEqual(names[:3], ["lock", "metadata", "socket"])
        self.assertLess(names.index("bind"), names.index("controller"))
        self.assertEqual(self.events[-1], ("add", 1, 0xFD, "rs-06"))
        self.assertEqual(str(self.events[0][1]), "/run/lock/astra-b601-can1.lock")
        self.assertEqual(self.bus.sent, [])
        filters = next(
            e[3]
            for e in self.events
            if e[0] == "setsockopt" and e[1:3] == (socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER)
        )
        filter_pairs = list(struct.iter_unpack("=II", filters))
        self.assertEqual({(identifier >> 24) & 31 for identifier, _ in filter_pairs}, {2, 17})
        # Linux routes any mask containing CAN_ERR_FLAG away from normal data frames.
        self.assertTrue(all(mask == 0xDF0000FF for _, mask in filter_pairs))
        with self.assertRaises(SessionFailed):
            self.transport.open()

    def test_unreviewed_default_vendor_is_rejected_before_socket_open(self):
        self.transport._controller_factory = transport_module._controller
        with patch.object(
            transport_module, "_verify_motorbridge", side_effect=TransportError("unreviewed ABI")
        ):
            with self.assertRaisesRegex(TransportError, "unreviewed ABI"):
                self.transport.open()
        self.assertEqual([e[0] for e in self.events], ["lock", "metadata", "lock.close"])

    def test_busy_ownership_lock_prevents_all_metadata_and_hardware_access(self):
        def busy(path):
            raise BlockingIOError("another controller owns CAN")

        self.transport._lock_factory = busy
        with self.assertRaises(BlockingIOError):
            self.transport.open()
        self.assertEqual(self.events, [])
        with self.assertRaises(SessionFailed):
            self.transport.open()

    def test_metadata_mismatch_fails_before_socket_and_releases_lock(self):
        for key, value in (
            ("channel", "can0"),
            ("driver", "mttcan"),
            ("usb_vendor", "ffff"),
            ("usb_product", "ffff"),
            ("bitrate", 500_000),
            ("flags", []),
            ("can_state", "BUS-OFF"),
            ("ctrlmode", ["LISTEN-ONLY"]),
            ("ctrlmode", ["LOOPBACK"]),
            ("type", 1),
        ):
            with self.subTest(key=key, value=value):
                old = self.metadata[key]
                self.metadata[key] = value
                transport = self.make_transport()
                self.events.clear()
                with self.assertRaises(TransportError):
                    transport.open()
                self.assertEqual([e[0] for e in self.events], ["lock", "metadata", "lock.close"])
                self.metadata[key] = old

    def test_positions_have_six_conservative_per_joint_timestamps(self):
        self.transport.open()
        sample = self.transport.read_positions()
        self.assertIsInstance(sample, PositionSample)
        self.assertEqual(len(sample.positions), 6)
        self.assertIsNone(sample.started_at)
        self.assertIsNone(sample.finished_at)
        self.assertEqual(sample.joint_timestamps[0], 100)
        self.assertLess(sample.joint_timestamps[-1], self.clock.now)
        self.assertEqual([e[1] for e in self.events if e[0] == "query"], list(range(1, 7)))

    def test_prepare_orders_strict_reads_writes_readback_and_aligned_hold(self):
        self.transport.open()
        self.events.clear()
        self.transport.prepare_axis("joint1", 0.125)
        calls = [e for e in self.events if e[0] != "blocking"]
        self.assertEqual(
            [e[0] for e in calls],
            [
                "disable",
                "query",
                "query",
                "write_f32",
                "write_u32",
                "query",
                "query",
                "query",
                "mit",
            ],
        )
        self.assertEqual(calls[0], ("disable", 1))
        self.assertEqual(calls[1], ("query", 1, 0x7005))
        self.assertEqual(calls[4], ("write_u32", 0x7028, 5000))
        self.assertLessEqual(calls[3][2], 0.3)
        self.assertEqual(calls[-1], ("mit", 0.125, 0.0, 50.0, 3.0, 0.0))
        self.assertFalse(self.transport.guard_readback["physical_watchdog_qualified"])

    def test_operator_gain_trial_changes_only_joint1_stiffness(self):
        profile = copy.deepcopy(PROFILE)
        profile["joints"][0]["kp"] = 100
        transport = self.make_transport(profile)
        profile["joints"][0]["kp"] = 5000
        try:
            transport.open()
            transport.prepare_axis("joint1", 0.125)
            transport.enable_axis("joint1")
            transport.send_axis("joint1", 0.126)
            self.assertEqual(transport.joint1_kp, 100.0)
            self.assertEqual(transport.guard_readback["commanded_kp"], 100.0)
            self.assertEqual(transport.guard_readback["commanded_kd"], 3.0)
            self.assertLessEqual(transport.guard_readback["torque_cap_nm"], 0.3)
            self.assertEqual(transport.guard_readback["watchdog_ticks"], 5000)
            self.assertEqual(
                [event for event in self.events if event[0] == "mit"],
                [
                    ("mit", 0.125, 0.0, 100.0, 3.0, 0.0),
                    ("mit", 0.126, 0.0, 100.0, 3.0, 0.0),
                ],
            )
            self.assertEqual(
                [event for event in self.events if event[0] == "add"], [("add", 1, 0xFD, "rs-06")]
            )
            transport.disable_axis("joint1")
        finally:
            transport.close()
        self.assertEqual(PROFILE["joints"][0]["kp"], 50.0)

    def test_unlisted_gains_are_refused_before_hardware_access(self):
        for kp in (0, 49, 51, 101, 500, float("nan"), True):
            profile = copy.deepcopy(PROFILE)
            profile["joints"][0]["kp"] = kp
            with self.subTest(kp=kp), self.assertRaises(ValueError):
                self.make_transport(profile)
        self.assertEqual(self.events, [])

    def test_prepare_must_observe_reset_before_config_or_target_writes(self):
        self.transport.open()
        self.bus.mode = 2  # Software inactivity does not establish physical RESET.

        def check_before_ack():
            self.assertFalse(
                any(e[0] in ("query", "write_f32", "write_u32", "mit") for e in self.events)
            )
            return self.bus.packet(status(0))

        self.bus.control_hook = lambda kind: [check_before_ack]
        self.transport.prepare_axis("joint1", 0.125)
        self.assertEqual(
            [e for e in self.events if e[0] in ("enable", "disable")], [("disable", 1)]
        )

    def test_failed_initial_reset_ack_cannot_reach_parameter_or_hold_writes(self):
        for response in (status(2), status(1), status(3)):
            transport = self.make_transport()
            transport.open()
            self.events.clear()
            self.bus.control_hook = lambda kind: [self.bus.packet(response)]
            with self.assertRaises(UnconfirmedOperation):
                transport.prepare_axis("joint1", 0.125)
            self.assertFalse(
                any(e[0] in ("query", "write_f32", "write_u32", "mit") for e in self.events)
            )
            transport.close()

    def test_cap_never_increases_existing_lower_cap(self):
        self.transport.open()
        self.bus.values[0x700B] = 0.125
        self.transport.prepare_axis("joint1", 0.125)
        self.assertEqual(self.transport.guard_readback["torque_cap_nm"], 0.125)

    def test_parameter_write_failure_is_preserved_and_stops_setup(self):
        self.transport.open()
        original = OSError("torque write failed")
        self.bus.errors["write_f32"] = original
        with self.assertRaises(OSError) as caught:
            self.transport.prepare_axis("joint1", 0.125)
        self.assertIs(caught.exception, original)
        self.assertFalse(any(e[0] in ("write_u32", "mit") for e in self.events))
        with self.assertRaises(SessionFailed):
            self.transport.read_positions()

    def test_wrong_mode_does_not_write_or_change_mode(self):
        self.transport.open()
        self.bus.values[0x7005] = 1
        with self.assertRaisesRegex(TransportError, "MIT"):
            self.transport.prepare_axis("joint1", 0.125)
        self.assertFalse(any(e[0] in ("write_f32", "write_u32", "mit") for e in self.events))
        with self.assertRaises(SessionFailed):
            self.transport.read_positions()

    def test_unsupported_watchdog_readback_prevents_hold_and_enable(self):
        self.transport.open()
        self.bus.unsupported.add(0x7028)
        with self.assertRaises(TransportError):
            self.transport.prepare_axis("joint1", 0.125)
        self.assertFalse(any(e[0] == "mit" for e in self.events))
        with self.assertRaises(SessionFailed):
            self.transport.enable_axis("joint1")

    def test_guard_readback_mismatch_or_changed_pose_prevents_hold(self):
        for fault in ("write", "pose"):
            with self.subTest(fault=fault):
                transport = self.make_transport()
                transport.open()
                self.events.clear()
                self.bus.writes_take_effect = fault != "write"
                self.bus.values[0x700B] = 1.0
                self.bus.positions[0] = 0.5 if fault == "pose" else 0.125
                with self.assertRaises(TransportError):
                    transport.prepare_axis("joint1", 0.125)
                self.assertFalse(any(e[0] == "mit" for e in self.events))
                transport.close()

    def test_invalid_axes_and_explicit_targets_never_write(self):
        self.transport.open()
        for method, args in (
            ("prepare_axis", (0.125,)),
            ("send_axis", (0.125,)),
            ("enable_axis", ()),
            ("disable_axis", ()),
        ):
            for axis in ("joint2", "joint6", "gripper", "joint7"):
                with self.assertRaises(ValueError):
                    getattr(self.transport, method)(axis, *args)
        for value in (math.nan, math.inf, True, PROFILE["joints"][0]["upper"] + 1e-8):
            with self.assertRaises(ValueError):
                self.transport.send_axis("joint1", value)
        self.assertFalse(
            any(e[0] in ("mit", "disable", "write_f32", "write_u32") for e in self.events)
        )

    def test_send_uses_reviewed_gains_and_preserves_original_error(self):
        self.transport.open()
        self.transport.prepare_axis("joint1", 0.125)
        self.transport.send_axis("joint1", 0.126)
        self.assertEqual(self.events[-1], ("mit", 0.126, 0.0, 50.0, 3.0, 0.0))
        original = OSError("injected write error")
        self.motor.errors["mit"] = original
        with self.assertRaises(OSError) as caught:
            self.transport.send_axis("joint1", 0.126)
        self.assertIs(caught.exception, original)
        with self.assertRaises(SessionFailed):
            self.transport.send_axis("joint1", 0.126)

    def test_enable_and_disable_observe_mode_but_never_claim_torque_confirmation(self):
        self.transport.open()
        self.transport.prepare_axis("joint1", 0.125)
        self.assertIsNone(self.transport.enable_axis("joint1"))
        outcome = self.transport.last_outcome
        self.assertTrue(outcome.attempted)
        self.assertTrue(outcome.transport_submitted)
        self.assertTrue(outcome.mode_acknowledged)
        self.assertEqual((outcome.mode_observed, outcome.fault_bits), (2, 0))
        self.assertFalse(outcome.physical_state_confirmed)
        self.assertEqual(
            FRAME.unpack(self.bus.sent[-1]), (EFF | (3 << 24) | (0xFD << 8) | 1, 8, bytes(8))
        )
        before = len(self.bus.sent)
        with self.assertRaisesRegex(TransportError, "already enabled"):
            self.transport.enable_axis("joint1")
        self.assertEqual(len(self.bus.sent), before)
        self.assertIsNone(self.transport.disable_axis("joint1"))
        outcome = self.transport.last_outcome
        self.assertTrue(outcome.mode_acknowledged)
        self.assertEqual(outcome.mode_observed, 0)
        self.assertFalse(outcome.physical_state_confirmed)
        self.assertEqual(
            FRAME.unpack(self.bus.sent[-1]), (EFF | (4 << 24) | (0xFD << 8) | 1, 8, bytes(8))
        )
        with self.assertRaisesRegex(TransportError, "preparation required"):
            self.transport.enable_axis("joint1")

    def test_enable_requires_preparation_without_any_command(self):
        self.transport.open()
        with self.assertRaisesRegex(TransportError, "preparation required"):
            self.transport.enable_axis("joint1")
        self.assertEqual(self.bus.sent, [])

    def test_disable_reset_can_retain_fault_bits_without_claiming_torque_off(self):
        self.transport.open()
        self.bus.fault_bits = 33
        self.transport.disable_axis("joint1")
        outcome = self.transport.last_outcome
        self.assertTrue(outcome.mode_acknowledged)
        self.assertEqual((outcome.mode_observed, outcome.fault_bits), (0, 33))
        self.assertFalse(outcome.physical_state_confirmed)
        with self.assertRaises(SessionFailed):
            self.transport.prepare_axis("joint1", 0.125)

    def test_failed_enable_preserves_primary_when_emergency_disable_also_fails(self):
        self.transport.open()
        self.transport.prepare_axis("joint1", 0.125)
        self.bus.control_hook = lambda kind: [self.bus.packet(status(1))]
        with self.assertRaises(UnconfirmedOperation) as caught:
            self.transport.enable_axis("joint1")
        primary = caught.exception
        with self.assertRaises(UnconfirmedOperation):
            self.transport.disable_axis("joint1")
        self.assertIs(self.transport._failure, primary)
        self.assertIs(self.transport._reader.failure, primary)
        self.assertEqual(primary.outcome.operation, "enable")
        self.assertEqual(self.transport.last_outcome.operation, "disable")

    def test_delayed_vendor_mit_return_latches_even_if_submission_completed(self):
        self.transport.open()
        self.transport.prepare_axis("joint1", 0.125)
        original = self.motor.send_mit

        def slow(*args):
            original(*args)
            self.clock.now += 0.051

        self.motor.send_mit = slow
        with self.assertRaisesRegex(TimeoutError, "may have been sent"):
            self.transport.send_axis("joint1", 0.126)
        with self.assertRaises(SessionFailed):
            self.transport.send_axis("joint1", 0.126)

    def test_disable_exception_keeps_unknown_submission_and_original_error(self):
        self.transport.open()
        original = OSError("disable socket send failed")
        self.bus.errors["disable"] = original
        with self.assertRaises(OSError) as caught:
            self.transport.disable_axis("joint1")
        self.assertIs(caught.exception, original)
        self.assertIsNone(self.transport.last_outcome.transport_submitted)
        self.assertFalse(self.transport.last_outcome.physical_state_confirmed)

    def test_reader_failure_permanently_inhibits_targets_but_allows_disable_attempt(self):
        self.transport.open()
        self.bus.response_hook = lambda *args: []
        with self.assertRaises(TimeoutError):
            self.transport.read_positions()
        with self.assertRaises(SessionFailed):
            self.transport.prepare_axis("joint1", 0.125)
        primary = self.transport._reader.failure
        self.transport.disable_axis("joint1")
        self.assertEqual(len(self.bus.sent), 2)
        self.assertIs(self.transport._reader.failure, primary)
        with self.assertRaises(SessionFailed):
            self.transport.read_positions()

    def test_partial_open_registration_failure_closes_resources(self):
        original = OSError("registration failed")
        self.controller.add_error = original
        with self.assertRaises(OSError) as caught:
            self.transport.open()
        self.assertIs(caught.exception, original)
        self.assertEqual(self.events[-3:], [("controller.close",), ("bus.close",), ("lock.close",)])

    def test_partial_socket_setup_failure_never_constructs_vendor(self):
        self.bus.option_error = OSError("timestamps unsupported")
        with self.assertRaises(OSError):
            self.transport.open()
        self.assertFalse(any(e[0] == "controller" for e in self.events))
        self.assertEqual(self.events[-2:], [("bus.close",), ("lock.close",)])

    def test_open_failure_preserves_primary_and_cleanup_and_lock(self):
        original, cleanup = OSError("registration failed"), OSError("bus close failed")
        self.controller.add_error, self.bus.close_error = original, cleanup
        with self.assertRaises(OpenError) as caught:
            self.transport.open()
        self.assertIs(caught.exception.primary, original)
        self.assertIs(caught.exception.cleanup.failures[0][1], cleanup)
        self.assertNotIn(("lock.close",), self.events)

    def test_close_attempts_all_resources_preserves_failures_and_can_retry(self):
        self.transport.open()
        errors = [OSError("motor close"), OSError("controller close"), OSError("socket close")]
        self.motor.close_error, self.controller.close_error, self.bus.close_error = errors
        self.events.clear()
        with self.assertRaises(CleanupError) as caught:
            self.transport.close()
        self.assertEqual([error for _, error in caught.exception.failures], errors)
        self.assertEqual(self.events, [("motor.close",), ("controller.close",), ("bus.close",)])
        self.motor.close_error = self.controller.close_error = self.bus.close_error = None
        self.transport.close()
        self.assertEqual(self.events[-1], ("lock.close",))
        with self.assertRaises(SessionFailed):
            self.transport.read_positions()
        with self.assertRaises(SessionFailed):
            self.transport.open()

    def test_core_integration_can_activate_with_mode_ack_and_close_without_torque_claim(self):
        self.transport.open()
        bounds = [(joint["lower"], joint["upper"]) for joint in PROFILE["joints"]]
        core = CommissioningCore(
            self.transport, bounds, max_feedback_age=0.15, persisted_stopped=False, clock=self.clock
        )
        core.activate(self.clock())
        self.assertEqual(core.state, State.ACTIVE)
        report = core.close()
        self.assertTrue(report.disable_acknowledged)
        self.assertFalse(report.torque_off_confirmed)

    def test_fake_end_to_end_first_goal_uses_strict_reads_and_only_joint1_control(self):
        self.transport.open()
        self.motor.follow_target = True
        bounds = [(joint["lower"], joint["upper"]) for joint in PROFILE["joints"]]
        core = CommissioningCore(
            self.transport, bounds, max_feedback_age=0.15, persisted_stopped=False, clock=self.clock
        )
        core.activate(self.clock())
        target = list(core.bounded_positions)
        target[0] += 0.005
        core.start_goal(target, 2.0, self.clock())
        for _ in range(30):
            self.clock.now += 0.06
            core.tick(self.clock())
            if core.goal_state == GoalState.SUCCEEDED:
                break
        self.assertEqual(core.goal_state, GoalState.SUCCEEDED)
        self.assertEqual(
            [e for e in self.events if e[0] in ("enable", "disable")],
            [("disable", 1), ("enable", 1)],
        )
        self.assertFalse(core.close().torque_off_confirmed)

    def test_core_retains_enable_fault_separately_from_failed_cleanup_ack(self):
        self.transport.open()
        bounds = [(joint["lower"], joint["upper"]) for joint in PROFILE["joints"]]
        core = CommissioningCore(
            self.transport, bounds, max_feedback_age=0.15, persisted_stopped=False, clock=self.clock
        )
        calls = []

        def responses(kind):
            calls.append(kind)
            # First disable prepares successfully; enable faults, then cleanup
            # receives RUN instead of RESET and must remain unacknowledged.
            frame = status(0) if len(calls) == 1 else status(2, 1 if kind == 3 else 0)
            return [self.bus.packet(frame)]

        self.bus.control_hook = responses
        with self.assertRaises(UnconfirmedOperation) as caught:
            core.activate(self.clock())
        self.assertIs(core.report.primary.error, caught.exception)
        self.assertFalse(core.report.disable_acknowledged)
        self.assertTrue(core.report.cleanup)
        self.assertEqual(core.state, State.STOPPED)
        self.bus.control_hook = lambda kind: [self.bus.packet(status(0))]
        core.close()


if __name__ == "__main__":
    unittest.main()
