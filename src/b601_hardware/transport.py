"""Single-owner, joint1-only transport for operator-run commissioning scripts.

Construction is hardware-free; open() takes the shared CAN lock and opens CAN.
Position/configuration reads use our socket, never MotorBridge's cached decoder.
One failed transaction permanently invalidates this instance: reconnecting or
retrying after a timeout requires a new, explicitly authorized session.

Explicit enable/disable use fresh type-2 RUN/RESET observations, not the vendor's
weak status ACK. These are protocol-mode acknowledgements, not physical torque
confirmation. Guard writes use bounded type 18 followed by strict readback.
Readback does not qualify physical watchdog/torque behavior. Socket deadlines
bound accepted replies, not Linux scheduling or MotorBridge's blocking MIT send.
The caller owns the durable stop latch and must honor it before prepare_axis().
Preparation deliberately disables only joint1 and requires fresh RESET status
before any guard write or HOLD staging; software INACTIVE is not motor RESET.
"""

from __future__ import annotations

import fcntl
import hashlib
import importlib.metadata
import importlib.util
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import json
import threading
import time
from dataclasses import dataclass, replace

from .core import JOINT1_KP_CHOICES, JOINT_NAMES, MEASURED_TOLERANCE, PositionSample


__all__ = [
    "Joint1Transport", "StrictParameterReader", "decode_parameter_reply", "decode_status_reply",
    "TransportError", "SessionFailed", "UnconfirmedOperation", "CommandOutcome",
    "CleanupError", "OpenError",
]

_FRAME = struct.Struct("=IB3x8s")
_EFF, _RTR, _ERR = 0x80000000, 0x40000000, 0x20000000
_HOST = 0xFD
_READ = 17
_PARAMETERS = {0x7005: "i8", 0x700B: "f32", 0x7019: "f32", 0x7028: "u32"}
_FORMATS = {"i8": "<b", "f32": "<f", "u32": "<I"}
# Linux UAPI SO_TIMESTAMPNS_NEW: two signed 64-bit seconds/nanoseconds fields.
_TIMESTAMPNS = 64
_TIMESPEC = struct.Struct("=qq")
_LOCK_PATH = Path("/run/lock/astra-b601-can1.lock")
_NATIVE_SHA256 = "eccf92c67a9782098a9523dc9824cb7ccc43baf0eba43ba305c2c10cd4ef9448"
_WRAPPER_SHA256 = {
    "core.py": "2dd908f992db2ce4641af6bdfdcd049d335211fc458c267875b908967a70dcf4",
    "abi.py": "63b70bcc0a52a40b01fe5420da1d50c502b6d025e9e3ebae689c88109120193f",
}


class TransportError(RuntimeError):
    pass


class SessionFailed(TransportError):
    pass


@dataclass(frozen=True)
class CommandOutcome:
    operation: str
    attempted: bool = False
    transport_submitted: bool | None = False
    mode_observed: int | None = None
    fault_bits: int | None = None
    mode_acknowledged: bool = False
    physical_state_confirmed: bool = False
    detail: str = "Not submitted"


class UnconfirmedOperation(TransportError):
    def __init__(self, outcome: CommandOutcome):
        self.outcome = outcome
        super().__init__(outcome.detail)


class CleanupError(TransportError):
    def __init__(self, failures):
        self.failures = tuple(failures)
        super().__init__("; ".join(f"{name}: {error}" for name, error in self.failures))


class OpenError(TransportError):
    def __init__(self, primary, cleanup):
        self.primary, self.cleanup = primary, cleanup
        super().__init__(f"Open failed: {primary}; cleanup also failed: {cleanup}")


def _number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("expected a finite number")
    return float(value)


def _parameter(motor_id, parameter, dtype):
    if type(motor_id) is not int or not 1 <= motor_id <= 6:
        raise ValueError("only commissioned arm motor IDs 1..6 may be queried")
    if type(parameter) is not int or _PARAMETERS.get(parameter) != dtype:
        raise ValueError("unsupported parameter or incorrect dtype")


def _reply_parts(frame, motor_id, communication_type, flags):
    if flags & (socket.MSG_DONTROUTE | socket.MSG_TRUNC | socket.MSG_CTRUNC):
        return None
    if len(frame) != _FRAME.size:
        return None
    identifier, dlc, data = _FRAME.unpack(frame)
    if (not identifier & _EFF or identifier & (_RTR | _ERR) or dlc != 8
            or (identifier >> 24) & 0x1F != communication_type
            or identifier & 0xFF != _HOST
            or (identifier >> 8) & 0xFF != motor_id):
        return None
    return identifier, data


def decode_parameter_reply(frame, motor_id, parameter, dtype, *, flags=0):
    """Return a matched value, None for unrelated traffic, or raise on bad data.

    The parameter error byte is CAN identifier bits 16..23, as in the reviewed
    connection checker. Kernel timestamps are checked by StrictParameterReader.
    """
    _parameter(motor_id, parameter, dtype)
    parts = _reply_parts(frame, motor_id, _READ, flags)
    if parts is None:
        return None
    identifier, data = parts
    if struct.unpack_from("<H", data)[0] != parameter:
        return None
    if (identifier >> 16) & 0xFF:
        raise TransportError(f"motor {motor_id} returned a parameter-read error for 0x{parameter:04x}")
    value = struct.unpack_from(_FORMATS[dtype], data, 4)[0]
    if dtype == "f32" and not math.isfinite(value):
        raise TransportError("non-finite motor feedback")
    return value


def decode_status_reply(frame, motor_id=1, *, flags=0):
    """Return (mode_state, fault_bits) from a matched private-protocol type 2.

    Layout: MotorBridge v0.5.6 motor_vendors/robstride/src/protocol.rs:234-244;
    also RobStride/Python_Sample robstride_dynamics/bus.py:294-305.
    Values: docs.rs/robstride/0.3.6/src/robstride/actuator_types.rs.html:137-142
    defines Reset=0, Calibration=1, Run=2. The manufacturer's bus.py:395-409
    sends zero-payload types 3/4 then receives status; :501-503 names disabled
    state Reset. A mode observation does not prove electrical torque removal.
    """
    if type(motor_id) is not int or motor_id != 1:
        raise ValueError("mode acknowledgements are restricted to motor 1")
    parts = _reply_parts(frame, motor_id, 2, flags)
    if parts is None:
        return None
    identifier, data = parts
    # A firmware-version reply also uses type 2; it is not motor status.
    if data[:3] == b"\x00\xc4\x56":
        return None
    return (identifier >> 22) & 3, (identifier >> 16) & 63


class StrictParameterReader:
    """Owns serialized strict reads and joint1 guard/control transactions.

    Timeout/error poisons the session; only explicit emergency type 4 can still
    be sent. Kernel timestamps exclude pre-request frames. RobStride has no
    transaction nonce: duplicated old replies arriving after a new request are
    indistinguishable on the wire. Sole bus ownership is a prerequisite.
    """

    def __init__(self, bus, *, timeout=0.05, clock=time.monotonic, wall_ns=time.time_ns):
        self.timeout = _number(timeout)
        if not 0 < self.timeout <= 0.05:
            raise ValueError("request timeout must be in (0, 0.05] seconds")
        self._bus, self._clock, self._wall_ns = bus, clock, wall_ns
        self._lock = threading.Lock()
        self.failure = None
        self.last_outcome = None
        self._enabled_since_ns = None

    def _remaining(self, deadline):
        remaining = deadline - self._clock()
        if remaining <= 0:
            raise TimeoutError("strict CAN request deadline expired; session invalidated")
        return remaining

    def read(self, motor_id, parameter, dtype):
        """Return (value, conservative monotonic request-start timestamp)."""
        _parameter(motor_id, parameter, dtype)
        return self._exchange(_READ, motor_id, struct.pack("<H", parameter) + bytes(6),
                              parameter=parameter, dtype=dtype)

    def write_guard(self, parameter, value):
        """Only the two reviewed guard writes; acknowledgement is strict readback."""
        if parameter == 0x700B:
            value = _number(value)
            if not 0 < value <= 0.3:
                raise ValueError("torque cap must be positive and at most 0.3 Nm")
            raw = struct.pack("<f", value)
        elif parameter == 0x7028 and type(value) is int and value == 5000:
            raw = struct.pack("<I", value)
        else:
            raise ValueError("only reviewed torque/watchdog guard writes are allowed")
        self._exchange(18, 1, struct.pack("<H", parameter) + bytes(2) + raw)

    def set_enabled(self, enabled):
        if type(enabled) is not bool:
            raise ValueError("enabled must be a boolean")
        # Never set type-4 byte0=1: that would clear motor faults.
        self._exchange(3 if enabled else 4, 1, bytes(8),
                       operation="enable" if enabled else "disable", emergency=not enabled)

    def _timestamp(self, ancillary):
        timestamps = [data for level, kind, data in ancillary
                      if level == socket.SOL_SOCKET and kind == _TIMESTAMPNS]
        if len(timestamps) != 1 or len(timestamps[0]) != _TIMESPEC.size:
            raise TransportError("matched reply lacks an intact kernel receive timestamp")
        seconds, nanos = _TIMESPEC.unpack(timestamps[0])
        if seconds < 0 or not 0 <= nanos < 1_000_000_000:
            raise TransportError("invalid kernel receive timestamp")
        return seconds * 1_000_000_000 + nanos

    def _fresh(self, ancillary, started, wall_start):
        received_ns = self._timestamp(ancillary)
        wall_now, now = self._wall_ns(), self._clock()
        if now < started or abs((wall_now - wall_start) / 1e9 - (now - started)) > 0.005:
            raise TransportError("realtime/monotonic clocks diverged during CAN request")
        if received_ns > wall_now:
            raise TransportError("kernel receive timestamp is in the future")
        return received_ns >= wall_start

    def _watch_running_status(self, frame, ancillary, flags):
        if self._enabled_since_ns is None:
            return
        status = decode_status_reply(frame, flags=flags)
        if status is None:
            return
        received_ns = self._timestamp(ancillary)
        if received_ns > self._wall_ns():
            raise TransportError("motor status receive timestamp is in the future")
        if received_ns >= self._enabled_since_ns:
            mode, faults = status
            if mode != 2 or faults:
                raise TransportError(f"Active joint1 reported mode={mode}, fault bits=0x{faults:02x}")

    def _exchange(self, kind, motor_id, payload, *, parameter=None, dtype=None,
                  operation=None, emergency=False):
        with self._lock:
            if self.failure is not None and not emergency:
                raise SessionFailed("previous CAN transaction failed; no retries in this session") from self.failure
            if emergency and (kind != 4 or motor_id != 1 or payload != bytes(8)):
                raise ValueError("only fault-retaining motor1 disable may bypass a failed session")
            if operation:
                self.last_outcome = CommandOutcome(operation)
            try:
                deadline = self._clock() + self.timeout
                # Drain before the first request and every later request, bounded
                # by the same transaction deadline. Never turn queued data fresh.
                self._bus.setblocking(False)
                while True:
                    self._remaining(deadline)
                    try:
                        frame, ancillary, flags, _ = self._bus.recvmsg(
                            _FRAME.size, socket.CMSG_SPACE(_TIMESPEC.size), socket.MSG_DONTWAIT)
                        if not emergency:
                            self._watch_running_status(frame, ancillary, flags)
                    except BlockingIOError:
                        break
                started = self._clock()
                wall_start = self._wall_ns()
                identifier = _EFF | (kind << 24) | (_HOST << 8) | motor_id
                request = _FRAME.pack(identifier, 8, payload)
                self._bus.settimeout(self._remaining(deadline))
                if operation:
                    self.last_outcome = replace(self.last_outcome, attempted=True, transport_submitted=None)
                if self._bus.send(request) != len(request):
                    raise TransportError("incomplete CAN request submission")
                if operation:
                    self.last_outcome = replace(self.last_outcome, transport_submitted=True,
                                                detail="Submitted; awaiting fresh mode status")
                self._remaining(deadline)
                if kind == 18:
                    return None, started
                while True:
                    self._bus.settimeout(self._remaining(deadline))
                    frame, ancillary, flags, _ = self._bus.recvmsg(
                        _FRAME.size, socket.CMSG_SPACE(_TIMESPEC.size))
                    self._remaining(deadline)  # Late userspace delivery is also rejected.
                    if not emergency:
                        self._watch_running_status(frame, ancillary, flags)
                    parts = _reply_parts(frame, motor_id, _READ if kind == _READ else 2, flags)
                    if parts is None:
                        continue
                    if parameter is not None and struct.unpack_from("<H", parts[1])[0] != parameter:
                        continue
                    if not self._fresh(ancillary, started, wall_start):
                        continue
                    if kind == _READ:
                        value = decode_parameter_reply(frame, motor_id, parameter, dtype, flags=flags)
                    else:
                        value = decode_status_reply(frame, motor_id, flags=flags)
                        if value is None:
                            continue
                        mode, faults = value
                        expected_mode = 2 if operation == "enable" else 0
                        self.last_outcome = replace(self.last_outcome, mode_observed=mode, fault_bits=faults,
                            detail=f"Fresh mode={mode}, faults=0x{faults:02x}; expected mode={expected_mode}")
                        if mode != expected_mode or (operation == "enable" and faults):
                            raise UnconfirmedOperation(self.last_outcome)
                        self.last_outcome = replace(self.last_outcome, mode_acknowledged=True,
                            detail=f"Fresh mode={mode} acknowledged; faults=0x{faults:02x}; physical torque unconfirmed")
                        self._enabled_since_ns = wall_start if operation == "enable" else None
                        if faults and self.failure is None:
                            self.failure = TransportError(f"Motor retains hardware fault bits 0x{faults:02x}")
                    self._remaining(deadline)
                    return value, started
            except BaseException as error:
                if self.failure is None:
                    self.failure = error
                if operation:
                    self.last_outcome = replace(self.last_outcome, mode_acknowledged=False,
                                                detail=f"{operation} failed: {error}")
                raise


def _interface_metadata(channel):
    """Read sysfs/ip metadata only; never configure the interface."""
    net = Path("/sys/class/net") / channel
    device = (net / "device").resolve(strict=True)
    driver = (device / "driver").resolve(strict=True).name
    usb = next((p for p in (device, *device.parents)
                if (p / "idVendor").is_file() and (p / "idProduct").is_file()), None)
    if usb is None:
        raise TransportError("CAN interface has no verifiable USB identity")
    result = subprocess.run(["ip", "-j", "-details", "link", "show", "dev", channel],
                            check=True, capture_output=True, text=True, timeout=2)
    links = json.loads(result.stdout)
    if len(links) != 1 or links[0].get("ifname") != channel:
        raise TransportError("ambiguous CAN interface metadata")
    link = links[0]
    info = link["linkinfo"]["info_data"]
    return {"channel": channel, "type": int((net / "type").read_text()), "driver": driver,
            "usb_vendor": (usb / "idVendor").read_text().strip(),
            "usb_product": (usb / "idProduct").read_text().strip(),
            "flags": link["flags"], "bitrate": info.get("bittiming", {}).get("bitrate"),
            "ctrlmode": info.get("ctrlmode", []), "can_state": info.get("state")}


def _ownership_lock(path):
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o660)
    stream = os.fdopen(fd, "a+")
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BaseException:
        stream.close()
        raise
    return stream


def _verify_motorbridge():
    """Verify the reviewed installed implementation before opening any CAN socket."""
    distribution = importlib.metadata.distribution("motorbridge")
    if distribution.version != "0.5.6":
        raise TransportError("commissioning requires reviewed MotorBridge 0.5.6")
    package = Path(distribution.locate_file("motorbridge")).resolve(strict=True)
    spec = importlib.util.find_spec("motorbridge")
    if spec is None or spec.origin is None or Path(spec.origin).resolve() != package / "__init__.py":
        raise TransportError("MotorBridge import path differs from the reviewed installed distribution")
    native = (package / "lib/libmotor_abi.so").resolve(strict=True)
    override = os.environ.get("MOTORBRIDGE_LIB")
    if override is not None and (not override or Path(override).expanduser().resolve() != native):
        raise TransportError("unreviewed MOTORBRIDGE_LIB override")
    for path, expected in [(native, _NATIVE_SHA256),
                           *((package / name, digest) for name, digest in _WRAPPER_SHA256.items())]:
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise TransportError(f"unreviewed MotorBridge content: {path.name}")
    return native


def _controller(channel):
    native = _verify_motorbridge()
    from motorbridge import Controller  # Imported only during explicit open().
    from motorbridge.abi import get_abi
    # A different ABI may already have been loaded before an environment change.
    if Path(get_abi().lib._name).resolve() != native:
        raise TransportError("loaded MotorBridge ABI is not the reviewed native library")
    return Controller(channel=channel)


def _profile(profile):
    expected = {"channel": "can1", "usb_vendor": "0c72", "usb_product": "000c",
                "driver": "peak_usb", "host_id": _HOST, "active_joint": "joint1",
                "coordinate_frame": "raw_motor_radians", "watchdog_ticks": 5000}
    if any(profile.get(key) != value for key, value in expected.items()):
        raise ValueError("profile does not identify the reviewed can1 PCAN/RS joint1 configuration")
    cap = _number(profile["torque_cap_nm"])
    tolerance = MEASURED_TOLERANCE
    age = _number(profile["feedback_timeout_s"])
    if not 0 < cap <= 0.3 or not 0 < tolerance <= math.radians(0.03) or not 0 < age <= 0.15:
        raise ValueError("profile exceeds commissioning cap, tolerance or feedback age")
    joints = tuple(dict(joint) for joint in profile["joints"])
    if len(joints) != 6:
        raise ValueError("profile must contain exactly joint1..joint6")
    for i, joint in enumerate(joints, 1):
        if (joint.get("name") != JOINT_NAMES[i - 1] or type(joint.get("motor_id")) is not int
                or joint["motor_id"] != i or joint.get("model") != ("rs-06" if i <= 3 else "rs-00")
                or _number(joint["lower"]) >= _number(joint["upper"])):
            raise ValueError("invalid joint mapping/model/bounds")
    if _number(joints[0]["kp"]) not in JOINT1_KP_CHOICES or _number(joints[0]["kd"]) != 3:
        raise ValueError("joint1 requires kp=50 (baseline) or kp=100 (operator trial), with kd=3")
    return joints, cap, tolerance, age


class Joint1Transport:
    """Owns the shared lock, strict socket and one registered MotorBridge motor.

    The core owns motion policy and durable-stop integration belongs to its
    wrapper. This adapter owns identity, wire validation and write scope. Do not
    separately acquire the shared lock in the wrapper: open() owns it here.
    Dependency factories exist for hardware-free tests, not alternative drivers.
    """

    def __init__(self, profile, *, socket_factory=socket.socket, controller_factory=_controller,
                 metadata_reader=_interface_metadata, lock_factory=_ownership_lock,
                 clock=time.monotonic, wall_ns=time.time_ns, request_timeout=0.05):
        self._joints, self._cap, self._tolerance, self._age = _profile(profile)
        self._timeout = _number(request_timeout)
        if not 0 < self._timeout <= 0.05:
            raise ValueError("request timeout must be in (0, 0.05]")
        self._socket_factory, self._controller_factory = socket_factory, controller_factory
        self._metadata_reader, self._lock_factory = metadata_reader, lock_factory
        self._clock, self._wall_ns = clock, wall_ns
        self._mutex = threading.RLock()
        self._ownership = self._bus = self._controller = self._reader = None
        self._motors = {}
        self._used = self._opened = self._prepared = False
        self._enabled = False
        self._failure = None
        self.last_outcome = None
        self.guard_readback = None

    @property
    def joint1_kp(self):
        return float(self._joints[0]["kp"])

    def open(self):
        with self._mutex:
            if self._used:
                raise SessionFailed("transport instances are single-use")
            self._used = True
            try:
                self._ownership = self._lock_factory(_LOCK_PATH)
                info = self._metadata_reader("can1")
                expected = {"channel": "can1", "type": 280, "driver": "peak_usb",
                            "usb_vendor": "0c72", "usb_product": "000c", "bitrate": 1_000_000,
                            "can_state": "ERROR-ACTIVE"}
                if (any(info.get(k) != v for k, v in expected.items())
                        or "UP" not in info.get("flags", [])
                        or not isinstance(info.get("ctrlmode"), list)
                        or any(mode in info["ctrlmode"] for mode in ("LISTEN-ONLY", "LOOPBACK", "FD"))):
                    raise TransportError("requires verified PCAN can1, peak_usb, UP 1-Mbps normal classic CAN")
                if self._controller_factory is _controller:
                    _verify_motorbridge()
                self._bus = self._socket_factory(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self._bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
                self._bus.setsockopt(socket.SOL_SOCKET, _TIMESTAMPNS, 1)
                # Kernel filter reduces traffic; the full decoder still checks
                # motor, host, index, error byte, flags and DLC independently.
                # CAN_ERR_FLAG in can_mask selects Linux's error-only filter list.
                self._bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                                      b"".join(struct.pack("=II", _EFF | (kind << 24) | _HOST,
                                                            _EFF | _RTR | 0x1F0000FF)
                                               for kind in (_READ, 2)))
                self._bus.bind(("can1",))
                self._reader = StrictParameterReader(self._bus, timeout=self._timeout,
                                                     clock=self._clock, wall_ns=self._wall_ns)
                self._controller = self._controller_factory("can1")
                # Other five motors are read through the strict socket only.
                self._motors["joint1"] = self._controller.add_robstride_motor(1, _HOST, "rs-06")
                self._opened = True
                return self
            except BaseException as primary:
                self._failure = primary
                try:
                    self.close()
                except CleanupError as cleanup:
                    raise OpenError(primary, cleanup) from primary
                raise

    def _ready(self):
        if not self._opened:
            raise SessionFailed("transport is not open")
        if self._failure is not None:
            raise SessionFailed("transport session failed; only scoped disable/close remain allowed") from self._failure
        if self._reader.failure is not None:
            raise SessionFailed("strict socket session failed") from self._reader.failure

    def _read(self, motor_id, parameter, dtype):
        self._ready()
        try:
            return self._reader.read(motor_id, parameter, dtype)
        except BaseException as error:
            self._failure = self._failure or error
            raise

    def read_positions(self):
        with self._mutex:
            values = [self._read(joint["motor_id"], 0x7019, "f32") for joint in self._joints]
            return PositionSample(tuple(v for v, _ in values),
                                  joint_timestamps=tuple(t for _, t in values))

    def _axis(self, axis):
        if axis != "joint1":
            raise ValueError("writes to axes other than joint1 are forbidden")

    def _target(self, position):
        value = _number(position)
        if not self._joints[0]["lower"] <= value <= self._joints[0]["upper"]:
            raise ValueError("joint1 target exceeds strict raw-radian bounds")
        return value

    def prepare_axis(self, axis, hold_position):
        """Explicit scoped disable/RESET, existing MIT verification, guards, HOLD.

        No mode change, zero, homing, fault clear or other-axis write. Disabling
        joint1 is deliberate activation preparation, never an open/read action.
        """
        with self._mutex:
            self._axis(axis)
            hold = self._target(hold_position)
            self._ready()
            if self._prepared:
                raise TransportError("joint1 was already prepared in this session")
            try:
                self.disable_axis(axis)
                mode, _ = self._read(1, 0x7005, "i8")
                if mode != 0:
                    raise TransportError("existing MIT run_mode=0 required; no mode change performed")
                current, _ = self._read(1, 0x700B, "f32")
                if current <= 0:
                    raise TransportError("positive finite existing torque cap required")
                requested = min(self._cap, current)
                cap = struct.unpack("<f", struct.pack("<f", requested))[0]
                if cap > requested:  # Do not round a conservative cap upwards.
                    bits = struct.unpack("<I", struct.pack("<f", cap))[0]
                    cap = struct.unpack("<f", struct.pack("<I", bits - 1))[0]
                if cap <= 0:
                    raise TransportError("torque cap cannot be represented conservatively as f32")
                self._reader.write_guard(0x700B, cap)
                self._reader.write_guard(0x7028, 5000)
                measured_cap, _ = self._read(1, 0x700B, "f32")
                ticks, _ = self._read(1, 0x7028, "u32")
                if measured_cap != cap or ticks != 5000:
                    raise TransportError("torque/watchdog readback mismatch; activation inhibited")
                position, sampled = self._read(1, 0x7019, "f32")
                if abs(position - hold) > self._tolerance or self._clock() - sampled > self._age:
                    raise TransportError("fresh pose-aligned HOLD required after guard preparation")
                self._send_mit(hold)
                self.guard_readback = {"run_mode": mode, "torque_cap_nm": cap, "watchdog_ticks": ticks,
                                       "commanded_kp": self.joint1_kp, "commanded_kd": 3.0,
                                       "physical_watchdog_qualified": False}
                self._prepared = True
            except BaseException as error:
                self._failure = self._failure or error
                raise

    def _send_mit(self, position):
        started = self._clock()
        self._motors["joint1"].send_mit(position, 0.0, self.joint1_kp, 3.0, 0.0)
        if self._clock() - started > self._timeout:
            raise TimeoutError("MotorBridge MIT submission exceeded the deadline; result may have been sent")

    def send_axis(self, axis, position):
        with self._mutex:
            self._axis(axis)
            target = self._target(position)
            self._ready()
            if not self._prepared:
                raise TransportError("joint1 preparation required before target submission")
            try:
                self._send_mit(target)
            except BaseException as error:
                self._failure = self._failure or error
                raise

    def enable_axis(self, axis):
        with self._mutex:
            self._axis(axis)
            self._ready()
            if not self._prepared:
                raise TransportError("joint1 preparation required")
            if self._enabled:
                raise TransportError("joint1 was already enabled; no enable replay")
            self._mode_command(True)
            self._enabled = True

    def disable_axis(self, axis):
        with self._mutex:
            self._axis(axis)
            if not self._opened or axis not in self._motors:
                raise SessionFailed("no open joint1 handle for disable")
            self._mode_command(False)
            self._prepared = self._enabled = False

    def _mode_command(self, enabled):
        try:
            self._reader.set_enabled(enabled)
        except BaseException as error:
            self._failure = self._failure or error
            raise
        finally:
            self.last_outcome = self._reader.last_outcome
        # RESET with retained faults is acknowledged but cannot authorize a new
        # activation. Successful emergency cleanup never clears the primary fault.
        self._failure = self._failure or self._reader.failure

    def close(self):
        """Attempt every resource close; never shutdown/disable implicitly.

        Failed resources and the ownership lock are retained for explicit cleanup
        retry. CleanupError keeps every error; a failing close cannot hand the
        shared CAN ownership lock to another controller prematurely.
        """
        with self._mutex:
            self._used, self._opened = True, False
            failures = []
            for axis, motor in tuple(self._motors.items()):
                try:
                    motor.close()
                except BaseException as error:
                    failures.append((f"{axis}.close", error))
                else:
                    del self._motors[axis]
            for name in ("_controller", "_bus"):
                resource = getattr(self, name)
                if resource is not None:
                    try:
                        resource.close()
                    except BaseException as error:
                        failures.append((name[1:] + ".close", error))
                    else:
                        setattr(self, name, None)
            if not failures and self._ownership is not None:
                try:
                    self._ownership.close()
                except BaseException as error:
                    failures.append(("ownership.close", error))
                else:
                    self._ownership = None
            if failures:
                raise CleanupError(failures)
