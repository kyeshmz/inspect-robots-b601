"""CAN bus identity and exclusive ownership shared by every B601 session.

Everything that varies between installations comes from the robot profile:

- ``channel``: the SocketCAN interface, for example ``can0``.
- ``driver``, ``usb_vendor``, ``usb_product`` (optional): pin the adapter. When
  present, the interface must match them exactly; when absent, any adapter is
  accepted as long as the bus itself is up at 1 Mbit/s in normal classic mode.
  BusClaim checks both every time a session takes the bus.
- ``can_lock`` (optional): the lock file every B601 process on this bus shares.
  Defaults to ``/run/lock/b601-<channel>.lock``.
- ``native_owner_lock`` (optional): another controller's lock file on the same
  bus. It is held too, so the two controllers can never drive motors together.

Nothing here configures the interface; the reads are sysfs and ``ip -details``.
"""

from __future__ import annotations

import fcntl
import json
import os
import subprocess
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import IO, Any, Protocol

from . import PathLike, Profile

__all__ = [
    "BITRATE",
    "BusClaim",
    "channel_name",
    "claim_native_bus",
    "interface_metadata",
    "interface_problems",
    "ownership_lock",
    "shared_lock_path",
]

BITRATE = 1_000_000  # RobStride motors only speak 1 Mbit/s classic CAN.
_ARPHRD_CAN = 280
_ADAPTER_PINS = ("driver", "usb_vendor", "usb_product")


class Closable(Protocol):
    def close(self) -> None: ...


def channel_name(profile: Mapping[str, Any]) -> str:
    """The profile's SocketCAN interface name, validated so it is safe in a path."""
    name = profile.get("channel")
    if (
        not isinstance(name, str)
        or not 0 < len(name) <= 15
        or name in (".", "..")
        or any(char in name for char in "/\0")
        or any(char.isspace() for char in name)
    ):
        raise ValueError("profile channel must be a network interface name such as 'can0'")
    return name


def shared_lock_path(profile: Mapping[str, Any]) -> Path:
    """The bus's shared lock file: can_lock, else /run/lock (or /tmp)/b601-<channel>.lock."""
    configured = profile.get("can_lock")
    if configured is not None:
        return Path(configured)
    directory = Path("/run/lock")
    if not os.access(directory, os.W_OK):
        directory = Path("/tmp")
    return directory / f"b601-{channel_name(profile)}.lock"


def ownership_lock(path: PathLike) -> IO[str]:
    """Take the exclusive lock at path without waiting; closing the returned stream releases it."""
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o660)
    stream = os.fdopen(fd, "a+")
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        stream.close()
        raise BlockingIOError(
            error.errno, f"another B601 session holds the CAN lock {path}"
        ) from None
    except BaseException:
        stream.close()
        raise
    return stream


class _NoNativeOwner:
    def close(self) -> None:
        pass


def claim_native_bus(path: PathLike | None) -> Closable:
    """Hold another controller's owner lock, if the profile names one."""
    if path is None:
        return _NoNativeOwner()
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
    stream = os.fdopen(fd, "a+")
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BaseException as error:
        stream.close()
        raise RuntimeError(
            f"CAN is owned by another controller ({path}); no handoff is performed"
        ) from error
    return stream


def interface_metadata(channel: str) -> dict[str, Any]:
    """Read sysfs/ip metadata only; never configure the interface."""
    net = Path("/sys/class/net") / channel
    if not net.exists():
        raise FileNotFoundError(
            f"CAN interface {channel} does not exist; check `ip link` and the profile channel"
        )
    device = (net / "device").resolve(strict=True)
    driver = (device / "driver").resolve(strict=True).name
    usb = next(
        (
            p
            for p in (device, *device.parents)
            if (p / "idVendor").is_file() and (p / "idProduct").is_file()
        ),
        None,
    )
    result = subprocess.run(
        ["ip", "-j", "-details", "link", "show", "dev", channel],
        check=True,
        capture_output=True,
        text=True,
        timeout=2,
    )
    links = json.loads(result.stdout)
    if len(links) != 1 or links[0].get("ifname") != channel:
        raise RuntimeError("ambiguous CAN interface metadata")
    link = links[0]
    info = link.get("linkinfo", {}).get("info_data", {})
    return {
        "channel": channel,
        "type": int((net / "type").read_text()),
        "driver": driver,
        "usb_vendor": None if usb is None else (usb / "idVendor").read_text().strip(),
        "usb_product": None if usb is None else (usb / "idProduct").read_text().strip(),
        "flags": link.get("flags", []),
        "bitrate": info.get("bittiming", {}).get("bitrate"),
        "ctrlmode": info.get("ctrlmode", []),
        "can_state": info.get("state"),
    }


def interface_problems(info: Mapping[str, Any], profile: Mapping[str, Any]) -> list[str]:
    """Everything that stops ``info`` from being the profile's usable bus; empty when ready."""
    channel = channel_name(profile)
    problems = []
    if info.get("channel") != channel:
        problems.append(f"metadata is for {info.get('channel')!r}, not {channel!r}")
    if info.get("type") != _ARPHRD_CAN:
        problems.append("not a CAN interface")
    if "UP" not in (info.get("flags") or []):
        problems.append(
            f"interface is down (sudo ip link set {channel} up type can bitrate {BITRATE})"
        )
    if info.get("bitrate") != BITRATE:
        problems.append(f"bitrate is {info.get('bitrate')}, motors need {BITRATE}")
    if info.get("can_state") != "ERROR-ACTIVE":
        problems.append(f"controller state is {info.get('can_state')}, expected ERROR-ACTIVE")
    ctrlmode = info.get("ctrlmode")
    if not isinstance(ctrlmode, list):
        problems.append("controller mode is unreadable")
    else:
        unsupported = [mode for mode in ("LISTEN-ONLY", "LOOPBACK", "FD") if mode in ctrlmode]
        if unsupported:
            problems.append(f"{', '.join(unsupported)} enabled; normal classic CAN is required")
    for key in _ADAPTER_PINS:
        if key in profile and info.get(key) != profile[key]:
            problems.append(f"{key} is {info.get(key)!r} but the profile pins {profile[key]!r}")
    return problems


class BusClaim:
    """Exclusive use of the profile's bus: both locks, then a check that the interface is ready.

    Held until close(). Nothing is configured: an interface that is down, at the wrong
    bitrate or not the pinned adapter is refused with the list of problems.
    """

    def __init__(
        self,
        profile: Profile,
        *,
        metadata_reader: Callable[[str], Mapping[str, Any]] = interface_metadata,
    ) -> None:
        channel = channel_name(profile)
        self._native = claim_native_bus(profile.get("native_owner_lock"))
        try:
            self._shared = ownership_lock(shared_lock_path(profile))
            try:
                problems = interface_problems(metadata_reader(channel), profile)
                if problems:
                    raise RuntimeError(f"{channel} is not ready: " + "; ".join(problems))
            except BaseException:
                self._shared.close()
                raise
        except BaseException:
            self._native.close()
            raise

    def close(self) -> None:
        """Release both locks, the shared one first."""
        try:
            self._shared.close()
        finally:
            self._native.close()
