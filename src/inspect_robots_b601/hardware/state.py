"""Durable stop latch, optionally bound to another controller's stop evidence.

Every new state directory starts stopped: the first activation needs an explicit
operator recovery. When the profile names a ``legacy_stop_latch`` (a stop file
written by another controller on the same arm), recovery acknowledges that exact
file content and any later change to it re-latches the stop.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import stat
import tempfile
import time
from pathlib import Path
from typing import Any

from . import PathLike


def private_write(path: Path, value: dict[str, Any] | bytes) -> None:
    """Atomically persist private operational state before reporting success."""
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if path.parent.is_symlink() or path.is_symlink():
        raise RuntimeError(f"Refusing symlinked operational state: {path}")
    temporary = None
    data = (
        value
        if isinstance(value, bytes)
        else (json.dumps(value, indent=2, allow_nan=False) + "\n").encode()
    )
    try:
        with tempfile.NamedTemporaryFile(mode="wb", dir=path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            os.fchmod(stream.fileno(), 0o600)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
        fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


class StopLedger:
    """The durable stop latch in a state directory, bound to a legacy latch file when given."""

    def __init__(self, state_dir: PathLike, legacy_latch_path: PathLike | None = None) -> None:
        self.directory = Path(state_dir)
        self.legacy_path = None if legacy_latch_path is None else Path(legacy_latch_path)
        self.path = self.directory / "stop-state.json"
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        if self.directory.is_symlink() or self.path.is_symlink():
            raise RuntimeError("Stop state must use regular files in a real directory")
        raw, self._observed_hash = self._legacy()
        if raw is not None:
            evidence = self.directory / f"legacy-{self._observed_hash}.json"
            if not evidence.exists():
                private_write(evidence, raw)
        if self.path.exists():
            self._state = json.loads(self.path.read_text())
            if (
                self._state.get("schema_version") != 1
                or type(self._state.get("stopped")) is not bool
                or not isinstance(self._state.get("acknowledged_legacy_sha256"), (str, type(None)))
            ):
                raise RuntimeError("Invalid commissioning stop state")
        else:
            self._state = {
                "schema_version": 1,
                "stopped": True,
                "acknowledged_legacy_sha256": None,
                "reason": "Initial commissioning requires explicit recovery",
            }
            private_write(self.path, self._state)

    def _legacy(self) -> tuple[bytes | None, str | None]:
        if self.legacy_path is None:
            return None, None
        fd = os.open(self.legacy_path, os.O_RDONLY | os.O_NOFOLLOW)
        with os.fdopen(fd, "rb") as stream:
            if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
                raise RuntimeError("Legacy stop evidence must be a regular file")
            raw = stream.read(65537)
        if len(raw) > 65536:
            raise RuntimeError("Legacy stop evidence is unexpectedly large")
        value = json.loads(raw)
        when = value.get("last_stop_unix_s")
        if (
            value.get("schema_version") != 1
            or value.get("latched") is not True
            or type(when) not in (int, float)
            or not math.isfinite(when)
            or when < 0
        ):
            raise RuntimeError("Legacy stop evidence is invalid; activation remains inhibited")
        return raw, hashlib.sha256(raw).hexdigest()

    @property
    def stopped(self) -> bool:
        """Whether the stop is latched here or the legacy file differs from the acknowledged one."""
        _, self._observed_hash = self._legacy()
        return (
            bool(self._state["stopped"])
            or self._state["acknowledged_legacy_sha256"] != self._observed_hash
        )

    def snapshot(self) -> dict[str, Any]:
        """The stored state with the live stopped flag, legacy hash and legacy latch path."""
        stopped = self.stopped
        return {
            **self._state,
            "stopped": stopped,
            "legacy_sha256": self._observed_hash,
            "legacy_stop_latch": None if self.legacy_path is None else str(self.legacy_path),
        }

    def _commit(self, value: dict[str, Any]) -> None:
        event = self.directory / f"event-{time.time_ns()}.json"
        private_write(event, value)
        private_write(self.path, value)
        self._state = value

    def stop(self, reason: str, details: dict[str, Any] | None = None) -> None:
        """Latch the stop for reason; this process stays stopped even if persisting it fails."""
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("Stop requires a reason")
        # Inhibit this process even if persistence fails.
        self._state = {
            **self._state,
            "stopped": True,
            "reason": reason,
            "details": details,
            "time_unix_s": time.time(),
        }
        self._commit(self._state)

    def acknowledge(self, details: dict[str, Any] | None = None) -> None:
        """Caller must obtain fresh feedback after its last stopped/snapshot check."""
        _, current = self._legacy()
        if current != self._observed_hash:
            raise RuntimeError(
                "Legacy stop changed during recovery; obtain new post-event feedback"
            )
        value = {
            "schema_version": 1,
            "stopped": False,
            "acknowledged_legacy_sha256": current,
            "reason": "Operator recovery to inactive state",
            "details": details,
            "time_unix_s": time.time(),
        }
        self._commit(value)
