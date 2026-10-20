"""Durable stop acknowledgement bound to the exact legacy stop evidence."""

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


def existing_path(path: str | Path, *, directory: bool = False) -> Path:
    """Reject symlinks before canonicalization and require an existing regular object."""
    result = Path(path).absolute()
    if any(part.is_symlink() for part in (result, *result.parents)):
        raise RuntimeError(f"Refusing symlinked operational path: {result}")
    mode = result.stat().st_mode
    if not (stat.S_ISDIR(mode) if directory else stat.S_ISREG(mode)):
        raise RuntimeError(f"Operational path has the wrong file type: {result}")
    return result


def _read_state(path: Path) -> dict[str, Any]:
    value: dict[str, Any] = json.loads(existing_path(path).read_text())
    if (
        value.get("schema_version") != 1
        or type(value.get("stopped")) is not bool
        or not isinstance(value.get("acknowledged_legacy_sha256"), (str, type(None)))
    ):
        raise RuntimeError("Invalid commissioning stop state")
    return value


def _read_legacy(path: Path) -> tuple[bytes, str]:
    existing_path(path)
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
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


def inspect_existing_stop_state(
    state_dir: str | Path, legacy_latch_path: str | Path
) -> dict[str, Any]:
    """Read the actual ledger schema without creating or archiving any evidence."""
    directory = existing_path(state_dir, directory=True)
    state = _read_state(directory / "stop-state.json")
    _, digest = _read_legacy(Path(legacy_latch_path))
    return {
        **state,
        "stopped": state["stopped"] or state["acknowledged_legacy_sha256"] != digest,
        "legacy_sha256": digest,
        "legacy_stop_latch": str(legacy_latch_path),
    }


class StopLedger:
    def __init__(
        self,
        state_dir: str | Path,
        legacy_latch_path: str | Path,
        *,
        existing_only: bool = False,
    ) -> None:
        self.directory = Path(state_dir)
        self.legacy_path = Path(legacy_latch_path)
        self.path = self.directory / "stop-state.json"
        self._existing_only = existing_only
        if existing_only:
            inspect_existing_stop_state(self.directory, self.legacy_path)
            self._state = _read_state(self.path)
            _, self._observed_hash = self._legacy()
            return
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        if self.directory.is_symlink() or self.path.is_symlink():
            raise RuntimeError("Stop state must use regular files in a real directory")
        raw, self._observed_hash = self._legacy()
        evidence = self.directory / f"legacy-{self._observed_hash}.json"
        if not evidence.exists():
            private_write(evidence, raw)
        if self.path.exists():
            self._state = _read_state(self.path)
        else:
            self._state = {
                "schema_version": 1,
                "stopped": True,
                "acknowledged_legacy_sha256": None,
                "reason": "Initial commissioning requires explicit recovery",
            }
            private_write(self.path, self._state)

    def _legacy(self) -> tuple[bytes, str]:
        return _read_legacy(self.legacy_path)

    @property
    def stopped(self) -> bool:
        if self._existing_only:
            current = inspect_existing_stop_state(self.directory, self.legacy_path)
            if current["stopped"]:
                self._state = {**self._state, "stopped": True}
        _, self._observed_hash = self._legacy()
        return bool(
            self._state["stopped"]
            or self._state["acknowledged_legacy_sha256"] != self._observed_hash
        )

    def snapshot(self) -> dict[str, Any]:
        stopped = self.stopped
        return {
            **self._state,
            "stopped": stopped,
            "legacy_sha256": self._observed_hash,
            "legacy_stop_latch": str(self.legacy_path),
        }

    def _commit(self, value: dict[str, Any]) -> None:
        if self._existing_only:
            existing_path(self.directory, directory=True)
            existing_path(self.path)
        event = self.directory / f"event-{time.time_ns()}.json"
        private_write(event, value)
        private_write(self.path, value)
        self._state = value

    def stop(self, reason: str, details: dict[str, Any] | None = None) -> None:
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
