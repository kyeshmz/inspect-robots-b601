# `inspect_robots_b601` package — module map

One Inspect Robots component, the hardware session behind it, and the operator tools.
The package is `mypy --strict` clean and ships `py.typed`.

## Modules

| Module | Responsibility |
|--------|----------------|
| `embodiment.py` | `B601Embodiment` (`b601`): six arm joints plus the gripper in absolute raw motor radians, optional wrist RealSense camera. Construction reads the profile (default `defaults/b601-rs.json`) and declares `.info` only. `reset()` opens an `ArmSession` and, with every operator flag set, activates it. `step()` clips each target to the bounds and `max_step`, holds joints that stalled against something, and walks targets at the session's velocity limit. `RUNTIME_REQUIREMENTS`, `DEVICE_SLOTS` (CAN `channel`), context manager. |
| `preflight.py` | `build` / `run_preflight` + the `inspect-robots-b601-preflight` CLI: conformance check and runtime requirements, no hardware. |
| `cli.py` | The `inspect-robots-b601` operator CLI: `check`, `read`, `recover`, `hold`, `probe`, `pose`, `gripper`, `agent`. Anything that can move or re-arm the arm requires `--confirm-supported-arm`. |
| `hardware/session.py` | `ArmSession`: enables every configured motor with per-axis gains and torque caps, a 50 Hz hold/command loop that never blocks on reads, gravity feed-forward, path walking, envelope and stall checks, and walk-home-before-disable. `read()` / `recover()` open the bus without enabling anything. |
| `hardware/bus.py` | `BusClaim`: the profile's CAN locks plus a read-only check that the interface is up, at 1 Mbit/s and the pinned adapter. Channel validation and default lock path. |
| `hardware/state.py` | `StopLedger`: the durable stop latch, optionally bound to another controller's stop file. Every new state directory starts latched. |
| `hardware/gravity.py` | Static gravity feed-forward and gripper-tip forward kinematics, vendored from Seeed's URDF. |
| `hardware/motor_log.py`, `hardware/thermal.py` | JSONL motor/temperature log and the thermal model it records. |
| `hardware/camera.py` | RealSense colour and depth reader. |
| `hardware/__init__.py` | Shared types: `Profile`, `PathLike`, and the `Motor` / `MotorController` protocols describing the part of Motorbridge's untyped API the package uses. |
| `defaults/` | The stock B601-RS profile and the `agent` command's Inspect defaults. |

## Key invariants

- **Construction is inert:** `B601Embodiment()` touches no CAN, motor or camera, so the
  registry (`factories["b601"]()`), `inspect-robots doctor` and preflight construct it freely.
- **Safety lives in `step()` and the session**, not in an optional Approver.
- **Hardware behind seams:** `ArmSession` takes `controller_factory`, `bus_factory` and
  `clock`, so the whole suite runs without CAN, Motorbridge or a camera.
- **Stops are durable:** a fault latches the `StopLedger`; only an explicit `recover`
  clears it, after fresh reads of every motor.
