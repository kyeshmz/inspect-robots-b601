# Adapter contract and ownership

The implementation follows the [Inspect adapter guide](https://docs.inspectrobots.org/guide/adapters/).
Inspect is pinned at 0.60.0; do not adopt newer setup-wizard APIs without testing
and deliberately updating that pin.

| Requirement | Implementation / evidence |
| --- | --- |
| Hardware-free construction | ROS and Motorbridge load only at reset/open; isolated wheel-import tests |
| Runtime requirements | `RUNTIME_REQUIREMENTS` on both registered embodiment classes |
| Finite action bounds | Nominal mock limits; explicit commissioned physical limits/profile |
| Honest action semantics | Absolute `joint_pos`, no Cartesian rotation, explicit joint labels |
| Aligned measured state | ROS `joint_pos` matches six/seven actions; direct `joint_pos` is one axis and `arm_joint_pos` observes six |
| Guardrails | Conformance checks plus independent bounds, step, speed and convergence checks inside the adapters/core |
| Self-paced lifecycle | Blocking action completion or independent bounded HOLD worker; explicit `close()` |
| Failure behavior | Fault latching; cancellation/disable uncertainty preserved, no automatic recovery |

Run `inspect-robots doctor --embodiment b601_ros` on a sourced ROS host. The
constructor/conformance API also works without ROS. For direct `doctor`, install
the Motorbridge extra and supply `-E profile=/absolute/reviewed/profile.json`.
Doctor does not open CAN, but it does check required runtime availability.

## Boundaries

- `embodiment.py`: ROS-facing spaces, configuration validation, measured-pose
  command limits and session fault state.
- `_ros.py`: one ROS context, subscriptions and outstanding action lifecycle.
- `motorbridge.py`: the one-axis Inspect contract and explicit activation gates.
- `b601_hardware`: the existing bounded commissioning core, strict wire checks,
  exclusive ownership, independent worker and durable stop ledger.
- `benchmarks.py`: scene goals, the deterministic policy and recorded-state
  scorer. It does not access hardware or enable motors.
- `evaluate.py`: simulation-only CLI policy/approver wiring. Physical commands
  have a different executable and require explicit profile/state paths.

One shared core is packaged rather than reimplementing safety in each backend.
The standalone repository is the package's maintained source; the original
`b601-rs-ros2` workspace was not deleted or rewritten during extraction. No
runtime code depends on that checkout.

Tests use injected sessions/transports rather than a production `fake=true`
switch. Synthetic profiles cannot open the real direct runtime. The fixed
can1/native-owner locks are deployment constraints, not configurable wizard
device slots; changing them requires separate hardware qualification.

The conformance kit cannot prove coordinate calibration, truthful feedback,
physical HOLD or emergency-stop behavior. `b601_measured_goal` checks recorded
samples only; use the operator commissioning procedure for real hardware.
