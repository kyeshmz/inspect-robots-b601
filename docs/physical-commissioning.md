# Operator-only physical commissioning

**Physical tracking readiness remains unresolved. Do not run a live LLM or VLA
until deterministic tracking, supported-arm behavior and stopping are qualified.**

This is separate from simulation. There is no physical CI job, automatic
"run all", controller handoff, automatic recovery or unattended trial sequence.
The package preserves the existing deployment-specific PCAN can1 identity,
native controller ownership lock, Motorbridge 0.5.6 content hashes and legacy
stop-evidence checks. An arbitrary B601/SDK installation is not qualified by
installing this wheel.

## Before any real CAN access

1. Support the entire arm/load mechanically; joints2–6 are **not held** by the
   direct commissioning backend. Clear joint1's movement area.
2. Have an independent cutoff immediately accessible. Python cleanup and a
   mode/disable acknowledgement do not prove physical torque-off.
3. Deliberately arrange exclusive ownership using the robot's existing reviewed
   operator procedure. This package will not stop another controller for you.
4. Supply the robot's actual reviewed profile and **existing shared persistent
   state directory**. Never use `tests/fixtures`, a fabricated stop latch, or a
   new per-evaluation directory to bypass a retained stop.
5. Resolve retained faults and failed tracking before considering recovery. Do
   not increase gains, speed or torque just to make a software test pass.

The following are commands for the operator to run **individually** after that
review; they are not a script to paste and execute as a group.

## Read only

```bash
b601-commission read \
  --profile /absolute/path/to/reviewed-profile.json \
  --state-dir /absolute/path/to/existing/.hardware-state
```

This opens CAN and takes ownership locks but never enables motors. Stop if the
profile, USB identity, SDK verification, ownership or feedback checks fail.

## Recovery, only after the cause is resolved

```bash
b601-commission recover --confirm-supported-arm \
  --profile /absolute/path/to/reviewed-profile.json \
  --state-dir /absolute/path/to/existing/.hardware-state
```

This acknowledges the exact retained evidence after fresh feedback, remains
inactive and does not clear hardware motor faults. It is not automatic repair.

## One deterministic joint1 trial

```bash
b601-commission test-joint1 --confirm-supported-arm \
  --profile /absolute/path/to/reviewed-profile.json \
  --state-dir /absolute/path/to/existing/.hardware-state
```

This actually enables joint1, requests **+0.005 raw rad over two seconds**, checks
measured convergence, then requests scoped disable. No other-axis enable, home,
zero, mode change or return move is performed. A report is saved in the existing
state directory; preserve the primary error and any cleanup uncertainty. Stop
on a failed or ambiguous result. Do not follow a failed trial with a model run.

The Inspect direct adapter's optional activation additionally requires all
explicit motion/operator flags and a passing profile-matched report from the
last ten minutes. No supplied evaluation command enables those flags. A passing
report is an accidental-use gate, not independent physical safety certification.

## Full ROS hardware

`B601Embodiment(simulated=False, action_low=..., action_high=...)` requires a
separately commissioned Seeed ROS controller and explicit bounds in its actual
coordinate frame. Arrange exclusive driver ownership outside this package.
Optional gripper control requires calibrated motor-radian endpoints and effort
configuration; cameras require existing, correctly timestamped compressed ROS
publishers. Neither reset nor close homes the arm or changes motor power.
