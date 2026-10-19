# inspect-robots-b601

Seeed reBot Arm **B601-RS** embodiments and measured-state benchmarks for
[Inspect Robots](https://github.com/robocurve/inspect-robots).

This standalone repository contains the adapters, the shared bounded joint1
commissioning core, regression tests, simulation evaluations and operator-only
physical test procedure. It does not include credentials, robot calibration,
stop-state files, ROS, vendor native binaries or a commissioned physical robot.

## Install

Use Python **3.10–3.12** on Linux; ROS 2 Humble uses Ubuntu 22.04's system Python
3.10 and a virtual environment created with `--system-site-packages`.

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[dev]'
inspect-robots list
python -m pytest
```

Optional extras: `[agent]` installs the pinned LLM client; `[motorbridge]` installs
the reviewed Motorbridge version. Inspect is pinned to 0.60.0 and NumPy below 2
for the Humble ABI. No install command starts a driver or enables a motor.

## Registered components

| Kind | Name | Purpose |
| --- | --- | --- |
| Embodiment | `b601_ros` | Six absolute ROS joint-radian targets; optional gripper and compressed cameras |
| Embodiment | `b601_motorbridge` | Joint1-only direct commissioning; all six joints observed |
| Task | `b601/hold` | Hold the initial measured pose |
| Task | `b601/joint1-nudge` | Small initial-pose-relative joint1 movement; other joints fixed |
| Policy | `b601_scripted` | Deterministic, bounded absolute joint targets |
| Scorer | `b601_measured_goal` | Measured final error, non-target drift and observed overshoot |

```python
from inspect_robots_b601 import B601Embodiment, MotorbridgeB601Embodiment
from inspect_robots.conformance import assert_embodiment_conformant

robot = B601Embodiment()  # Simulation by default; construction does not import ROS.
assert_embodiment_conformant(robot.info)
robot.close()
```

Both constructors are hardware-free. Connections happen at `reset()`. Direct
Motorbridge requires an explicit reviewed profile; no default calibration is
shipped. Its deployment-specific can1/PCAN identity, native-owner locks, legacy
stop evidence and verified SDK hashes remain enforced.

## Test ladder

1. **Offline:** `python -m pytest` — conformance, compatibility, fake motors,
   wire protocol, ownership, durable stops, policy-pause HOLD, measured scoring,
   mocked LLM responses and the upstream CubePick smoke. No real CAN or model API.
2. **ROS protocol:** `bash scripts/test-ros.sh` — actual ROS action servers with
   fake joints/cameras on a dedicated localhost-only test domain.
3. **Official ROS mock:** launch the official mock separately, then run
   `b601-evaluate ros-mock --task hold` and `--task joint1-nudge`.
4. **Agent simulation:** opt-in `b601-evaluate agent-sim`, with an explicit model
   and paid-inference acknowledgement, against the verified mock only.
5. **Physical commissioning:** separate `b601-commission` operator commands;
   never run by pytest, GitHub Actions or the simulation evaluator.

See [testing and simulation](docs/testing.md), [adapter contract](docs/adapters.md)
and [physical commissioning](docs/physical-commissioning.md). Run one stage at a
time and stop on a failure. Negative safety cases stay unit/protocol tests, not
physical experiments or automatically chained evaluation tasks.

The scorer reads recorded **measured** positions, not policy targets or run
status. It rejects missing/malformed measurements, target error, other-joint
drift and observed overshoot. A completed run is not automatically a successful
task. Scoring only covers recorded samples; it does not certify continuous-time
motion or physical safety.

## Distribution

```bash
python -m build
python -m twine check dist/*
python -m venv /tmp/opencode/b601-wheel-check
/tmp/opencode/b601-wheel-check/bin/python -m pip install dist/*.whl
/tmp/opencode/b601-wheel-check/bin/python -I tests/packaging/test_installed.py
```

The wheel includes both Python packages and all registered benchmarks. Consumers
need no source checkout or `PYTHONPATH` override. This repository does not
automatically publish to PyPI.

## Limits

- ROS uses one action at a time and fresh feedback. These are **not**
  collision-checked MoveIt plans. Physical bounds and gripper endpoints must be
  commissioned in the driver's coordinate frame.
- Direct Motorbridge is a **joint1-only prototype**: joints2–6 are observed,
  not enabled or held. It has no camera, gripper, Cartesian IK, home or zeroing.
- Physical tracking readiness remains unresolved. Software tests, mock results,
  worker HOLD and disable acknowledgements are not proof of physical torque-off.
- No automatic controller handoff, stop acknowledgement, recovery, gain increase
  or full-arm activation is provided. Support the arm/load and keep an
  independent cutoff accessible for operator trials.

See [ecosystem options](docs/ecosystem.md) for WorldEvals, StationeryBench, Isaac
Lab, XPolicyLab and the remaining integration prerequisites.
