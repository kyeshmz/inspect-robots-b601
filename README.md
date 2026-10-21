<div align="center">

# inspect-robots-b601

Run [Inspect Robots](https://github.com/robocurve/inspect-robots) evals on real
Seeed [reBot Arm B601-RS](https://github.com/Seeed-Projects/reBot-DevArm) arms,
driven directly over SocketCAN with [Motorbridge](https://motorbridge.seeedstudio.com).

![Status: alpha](https://img.shields.io/badge/status-alpha-blue)
[![License: MIT](https://img.shields.io/badge/license-MIT-green)](LICENSE)
[![Built on Inspect Robots](https://img.shields.io/badge/built%20on-Inspect%20Robots-indigo)](https://github.com/robocurve/inspect-robots)

</div>

> [!NOTE]
> This project is in early development. The API may change between releases, so pin a version before depending on it.

Inspect Robots has two swappable inputs: a `Policy` (the brain) and an `Embodiment`
(the robot body + world). This package provides the embodiment for the B601-RS, so
any embodiment-agnostic Inspect Robots task runs on a real arm:

- **`b601` embodiment**: the six RobStride arm joints plus the gripper, as 7-D
  absolute joint positions in raw motor radians (`joint1` … `joint6`, `gripper`),
  with an optional wrist RealSense camera. It has a hard clamp inside `step()`,
  per-axis torque caps, gravity feed-forward, a durable stop latch, and walks the
  arm home before it disables the motors.

Pair it with Inspect Robots's LLM `agent` policy, or with any policy that declares
the same 7-D `joint_pos` contract.

```bash
inspect-robots run --instruction "Open the gripper, then close it" --policy agent -P model=provider/model \
  --embodiment b601 -E channel=can0 -E allow_motion=true -E activate_on_reset=true -E operator_supported_arm=true
```

> This is the B601-RS sibling of
> [inspect-robots-so101](https://github.com/robocurve/inspect-robots-so101) (SO-ARM + LeRobot) and
> [inspect-robots-yam](https://github.com/robocurve/inspect-robots-yam) (bimanual YAM + MolmoAct2).
> Same Inspect Robots contract, different body.

## Install (on the computer wired to the arm)

```bash
uv pip install "inspect-robots-b601[motorbridge,agent] @ git+https://github.com/kyeshmz/inspect-robots-b601"
```

- `motorbridge` → Motorbridge 0.5.6, the RobStride driver (Linux x86_64 or aarch64).
- `agent` → `inspect-robots-agent`, the LLM agent policy.
- Python 3.10–3.12. For the camera, install `pyrealsense2` from pip or, on a
  Jetson, from your librealsense build.

From a checkout, `scripts/inspect-robots-b601 setup` creates `.inspect-venv` with
the checkout installed in editable mode.

## Preflight: prove the embodiment before any motion

```bash
inspect-robots-b601-preflight                              # declared spaces + Motorbridge installed
inspect-robots-b601-preflight --camera realsense+depth     # also declare the wrist camera
inspect-robots-b601-preflight --profile my-arm.json --dry-run
inspect-robots doctor --embodiment b601                    # the framework's own audit
```

Preflight constructs the embodiment without opening CAN, runs Inspect Robots's
conformance check, and checks that the motor runtime is installed. It does not
prove the bus or the motors are ready; the next section does.

## Bring up the arm (once)

**1. Bring up the bus.** RobStride motors need 1 Mbit/s classic CAN:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

**2. Describe the arm.** The embodiment reads a JSON *profile*. The bundled one
(`inspect_robots_b601/defaults/b601-rs.json`) describes a stock B601-RS on `can0`
with no other controller on the bus: motor IDs 1–6 for the arm (RS06 on joints 1–3,
RS00 on 4–6), an RS00 gripper on ID 7, joint limits, gains and torque caps. If only
the interface differs, pass `-E channel=can1` or let `inspect-robots setup` fill it
(the embodiment declares a `can` device slot). Otherwise copy the profile, edit it,
and pass `-E profile=...` or set `B601_PROFILE`:

| Key | Meaning |
| --- | --- |
| `channel` | SocketCAN interface, e.g. `can0` |
| `driver`, `usb_vendor`, `usb_product` | Optional. Pin the adapter; any other adapter is refused |
| `can_lock` | Optional. Lock file shared by every B601 session on this bus (default `/run/lock/b601-<channel>.lock`) |
| `native_owner_lock` | Optional. Another controller's lock file, held too so the two never drive the motors together |
| `legacy_stop_latch` | Optional. Another controller's stop file; any change to it re-latches the stop |
| `wrist_camera` | Optional. Jaw calibration for the depth camera; without it observations carry no `jaw_range` |
| `gravity_feedforward`, `joints`, `gripper` | B601-RS facts: limits, motor IDs, models, gains, torque caps. Change only after review |

**3. Check, read, recover.** With the arm supported and a cutoff in reach:

```bash
inspect-robots-b601 check                                # interface up, 1 Mbit/s, adapter pins; opens nothing
inspect-robots-b601 read                                 # fresh positions and fault reports; enables nothing
inspect-robots-b601 recover --confirm-supported-arm      # acknowledge the stop latch
```

Every new state directory (`.hardware-state` by default) starts with the stop
latched, and every fault latches it again. `recover` reads every motor afresh,
refuses while any motor reports a fault, and never enables anything.

## Run on hardware

`inspect-robots-b601 agent --confirm-supported-arm` runs the agent policy with the
bundled run defaults (`defaults/agent.ini`: model, wrist camera, rates; pass your
own with `--inspect-config`), walks a scripted ready pose first, and leaves the
grading to a person. In Python:

```python
from inspect_robots import Scene, Task, eval, operator_scorer
from inspect_robots.approver import DeltaLimitApprover
from inspect_robots.registry import resolve
from inspect_robots_b601 import B601Embodiment

def grade_trial(record, _scene):
    record.operator_judgement = input("Outcome? [y/n/partial/skip]: ")
    record.operator_note = input("Grader note (optional): ")

task = Task(
    name="operator-graded-gripper",
    scenes=[Scene(id="gripper", instruction="Open the gripper, then close it")],
    scorer=operator_scorer(),
    max_steps=20,
)
emb = B601Embodiment(
    channel="can0",
    camera="realsense+depth:640x480@30",
    allow_motion=True, activate_on_reset=True, operator_supported_arm=True,
)
pol = resolve("policy", "agent", model="provider/model")

with emb:  # walks home and disables the motors, even if the eval raises
    (log,) = eval(task, pol, emb,
                  approver=DeltaLimitApprover(emb.info.action_space),
                  before_scoring=grade_trial)
print(log.status)
```

Observations carry `joint_pos` (7), `blocked` (one entry per arm joint: ±1 when a
joint stalled against something in that direction), `tip` (gripper tip x, y, z in
metres and pitch in degrees), and, with a depth camera and a calibrated profile,
`jaw_range`. Each trial is appended to `logs/motorbridge-trials.jsonl` beside the
state directory, and each session writes a motor and temperature log to
`logs/motor/`.

### Operator commands

Commands that move or re-arm the arm require `--confirm-supported-arm`: the arm
and load are supported and an independent cutoff is within reach.

| Command | What it does |
| --- | --- |
| `check` | Interface and install check; opens nothing |
| `read` | Fresh positions and fault reports of every motor, and the stop latch |
| `recover` | Acknowledge a latched stop after fresh reads |
| `probe --joint N` | Move one axis a small amount and report the measured change |
| `hold` | Enable every motor at its measured pose, hold, walk home, disable |
| `pose --stage ...` | Walk staged poses, take a wrist-camera frame at each, walk home |
| `gripper` | Low-torque probe of the gripper alone |
| `agent` | The agent policy on the `b601` embodiment |

## Safety

- **Hard clamp backstop.** Every target is clipped to the profile's joint limits
  and to `max_step` from the measured pose *inside* `step()`, independent of any
  Inspect Robots `Approver`. Non-finite or wrong-shaped actions raise `SafetyAbort`.
  Use `DeltaLimitApprover` as a second layer.
- **Bounded motion.** Targets are walked at 0.05 rad/s (0.5 rad/s for the gripper),
  never leading the measured pose by more than 0.09 rad. A joint that stalls holds
  where it stopped and is reported in `blocked` instead of being pushed harder.
- **Torque.** Per-axis torque caps are written and read back before enabling; the
  gripper's holding force is capped at about 1 Nm.
- **Shutdown.** Motors are disabled only after walking back to the pose they were
  enabled at, unless CAN control is lost. A blocked home keeps holding and retrying
  until cleared; Ctrl-C then disables the motors where they are.
- **Durable stops.** A fault latches the stop in the state directory; nothing
  moves again until `recover`.
- **One owner per bus.** Every session takes the profile's CAN locks and checks the
  interface before touching a motor. It never stops or hands off another controller.
- **Opt-in motion.** Construction and `reset()` without `allow_motion`,
  `activate_on_reset` and `operator_supported_arm` never enable a motor.
- **Not a collision checker.** Keep the arm supported and a physical cutoff within
  reach; a disable acknowledgement is not proof of torque-off.

## Configuration

`B601Embodiment` keyword arguments, all scalars and settable with `-E key=value`:

| Argument | Default | Meaning |
| --- | --- | --- |
| `profile` | bundled `b601-rs.json` | Robot profile path |
| `channel` | the profile's | SocketCAN interface; replaces the profile's |
| `state_dir` | `.hardware-state` | Stop latch; `logs/` is written beside it |
| `allow_motion`, `activate_on_reset`, `operator_supported_arm` | `false` | All three enable the motors at `reset()` |
| `control_hz` | `1.0` | Declared control rate, 0.2–2 |
| `max_step` | `0.5` | Largest arm move per step, in rad (clipped, not rejected) |
| `step_size` | `0.1` | Per-step size the agent plans with, in rad |
| `camera` | `""` | `realsense[+depth][:WxH@FPS]`; empty for no camera |
| `ready` | `""` | Stages walked after activation, `name=offset,...;...` |
| `startup` | `""` | Small checked deltas after activation, `name=delta,...` |
| `hold_seconds` | `0` | Keep the final pose energized before walking home |

The CLI reads `B601_PROFILE` and `B601_STATE_DIR` from the environment; in a
checkout, `scripts/inspect-robots-b601` loads them from `.env` (see
[`.env.example`](.env.example)).

## Development

```bash
uv venv && uv pip install -e ".[dev]"
uv run pytest
uv run mypy                                    # --strict over src/inspect_robots_b601
```

The suite needs no CAN, Motorbridge, camera or model: the motor controller, bus,
clock and session are injected, and an end-to-end test runs a full `eval()` through
the real session on fake motors. `tests/packaging/test_installed.py` checks a built
wheel in a clean environment (`python -I tests/packaging/test_installed.py`). Set
`REBOTARM_CONTROL_PY` to a checkout of Seeed's reBotArm_control_py to recheck the
vendored gravity model against its URDF.

## License

[MIT](LICENSE)
