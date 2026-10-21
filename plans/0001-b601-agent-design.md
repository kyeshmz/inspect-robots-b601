# 0001: B601-RS embodiment + agent policy pairing

Status: accepted (2026-10-21)

## Goal

Ship the B601-RS sibling of inspect-robots-so101, -yam and -franka. It is a plugin
package that registers one `b601` embodiment: a real Seeed reBot Arm B601-RS driven
directly over SocketCAN. Any embodiment-agnostic Inspect Robots task then runs on
the arm. The `b601` embodiment declares a 7-D `joint_pos` contract that the
LLM `agent` policy (inspect-robots-agent) binds to, so
`inspect_robots.compat.check_compatibility` passes for every registered task.

| Robot | Embodiment | Package | Action space | Policies |
| --- | --- | --- | --- | --- |
| Seeed reBot Arm B601-RS | `b601` | `inspect-robots-b601` | 7-D `joint_pos` (6 joints + gripper) | `agent` |

## Stack decision

- **Driver: Motorbridge 0.5.6** (`motorbridge` on PyPI, Seeed's RobStride driver).
  - It talks MIT-mode frames to RS06 (joints 1–3) and RS00 (joints 4–6, gripper)
    motors over SocketCAN, with no ROS in the loop.
  - It ships cp310–cp313 Linux wheels and no type information. The package
    therefore types only the slice it uses (`hardware.Motor`,
    `hardware.MotorController`) and keeps Motorbridge an optional extra,
    imported lazily by the session.
- **Policy: the LLM `agent`** from inspect-robots-agent.
  - No public VLA checkpoint targets this arm yet.
  - The agent plans absolute joint targets from the declared `max_step` and
    finishes with `done`/`give_up`, so it needs no model-specific action
    conversion.
- **Camera: RealSense D405 on the wrist.**
  - Colour comes from pyrealsense2. Depth comes from the RealSense Enhanced
    Depth close-range improver (`rs_depth`) when it is installed; otherwise
    the camera's own depth is used.
  - A profile's `wrist_camera.depth_backend` pins the depth its jaw
    calibration was measured with, and a run whose camera starts with any
    other depth is refused.

## Contract

- **Action:** absolute raw motor radians, `joint1`…`joint6` then `gripper`.
  - `ActionSemantics(control_mode="joint_pos", frame="base", gripper="continuous")`.
  - The declared `max_step` is `step_size` (default 0.1 rad) per arm joint and
    0.3 rad for the gripper.
- **Observation state:**
  - `joint_pos` (7, rad);
  - `blocked` (6, ±1 for a joint stalled in that direction);
  - `tip` (gripper tip x, y, z in metres from the base, plus pitch in degrees);
  - with a calibrated depth camera, `jaw_range` (2, m).
- **Camera:** an optional `front` wrist camera, plus a `front_depth` extra with
  the depth image.
- **Capabilities:** `self_paced`; `control_hz` is declared (0.2–2 Hz, default 1).
- **Device and option slots:** a `can` device slot (`channel`) and three option
  slots (`allow_motion`, `operator_supported_arm`, `activate_on_reset`). With
  them, `inspect-robots setup` writes a config that can move the arm.

## Construction is inert

`B601Embodiment()` reads the profile and declares `.info`, and nothing else. It
touches no CAN, motor or camera. This lets the registry, `inspect-robots doctor`,
`inspect-robots list` and preflight construct it freely. Hardware opens in the
first `reset()`, and only after the camera has started.

## Limits on every move

Every agent action passes through the layers below in order. Each layer can
only shorten the move, never lengthen it. The smallest per-step limit wins.

| Layer | Limit | Default | Set by |
| --- | --- | --- | --- |
| Agent planning | Per-step change = declared `max_step` × min(`max_speed_frac` / 0.1, 1) | 0.07 rad (arm), 0.21 rad (gripper) with the bundled `agent.ini` (`max_speed_frac = 0.07`) | `-P max_speed_frac=` |
| Agent planning | One `move_joints` call lasts at most 10 s, i.e. 10 steps at 1 Hz | about 0.7 rad per call | `-E control_hz=` (more steps per call) |
| Episode budget | Environment steps and model calls per trial | `max_steps = 20`, `max_llm_calls = 3` | `--max-steps`, `-P max_llm_calls=` |
| Inspect guardrails | Clamp to bounds, then per-step delta limit = declared `max_step` | 0.1 rad (arm), 0.3 rad (gripper) | `-E step_size=` (≤ 0.2), `--max-action-delta` |
| `step()` | Clip to the profile's joint limits (+0.01 rad) | profile `lower`/`upper` | profile |
| `step()` | Clip each arm target to measured ± `max_step` | 0.5 rad (gripper: bounds only) | `-E max_step=` (≤ 1.0) |
| `step()` | Hold a blocked joint until the agent moves it back | — | — |
| Session | Walk at a fixed speed, never leading the measured pose by more than a small reach | 0.05 rad/s, 0.09 rad (gripper 0.5 rad/s, 0.45 rad) | code (`MAX_VELOCITY`, `HOLD_REACH`) |
| Session | A joint that stops progressing for 5 s is reported as stalled | — | code (`PATH_STALL_S`) |
| Session | Measured pose more than 0.6 rad from the command (gripper 5 rad) faults and latches the stop | — | code (`MAX_EXCURSION`) |
| Motors | Per-axis torque caps written and read back before enabling | 36 Nm (joints 1–3), 14 Nm (4–6), 1 Nm (gripper) | profile `torque_cap_nm` |
| Ready pose | Each ready offset | ≤ 1.2 rad | `-E ready=` |

Widening a move, from the cheapest change to the most involved:

1. Raise `max_speed_frac` to 0.1. Agent steps grow from 0.07 to 0.1 rad.
2. Raise `step_size` to 0.2. This doubles the declared `max_step`, and with it
   both the agent's plan and the Inspect delta limit.
3. Anything larger needs code changes:
   - lifting the `step_size ≤ 0.2` and `max_step ≤ 1.0` checks;
   - making `MAX_VELOCITY` a profile key, since walking speed stays
     0.05 rad/s whatever the step size.

## Safety

Safety lives in `step()` and the session, not in an optional approver:

- **Opt-in motion:** no motor enables without all three of `allow_motion`,
  `operator_supported_arm` and `activate_on_reset`.
- **Operator gate:** with Inspect's operator console, the operator confirms
  that the arm is supported and the cutoff is in reach before the motors
  enable and before each later trial. `--no-prompt` skips it.
- **Durable stops:** any fault latches the `StopLedger`. Nothing moves again
  until `inspect-robots-b601 recover`, which reads every motor afresh and
  refuses while any reports a fault.
- **One owner per bus:** each session takes the profile's CAN locks and checks
  that the interface is up at 1 Mbit/s on the pinned adapter. It never stops
  or hands off another controller.
- **Walk home before disable:** the session walks back to its activation pose
  before disabling. A blocked home keeps holding and retrying.

## Trials

Each `reset()` starts a trial:

1. Record the previous trial.
2. Walk home.
3. Confirm with the operator.
4. Walk the ready and startup moves.

Trials are appended to `logs/motorbridge-trials.jsonl` beside the state
directory. Each record holds the start and final poses, any fault, the depth
backend and a final camera frame. Machine-specific profiles and hardware runs
live outside this repository, so the package stays usable by any B601-RS owner.

## Testing

- Hardware sits behind seams: `ArmSession` takes `controller_factory`,
  `bus_factory` and `clock`. The suite therefore runs without CAN, Motorbridge,
  a camera or a model.
- `tests/test_eval_end_to_end.py` runs a full `eval()` through the real session
  on fake motors.
- `tests/test_ecosystem.py` drives the real `inspect-robots` CLI against a fake
  model. It covers `run`, `eval-set`, every registered task, the log tools and
  the setup wizard.
- `tests/packaging/test_installed.py` checks a built wheel in a clean
  environment.
