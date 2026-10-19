# Testing, one stage at a time

## 1. Offline: the default gate

```bash
python -m pip install -e '.[dev]'
python -m pytest -v
```

This includes adapter conformance, registry wiring, invalid targets, latched
faults, fresh-feedback and protocol validation, competing ownership, durable
stops, tracking failures and independent HOLD during policy pauses. Fake-motor
evaluations traverse the actual commissioning core and worker. Mocked HTTP
responses traverse the real pinned Inspect agent client and its tool-call
mapping, without credentials or paid inference. A test guard rejects attempts
to construct a real SocketCAN socket.

`tests/fixtures/joint1.synthetic.json` is **not robot calibration**. It is marked
`test_only`; `DirectSession` refuses it unless a fake transport is injected. No
test manufactures operational recovery evidence for a real robot.

GitHub Actions runs this gate on Python 3.10, 3.11 and 3.12. Its separate package
job builds an sdist and a wheel from that sdist, checks metadata, and installs
the wheel into a fresh environment before testing discovery and lazy imports.

## 2. Actual ROS protocol, fake controller

Requires a ROS 2 Humble environment with `rclpy`, `control_msgs`, sensor and
trajectory messages, Cyclone DDS and the pinned Seeed `rebotarm_msgs` overlay.
The CI container builds **only the message package**, not a hardware driver.

```bash
source /opt/ros/humble/setup.bash
source /path/to/messages-or-seeed-workspace/install/setup.bash
source .venv/bin/activate  # Created with the ROS system Python and --system-site-packages.
python -m pip install 'pip==25.3'  # Editable-build support with Humble's system packages.
python -m pip install -e '.[test]'
bash scripts/test-ros.sh
```

The script chooses localhost-only ROS domain **163**, unsets remote-discovery
overrides and refuses to proceed if imports are missing. Do not share this
domain with other controllers. Tests host action servers with synthetic
feedback/images and cover rejection, abort, result codes, late acceptance,
timeouts, cancellation, stale frames and measured benchmark scoring. No CAN
device or model is needed. Missing ROS is not treated as a passing protocol test.

## 3. Official Seeed mock, deterministic policy

Use the existing reviewed ROS workspace; this Python package does not install
system packages, clone controller code during import, or launch a physical
driver. The mock profile's nominal limits are referenced to Seeed commit
`7506dc0fc5c2a7787dcb974d89d13f9f49d16d61`.

In both terminals, source Humble, the workspace overlay and the matching venv:

```bash
export ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_DISCOVERY_SERVER ROS_STATIC_PEERS
```

Terminal one:

```bash
ros2 launch rebotarm_moveit_config demo.launch.py model:=rs use_rviz:=false
```

Terminal two, **one evaluation at a time**:

```bash
b601-evaluate ros-mock --task hold --log-dir logs/ros-hold
b601-evaluate ros-mock --task joint1-nudge --log-dir logs/ros-nudge
inspect-robots view logs/
```

The evaluator always constructs `B601Embodiment(simulated=True)`. It offers no
hardware embodiment flag. Before any goal, the adapter reads the controller's
robot description and requires `ReBot_Arm_RS` with exactly
`mock_components/GenericSystem`. Domain isolation alone is not a physical
safety boundary; keep unrelated controllers off the domain.

Both commands exit nonzero on execution failure **or** a failed measured score.
The default nudge is +0.0015 rad, with a 0.0006-rad scoring tolerance. The fixed
policy moves toward a goal relative to the trial's initial measured pose; it
does not keep accumulating nudges every step. No automatic homing or return
motion is performed between trials.

The manually dispatched **Official Seeed mock** workflow runs this stage on a
GitHub-hosted Humble container. It builds only the pinned description/bringup
and MoveIt configuration packages, installs no motor SDK, and starts only the
official `GenericSystem` mock. `scripts/smoke-official-mock.py` owns that launch,
verifies it before evaluation, runs both benchmarks and shuts down only its own
process group. Do not run that wrapper alongside another domain-61 mock. Use
`b601-evaluate` to attach to an existing mock instead.

## 4. LLM in simulation, explicitly opt in

Only after stages 1–3 pass:

```bash
python -m pip install -e '.[agent]'
# Configure provider credentials securely in the shell, not in Git.
b601-evaluate agent-sim --task joint1-nudge \
  --model openai/YOUR_AUTHORIZED_MODEL --wire responses \
  --allow-paid-inference --log-dir logs/agent-sim
```

This can incur API charges. It requires the same verified mock and isolated
environment as stage 3. The runner caps the trial at 10 control steps and five
model calls, uses a conservative interpolation speed, and installs clamp and
delta approvers. Requests never fall back to physical hardware. Mocked client
tests do not establish actual model access, credentials or policy competence.

To use the registered tasks in another Inspect integration, select
`--task b601/hold` or `--task b601/joint1-nudge`, a compatible policy, and your
explicitly configured embodiment. The dedicated evaluator is the recommended
simulation entry point because it does not inherit an arbitrary embodiment or
policy from a user's global Inspect config.

## 5. Physical commissioning

Follow [the separate operator procedure](physical-commissioning.md). It is not
a pytest marker or CI job. Nothing automatically progresses to this stage.
