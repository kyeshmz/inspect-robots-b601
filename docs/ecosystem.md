# Which upstream packages to use

The banner uses the [official Hugging Face logo](https://huggingface.co/brand)
to identify the optional model ecosystem. This does not install a Hugging Face
client, download a checkpoint or make an arbitrary model B601-compatible.

The immediate testing plan uses [Inspect Robots](https://github.com/robocurve/inspect-robots)
and its [adapter conformance kit](https://docs.inspectrobots.org/guide/adapters/).
The core's built-in CubePick runs in the offline suite to check the evaluation
pipeline. Its 2-D Cartesian scripted policy is intentionally incompatible with
the B601's one/six/seven absolute joint-radian action dimensions.

## Benchmarks

[WorldEvals](https://github.com/robocurve/worldevals) is a catalog, not a motor
test suite. [StationeryBench](https://github.com/robocurve/stationerybench) has
five bimanual tasks and an abstract mock. Its mock can check installation,
epochs and reporting in an isolated environment; it is not a B601 dynamics
model. Real tasks are not directly applicable to one B601.

The optional, manually dispatched `ecosystem` workflow installs StationeryBench
at a fixed commit and runs two scripted mock trials. It never selects a B601
embodiment. KitchenBench was listed in WorldEvals but its linked repository
returned 404 during the initial review, so it is not an automated dependency.

## Policies and simulators

| Package | Use | Missing B601 prerequisites |
| --- | --- | --- |
| `inspect-robots-agent` | Tool-calling LLM; tested offline with a mock Responses transport; live mock stage is explicit opt-in | Authorized model/key for live inference; physical qualification for any real run |
| `inspect-robots-isaacsim` | Physics-based Isaac Lab evaluation | NVIDIA/Isaac environment, B601 model/task, calibrated observation/action mapping; default Franka is not B601 |
| `inspect-robots-xpolicylab` | Clients for ACT, SmolVLA, pi0, GR00T and other served policies | Compatible checkpoint, camera streams, joint ordering/units/rate/gripper mapping; no generic zero-shot hardware qualification |
| `inspect-robots-capx` | Code-as-policy with perception and IK helpers | Camera/gripper setup, B601 IK model; depth/intrinsics/extrinsics for grasp planning; externally isolated execution of model-generated Python |
| `inspect-robots-ros` | Generic rosbridge topic publisher | Does not send the trajectory/gripper actions required by Seeed's physical driver; not a replacement for this adapter |
| SO101/YAM/Franka adapters | Reference designs for injected tests, preflight and lifecycle | Different motor buses, joint dimensions and sometimes units; do not reuse their drivers or checkpoints unchanged |

These integrations are prerequisites/roadmap items, not claims that installing
every plugin makes full-arm manipulation work. Keep experimental simulator/model
servers in separate environments and validate compatibility against the pinned
Inspect version before changing the robot environment.
