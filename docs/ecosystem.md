# B601 in the RoboCurve ecosystem

This is an independent community adapter for the Inspect Robots plugin
contract. It is not a RoboCurve-supported robot or a general-purpose VLA stack.
Use the B601's own hold and joint1-nudge tasks for bounded checks. Other robots'
drivers, checkpoints, and benchmark results do not establish B601 compatibility.

The banner uses the [official Hugging Face logo](https://huggingface.co/brand)
to identify the optional model ecosystem. This does not install a Hugging Face
client, download a checkpoint or make an arbitrary model B601-compatible.

The software checks use [Inspect Robots](https://github.com/robocurve/inspect-robots)
and its [adapter conformance kit](https://docs.inspectrobots.org/guide/adapters/).
The core's built-in CubePick runs in the offline suite to check the evaluation
pipeline. Its 2-D Cartesian scripted policy is intentionally incompatible with
the B601's absolute joint-radian action dimensions. Those are one for direct
joint1 commissioning, six for ROS, or seven for ROS with a calibrated gripper.

## Repository conventions

The presentation follows the sibling adapters' install, preflight, contract,
hardware, and development structure. The README keeps a no-connection contract
check before ROS setup, like the siblings' metadata-only preflights. It uses
existing conformance APIs rather than adding another command.

The [framework adapter guide](https://github.com/robocurve/inspect-robots/blob/2e2d064edc1a8afa7ba8f08bea3fd340f76eef88/docs/guide/adapters.md#the-conformance-kit)
documents hardware-free constructors and connections at `reset()`.
[Franka](https://github.com/robocurve/inspect-robots-franka/blob/cb46d749162b600ec65b234322fadaf89f760376/README.md#preflight)
and [Unitree G1](https://github.com/robocurve/inspect-robots-unitree-g1/blob/3b43fc7e11b08d7e736e7b89decdfde7ed3519cd/README.md#preflight)
separate preflight declarations from physical qualification.
[StationeryBench](https://github.com/robocurve/stationerybench/blob/43d5dd380eeb9f555fe691a896a076cf43d3b18b/README.md#quick-start)
labels its abstract mock's limits. [RoboHarm](https://github.com/robocurve/roboharm/blob/5952f0035037604265d5cf38fa24938d4acf7133/README.md#what-is-included)
distinguishes packaged tools from missing deployment artifacts.

Those conventions do not imply identical capabilities or project policies.
B601 retains its reviewed Inspect 0.60.0 and optional agent 0.28.0 pins,
Python 3.10 through 3.12 range, NumPy below 2, and 75% offline coverage floor.
It has no PyPI publication, selected project license, pretrained policy client,
or full-arm Motorbridge controller. Sibling PyPI, MIT, and 100% coverage badges
are not applicable here.

## Reference repositories

The public [RoboCurve repository catalog](https://github.com/orgs/robocurve/repositories)
contained 15 repositories in the 2026-10-20 review. The table links to the
source snapshots used for this comparison, not guaranteed compatible releases.
All six sibling adapter READMEs, package declarations, preflight modules, and
CI gates were inspected. The framework, benchmarks, and registries also informed
the structure. Training and editorial projects were categorized from their
READMEs rather than treated as adapter templates.

| Repository snapshot | Role | Relevance to B601 |
| --- | --- | --- |
| [inspect-robots](https://github.com/robocurve/inspect-robots/blob/2e2d064edc1a8afa7ba8f08bea3fd340f76eef88/README.md) | Framework | Component discovery, conformance, separate plugins, and log reports. |
| [inspect-robots-agibot-a2](https://github.com/robocurve/inspect-robots-agibot-a2/blob/6d5fe351450d4a0f9e165549d104d8fdc5fdcdee/README.md) | Humanoid adapter | Explicit joint layout, client versus robot setup, and preflight limits. Not a B601 driver. |
| [inspect-robots-franka](https://github.com/robocurve/inspect-robots-franka/blob/cb46d749162b600ec65b234322fadaf89f760376/README.md) | Arm adapter | Install, preflight, safety, units, and configuration order. DROID actions are not B601 actions. |
| [inspect-robots-so101](https://github.com/robocurve/inspect-robots-so101/blob/096d89e12a9e31228fa57a7e5b7d236b09f3db6e/README.md) | Arm adapter | Calibration prerequisites and unit caveats. Its six dimensions include a gripper and use different units. |
| [inspect-robots-unitree-g1](https://github.com/robocurve/inspect-robots-unitree-g1/blob/3b43fc7e11b08d7e736e7b89decdfde7ed3519cd/README.md) | Humanoid adapter | Separate declared compatibility, timing, and physical control limits. |
| [inspect-robots-widowx](https://github.com/robocurve/inspect-robots-widowx/blob/790146bc158b3eecb587bd0c367d0a8d55c2455f/README.md) | Arm adapter | Pinned install sources and explicit supported-hardware scope. Its Cartesian deltas are incompatible. |
| [inspect-robots-yam](https://github.com/robocurve/inspect-robots-yam/blob/3b1e0856671e19bfc575e74a8c8efba03ca5b242/README.md) | Bimanual adapter | Policy versus embodiment explanation, preflight, and hold-verification caveats. Not a B601 model or driver. |
| [stationerybench](https://github.com/robocurve/stationerybench/blob/43d5dd380eeb9f555fe691a896a076cf43d3b18b/README.md) | Benchmark | Hardware-free first result, task table, and separate physical protocol. Bimanual tasks do not directly fit one B601. |
| [roboharm](https://github.com/robocurve/roboharm/blob/5952f0035037604265d5cf38fa24938d4acf7133/README.md) | Benchmark and experiment tools | Synthetic first-run examples and explicit replication gaps. Not a motor qualification suite. |
| [clapboardbench](https://github.com/robocurve/clapboardbench/blob/2f68726a8347dad847a4c6532fe7e842d1d95d5c/README.md) | Published real-robot traces | Reader-first report links and provenance. YAM results do not transfer to B601. |
| [worldevals](https://github.com/robocurve/worldevals/blob/d2225d97c86640a598e05f257ed57bd9e58a30c7/README.md) | Benchmark catalog | Distinguishes available benchmarks from locally installed tasks. B601 is not listed in the reviewed catalog. |
| [worldpolicies](https://github.com/robocurve/worldpolicies/blob/59962bc3bb53129e122d822fa44822f9d17a913a/README.md) | Checkpoint registry | Records embodiment and evaluation protocol. No B601 checkpoint is listed in the reviewed registry. |
| [dreamzero-yam](https://github.com/robocurve/dreamzero-yam/blob/17c355f96e27d10d8138ee14029bad093a0509dd/README.md) | YAM training experiment | Model provenance reference only. No B601 training or serving integration. |
| [gr00t-n1.7-so-101](https://github.com/robocurve/gr00t-n1.7-so-101/blob/415bc4a0ca11797cc56bfc965a07133af9143b75/README.md) | SO-101 training experiment | Another embodiment's fine-tune, not a B601 policy. |
| [llm-token-speed](https://github.com/robocurve/llm-token-speed/blob/32015a9bb3086d6ee8975c620a26c77377f1ce28/README.md) | Editorial data and chart | Unrelated to adapter installation or robot control. |

## Benchmarks

[WorldEvals](https://github.com/robocurve/worldevals) is a catalog, not a motor
test suite. [StationeryBench](https://github.com/robocurve/stationerybench) has
five bimanual tasks and an abstract mock. Its mock can check installation,
epochs and reporting in an isolated environment; it is not a B601 dynamics
model. Real tasks are not directly applicable to one B601.

The optional, manually dispatched `ecosystem` workflow installs StationeryBench
at a fixed commit and runs two scripted mock trials. It never selects a B601
embodiment. KitchenBench and VibeCheckBench appear in the reviewed WorldEvals
catalog but not in the public organization repository listing used above.
They are not automated dependencies here.

## Tested package compatibility

The 2026-10-20 software review used the public B601 v0.4.0 wheel on Linux with
Python 3.12, Inspect 0.60.0, and NumPy below 2. The following exact base packages
co-installed in one fresh environment. `pip check`, both CLI plugin lists,
and the eight installed B601 package tests passed. No hardware, training,
CUDA, or serving extras were installed.

| Packages | Tested versions | What the result establishes |
| --- | --- | --- |
| `inspect-robots-agent` | 0.28.0 | Co-installation and discovery. A separate isolated pair also passed the real B601 evaluation, JSON readback, and HTML report checks described below. |
| `inspect-robots-isaacsim`, `inspect-robots-xpolicylab`, `inspect-robots-capx`, `inspect-robots-ros` | 0.1.1, 0.1.0, 0.3.1, 0.1.0, respectively | Base dependency resolution and plugin discovery, not B601 runtime support. |
| `inspect-robots-franka`, `inspect-robots-yam`, `inspect-robots-so101` | 0.1.0, 0.37.0, 0.4.0, respectively | Base co-installation with B601, not interchangeable drivers, action dimensions, or units. |
| `inspect-robots-widowx`, `inspect-robots-unitree-g1`, `inspect-robots-agibot-a2` | 0.1.0, 0.1.1, 0.1.0, respectively | Base co-installation with B601, not qualification for those robots or checkpoints. |

Co-installation is not a recommendation to install every plugin. Some sibling
packages register the same policy names, including `openpi` and `gr00t`.
A listed name does not identify which package's factory won discovery. Use
separate environments and verify the selected factory before evaluating a
specific policy.

The [installed-wheel agent acceptance](testing.md#installed-wheel-agent-acceptance)
goes beyond imports. Both the public B601 v0.4.0 wheel and the branch wheel
passed two real Inspect evaluations with agent 0.28.0, fake ROS I/O, and mocked
HTTP responses. The nudge scored 1.0. `done` without motion scored 0.0. Both
logs reloaded and rendered through the installed CLI. No live model or robot
was involved. Live inference still requires an authorized provider and the
[separate official ROS mock procedure](testing.md#4-llm-in-simulation-explicitly-opt-in).

Confirmed limits remain:

- CaP-X 0.3.1 rejects the default B601 configuration at bind time because no
  gripper is declared. Camera, perception, IK, calibration, and isolated
  generated-code execution also remain unqualified. Installing the package
  does not supply those integrations.
- XPolicyLab 0.1.0 defaults to an eight-dimensional action space. Default B601
  ROS actions have six dimensions. A compatible checkpoint and explicit
  observation/action mapping are required before evaluation.
- IsaacSim 0.1.1 defaults to Franka, not B601. Its base pip package does not
  supply Isaac Lab, GPU assets, or a B601 model/task.
- Generic ROS 0.1.0 publishes rosbridge topics. It does not replace Seeed's
  trajectory-action adapter.
- The reviewed [RoboHarm snapshot](https://github.com/robocurve/roboharm/blob/5952f0035037604265d5cf38fa24938d4acf7133/pyproject.toml)
  pins Inspect 0.58.0 and its optional agent to 0.26.0. Those pins conflict
  with B601's reviewed 0.60.0 and 0.28.0 pins. Use a separate environment.
- StationeryBench was absent from public PyPI in this review. The optional
  workflow uses its pinned Git source. Its abstract bimanual mock is not a
  B601 simulation or behavior test.

Registry catalogs, published traces, and robot-specific training repositories
are reference material, not installable B601 policy promises. Physical
qualification is separate from every software result in this section.

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
