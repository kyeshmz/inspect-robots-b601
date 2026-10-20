# Changelog

This package has its own [SemVer policy](docs/versioning.md); its version does
not track the Inspect Robots framework's version.

## [0.5.0] - Unreleased

### Added
- `b601-physical-agent check|run` for one operator-attended direct joint1 hold or +0.0015-rad agent trial, separate from simulation and deterministic qualification.
- Read-only existing-state inspection, existing-only runtime acquisition, a worker-enforced active deadline, terminal trial abort, and retained immutable cleanup receipts.
- Fixed measured-origin corridor checks over provider targets, complete interpolated chunks, and actual dispatch. Owned HTTP I/O has bounded attempts, body size, total request lifetime, and cancellation.
- Installed CLI acceptance and real-agent/direct-core tests with fake external boundaries. These tests do not qualify a physical arm.

### Changed
- Direct action semantics publish the existing 0.002-rad max-step limit. Physical activation rechecks the captured profile and actual tracking report after opening.
- The direct adapter retains its session after close, including failed cleanup. The new runner checks its final receipt, required logs, and summary persistence before returning success.

### Installation
- Build this branch's 0.5.0 wheel with the `agent` and `motorbridge` extras. There is no v0.5.0 release or PyPI publication yet.
- Public 0.4.0 wheels and tags are unchanged and do not contain the new command. See [the attended physical-agent guide](docs/physical-agent.md).

## [0.4.0] - 2026-10-20

### Removed
- Cloudflare Realtime integration, network role CLI, optional dependencies, credentials template and transport-specific tests.
- Hosted-service setup from the README and operator/robot quickstart. The repository is a local Inspect Robots integration again.

### Preserved
- Pip-installable package, B601 adapters, measured benchmarks, strict typing, CI and optional PyPI publishing workflow.
- Official Hugging Face artwork, centered banner badges and simplified arm diagram.
- Existing motion limits, controller ownership and physical commissioning gates.

### Migration
- Upgrade to this version without the old `realtime` extra; `b601-realtime` is no longer provided.
- Run `b601-evaluate` beside the ROS mock or use the documented local commissioning procedure. No replacement network service is required.
- Older version tags and release artifacts remain unchanged; this is the new local-only release.

## [0.3.0] - 2026-10-19

### Added
- Optional `realtime` extra and operator/robot CLIs using new Cloudflare Realtime SFU DataChannels, without a tunnel or public robot listener.
- Authenticated, size/age/sequence-bounded status, stop and fixed simulation requests; remote physical activation remains unavailable.
- Robot-local ownership and a read-only physical feedback endpoint; no remote joint-vector, recovery, homing or gain API.
- Separate-computer setup guide, versioned pip wheel installation and an explicit PyPI trusted-publishing workflow.
- Cloudflare API contract tests and actual local aiortc DataChannel/reply tests, without live cloud credentials.

### Changed
- Used the official Hugging Face logo, centered all banner badge text and simplified the arm diagram.
- Made end-user installation and operator/robot responsibilities the primary README entry points.

## [0.2.1] - Development snapshot

### Added
- Strict mypy checks for every shipped source module and Inspect protocol contracts.
- PEP 561 `py.typed` markers and clean-wheel consumer typing checks.
- Public API snapshots, release-tag/metadata checks and compatibility documentation.
- CI/quality/upstream badges and an attributed Inspect Robots / RoboCurve / Seeed Studio banner.
- Explicit branch-coverage reporting and a 75% offline baseline gate, including the ROS module.

### Changed
- Added precise public annotations and typed native/ROS boundary contracts without changing motion limits.
- Shared the fake ROS fixture through pytest's standard fixture discovery.

## [0.2.0] - 2026-10-19

### Added
- Standalone B601 ROS and joint1-only Motorbridge embodiments.
- Measured hold/nudge benchmarks and deterministic policy.
- Offline, real-Humble fake-server, installed-wheel and official Seeed mock checks.
- Optional StationeryBench mock workflow and operator-only physical commissioning tools.
