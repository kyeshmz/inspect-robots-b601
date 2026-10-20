# Changelog

This package has its own [SemVer policy](docs/versioning.md); its version does
not track the Inspect Robots framework's version.

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
