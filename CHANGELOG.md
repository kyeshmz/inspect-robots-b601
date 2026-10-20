# Changelog

This package has its own [SemVer policy](docs/versioning.md); its version does
not track the Inspect Robots framework's version.

## [0.2.1] - Unreleased

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
