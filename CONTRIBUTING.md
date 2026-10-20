# Contributing

This is an independent adapter, not part of the upstream Inspect Robots project.
We use its [adapter contract](https://docs.inspectrobots.org/guide/adapters/) and
follow its separation of components, lazy hardware loading and typed public API.
Upstream [development conventions](https://github.com/robocurve/inspect-robots/blob/main/CONTRIBUTING.md)
are the reference, not a claim that every upstream project policy is already met.

```bash
python -m pip install -e '.[dev]'
ruff check src tests scripts
ruff format --check src tests scripts
python -m mypy
python -m pytest --cov
python -m scripts.check_version
```

Strict mypy covers **all shipped Python source**, including the hardware core,
and static consumer/protocol contracts. Only missing stubs for optional ROS and
Motorbridge dependencies are exempted; no first-party module is skipped. `Any`
is confined to dynamic ROS/native messages, external JSON/config/metadata and
framework registry boundaries. Existing dynamic negative-test fixtures run
through pytest rather than being advertised as strict-typed tests.

Ruff checks imports, Python 3.10-compatible modernization and basic correctness;
its formatter runs on source, tests and scripts. This is a deliberately smaller
lint ruleset than upstream's public-docstring/style rules.

## Coverage is not a safety certificate

We report branch coverage and enforce a **75% offline floor** over both packages,
including `_ros.py` even though it cannot execute without ROS. ROS protocol and
official mock behavior are checked in separate Humble jobs. **We do not claim
upstream's 100% line/branch coverage gate.** Do not remove hardware/ROS code from
the denominator or add blanket exclusions to make a badge look better. Improve
tests rather than weakening the baseline; no live CAN or paid model is needed
for the default suite.

See [versioning](docs/versioning.md) before changing public signatures, entry
points, action units/order or safety gates. Keep physical trials operator-only;
never add a physical test to pytest, Actions or an automatic "run all" script.
