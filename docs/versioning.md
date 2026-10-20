# Public API, compatibility and versioning

This independent plugin follows [SemVer 2.0.0](https://semver.org/). The package's
version is owned by `pyproject.toml`; it is not the version of Inspect Robots,
Motorbridge, ROS or a hardware/firmware qualification.

## Compatibility baseline

- Inspect Robots **0.60.0**, the current reviewed upstream release, is pinned
  exactly. The optional agent is reviewed at **0.28.0**.
- Python **3.10–3.12**, NumPy below 2, and ROS 2 Humble/system Python 3.10 are
  deliberate compatibility constraints. Upstream's broader Python support does
  not establish compatibility with Humble's ABI for this plugin.
- Upgrading either Inspect or the agent is a separate reviewed change: run
  strict typing, protocol/API tests, offline and installed-wheel checks, real
  Humble action tests and the official mock workflow before changing the pin.
- `scripts/check_version.py` checks stable version syntax, a matching changelog
  entry, the reviewed compatibility pin/badge and exact `vX.Y.Z` tag identity.

## Public surface

The compatibility commitment covers the root exports `B601Embodiment` and
`MotorbridgeB601Embodiment`, their documented constructors/reset/step/close
methods, the benchmark factories/policy/scorer, registered Inspect entry-point
names, CLI arguments, and documented action/observation units, order and safety
gates. It does not cover private modules, the test helpers or development scripts.

`tests/fixtures/public-api.json` snapshots signatures, root exports and entry
points. `tests/typing/contracts.py` assigns our classes to Inspect's actual
`Embodiment`, `Policy` and `Scorer` protocols. A snapshot failure is a review
signal, not permission to regenerate a golden file blindly. Behavior, units and
safety compatibility still require human review and runtime tests.

## Version selection

While this package is **0.y.z**, its API is still experimental:

- Compatible fixes, annotations, documentation and development tooling: patch.
- New functionality or an incompatible public-contract change: minor, with a
  migration note when behavior changes. Do not hide an action-unit or safety-gate
  change in a patch.
- At 1.0.0 and later: incompatible API changes require a major version; compatible
  additions require a minor; compatible fixes require a patch.

Versioned releases are immutable. Do not replace an existing tag or published
artifact. A tag is `vX.Y.Z`; Python package metadata uses `X.Y.Z`. Development
changes can accumulate under an unreleased changelog entry until that version is
deliberately tagged. Version 0.3.0 adds the optional two-computer Realtime CLI,
so it is a minor increment rather than a patch. GitHub release assets and PyPI
publication are separate: a public wheel can be installed directly before the
PyPI account's trusted publisher is configured. See [publishing](publishing.md).

## Before tagging

```bash
python -m pip install -e '.[dev]'
ruff check src tests scripts
ruff format --check src tests scripts
python -m mypy
python -m pytest --cov
python -m scripts.check_version
python -m scripts.check_version --tag v0.3.0  # Use the version in pyproject.toml.
python -m build
python -m twine check dist/*
```

Run the ROS and official mock workflows as well. Regenerate an intentionally
changed API baseline with `python -m scripts.public_api`, review the diff and
choose the corresponding version increment before committing it. CI checks
tags but never creates, force-updates or publishes them.
