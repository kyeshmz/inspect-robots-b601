---
name: verify-b601-onboarding
description: Verify the B601 README installation, hardware-free discovery, and adapter contract through the published wheel and existing package tests. Use for onboarding or presentation changes. Never connect to a robot.
---

# Verify B601 onboarding

## Launch

Use Linux and Python 3.10, 3.11, or 3.12. Locate an installed Python 3.12 interpreter before running the commands. If `python3.12` is not on the path, use its absolute executable path. Do not use a newer system Python. Resolve the host scratch directory and create a unique run directory. Keep its absolute path as `RUN`. From the repository root, run:

```bash
python3.12 -m venv "$RUN/consumer"
"$RUN/consumer/bin/python" -m pip install 'https://github.com/kyeshmz/inspect-robots-b601/releases/download/v0.4.0/inspect_robots_b601-0.4.0-py3-none-any.whl' > "$RUN/install.log" 2>&1
```

Installation needs network access. Subsequent discovery and contract checks need no ROS, CAN, model key, or network service. There is no long-lived process to launch.

## Doctor

```bash
"$RUN/consumer/bin/python" -m pip check
"$RUN/consumer/bin/python" -I -c 'from importlib.metadata import version; print(version("inspect-robots-b601")); print(version("inspect-robots"))'
```

Expect `0.4.0` and `0.60.0`. Do not infer physical readiness from these checks.

## Drive

Follow [the feature map](features/README.md). Run the README's Python example verbatim with the consumer interpreter in isolated mode. Then run the existing installed-package checks:

```bash
"$RUN/consumer/bin/inspect-robots" list embodiments > "$RUN/embodiments.txt"
"$RUN/consumer/bin/python" -I tests/packaging/test_installed.py > "$RUN/package-tests.txt" 2>&1
"$RUN/consumer/bin/b601-evaluate" --help > "$RUN/evaluate-help.txt"
"$RUN/consumer/bin/b601-commission" --help > "$RUN/commission-help.txt"
```

Expect both B601 registry names, eight passing package checks, and successful help commands. The tests inject fake transport at the existing boundary. Do not call `reset()` outside those tests. Do not run a commissioning command, ROS driver, model, or publication workflow.

For a changed checkout, run the existing quality commands in `CONTRIBUTING.md`. Build to a separate directory with `python -m build --outdir "$RUN/dist"`. Repeat the package checks in a second fresh environment with that wheel. The published wheel proves the release instructions; the newly built wheel proves the branch artifact.

## Evidence

Retain the install log, version output, registry output, exact Python example, stdout, package-test output, and quality results under `RUN`. Record the Git SHA and uncommitted diff tested. Inspect README links and the GitHub-rendered README. Capture desktop and narrow-window screenshots if layout changes. Verify the banner loads and no new horizontal page overflow appears.

## Cleanup

All commands above are short-lived. No robot session, port, or server needs teardown. Deactivate the environment if you activated it. Leave evidence and environments in the unique run directory for review. Remove nothing outside that directory. Do not delete old worktrees or environments as part of this check.

## Helpers

Reuse `tests/packaging/test_installed.py` and the existing quality commands. No additional driver is required. Update the feature map when the documented first-run path changes.
