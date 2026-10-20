---
name: verify-b601-physical-agent
description: Verify the unreleased B601 physical-agent installed CLI and real-agent/direct-core behavior using only fake external boundaries. Use for changes to physical_agent, direct ownership, state inspection, or finalization. Never operate hardware.
---

# Verify B601 physical-agent software

## Launch

Use Linux and an absolute Python 3.10 through 3.12 executable. Set `PYTHON` to
that executable. Resolve the host scratch directory and create a unique directory
under it as `RUN`. Run these commands from the feature repository root:

```bash
"$PYTHON" -m venv "$RUN/build"
"$RUN/build/bin/python" -m pip install -e '.[dev]'
"$RUN/build/bin/python" -m build --outdir "$RUN/dist"
"$PYTHON" -m venv "$RUN/consumer"
"$RUN/consumer/bin/python" -m pip install "$RUN/dist/inspect_robots_b601-0.5.0-py3-none-any.whl[agent,motorbridge,test]"
```

Installation downloads packages. No later step needs a model service, robot,
ROS, CAN, or real credential. There is no long-lived application to start.
Do not use the public 0.4.0 wheel for the new command. Do not publish the wheel.

## Doctor

```bash
"$RUN/consumer/bin/python" -m pip check
"$RUN/consumer/bin/python" -I -c 'from importlib.metadata import version; print(version("inspect-robots-b601")); print(version("inspect-robots")); print(version("inspect-robots-agent")); print(version("motorbridge"))'
"$RUN/consumer/bin/b601-physical-agent" --help
```

Expect 0.5.0, 0.60.0, 0.28.0, and 0.5.6. Stop on an unexpected version or
editable consumer import. The installed acceptance verifies exact origins.

## Drive

Follow [the feature map](features/README.md). Run the actual installed artifacts:

```bash
"$RUN/consumer/bin/python" -I tests/packaging/test_installed.py > "$RUN/package-tests.txt" 2>&1
"$RUN/consumer/bin/python" -I tests/test_agent.py --expected-b601-version 0.5.0 --artifacts-dir "$RUN/previous-agent" > "$RUN/previous-agent.txt" 2>&1
"$RUN/consumer/bin/python" -I tests/packaging/test_physical_installed.py --artifacts-dir "$RUN/physical" > "$RUN/physical-installed.txt" 2>&1
```

The last runner drives the installed CLI with sockets, session creation, ledger
construction, and provider subprocesses forbidden for help/check/rejection.
It compares all fixture bytes before and after. It then runs the real installed
agent, adapter, core, HOLD worker, evaluator, scorer, logs, and HTML viewer.
Only external motor and ownership are fake in the joined cases. Provider input
is redirected to a local fake HTTP server while the actual provider-only child
runs. Positive nudge, false done, and reflected provider failures share this
path, including the installed HTML viewer. Separate HTTP tests cover a slow
body and total-lifetime cancellation. No external model request is allowed.

Do not run `b601-physical-agent run`, `b601-commission`, `reset()`, ROS, CAN,
recovery, or operator procedures outside these isolated tests. The generated
profile/state/report files are temporary software fixtures. They must never be
copied into an operational state directory or treated as qualification.

## Evidence

Keep build/install output, exact Git SHA and dirty diff, `pip check`, versions,
and the three acceptance outputs under `RUN`. The physical evidence directory
contains the zero-effect CLI transcript, runtime regression output, synthetic
case logs and summaries, simulation rejection, and installed HTML report.
Inspect literal outcomes. The fake positive nudge measures joint1 at 0.2015
from 0.2 and scores 1. Done without motion scores 0. Cleanup failures must not
return overall success, even after score 1. None of this proves actual-arm
readiness, torque-off, hard real-time enforcement, or physical qualification.

## Cleanup

The commands are short-lived. The HTTP test shuts down its own local fake
server and kills and joins a stalled provider-only child before returning.
Do not kill by process name or touch unrelated processes, worktrees, state, or
environments. Leave `RUN` intact for review. Confirm that the CLI transcript,
runtime output, and HTML report still exist after all commands finish.

## Helpers

Use `tests/packaging/test_physical_installed.py` as shown above. Do not replace
it with source-only assertions. Update this skill and its feature map when
the command or acceptance paths change.
