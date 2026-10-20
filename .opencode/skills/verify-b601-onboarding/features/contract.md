# Inspect the contract without a robot

## Sub-features

Import the public ROS embodiment, check conformance, inspect its six-joint action shape, and close it without a connection.

## How to get to it (user POV)

After installation, run the Python block under the README's first-success section.

## Driving it with the Python CLI

Save the exact README block under the run directory. Run it with `"$RUN/consumer/bin/python" -I` and retain stdout. Confirm the stated readiness message. Use the existing installed-package test to confirm lazy imports and the absence of a ROS session. This verifies metadata and plugin wiring, not a simulated motion trial.

## Gotchas

The default `simulated=True` selects the official ROS mock backend. It is not an in-process simulator. `reset()` requires a sourced ROS runtime and a verified mock. Do not add it to this example. Motorbridge needs an explicit reviewed profile and offers joint1-only commissioning, not six-axis control.
