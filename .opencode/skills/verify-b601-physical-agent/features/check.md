# Configuration check

## Sub-features

Installed help, required explicit inputs, independent acknowledgements, exact
report/profile matching, existing shared stop state, and write-free inspection.

## How to get to it (user POV)

Install the unreleased 0.5.0 wheel and invoke `b601-physical-agent --help` or
`b601-physical-agent check` with reviewed paths and explicit provider selection.
Do not use actual operational paths for this software recipe.

## Driving it with Python

Run `tests/packaging/test_physical_installed.py` with the isolated consumer as
specified in the skill. Expect its installed CLI test to pass with sockets,
session construction, ledger construction, and provider subprocesses forbidden.
Expect missing consent or credentials to exit 2, valid synthetic check to exit
0, all fixture bytes unchanged, and no new log directory or native SDK import.

## Gotchas

Check does not verify physical support, ownership, live feedback, credentials
against a provider, motor disable, or readiness. Never make operational files
from synthetic evidence. Global Inspect defaults must not select a model or
backend for this command.
