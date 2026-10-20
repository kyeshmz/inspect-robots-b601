# Run one attended direct joint1 agent trial

This guide describes **0.5.0, unreleased**, on `feat/guarded-physical-agent`.
The immutable public 0.4.0 wheel does not contain `b601-physical-agent`.
There is no v0.5.0 release or PyPI publication yet.

This command is for a qualified operator on the reviewed robot host. It controls
**joint1 only**, in absolute raw motor radians. Joints2-6 are observed, not held.
Mechanically support them and the load. Keep an independent physical cutoff
accessible throughout the trial. Do not run this guide as an unattended script.

Software tests do not establish physical readiness. No actual-arm agent trial
is claimed. The deployment host, reviewed profile, shared state, recent tracking
report, authorized model, and operator still need deployment-specific review.

## Install the new branch artifact

Use Linux and Python 3.10 through 3.12. In the reviewed feature checkout, build
and install a new wheel in a robot-local virtual environment:

```bash
git switch feat/guarded-physical-agent
python3.12 -m venv .build-venv
.build-venv/bin/python -m pip install 'build>=1.2,<2'
.build-venv/bin/python -m build --outdir dist-physical
python3.12 -m venv .physical-venv
.physical-venv/bin/python -m pip install './dist-physical/inspect_robots_b601-0.5.0-py3-none-any.whl[agent,motorbridge]'
.physical-venv/bin/python -m pip check
.physical-venv/bin/python -c 'from importlib.metadata import version; print(version("inspect-robots-b601"))'
.physical-venv/bin/b601-physical-agent --help
```

Expect `0.5.0`. Review the exact source commit before installation. Keep release
artifacts and tags unchanged. This does not deploy, configure CAN, acquire a
controller, or qualify a Motorbridge installation. The runtime still verifies
Motorbridge 0.5.6 content hashes and ABI, can1 PCAN identity, and both existing
ownership locks. Inspect 0.60.0, agent 0.28.0, and NumPy below 2 remain pinned.

## Check existing configuration without hardware effects

Set these variables yourself. Use actual reviewed absolute paths, not repository
test fixtures. `STATE_DIR` must be the existing shared commissioning directory.
Its regular `stop-state.json` and the profile's actual legacy stop evidence must
exist. Do not initialize a new ledger to make a check pass.

`TRACKING_REPORT` must identify an existing successful deterministic direct
joint1 report for the same exact profile bytes. It must be at most 600 seconds
old. `LOG_DIR` must be a new path beneath an existing real directory.
`PROVIDER_URL` is the explicitly chosen HTTPS Responses API base URL.
`MODEL` is that endpoint's explicit model identifier. Provision the credential
in `B601_PROVIDER_KEY` through your normal secret handling. Do not put it on the
command line, in the profile, or in logs.

```bash
common=(
  --profile "${PROFILE:?Set the reviewed profile path}"
  --state-dir "${STATE_DIR:?Set the existing shared state directory}"
  --tracking-report "${TRACKING_REPORT:?Set the existing deterministic report path}"
  --log-dir "${LOG_DIR:?Set a new output directory path}"
  --task joint1-nudge
  --provider-url "${PROVIDER_URL:?Select the HTTPS provider endpoint}"
  --model "${MODEL:?Select the model explicitly}"
  --api-key-env B601_PROVIDER_KEY
)
.physical-venv/bin/b601-physical-agent check "${common[@]}"
```

The check reads configuration and existing evidence. It does not open CAN,
import the native SDK, acquire ownership, create or archive ledger state, or
call a provider. It checks that the selected credential is nonempty, not that
it works. It cannot verify mechanical support, controller inactivity, actual
feedback, motor disable, or physical readiness. Runtime gates remain authoritative.
Missing, malformed, symlinked, retained-stop, expired, or mismatched inputs fail.

## Qualify separately, only with the operator present

Follow [physical commissioning](physical-commissioning.md) first. A retained
stop is not an invitation to recover automatically. This agent command never
recovers, acknowledges stop evidence, changes mode, or hands off controllers.

When the deployment and operator prerequisites are satisfied, invoke the
deterministic test individually:

```bash
.physical-venv/bin/b601-commission test-joint1 \
  --profile "${PROFILE:?}" --state-dir "${STATE_DIR:?}" \
  --confirm-supported-arm
```

This is a real physical action, +0.005 rad over 2 seconds. Review its measured
result and cleanup. Select its actual successful `report_path` as
`TRACKING_REPORT`, rebuild the `common` array above, and check again. Do not
generate, copy from tests, or edit qualification evidence to satisfy the gate.
The report is an accidental-use gate, not certified or cryptographic proof.

## Run one trial individually

Before this command, confirm that the operator is at the cutoff, all inactive
joints and the load are supported, the controller is available without a
handoff, and paid inference is authorized. Do not run it if any condition is
unresolved. The four flags are explicit operator statements, not software tests.

```bash
.physical-venv/bin/b601-physical-agent run "${common[@]}" \
  --allow-physical-motion \
  --confirm-supported-arm \
  --confirm-independent-cutoff \
  --allow-paid-inference
```

The only task choices are `joint1-nudge`, a +0.0015-rad relative goal, and `hold`.
The trial captures one measured initial pose. Its allowed joint1 interval is
the initial-to-goal interval plus 0.0006 rad tolerance, intersected with the
reviewed profile bounds. The interval never moves with subsequent actions.
The core checks its actual activation samples against that frozen origin and
goal before preparation and again before enable. Inconsistent drift or a newly
infeasible goal fails without shifting the trial origin.
An infeasible goal, invalid provider target, complete invalid interpolated
chunk, or violating dispatch terminates the trial and follows existing stop
persistence. Targets are vetoed, not clamped into apparent success.

There is one scene and epoch, at most six environment steps, and at most three
actual HTTP attempts with one attempt per agent call. The existing max-step
semantics cap interpolation at 0.002 rad. The existing core retains its separate
0.01-rad activation envelope, 0.01-rad/s command limit, fresh-feedback gates,
gain choices, torque cap, convergence checks, and independent HOLD worker.
There are no tuning, backend, joint-list, freeform-task, or prior-learning flags.
Inspect global defaults and generic environment routing are not inherited.

Each provider-only HTTP child has a five-second total lifetime including body
consumption. Requests and responses have one-MiB body limits. The parent kills
and joins a timed-out or interrupted HTTP child. No abandoned evaluation thread
can later apply a reply. Redirects, proxies, and HTTP retries are disabled.
HTTP failure bodies and Responses failure messages are withheld before the
agent can log them. Terminal diagnostics retain the HTTP status or failure
type, not provider error text that might reflect the selected credential.
The runner owns the injected transport because agent 0.28 has no public policy
timeout parameter or close method.

The existing motor worker latches a stop when the 30-second active deadline
expires, even while inference waits. Python scheduling, filesystem stalls,
kernel calls, native driver blocking, and the GIL can delay enforcement or
cleanup. These bounds are **not hard real-time or a safety-rated e-stop**.
HOLD and a disable acknowledgement do not confirm physical torque-off.

## Inspect the measured outcome

The command exits zero only after an error-free measured score of 1, agent
`done`, a clean final hardware receipt, acknowledged disable, complete resource
cleanup, and required log and summary persistence. `done` and `give_up` only
produce hold actions. Neither stops the motor or proves task completion.
Budget exhaustion, give-up, interruptions, and cleanup uncertainty fail.

Inspect `physical-agent-summary.json` in `LOG_DIR`. Its distinct
`b601-physical-agent-summary-v1` schema cannot qualify another run. It retains
the measured score even if later cleanup fails. `torque_off_confirmed` is
always false. If summary persistence fails, the command returns nonzero and
prints that failure instead of claiming a saved successful result.

Use the `eval_log` path from the summary for the installed Inspect viewer:

```bash
.physical-venv/bin/inspect-robots view "${EVAL_LOG:?Select the summary eval_log path}" \
  --no-video -o "${REPORT_HTML:?Select the HTML output path}"
```

Read the measured trajectory score, action log, transcript, primary error, and
all cleanup failures. A failed worker or resource close can leave ownership or
disable uncertain. Use the independent cutoff and review the failure. Never
retry, recover, reconnect, reset, or repeat a physical trial automatically.

See [software-only verification](testing.md#physical-agent-software-acceptance)
for the fake-external acceptance recipe. That recipe never authorizes this
operator procedure.
