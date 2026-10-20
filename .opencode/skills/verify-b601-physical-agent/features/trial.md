# Bounded trial and measured outcome

## Sub-features

One initial-pose corridor, pre-prepare and pre-enable activation guards,
full-chunk and dispatch vetoes, one reset, existing stop enforcement, worker
deadline, bounded HTTP, measured scoring, final cleanup receipts, result
persistence, and installed report generation.

## How to get to it (user POV)

The operator command is `b601-physical-agent run` after separate deployment
qualification and explicit consent. Do not execute that operator path during
software verification. Reach it only through the fake-external acceptance.

## Driving it with Python

Run the installed acceptance recipe. Expect a fake 0.2-to-0.2015 joint1 nudge
with score 1 and clean finalization. Expect done without motion to score 0.
Both outcomes must pass through the real provider child and loopback HTTP,
real agent, core, worker, scorer, and installed HTML viewer. Reflected-key HTTP
failures and HTTP-200 Responses failures must use the same joined path, fail
without retry, and leave no selected credential in logs, summaries, stdout,
stderr, or HTML. Activation drift and newly infeasible goals must stop before
prepare or enable without changing the frozen origin. Expect invalid full
chunks, cumulative corridor violations, late replies, state/report changes,
stale feedback, provider errors, interrupts, exhausted budgets, and cleanup
or persistence failures to fail without recovery or another physical trial.

## Gotchas

Only fake motor and provider boundaries are permitted. The local fake HTTP
server is not a model service. All synthetic summaries record torque-off as
unconfirmed. A successful software fixture is not an actual-arm trial.
