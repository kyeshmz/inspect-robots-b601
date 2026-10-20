# Bounded trial and measured outcome

## Sub-features

One initial-pose corridor, full-chunk and dispatch vetoes, one reset, existing
stop enforcement, worker deadline, bounded HTTP, measured scoring, final
cleanup receipts, result persistence, and installed report generation.

## How to get to it (user POV)

The operator command is `b601-physical-agent run` after separate deployment
qualification and explicit consent. Do not execute that operator path during
software verification. Reach it only through the fake-external acceptance.

## Driving it with Python

Run the installed acceptance recipe. Expect a fake 0.2-to-0.2015 joint1 nudge
with score 1 and clean finalization. Expect done without motion to score 0.
Expect invalid full chunks, cumulative corridor violations, late replies,
state/report changes, stale feedback, provider errors, interrupts, exhausted
budgets, and cleanup/persistence failures to fail without recovery or another
physical trial. Expect a usable HTML report from the real installed viewer.

## Gotchas

Only fake motor and provider boundaries are permitted. The local fake HTTP
server is not a model service. All synthetic summaries record torque-off as
unconfirmed. A successful software fixture is not an actual-arm trial.
