"""Simulation-only evaluation entry point; physical commissioning is separate."""

from __future__ import annotations

import argparse
import json
import os

from inspect_robots import eval
from inspect_robots.approver import ChainApprover, ClampApprover, DeltaLimitApprover
from inspect_robots.registry import resolve

from .benchmarks import JointGoalPolicy, hold, joint1_nudge
from .embodiment import B601Embodiment


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("ros-mock", "agent-sim"))
    parser.add_argument("--task", choices=("hold", "joint1-nudge"), default="hold")
    parser.add_argument("--log-dir", default="logs/b601-simulation")
    parser.add_argument("--model", help="Explicit provider/model for agent-sim only")
    parser.add_argument("--wire", choices=("chat", "responses"), default="responses")
    parser.add_argument("--allow-paid-inference", action="store_true")
    args = parser.parse_args(argv)
    if args.stage == "agent-sim" and not (args.model and args.allow_paid_inference):
        parser.error("agent-sim requires --model and --allow-paid-inference")
    if args.stage == "ros-mock" and (args.model or args.allow_paid_inference):
        parser.error("ros-mock is deterministic and accepts no inference options")
    if os.environ.get("ROS_DOMAIN_ID") != "61" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("Simulation requires ROS_DOMAIN_ID=61 and ROS_LOCALHOST_ONLY=1")
    if os.environ.get("ROS_DISCOVERY_SERVER") or os.environ.get("ROS_STATIC_PEERS"):
        parser.error("Unset ROS_DISCOVERY_SERVER and ROS_STATIC_PEERS for isolated simulation")
    robot = B601Embodiment(simulated=True, max_step=0.002, position_tolerance=0.0006)
    policy = None
    try:
        policy = JointGoalPolicy() if args.stage == "ros-mock" else resolve(
            "policy", "agent", model=args.model, wire=args.wire, effort="low",
            max_speed_frac=0.0004, max_llm_calls=5,
        )
        task = (hold if args.task == "hold" else joint1_nudge)(
            max_steps=3 if args.stage == "ros-mock" else 10,
        )
        space = robot.info.action_space
        approver = ChainApprover(ClampApprover(space), DeltaLimitApprover(space, max_delta=0.002))
        log, = eval(task, policy, robot, approver=approver, log_dir=args.log_dir)
        passed = log.status == "success" and log.results.metrics.get("b601_measured_goal") == 1.0
        print(json.dumps({"status": log.status, "metrics": log.results.metrics, "passed": passed}))
        return 0 if passed else 1
    finally:
        try:
            robot.close()
        finally:
            close = getattr(policy, "close", None)
            if callable(close):
                close()


if __name__ == "__main__":
    raise SystemExit(main())
