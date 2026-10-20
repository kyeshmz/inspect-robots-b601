"""Snapshot the documented public Python surface without importing hardware runtimes."""

from __future__ import annotations

import inspect
import json
from importlib.metadata import distribution
from typing import Any

import inspect_robots_b601
from inspect_robots_b601.benchmarks import JointGoalPolicy, MeasuredJointGoal, hold, joint1_nudge


def signature(value: Any) -> dict[str, Any]:
    """Record stable parameter names, kinds, defaults and annotations."""
    result = []
    interface = inspect.signature(value)
    for parameter in interface.parameters.values():
        if parameter.name == "self":
            continue
        result.append(
            {
                "name": parameter.name,
                "kind": parameter.kind.name,
                "annotation": ""
                if parameter.annotation is inspect.Parameter.empty
                else str(parameter.annotation),
                "default": "<required>"
                if parameter.default is inspect.Parameter.empty
                else repr(parameter.default),
            }
        )
    return {
        "parameters": result,
        "returns": ""
        if interface.return_annotation is inspect.Signature.empty
        else str(interface.return_annotation),
    }


def snapshot() -> dict[str, Any]:
    """Make public API/entry-point changes visible to review and version selection."""
    factories = {
        **{name: getattr(inspect_robots_b601, name) for name in inspect_robots_b601.__all__},
        "hold": hold,
        "joint1_nudge": joint1_nudge,
        "JointGoalPolicy": JointGoalPolicy,
        "MeasuredJointGoal": MeasuredJointGoal,
    }
    surfaces = {name: signature(factory) for name, factory in factories.items()}
    for name, factory in factories.items():
        if inspect.isclass(factory):
            for method in ("reset", "step", "close", "bind", "act", "__call__"):
                if hasattr(factory, method) and method != "__call__":
                    surfaces[f"{name}.{method}"] = signature(getattr(factory, method))
                elif method == "__call__" and method in factory.__dict__:
                    surfaces[f"{name}.{method}"] = signature(factory.__call__)
    points = distribution("inspect-robots-b601").entry_points
    return {
        "schema_version": 1,
        "exports": list(inspect_robots_b601.__all__),
        "signatures": surfaces,
        "entry_points": {f"{point.group}:{point.name}": point.value for point in points},
    }


if __name__ == "__main__":
    print(json.dumps(snapshot(), indent=2, sort_keys=True))
