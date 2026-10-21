"""Preflight: prove the B601 embodiment is ready for Inspect Robots *before any motion*.

``inspect-robots-b601-preflight`` constructs the embodiment from a profile (no CAN, no
motors, no camera; only its declared ``.info``), runs Inspect Robots's conformance check,
and checks that the motor runtime is installed, and so is the RealSense Enhanced Depth
package when the profile's depth camera calibration needs close-range depth. It prints
the report and exits non-zero on any error.

A green preflight means the action space, semantics, state and cameras are declared
correctly and Motorbridge is importable. It does not prove the bus or the motors are
ready: ``inspect-robots-b601 check`` and ``inspect-robots-b601 read`` do that.
"""

from __future__ import annotations

import argparse
import json
import os
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any

from inspect_robots.conformance import (
    ConformanceReport,
    check_embodiment,
    missing_runtime_requirements,
)

from inspect_robots_b601.embodiment import B601Embodiment
from inspect_robots_b601.hardware.camera import close_range_problem


@dataclass(frozen=True)
class PreflightReport:
    """Conformance findings plus any runtime requirement that is not installed."""

    conformance: ConformanceReport
    missing: dict[str, str] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        """Whether the embodiment conforms and its runtime is installed."""
        return self.conformance.ok and not self.missing


def build(profile: str = "", **kwargs: Any) -> B601Embodiment:
    """Construct the embodiment without connecting to anything."""
    return B601Embodiment(profile=profile, **kwargs)


def run_preflight(embodiment: B601Embodiment | None = None) -> PreflightReport:
    """Return the preflight report for ``embodiment`` (default: the stock profile)."""
    embodiment = embodiment if embodiment is not None else build()
    missing = dict(missing_runtime_requirements(type(embodiment)))
    if embodiment.depth_backend == "close-range":
        problem = close_range_problem()
        if problem is not None:
            missing["rs_depth"] = (
                f"install the RealSense Enhanced Depth package (librealsense2-enhanced-depth); "
                f"wrist_camera.depth_backend is close-range ({problem})"
            )
    return PreflightReport(check_embodiment(embodiment.info), missing)


def _format_human(report: PreflightReport, *, dry_run: bool) -> str:
    lines = [report.conformance.summary()]
    lines += [
        f"  [error] missing {module}: {command}" for module, command in report.missing.items()
    ]
    lines.append("OK: ready for Inspect Robots." if report.ok else "NOT READY.")
    if dry_run:
        lines.append("(dry-run) No motion will be commanded.")
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point. Returns a process exit code (non-zero on any error)."""
    parser = argparse.ArgumentParser(
        prog="inspect-robots-b601-preflight", description=__doc__.split("\n\n")[0]
    )
    parser.add_argument(
        "--profile",
        default=os.environ.get("B601_PROFILE", ""),
        help="robot profile (default: $B601_PROFILE, else the bundled stock B601-RS)",
    )
    parser.add_argument(
        "--camera", default="", help="camera to declare, as in -E camera=, e.g. realsense+depth"
    )
    parser.add_argument("--json", action="store_true", help="emit the report as JSON")
    parser.add_argument("--dry-run", action="store_true", help="affirm no motion will be commanded")
    args = parser.parse_args(argv)

    report = run_preflight(build(args.profile, camera=args.camera))
    if args.json:
        print(
            json.dumps(
                {
                    "ok": report.ok,
                    "embodiment": report.conformance.embodiment,
                    "issues": [
                        {"severity": issue.severity, "code": issue.code, "message": issue.message}
                        for issue in report.conformance.issues
                    ],
                    "missing": report.missing,
                    "dry_run": args.dry_run,
                },
                indent=2,
            )
        )
    else:
        print(_format_human(report, dry_run=args.dry_run))
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
