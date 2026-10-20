"""Validate stable SemVer metadata, changelog coverage and optional release tags."""

from __future__ import annotations

import argparse
import os
import re
from pathlib import Path

try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib

ROOT = Path(__file__).resolve().parents[1]
STABLE_VERSION = re.compile(r"(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)")


def validate(root: Path = ROOT, *, tag: str | None = None) -> str:
    """Reject inconsistent project metadata; never create or modify a release."""
    project = tomllib.loads((root / "pyproject.toml").read_text())["project"]
    version = project["version"]
    if not isinstance(version, str) or STABLE_VERSION.fullmatch(version) is None:
        raise ValueError(
            "Package version must be stable SemVer MAJOR.MINOR.PATCH without leading zeros"
        )
    if tag is not None and tag != f"v{version}":
        raise ValueError(f"Release tag {tag!r} must exactly match package version v{version}")
    changelog = (root / "CHANGELOG.md").read_text()
    if not re.search(rf"^## \[{re.escape(version)}\](?:\s|$)", changelog, re.MULTILINE):
        raise ValueError(f"CHANGELOG.md needs an entry for {version}")
    core = [
        dependency
        for dependency in project["dependencies"]
        if dependency.startswith("inspect-robots==")
    ]
    if len(core) != 1 or STABLE_VERSION.fullmatch(core[0].split("==")[1]) is None:
        raise ValueError("Declare exactly one reviewed, exact Inspect Robots compatibility pin")
    core_version = core[0].split("==")[1]
    if f"Inspect%20Robots-{core_version}" not in (root / "README.md").read_text():
        raise ValueError("The README compatibility badge must match the reviewed Inspect pin")
    return version


def main() -> int:
    """Check the current project and, on tag builds, its immutable release identity."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--tag",
        default=os.environ.get("GITHUB_REF_NAME")
        if os.environ.get("GITHUB_REF_TYPE") == "tag"
        else None,
    )
    args = parser.parse_args()
    try:
        version = validate(tag=args.tag)
    except ValueError as error:
        parser.error(str(error))
    print(f"Version metadata, changelog and compatibility pin agree: {version}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
