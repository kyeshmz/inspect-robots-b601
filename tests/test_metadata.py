"""Public metadata and artwork must agree with the package's real compatibility."""

import hashlib
import json
import xml.etree.ElementTree as ET
from importlib.metadata import version

import pytest
from PIL import Image
from scripts.check_version import ROOT, validate
from scripts.public_api import snapshot


def test_version_changelog_and_installed_metadata_agree():
    assert validate() == version("inspect-robots-b601")
    assert validate(tag=f"v{version('inspect-robots-b601')}") == validate()


def test_public_api_snapshot_requires_an_explicit_review():
    expected = json.loads((ROOT / "tests/fixtures/public-api.json").read_text())
    assert snapshot() == expected, (
        "Review the public API change and SemVer impact before updating this snapshot"
    )


@pytest.fixture
def version_project(tmp_path):
    (tmp_path / "pyproject.toml").write_text(
        '[project]\nversion = "0.2.1"\ndependencies = ["inspect-robots==0.60.0"]\n'
    )
    (tmp_path / "CHANGELOG.md").write_text("## [0.2.1] - Unreleased\n")
    (tmp_path / "README.md").write_text("Inspect%20Robots-0.60.0")
    return tmp_path


@pytest.mark.parametrize("invalid", ["0.02.1", "v0.2.1", "0.2", "0.2.1rc1", "1.0.0-rc.1"])
def test_invalid_stable_versions_are_rejected(version_project, invalid):
    project = version_project / "pyproject.toml"
    project.write_text(project.read_text().replace('"0.2.1"', json.dumps(invalid)))
    with pytest.raises(ValueError, match="SemVer"):
        validate(version_project)


def test_wrong_tag_missing_changelog_and_stale_badge_are_rejected(version_project):
    with pytest.raises(ValueError, match="exactly match"):
        validate(version_project, tag="v0.3.0")
    (version_project / "CHANGELOG.md").write_text("## [0.2.0]\n")
    with pytest.raises(ValueError, match="CHANGELOG"):
        validate(version_project)
    (version_project / "CHANGELOG.md").write_text("## [0.2.1]\n")
    (version_project / "README.md").write_text("Inspect%20Robots-0.59.0")
    with pytest.raises(ValueError, match="badge"):
        validate(version_project)


def test_unreviewed_inspect_range_is_not_advertised_as_an_exact_match(version_project):
    project = version_project / "pyproject.toml"
    project.write_text(project.read_text().replace("inspect-robots==", "inspect-robots>="))
    with pytest.raises(ValueError, match="reviewed"):
        validate(version_project)


def test_banner_is_self_contained_attributed_and_rendered():
    svg = ROOT / "assets/banner.svg"
    root = ET.fromstring(svg.read_text())
    for name in ("Inspect Robots", "RoboCurve", "Seeed Studio", "Hugging Face"):
        assert name in svg.read_text()
    for node in root.iter():
        assert not node.tag.endswith("script")
        assert not any(
            key.endswith("href") and value.startswith(("http:", "https:"))
            for key, value in node.attrib.items()
        )
    assert "MIT License" in (ROOT / "assets/NOTICE.md").read_text()
    with Image.open(ROOT / "assets/banner.png") as image:
        assert image.format == "PNG"
        assert image.size == (1600, 600)


def test_banner_uses_official_huggingface_paths_and_geometrically_centered_badges():
    namespace = "{http://www.w3.org/2000/svg}"
    source = ROOT / "assets/huggingface-logo.svg"
    assert (
        hashlib.sha256(source.read_bytes()).hexdigest()
        == "942cad1ccda905ac5a659dfd2d78b344fccfb84a8a3ac3721e08f488205638a0"
    )
    original = ET.fromstring(source.read_bytes())
    banner = ET.fromstring((ROOT / "assets/banner.svg").read_bytes())
    embedded = banner.find(f".//{namespace}svg[@id='hugging-face-logo']")
    assert embedded is not None
    assert [(node.tag, node.attrib) for node in original] == [
        (node.tag, node.attrib) for node in embedded
    ]
    badges = banner.findall(f".//{namespace}g[@data-badge='true']")
    assert len(badges) == 4
    for badge in badges:
        box, text = badge.find(namespace + "rect"), badge.find(namespace + "text")
        assert text.get("text-anchor") == "middle"
        assert text.get("dominant-baseline") == "central"
        assert float(text.get("x")) == float(box.get("x")) + float(box.get("width")) / 2
        assert float(text.get("y")) == float(box.get("y")) + float(box.get("height")) / 2


def test_quickstart_is_local_only_and_does_not_claim_pypi_publication():
    guide = (ROOT / "docs/quickstart.md").read_text()
    assert "Operator computer" in guide and "Robot computer" in guide
    assert "python -m pytest" in guide
    assert "b601-evaluate ros-mock" in guide
    for removed in ("CLOUDFLARE_", "b601-realtime", "[realtime]"):
        assert removed not in guide
        assert removed not in (ROOT / "README.md").read_text()
    assert "v" + version("inspect-robots-b601") in guide
    publishing = (ROOT / "docs/publishing.md").read_text()
    assert "pending publisher" in publishing
    assert "publish.yml" in publishing


def test_cloud_transport_is_not_part_of_the_shipped_source_or_metadata():
    from importlib.metadata import distribution

    project = distribution("inspect-robots-b601")
    assert "realtime" not in project.metadata.get_all("Provides-Extra", [])
    assert all(point.name != "b601-realtime" for point in project.entry_points)
    assert all("aiortc" not in requirement.lower() for requirement in project.requires)
    assert not (ROOT / "src/inspect_robots_b601/realtime.py").exists()
    assert not (ROOT / "src/inspect_robots_b601/_realtime/__init__.py").exists()
