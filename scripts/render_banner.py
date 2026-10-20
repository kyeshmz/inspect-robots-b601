"""Embed the unmodified official HF artwork and render the portable README banner."""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
START = "<!-- BEGIN OFFICIAL HUGGING FACE LOGO -->"
END = "<!-- END OFFICIAL HUGGING FACE LOGO -->"


def main() -> None:
    import cairosvg

    ET.register_namespace("", "http://www.w3.org/2000/svg")
    logo = ET.fromstring((ROOT / "assets/huggingface-logo.svg").read_bytes())
    logo.attrib.update(id="hugging-face-logo", x="1207", y="24", width="78", height="78")
    banner = ROOT / "assets/banner.svg"
    content, count = re.subn(
        re.escape(START) + r".*?" + re.escape(END),
        lambda _: START + "\n    " + ET.tostring(logo, encoding="unicode") + "\n    " + END,
        banner.read_text(),
        flags=re.DOTALL,
    )
    if count != 1:
        raise ValueError("The banner must contain exactly one managed official-logo block")
    banner.write_text(content)
    cairosvg.svg2png(bytestring=content.encode(), write_to=str(ROOT / "assets/banner.png"))


if __name__ == "__main__":
    main()
