# Banner artwork and names

`banner.svg` is the editable source; `banner.png` is its raster rendering. The
robot arm is an original schematic illustration, not a Seeed CAD model. The
RoboCurve and Seeed Studio lettering and decorative motifs are original banner
graphics, not copies of their official logo assets.

The Hugging Face icon is the **official, unmodified brand asset** downloaded
from <https://huggingface.co/datasets/huggingface/brand-assets/resolve/main/hf-logo.svg>,
as provided by <https://huggingface.co/brand>. `huggingface-logo.svg` preserves
the original file; the banner embeds its original paths/colors at a smaller
viewport. Its use identifies the optional model ecosystem, not a bundled model
integration, sponsorship or endorsement. Hugging Face's trademark remains its
owner's property.

Original asset SHA-256: `942cad1ccda905ac5a659dfd2d78b344fccfb84a8a3ac3721e08f488205638a0`.

The small Inspect Robots inspector icon is adapted from
[`robocurve/inspect-robots/docs/assets/inspect-robots-logo.svg`](https://github.com/robocurve/inspect-robots/blob/main/docs/assets/inspect-robots-logo.svg),
distributed under the upstream MIT license reproduced below. Its blink
animation is omitted in this static banner.

Inspect Robots/RoboCurve, Seeed Studio and Hugging Face names and marks identify the upstream
framework and hardware. They belong to their respective owners. This repository
is an independent community integration, not an official product, sponsorship or
endorsement by any of these organizations.

## Inspect Robots icon license

MIT License

Copyright (c) 2026 robocurve

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

To render locally with the optional development artwork dependencies:

```bash
python -m pip install -e '.[art]'
python -m scripts.render_banner
```
