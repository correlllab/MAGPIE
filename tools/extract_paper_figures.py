#!/usr/bin/env python3
"""Cut the figures used in the docs out of the paper PDF that ships in docs/paper.

Every picture in the README that shows the real hardware comes from the paper,
so it is extracted here rather than pasted in by hand — the provenance of each
one is a line of code you can read.

    python3 -m pip install pymupdf pillow numpy scipy
    python3 tools/extract_paper_figures.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pymupdf
from PIL import Image, ImageDraw
from scipy import ndimage

Image.MAX_IMAGE_PIXELS = None

REPO = Path(__file__).resolve().parent.parent
PDF = REPO / "docs" / "paper" / "2402.06018-magpie.pdf"
OUT = REPO / "docs" / "img"

# (page, xref) of each embedded image, and what the paper calls it
FIGURES = {
    "fig1": (0, 33),  # the hand over the block-stacking scene
    "fig2": (1, 81),  # torque diagram of the four-bar
    "fig3": (2, 82),  # CAD: top, bottom, exploded
    "fig4": (3, 119),  # force/aperture traces on the mustard bottle
    "fig5": (3, 121),  # Siemens gear assembly
    "fig6": (3, 129),  # replanned tower build
}


def extract(doc: pymupdf.Document, page: int, xref: int) -> Image.Image:
    data = doc.extract_image(xref)
    if data["width"] * data["height"] == 0:  # pragma: no cover - defensive
        raise ValueError(f"empty image at xref {xref}")
    import io

    return Image.open(io.BytesIO(data["image"])).convert("RGB")


def drop_black_background(img: Image.Image, tolerance: int = 40) -> Image.Image:
    """Make the render background transparent, keeping black *parts* opaque.

    Filled from the border rather than by thresholding the whole image: the
    servos in the CAD views are black too, and a plain threshold eats them.
    The cut-outs in the plates are background as well but the border fill can
    not reach them, so they are cleared in a second pass — a pocket of black
    ringed by saturated plastic is a hole you are looking through, while black
    surrounded by grey is a servo.
    """
    rgba = img.convert("RGBA")
    marker = (255, 0, 255, 0)
    w, h = rgba.size
    for seed in ((0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1), (w // 2, 0), (w // 2, h - 1)):
        if sum(rgba.getpixel(seed)[:3]) <= tolerance * 3:
            ImageDraw.floodfill(rgba, seed, marker, thresh=tolerance)

    a = np.array(rgba)
    a[..., 3] = np.where((a[..., :3] == marker[:3]).all(-1), 0, 255)
    _clear_enclosed_background(a)
    return Image.fromarray(a, "RGBA")


def _clear_enclosed_background(a: np.ndarray, ring_saturation: int = 55) -> None:
    """Clear black pockets ringed by saturated colour (holes through a part)."""
    rgb = a[..., :3].astype(int)
    saturation = rgb.max(-1) - rgb.min(-1)
    black = (a[..., 3] > 0) & (rgb.max(-1) <= 14)
    labels, count = ndimage.label(black)
    if not count:
        return
    pad = 4
    for index, window in enumerate(ndimage.find_objects(labels), start=1):
        if window is None:
            continue
        # work in the blob's own neighbourhood; whole-image masks per component
        # turn this into a minute of numpy for a picture
        ys, xs = window
        ys = slice(max(ys.start - pad, 0), min(ys.stop + pad, a.shape[0]))
        xs = slice(max(xs.start - pad, 0), min(xs.stop + pad, a.shape[1]))
        blob = labels[ys, xs] == index
        ring = ndimage.binary_dilation(blob, iterations=3) & ~blob & (a[ys, xs, 3] > 0)
        if ring.any() and saturation[ys, xs][ring].mean() >= ring_saturation:
            a[ys, xs, 3][blob] = 0


def save(img: Image.Image, name: str, max_width: int, quality: int = 88) -> None:
    if img.width > max_width:
        img = img.resize((max_width, round(img.height * max_width / img.width)), Image.LANCZOS)
    path = OUT / name
    if path.suffix == ".jpg":
        img.convert("RGB").save(path, quality=quality, optimize=True, progressive=True)
    else:
        img.save(path, optimize=True)
    print(f"{path.relative_to(REPO)}  {img.width}×{img.height}  {path.stat().st_size // 1024} KB")


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    doc = pymupdf.open(PDF)
    figs = {name: extract(doc, page, xref) for name, (page, xref) in FIGURES.items()}

    save(figs["fig1"], "hero.jpg", 1200)
    save(figs["fig2"], "linkage-torque.png", 520)

    cad = drop_black_background(figs["fig3"])
    save(cad, "cad-views.png", 1600)
    # the exploded view alone, for the assembly guide
    save(cad.crop((int(cad.width * 0.50), 0, cad.width, cad.height)), "exploded.png", 900)

    save(figs["fig4"], "force-trace.jpg", 1600)
    save(figs["fig5"], "siemens-assembly.jpg", 1800)
    save(figs["fig6"], "replanning.jpg", 1800)


if __name__ == "__main__":
    main()
