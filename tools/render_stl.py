#!/usr/bin/env python3
"""Render the printed parts of the MAGPIE hand as figures for the docs.

A small software rasteriser (numpy + Pillow, nothing else) so the pictures in
the README can be regenerated from the STLs by anyone who edits a part:

    python3 tools/render_stl.py

Writes one PNG per part into docs/img/printed/ plus a labelled contact sheet of
the whole print set.  Backgrounds are transparent so the figures read on both
the light and the dark GitHub theme.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO = Path(__file__).resolve().parent.parent
STL_DIR = REPO / "hand" / "stls"
OUT_DIR = REPO / "docs" / "img" / "printed"

SS = 3  # supersampling factor
ALBEDO = np.array([0.50, 0.55, 0.63])  # mid slate: legible on white and on black
CREASE_DEG = 35.0  # smooth across facets flatter than this, keep edges sharp


# ---------------------------------------------------------------- STL loading

def load_stl(path: Path) -> np.ndarray:
    """Return an (n, 3, 3) array of triangle vertices, in millimetres."""
    raw = path.read_bytes()
    if raw[:5].lower() == b"solid" and b"facet normal" in raw[:512]:
        return _load_ascii_stl(raw)
    count = struct.unpack("<I", raw[80:84])[0]
    body = np.frombuffer(raw[84 : 84 + count * 50], dtype=np.uint8).reshape(count, 50)
    return np.frombuffer(body[:, 12:48].tobytes(), dtype="<f4").reshape(count, 3, 3).astype(np.float64)


def _load_ascii_stl(raw: bytes) -> np.ndarray:
    verts = [
        [float(x) for x in line.split()[1:4]]
        for line in raw.decode("utf-8", "replace").splitlines()
        if line.strip().startswith("vertex")
    ]
    return np.asarray(verts, dtype=np.float64).reshape(-1, 3, 3)


def vertex_normals(tris: np.ndarray) -> np.ndarray:
    """Per-corner normals, averaged over adjacent faces within the crease angle.

    Curved surfaces (bearing seats, fillets) come out smooth; the flat faces of
    a printed plate keep their hard edges.  The crease test is made against each
    *face's own* normal rather than against the averaged one — otherwise a large
    flat triangle that happens to touch a hole rim drags the rim's normals into
    its average and paints a comet-shaped streak across the face.
    """
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]
    face = np.cross(e1, e2)
    area = np.linalg.norm(face, axis=1, keepdims=True)
    face_n = face / np.maximum(area, 1e-12)

    # weld corners that sit on the same point (STL repeats every vertex)
    corners = tris.reshape(-1, 3)
    _, inverse = np.unique(np.round(corners, 4), axis=0, return_inverse=True)
    inverse = inverse.ravel()
    face_of_corner = np.repeat(np.arange(len(tris)), 3)

    order = np.argsort(inverse, kind="stable")
    bounds = np.searchsorted(inverse[order], np.arange(inverse.max() + 2))
    cos_crease = math.cos(math.radians(CREASE_DEG))

    out = np.repeat(face_n, 3, axis=0).copy()
    for lo, hi in zip(bounds[:-1], bounds[1:]):
        if hi - lo < 2:
            continue
        idx = order[lo:hi]
        faces = face_of_corner[idx]
        fn = face_n[faces]
        weight = area[faces, 0]
        blend = (fn @ fn.T >= cos_crease) * weight
        out[idx] = blend @ fn

    out /= np.maximum(np.linalg.norm(out, axis=1, keepdims=True), 1e-12)
    return out.reshape(-1, 3, 3)


# ------------------------------------------------------------------- geometry

def look_at(azimuth_deg: float, elevation_deg: float) -> np.ndarray:
    """World -> camera rotation for an orbit camera with +Z up."""
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    forward = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(up, forward)
    right /= np.linalg.norm(right)
    cam_up = np.cross(forward, right)
    return np.stack([right, cam_up, forward])  # rows: x_cam, y_cam, depth


@dataclass
class Framebuffer:
    """Colour + depth + normal buffers, shared by every part on a contact sheet."""

    width: int
    height: int

    def __post_init__(self):
        self.color = np.zeros((self.height, self.width, 3))
        self.depth = np.full((self.height, self.width), np.inf)
        self.normal = np.zeros((self.height, self.width, 3))
        self.mask = np.zeros((self.height, self.width), dtype=bool)


def rasterise(fb: Framebuffer, xy: np.ndarray, z: np.ndarray, normals: np.ndarray) -> None:
    """Z-buffered scanline fill of camera-space triangles already in pixels."""
    lo = np.floor(xy.min(axis=1)).astype(int)
    hi = np.ceil(xy.max(axis=1)).astype(int)
    on_screen = (hi[:, 0] >= 0) & (hi[:, 1] >= 0) & (lo[:, 0] < fb.width) & (lo[:, 1] < fb.height)

    for i in np.nonzero(on_screen)[0]:
        x0, x1 = max(lo[i, 0], 0), min(hi[i, 0] + 1, fb.width)
        y0, y1 = max(lo[i, 1], 0), min(hi[i, 1] + 1, fb.height)
        if x0 >= x1 or y0 >= y1:
            continue

        (ax, ay), (bx, by), (cx, cy) = xy[i]
        denom = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(denom) < 1e-9:
            continue

        px, py = np.meshgrid(np.arange(x0, x1) + 0.5, np.arange(y0, y1) + 0.5)
        w0 = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / denom
        w1 = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / denom
        w2 = 1.0 - w0 - w1
        inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        if not inside.any():
            continue

        zz = w0 * z[i, 0] + w1 * z[i, 1] + w2 * z[i, 2]
        window = fb.depth[y0:y1, x0:x1]
        write = inside & (zz < window)
        if not write.any():
            continue

        n = (w0[..., None] * normals[i, 0] + w1[..., None] * normals[i, 1] + w2[..., None] * normals[i, 2])
        n /= np.maximum(np.linalg.norm(n, axis=-1, keepdims=True), 1e-12)

        window[write] = zz[write]
        fb.normal[y0:y1, x0:x1][write] = n[write]
        fb.mask[y0:y1, x0:x1] |= write


def draw_part(
    fb: Framebuffer,
    tris: np.ndarray,
    rotation: np.ndarray,
    scale: float,
    centre_px: tuple[float, float],
    pivot: np.ndarray,
) -> None:
    """Project one mesh into the shared framebuffer with an orthographic camera."""
    normals = vertex_normals(tris)
    cam = (tris - pivot) @ rotation.T
    xy = np.empty(cam.shape[:2] + (2,))
    xy[..., 0] = centre_px[0] + cam[..., 0] * scale
    xy[..., 1] = centre_px[1] - cam[..., 1] * scale
    rasterise(fb, xy, cam[..., 2], normals @ rotation.T)


# ------------------------------------------------------------------- shading

def _shift(a: np.ndarray, dy: int, dx: int, fill: float) -> np.ndarray:
    out = np.full_like(a, fill)
    ys = slice(max(dy, 0), a.shape[0] + min(dy, 0))
    xs = slice(max(dx, 0), a.shape[1] + min(dx, 0))
    yd = slice(max(-dy, 0), a.shape[0] + min(-dy, 0))
    xd = slice(max(-dx, 0), a.shape[1] + min(-dx, 0))
    out[ys, xs] = a[yd, xd]
    return out


def ambient_occlusion(
    depth: np.ndarray, normal: np.ndarray, mask: np.ndarray, radius_px: int, world_per_px: float
) -> np.ndarray:
    """Cheap screen-space AO: how much of the neighbourhood stands in front.

    The comparison is against the *tangent plane* rather than against the pixel's
    own depth.  A flat plate seen at an angle has a depth gradient across it, so
    a plain depth test reads that gradient as occlusion and paints diagonal
    streaks across every flat face.
    """
    occluders = np.zeros(depth.shape)
    samples = 0
    finite = np.where(mask, depth, 0.0)
    nx, ny, nz = normal[..., 0], normal[..., 1], normal[..., 2]
    nz_safe = np.where(np.abs(nz) < 1e-3, np.sign(nz) * 1e-3 + 1e-9, nz)

    for r in (max(radius_px // 3, 1), max((2 * radius_px) // 3, 2), radius_px):
        bias = world_per_px * (1.5 + 0.05 * r)
        for dy, dx in ((r, 0), (-r, 0), (0, r), (0, -r), (r, r), (r, -r), (-r, r), (-r, -r)):
            # depth the surface would have at this offset if it just kept going
            expected = finite - (nx * (dx * world_per_px) - ny * (dy * world_per_px)) / nz_safe
            neighbour = _shift(finite, dy, dx, 0.0)
            valid = _shift(mask, dy, dx, False)
            occluders += np.where(valid & (neighbour < expected - bias), 1.0, 0.0)
            samples += 1
    return 1.0 - 0.6 * (occluders / max(samples, 1))


def shade(fb: Framebuffer, world_per_px: float, albedo: np.ndarray = ALBEDO) -> Image.Image:
    n = fb.normal
    depth = np.where(fb.mask, fb.depth, 0.0)

    key = np.array([-0.45, 0.55, -0.70]); key /= np.linalg.norm(key)
    fill = np.array([0.75, 0.10, -0.55]); fill /= np.linalg.norm(fill)
    rim = np.array([0.30, -0.85, 0.35]); rim /= np.linalg.norm(rim)

    n_key = np.clip((n * key).sum(-1), 0, 1)
    n_fill = np.clip((n * fill).sum(-1), 0, 1)
    n_rim = np.clip((n * rim).sum(-1), 0, 1) ** 2

    sky = 0.5 + 0.5 * n[..., 1]  # hemispheric ambient, brighter from above
    ao = ambient_occlusion(depth, n, fb.mask, max(int(5 * SS), 3), world_per_px)

    view = np.array([0.0, 0.0, -1.0])
    half = key + view
    half /= np.linalg.norm(half)
    spec = np.clip((n * half).sum(-1), 0, 1) ** 48

    light = (
        albedo * (0.24 * sky * ao)[..., None]
        + albedo * (0.62 * n_key * (0.35 + 0.65 * ao))[..., None]
        + albedo * np.array([0.85, 0.92, 1.0]) * (0.18 * n_fill)[..., None]
        + np.array([1.0, 0.93, 0.85]) * (0.20 * n_rim)[..., None]
        + 0.16 * spec[..., None]
    )

    # contact edge: darken where the surface turns away at a silhouette
    edge = np.zeros(depth.shape)
    for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        edge = np.maximum(edge, 1.0 - np.abs((n * _shift(n, dy, dx, 0.0)).sum(-1)))
    light *= (1.0 - 0.35 * np.clip(edge * 1.6, 0, 1))[..., None]

    srgb = np.clip(light, 0, 1) ** (1 / 2.2)
    rgba = np.zeros(depth.shape + (4,), dtype=np.uint8)
    rgba[..., :3] = (srgb * 255).round().astype(np.uint8)
    rgba[..., 3] = np.where(fb.mask, 255, 0)
    img = Image.fromarray(rgba, "RGBA")
    return img.resize((img.width // SS, img.height // SS), Image.LANCZOS)


# --------------------------------------------------------------------- parts

PARTS = [
    ("structure/base_top.STL", "Top base", 1),
    ("structure/base_bottom.STL", "Bottom base", 1),
    ("structure/cover_top.STL", "Top base cover", 1),
    ("structure/cover_bottom.STL", "Bottom base cover", 1),
    ("4-bar/crank.STL", "Servo crank", 2),
    ("4-bar/coupler.STL", "Servo coupler", 4),
    ("4-bar/rocker.STL", "Servo rocker", 2),
    ("4-bar/finger.STL", "Finger", 2),
]

VIEW = (38.0, 26.0)  # azimuth, elevation


def trim(img: Image.Image, pad_frac: float = 0.04) -> Image.Image:
    """Crop to what was actually drawn, so a thin part is not mostly margin."""
    box = img.getchannel("A").getbbox()
    if box is None:
        return img
    pad = int(max(box[2] - box[0], box[3] - box[1]) * pad_frac)
    return img.crop((box[0] - pad, box[1] - pad, box[2] + pad, box[3] + pad))


def render_part(tris: np.ndarray, size: int = 900, margin: float = 0.10) -> Image.Image:
    rot = look_at(*VIEW)
    pivot = (tris.reshape(-1, 3).min(axis=0) + tris.reshape(-1, 3).max(axis=0)) / 2
    cam = (tris.reshape(-1, 3) - pivot) @ rot.T
    extent = max(np.ptp(cam[:, 0]), np.ptp(cam[:, 1]))
    scale = size * SS * (1 - 2 * margin) / extent
    fb = Framebuffer(size * SS, size * SS)
    draw_part(fb, tris, rot, scale, (fb.width / 2, fb.height / 2), pivot)
    return shade(fb, 1.0 / scale)


def render_sheet(meshes: list[tuple[str, int, np.ndarray]], cols: int = 4, cell: int = 620) -> Image.Image:
    """All parts at one common scale, so their real sizes compare."""
    rot = look_at(*VIEW)
    rows = math.ceil(len(meshes) / cols)
    label_px = 104
    fb = Framebuffer(cols * cell * SS, rows * (cell + label_px) * SS)

    widest = 0.0
    for _, _, tris in meshes:
        pivot = (tris.reshape(-1, 3).min(axis=0) + tris.reshape(-1, 3).max(axis=0)) / 2
        cam = (tris.reshape(-1, 3) - pivot) @ rot.T
        widest = max(widest, np.ptp(cam[:, 0]), np.ptp(cam[:, 1]))
    scale = cell * SS * 0.80 / widest

    for i, (_, _, tris) in enumerate(meshes):
        col, row = i % cols, i // cols
        cx = (col + 0.5) * cell * SS
        cy = (row * (cell + label_px) + 0.5 * cell) * SS
        pivot = (tris.reshape(-1, 3).min(axis=0) + tris.reshape(-1, 3).max(axis=0)) / 2
        draw_part(fb, tris, rot, scale, (cx, cy), pivot)

    img = shade(fb, 1.0 / scale)
    draw = ImageDraw.Draw(img)
    name_font = _font(40)
    dim_font = _font(31)
    for i, (label, qty, tris) in enumerate(meshes):
        col, row = i % cols, i // cols
        cx = int((col + 0.5) * cell)
        base = int(row * (cell + label_px) + cell)
        size_mm = np.ptp(tris.reshape(-1, 3), axis=0)
        # mid grey reads on either theme
        draw.text((cx, base), f"{label}  ×{qty}", fill=(138, 145, 156, 255), font=name_font, anchor="ma")
        draw.text(
            (cx, base + 48),
            "%.0f × %.0f × %.0f mm" % tuple(sorted(size_mm, reverse=True)),
            fill=(122, 129, 140, 255),
            font=dim_font,
            anchor="ma",
        )
    return img


def _font(size: int) -> ImageFont.FreeTypeFont:
    for candidate in (
        "/System/Library/Fonts/Supplemental/Helvetica.ttc",
        "/System/Library/Fonts/HelveticaNeue.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ):
        try:
            return ImageFont.truetype(candidate, size)
        except OSError:
            continue
    return ImageFont.load_default()


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    meshes = []
    for rel, label, qty in PARTS:
        tris = load_stl(STL_DIR / rel)
        meshes.append((label, qty, tris))
        out = OUT_DIR / (Path(rel).stem.lower() + ".png")
        trim(render_part(tris)).save(out)
        print(f"{out.relative_to(REPO)}  ({len(tris)} triangles)")

    sheet = OUT_DIR.parent / "print-set.png"
    render_sheet(meshes).save(sheet)
    print(f"{sheet.relative_to(REPO)}")


if __name__ == "__main__":
    main()
