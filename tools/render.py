"""Render the layout to a PNG with six views (for quick visual checks without a viewer).

Usage: uv run tools/render.py out.png [--no-board] [--hide label1,label2] [--set layout.blocks=solid]
"""
import argparse
import colorsys
import sys
from pathlib import Path

import numpy as np
import pyvista as pv

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from micras import layout, params  # noqa: E402

VIEWS = {
    "iso": ((1, 1, 1), (0, 0, 1)), "top": ((0, 0, 1), (1, 0, 0)), "side (left)": ((0, 1, 0), (0, 0, 1)),
    "front": ((1, 0, 0), (0, 0, 1)), "rear iso": ((-1, -1, 0.8), (0, 0, 1)), "bottom": ((0, 0, -1), (1, 0, 0)),
}


def mesh(shape, tol=0.05):
    v, t = shape.tessellate(tol, 0.3)
    if not t:
        return None
    faces = np.hstack([np.full((len(t), 1), 3), np.array(t)]).ravel()
    return pv.PolyData(np.array([tuple(p) for p in v]), faces)


BOARD_STL = Path(__file__).resolve().parents[1] / "ref/board.stl"


def board_mesh():
    """Populated board mesh from ref/board.stl (tools/export_board.py), in the robot frame."""
    m = pv.read(BOARD_STL)
    # kicad-cli --grid-origin: robot (x, y) = (Y - 67.1112, 64.1608 - X); STEP/STL Z=0 is the board bottom
    pts = np.asarray(m.points, dtype=float)
    m.points = np.column_stack([pts[:, 1] - 67.1112, 64.1608 - pts[:, 0], pts[:, 2] + layout.P.board.bottom_z])
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--no-board", action="store_true")
    ap.add_argument("--hide", default="")
    ap.add_argument("--extra", default="", help="module:function returning {label: shape}")
    ap.add_argument("--view", default="", help="render one large view instead of six: " + ", ".join(VIEWS))
    ap.add_argument("--set", action="append", default=[], help="override a parameter, e.g. layout.blocks=solid")
    args = ap.parse_args()
    for kv in args.set:
        params.override(kv)
    from micras import assembly
    parts = assembly.bought()  # bought parts, including the fan motor
    if args.extra:
        mod, fn = args.extra.split(":")
        parts.update(getattr(__import__(mod, fromlist=[fn]), fn)())
    hide = [h for h in args.hide.split(",") if h]
    items = [(k, v) for k, v in parts.items() if not any(h in k for h in hide)]
    meshes = [(k, mesh(v), getattr(v, "color", None)) for k, v in items]
    if not args.no_board:
        meshes.append(("brd:board", board_mesh(), None))
    pv.OFF_SCREEN = True
    views = {args.view: VIEWS[args.view]} if args.view else VIEWS
    single = len(views) == 1
    pl = pv.Plotter(off_screen=True, shape=(1, 1) if single else (2, 3), window_size=(1600, 1200) if single else (1800, 1100))
    if single:
        pl.enable_anti_aliasing("ssaa")
    for i, (name, (d, up)) in enumerate(views.items()):
        if not single:
            pl.subplot(i // 3, i % 3)
            pl.add_text(name, font_size=10)
        for j, (k, m, c) in enumerate(meshes):
            if m is None:
                continue
            if k.startswith("brd:"):
                col = (0.45, 0.62, 0.48)
            else:
                col = tuple(c) [:3] if c is not None else colorsys.hsv_to_rgb((j * 0.137) % 1, 0.55, 0.95)
            pl.add_mesh(m, color=col, smooth_shading=single, specular=0.3 if single else 0.0)
        pl.camera.focal_point = (0, 0, 10)
        pl.camera.position = tuple(np.array((0, 0, 10)) + np.array(d) * 300)
        pl.camera.up = up
        pl.reset_camera()
        pl.camera.zoom(1.3)
    pl.screenshot(args.out)


if __name__ == "__main__":
    main()
