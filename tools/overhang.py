"""Printability check: find overhangs of a part in its print orientation.

Faces whose downward normal is steeper than the limit (default 45 deg from vertical) and that are not on
the bed are reported, grouped, and rendered in red. Horizontal "ceiling" faces are bridges: fine when
both ends are supported and the span is short, which the report shows as the region size.

Usage: uv run tools/overhang.py <part> [--flip] [--limit 45] [--png out.png]
  --flip   print upside down (rotate 180 deg about x)
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pyvista as pv

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from micras import assembly  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("part")
    ap.add_argument("--flip", action="store_true")
    ap.add_argument("--limit", type=float, default=45.0)
    ap.add_argument("--png", default="")
    args = ap.parse_args()
    part = assembly.printed()[args.part]
    verts, tris = part.tessellate(0.05, 0.2)
    pts = np.array([tuple(v) for v in verts])
    if args.flip:
        pts[:, 1] *= -1
        pts[:, 2] *= -1
    pts[:, 2] -= pts[:, 2].min()
    faces = np.hstack([np.full((len(tris), 1), 3), np.array(tris)]).ravel()
    mesh = pv.PolyData(pts, faces).compute_normals(cell_normals=True, point_normals=False, auto_orient_normals=True)
    n = mesh.cell_data["Normals"]
    centers = mesh.cell_centers().points
    areas = mesh.compute_cell_sizes(length=False, volume=False).cell_data["Area"]
    down = -n[:, 2]
    bad = (down > np.cos(np.radians(args.limit))) & (centers[:, 2] > 0.2)
    ceiling = bad & (down > 0.98)
    print(f"{args.part}: height {pts[:, 2].max():.1f} mm, overhang area {areas[bad].sum():.1f} mm2 "
          f"({areas[ceiling].sum():.1f} of it horizontal bridges)")
    # group overhang faces into connected regions
    sub = mesh.extract_cells(np.where(bad)[0])
    if sub.n_cells:
        conn = sub.connectivity()
        ids = conn.cell_data["RegionId"]
        sizes = conn.compute_cell_sizes(length=False, volume=False).cell_data["Area"]
        cc = conn.cell_centers().points
        regions = []
        for rid in np.unique(ids):
            m = ids == rid
            a = sizes[m].sum()
            if a < 0.5:
                continue
            lo, hi = cc[m].min(0), cc[m].max(0)
            regions.append((a, cc[m].mean(0), hi - lo))
        for a, c, ext in sorted(regions, key=lambda r: -r[0])[:15]:
            print(f"  {a:7.1f} mm2 at print ({c[0]:6.1f},{c[1]:6.1f},{c[2]:5.1f})  extent {ext[0]:5.1f} x {ext[1]:5.1f}")
    if args.png:
        pv.OFF_SCREEN = True
        pl = pv.Plotter(off_screen=True, shape=(1, 2), window_size=(1800, 900))
        good = mesh.extract_cells(np.where(~bad)[0])
        for i, view in enumerate([(1, -1.2, -0.8), (-1, 1.2, -0.8)]):
            pl.subplot(0, i)
            pl.add_mesh(good, color="#bbbbbb", lighting=False, show_edges=True, edge_color="#777777", line_width=0.3)
            if bad.any():
                pl.add_mesh(mesh.extract_cells(np.where(bad)[0]), color="#e02020", lighting=False)
            pl.view_vector(view, viewup=(0, 0, 1))
            pl.reset_camera()
            pl.add_text("print orientation, seen from below: red = overhang", font_size=9)
        pl.screenshot(args.png)


if __name__ == "__main__":
    main()
