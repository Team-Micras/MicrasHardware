"""Printability check: find overhangs of a part in its print orientation.

Faces whose downward normal is steeper than the limit (default 45 deg from vertical) and that are not on
the bed are reported, grouped, and rendered in red. Horizontal "ceiling" faces are bridges: fine when
both ends are supported and the span is short, which the report shows as the region size.

Usage: uv run tools/overhang.py <part> [--flip | --down nx,ny,nz] [--limit 45] [--png out.png]
  --flip   print upside down (rotate 180 deg about x)
  --down   print lying on the face whose outward normal is (nx, ny, nz)
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
    ap.add_argument("--down", default="", help="nx,ny,nz: rotate so this face normal points onto the bed")
    ap.add_argument("--png", default="")
    args = ap.parse_args()
    part = assembly.printed()[args.part]
    verts, tris = part.tessellate(0.05, 0.2)
    pts = np.array([tuple(v) for v in verts])
    real = pts.copy()
    if args.flip:
        pts[:, 1] *= -1
        pts[:, 2] *= -1
    if args.down:
        a = np.array([float(v) for v in args.down.split(",")])
        a /= np.linalg.norm(a)
        b = np.array([0.0, 0.0, -1.0])
        v, c = np.cross(a, b), float(np.dot(a, b))
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rot = np.eye(3) + vx + vx @ vx / (1 + c)
        pts = pts @ rot.T
    pts[:, 2] -= pts[:, 2].min()
    faces = np.hstack([np.full((len(tris), 1), 3), np.array(tris)]).ravel()
    mesh = pv.PolyData(pts, faces)
    # normals from the triangle winding (OCC tessellation is wound outwards); the per-face tessellation is
    # not watertight, so automatic orientation would be unreliable
    tri = np.array(tris)
    n = np.cross(pts[tri[:, 1]] - pts[tri[:, 0]], pts[tri[:, 2]] - pts[tri[:, 0]])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    centers = mesh.cell_centers().points
    areas = mesh.compute_cell_sizes(length=False, volume=False).cell_data["Area"]
    down = -n[:, 2]
    bad = (down > np.cos(np.radians(args.limit))) & (centers[:, 2] > 0.2)
    ceiling = bad & (down > 0.98)
    print(f"{args.part}: height {pts[:, 2].max():.1f} mm, overhang area {areas[bad].sum():.1f} mm2 "
          f"({areas[ceiling].sum():.1f} of it horizontal bridges)")
    # group overhang faces into connected regions (with their position in the robot frame)
    mesh.cell_data["real"] = real[np.array(tris)].mean(axis=1)
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
            rc = conn.cell_data["real"][m].mean(0)
            regions.append((a, cc[m].mean(0), hi - lo, rc))
        for a, c, ext, rc in sorted(regions, key=lambda r: -r[0])[:15]:
            print(f"  {a:7.1f} mm2 at print height {c[2]:5.1f}, extent {ext[0]:5.1f} x {ext[1]:5.1f} | "
                  f"robot frame ({rc[0]:6.1f},{rc[1]:6.1f},{rc[2]:5.1f})")
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
