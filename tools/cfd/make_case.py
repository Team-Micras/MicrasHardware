"""Write an OpenFOAM (v2512, opencfd/openfoam-default) case for one suction-fan impeller: steady RANS, MRF.

    tools/capped.sh uv run tools/cfd/make_case.py --impeller radial --mesh medium [--suction 1000] [--rpm 42600]

Inputs from build/cfd/ (exported by tools/export.py, not regenerated here): impeller_<name>.stl, fan_mount.stl,
fan_motor.stl, geometry.json (mm, fan axis at x = y = 0, z up from the floor). The case is written in metres to
build/cfd/runs/<impeller>_<mesh>/ and meshed/solved by tools/cfd/run_case.sh inside docker (tools/cfd/cfd.sh).

Domain (axisymmetric shell, generated here as a closed STL with patches):
- plenum: the sealed cavity under the board, a Ø40 x 1.2 disk (floor z 0, board underside z 1.2) fed through its rim
  ("inlet") at a fixed total pressure -suction (the skirt leak comes in from the board's edge);
- the Ø15 hole through the 1.04 mm board, then the open air above the board top: Ø80, up to z 42 ("ambient", fixed
  total pressure 0 in, static 0 out);
- removed from it: the impeller (rotating: MRF zone), the fan mount (collar, plate and arm) and the motor.
Mesh: snappyHexMesh on a uniform background of size dx (coarse 3.0, medium 2.4, fine 1.92 mm: ratio 1.25), with the
face/neck seal gaps at level 5 (dx/32: 3.2 / 4 / 5 cells across the 0.3 mm gaps) and the impeller at level 4.
"""

import argparse
import json
import math
import re
import shutil
import struct
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "build" / "cfd"
TEMPLATE = Path(__file__).resolve().parent / "template"
MESHES = {"coarse": 3.0, "medium": 2.4, "fine": 1.92}  # background cell (mm); every level halves it
RHO, NU = 1.2, 1.5e-5

# domain (mm)
R_PLENUM, R_AMB, Z_TOP = 20.0, 40.0, 42.0


def read_stl(path):
    d = path.read_bytes()
    n = struct.unpack("<I", d[80:84])[0]
    a = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return a["v"].astype(float)


def write_stl_ascii(path, solids):
    """solids: {name: (n, 3, 3) triangles in metres}."""
    with open(path, "w") as f:
        for name, tris in solids.items():
            f.write(f"solid {name}\n")
            for t in tris:
                nrm = np.cross(t[1] - t[0], t[2] - t[0])
                ln = np.linalg.norm(nrm)
                nrm = nrm / ln if ln > 0 else nrm
                f.write(f" facet normal {nrm[0]:.6e} {nrm[1]:.6e} {nrm[2]:.6e}\n  outer loop\n")
                for p in t:
                    f.write(f"   vertex {p[0]:.9e} {p[1]:.9e} {p[2]:.9e}\n")
                f.write("  endloop\n endfacet\n")
            f.write(f"endsolid {name}\n")


def revolve(profile, nseg):
    """Triangles of the surface swept by a polyline [(r, z), ...] about z (outward normals for a profile running
    counter-clockwise round the fluid in the (r, z) half-plane)."""
    th = np.linspace(0, 2 * math.pi, nseg + 1)
    tris = []
    for (r0, z0), (r1, z1) in zip(profile[:-1], profile[1:]):
        for a, b in zip(th[:-1], th[1:]):
            p00 = (r0 * math.cos(a), r0 * math.sin(a), z0)
            p01 = (r0 * math.cos(b), r0 * math.sin(b), z0)
            p10 = (r1 * math.cos(a), r1 * math.sin(a), z1)
            p11 = (r1 * math.cos(b), r1 * math.sin(b), z1)
            if r0 > 1e-12:
                tris.append((p00, p01, p10))
            if r1 > 1e-12:
                tris.append((p01, p11, p10))
    return np.array(tris)


def domain_surfaces(g, nseg=360):
    """The fluid domain's outer shell (mm), split into patches; normals point out of the fluid."""
    zb, zt, rh = g["board_bottom_z"], g["board_top_z"], g["board_hole_d"] / 2
    # profile runs counter-clockwise round the fluid seen in the (r, z) plane, so the normal (dz, -dr) points out
    parts = {
        "floor": [(0.0, 0.0), (R_PLENUM, 0.0)],
        "inlet": [(R_PLENUM, 0.0), (R_PLENUM, zb)],
        "board": [(R_PLENUM, zb), (rh, zb), (rh, zt), (R_AMB, zt)],
        "ambient": [(R_AMB, zt), (R_AMB, Z_TOP), (0.0, Z_TOP)],
    }
    return {k: revolve(prof, nseg) for k, prof in parts.items()}


def ring_solid(r_in, r_out, z0, z1, nseg=180):
    """Closed annular ring (mm) used as a refinement region."""
    prof = [(r_in, z1), (r_in, z0), (r_out, z0), (r_out, z1), (r_in, z1)]
    return revolve(prof, nseg)


def fmt_vec(v):
    return "(" + " ".join(f"{x:.9g}" for x in v) + ")"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--impeller", required=True, choices=["radial", "inducer", "backward"])
    ap.add_argument("--mesh", default="medium", choices=list(MESHES))
    ap.add_argument("--suction", type=float, default=None, help="Pa; default from geometry.json")
    ap.add_argument("--rpm", type=float, default=None, help="default from geometry.json")
    ap.add_argument("--np", type=int, default=12, help="MPI ranks (12 measured fastest on the 155H)")
    ap.add_argument("--name", default=None)
    args = ap.parse_args()

    g = json.loads((SRC / "geometry.json").read_text())
    h = g["heights"]
    rpm = args.rpm or g["design_rpm"]
    suction = args.suction if args.suction is not None else g["design_suction_pa"]
    omega = rpm * 2 * math.pi / 60  # counter-clockwise seen from +z: +z axis
    dx = MESHES[args.mesh]
    name = args.name or f"{args.impeller}_{args.mesh}"
    case = SRC / "runs" / name
    if case.exists():
        raise SystemExit(f"{case} exists; remove it first")
    shutil.copytree(TEMPLATE, case)
    tri = case / "constant" / "triSurface"
    tri.mkdir(parents=True, exist_ok=True)

    mm = 1e-3
    # --- surfaces (metres)
    write_stl_ascii(tri / "domain.stl", {k: v * mm for k, v in domain_surfaces(g).items()})
    write_stl_ascii(tri / "impeller.stl", {"impeller": read_stl(SRC / f"impeller_{args.impeller}.stl") * mm})
    write_stl_ascii(tri / "mount.stl", {"mount": read_stl(SRC / "fan_mount.stl") * mm})
    write_stl_ascii(tri / "motor.stl", {"motor": read_stl(SRC / "fan_motor.stl") * mm})
    rh = g["board_hole_d"] / 2
    neck_in = g["neck_id"] / 2
    zt, zb = g["board_top_z"], g["board_bottom_z"]
    # seal region: neck gap (r 7.2..7.5 in the hole), face gap (r 7.2..11, 0.3 high) and the tip corner
    write_stl_ascii(tri / "sealring.stl", {"sealring": ring_solid(neck_in - 0.15, g["impeller_d2"] / 2 + 0.4,
                                                                  zb + 0.02, h["shroud"] + 0.08) * mm})

    # --- background mesh: uniform dx; board top on a grid plane (+ a small offset so no surface lies on faces)
    eps = 0.0037
    nxy = math.ceil(2 * (R_AMB + 0.6) / dx)
    nxy += nxy % 2
    half = nxy * dx / 2
    k_below = math.ceil((zt + 0.6) / dx)
    z0 = zt + eps - k_below * dx
    nz = math.ceil((Z_TOP + 0.6 - z0) / dx)
    z1 = z0 + nz * dx
    off = 0.0011  # keep the axis off grid faces
    lo = (-half + off, -half + off, z0)
    hi = (half + off, half + off, z1)

    zr0, zr1 = 1.25, 7.3  # MRF zone A (whole impeller below the backplate top 6.94; the mount arm stays above 7.6)
    subs = {
        "NPROCS": str(args.np),
        "XMIN": f"{lo[0]*mm:.9g}", "YMIN": f"{lo[1]*mm:.9g}", "ZMIN": f"{lo[2]*mm:.9g}",
        "XMAX": f"{hi[0]*mm:.9g}", "YMAX": f"{hi[1]*mm:.9g}", "ZMAX": f"{hi[2]*mm:.9g}",
        "NX": str(nxy), "NY": str(nxy), "NZ": str(nz),
        "LOCATION": fmt_vec(np.array((-25.0, 12.3, 25.0)) * mm),
        "ROTOR_P1": fmt_vec(np.array((0, 0, zb - 0.6)) * mm),
        "ROTOR_P2": fmt_vec(np.array((0, 0, h["back_tip"] + 1.2)) * mm),
        "ROTOR_R": f"{(g['impeller_d2']/2 + 0.6)*mm:.9g}",
        "NEAR_P1": fmt_vec(np.array((0, 0, -0.5)) * mm),
        "NEAR_P2": fmt_vec(np.array((0, 0, h["plate"] + 1.5)) * mm),
        "NEAR_R": f"{16.0*mm:.9g}",
        "JET_P1": fmt_vec(np.array((0, 0, -0.5)) * mm),
        "JET_P2": fmt_vec(np.array((0, 0, 12.0)) * mm),
        "JET_R": f"{27.0*mm:.9g}",
        "PLATE_P1": fmt_vec(np.array((0, 0, h["plate"] - 0.4)) * mm),
        "PLATE_P2": fmt_vec(np.array((0, 0, h["motor"] + 0.2)) * mm),
        "PLATE_R": f"{4.2*mm:.9g}",
        "ZONEA_P1": fmt_vec(np.array((0, 0, zr0)) * mm),
        "ZONEA_P2": fmt_vec(np.array((0, 0, zr1)) * mm),
        "ZONEA_R": f"{(g['impeller_d2']/2 + 0.5)*mm:.9g}",
        "ZONEB_P1": fmt_vec(np.array((0, 0, zr1 - 0.1)) * mm),
        "ZONEB_P2": fmt_vec(np.array((0, 0, h["motor"] - 0.15)) * mm),
        "ZONEB_R": f"{5.5*mm:.9g}",
        # flow-measurement disks in the hole (z 1.8: below the neck's lip at 2.24, above its bottom at 1.34)
        "MEAS_ORIGIN": fmt_vec(np.array((0, 0, 1.8)) * mm),
        "EYE_R": f"{(neck_in + g['neck_od']/2) / 2 * mm:.9g}",
        "HOLE_R": f"{(rh + 0.3)*mm:.9g}",
        "OMEGA": f"{omega:.6f}",
        "RPM": f"{rpm:.1f}",
        "P0_INLET": f"{-suction / RHO:.6f}",
        "NU": f"{NU:.6g}",
        "PROBE_PLENUM_A": fmt_vec(np.array((10.0, 0.0, 0.6)) * mm),
        "PROBE_PLENUM_B": fmt_vec(np.array((0.0, -15.0, 0.6)) * mm),
        "PROBE_HOLE": fmt_vec(np.array((0.0, 0.0, 1.2)) * mm),
        "PROBE_EYE": fmt_vec(np.array((0.0, 3.5, 2.5)) * mm),
        "PROBE_GAP": fmt_vec(np.array((9.0, 0.0, (zt + h['shroud']) / 2)) * mm),
        "GAPLINE_A0": fmt_vec(np.array((9.0, 0.3, zt - 0.05)) * mm),
        "GAPLINE_A1": fmt_vec(np.array((9.0, 0.3, h["shroud"] + 0.05)) * mm),
        "GAPLINE_B0": fmt_vec(np.array((g["neck_od"] / 2 - 0.05, 0.3, 1.8)) * mm),
        "GAPLINE_B1": fmt_vec(np.array((rh + 0.05, 0.3, 1.8)) * mm),
        "GAPLINE_C0": fmt_vec(np.array((10.6, 0.3, zt - 0.05)) * mm),
        "GAPLINE_C1": fmt_vec(np.array((10.6, 0.3, h["shroud"] + 0.05)) * mm),
        "PROBE_TIP": fmt_vec(np.array((0.0, 11.6, 4.0)) * mm),
    }
    for f in case.rglob("*"):
        if f.is_file() and f.suffix not in (".stl",):
            txt = f.read_text()
            if "@@" in txt:
                for k, v in subs.items():
                    txt = txt.replace(f"@@{k}@@", v)
                if re.search(r"@@[A-Z_0-9]+@@", txt):
                    raise SystemExit(f"unsubstituted key in {f}")
                f.write_text(txt)
    meta = dict(impeller=args.impeller, mesh=args.mesh, dx_mm=dx, rpm=rpm, omega=omega, suction_pa=suction,
                rho=RHO, nu=NU, np=args.np)
    (case / "case.json").write_text(json.dumps(meta, indent=1))
    print(case)


if __name__ == "__main__":
    main()
