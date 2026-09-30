"""Structural check of the printed parts: linear FEA under rough worst-case loads, to flag anything not viable.

Each part is meshed with gmsh (quadratic tets) and solved with scikit-fem + pyamg. Loads come from simple physics
(robot 89 g):
  crash  1 m/s into a wall through the TPU bumper (~0.5 mm travel): 100 g deceleration
  drop   30 cm onto the wheels (2.4 m/s, ~1 mm tire squash): 300 g
  plus handling pushes, the impeller's spin and the fan mount's clamp.
Parts carry the inertia of what they hold as forces on their seats, and their own mass as a body force.

The stress reported is von Mises at the 99.9 % volume percentile (the raw peak sits in sharp corners, where
linear FEA does not converge), against a design strength already knocked down for printing:
  resin 35 MPa (standard / ABS-like, UTS 35-50), PETG 35 MPa along its layers (0.75 x 47 yield) and 12 MPa of
  tension across them (0.7 x 18 interlayer adhesion; checked with each part's print orientation), TPU 8.6 MPa.
Short events need a safety factor of 2, sustained loads 3 (resin and PETG creep). A FLAG means "look here":
the loads are estimates.

Usage: uv run tools/fea.py [part ...] [--h 0.5] [--png]
  part   part names or prefixes (default: every printed part, one of each mirrored pair)
  --h    target element size in mm
  --dry  only mesh each part and check that every support and load finds its faces (seconds, no solving)
  --png  also render each case (build/fea/<part>_<case>.png, stress on the deformed shape, x10)
"""
import argparse
import resource
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

import gmsh
import numpy as np
import pyamg
from skfem import Basis, ElementTetP2, ElementVector, FacetBasis, Functional, LinearForm, MeshTet, asm, condense
from skfem.helpers import eye, sym_grad, trace
from skfem.models.elasticity import lame_parameters, linear_elasticity

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from build123d import Pos, export_step  # noqa: E402

from micras import assembly, drive, fan, frame, front  # noqa: E402
from micras.params import P  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "build/fea"
G = 9.81
CRASH, DROP = 100 * G, 300 * G  # m/s2
# E MPa, Poisson, design strength MPa, density g/mm3
MAT = {"resin": (1800, 0.38, 35, 1.15e-3), "petg": (1500, 0.39, 35, 1.27e-3), "tpu": (26, 0.45, 8.6, 1.21e-3)}
LAYER = {"petg": 12.0}  # tension across the layers, MPa (FDM)
# build direction (up in the printer) of the FDM parts, in the robot frame (assembly.MATERIALS)
BUILD = {"basket": np.array((0.0, 0, -1))}  # printed upside down


@dataclass
class Case:
    name: str
    fixed: callable  # sel(c, n) -> bool array over boundary facets (centres c, outward normals n: (3, nf))
    loads: list = field(default_factory=list)  # [(sel, (Fx, Fy, Fz) total N, spread evenly over the area)]
    accel: tuple = (0, 0, 0)  # the part's acceleration, m/s2 (its own mass then loads it with -rho*a)
    spin: tuple = None  # (x, y, rad/s) about a vertical axis: centrifugal body force
    squeeze: tuple = None  # (sel, x, y, mm): an inward pressure on those faces, scaled until they move that far
    # in towards a vertical axis on average (a clamp fit; a pressure, not a forced displacement, which would put
    # a false stress peak at the edge of the faces)
    sf: float = 2.0  # safety factor needed


# ---- mesh and solver ------------------------------------------------------------------------

def mesh(shape, h):
    """Quadratic-ready linear tets of a build123d shape. Through STEP (gmsh reads build123d's BREP less
    reliably); if a setting fails on a part, the next one is tried."""
    with tempfile.TemporaryDirectory() as tmp:
        path = f"{tmp}/part.step"
        export_step(shape, path)
        for heal, algo in ((0.02, 10), (0, 10), (0.02, 1)):  # 10 = HXT, 1 = Delaunay
            gmsh.initialize(["-nt", "4"])
            gmsh.option.setNumber("General.Terminal", 0)
            try:
                gmsh.model.occ.importShapes(path)
                if heal:
                    gmsh.model.occ.healShapes(tolerance=heal)
                gmsh.model.occ.synchronize()
                for k, v in {"MeshSizeMax": h, "MeshSizeMin": h / 5, "MeshSizeFromCurvature": 12,
                             "Algorithm3D": algo}.items():
                    gmsh.option.setNumber("Mesh." + k, v)
                gmsh.model.mesh.generate(3)
                tags, xyz, _ = gmsh.model.mesh.getNodes()
                et, _, en = gmsh.model.mesh.getElements(3)
                break
            except Exception as e:
                err = e
            finally:
                gmsh.finalize()
        else:
            raise RuntimeError(f"gmsh could not mesh the part: {err}")
    idx = np.zeros(int(tags.max()) + 1, int)
    idx[tags.astype(int)] = np.arange(len(tags))
    t = idx[np.asarray(en[list(et).index(4)], int).reshape(-1, 4)]
    used = np.unique(t)
    re = np.full(len(tags), -1)
    re[used] = np.arange(len(used))
    return MeshTet(xyz.reshape(-1, 3)[used].T.copy(), re[t].T.copy())


def facets(m, sel):
    """Boundary facets whose centre and outward normal satisfy sel."""
    f = m.boundary_facets()
    p = m.p[:, m.facets[:, f]]
    n = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0], axis=0)
    n /= np.linalg.norm(n, axis=0)
    c = p.mean(axis=1)
    n *= np.sign(np.sum(n * (c - m.p[:, m.t[:, m.f2t[0, f]]].mean(axis=1)), axis=0))
    return f[np.asarray(sel(c, n), bool)]


class Model:
    """Stiffness assembled once per part (in chunks, to bound memory); AMG hierarchy cached per support set."""

    def __init__(self, m, mat, chunk=4000):
        self.m, self.chunk = m, chunk
        E, nu, _, self.rho = MAT[mat]
        self.lam, self.mu = lame_parameters(E, nu)
        self.e = ElementVector(ElementTetP2())
        self.b0 = Basis(m, self.e, elements=np.arange(1))
        self.K, self.amg = None, {}
        for cb in self.chunks():
            k = asm(linear_elasticity(self.lam, self.mu), cb).tocsr()
            self.K = k if self.K is None else self.K + k

    def chunks(self):
        n = self.m.t.shape[1]
        for s in range(0, n, self.chunk):
            yield Basis(self.m, self.e, intorder=2, elements=np.arange(s, min(s + self.chunk, n)), dofs=self.b0.dofs)

    def solve(self, case, build=None):
        """Displacement at the nodes, von Mises per tet, and (with a build direction) the largest tension
        across the layers per tet."""
        m, b0 = self.m, self.b0
        f = np.zeros(b0.N)
        bf = -np.asarray(case.accel, float) * self.rho * 1e-3  # N/mm3
        if bf.any():
            for cb in self.chunks():
                f += asm(LinearForm(lambda v, w: bf[0] * v[0] + bf[1] * v[1] + bf[2] * v[2]), cb)
        if case.spin:
            cx, cy, om = case.spin
            k = self.rho * 1e-3 * om ** 2 * 1e-3  # N/mm3 per mm of radius
            for cb in self.chunks():
                f += asm(LinearForm(lambda v, w: k * ((w.x[0] - cx) * v[0] + (w.x[1] - cy) * v[1])), cb)
        for sel, F in case.loads:
            fb = FacetBasis(m, self.e, facets=facets(m, sel), dofs=b0.dofs)
            area = Functional(lambda w: 1.0 + 0 * w.x[0]).assemble(fb)
            tr = np.asarray(F, float) / area
            f += asm(LinearForm(lambda v, w: tr[0] * v[0] + tr[1] * v[1] + tr[2] * v[2]), fb)
        fixed = facets(m, case.fixed)
        assert len(fixed), f"{case.name}: no supported faces"
        D = b0.get_dofs(fixed).all()
        if case.squeeze:
            sel, sx, sy, delta = case.squeeze
            sq = facets(m, sel)
            fb = FacetBasis(m, self.e, facets=sq, dofs=b0.dofs)
            area = Functional(lambda w: 1.0 + 0 * w.x[0]).assemble(fb)

            def inward(v, w):
                r = np.sqrt((w.x[0] - sx) ** 2 + (w.x[1] - sy) ** 2)
                return -((w.x[0] - sx) * v[0] + (w.x[1] - sy) * v[1]) / r / area
            f += asm(LinearForm(inward), fb)
        u = np.zeros(b0.N)
        key = hash(D.tobytes())
        if key not in self.amg:
            Kc, _, _, I = condense(self.K, f, D=D)
            comp, (x, y, z) = np.arange(b0.N)[I] % 3, b0.doflocs[:, I]
            B = np.zeros((len(I), 6))  # rigid-body modes for the aggregation
            for c in range(3):
                B[comp == c, c] = 1
            B[comp == 0, 3], B[comp == 1, 3] = -y[comp == 0], x[comp == 1]
            B[comp == 1, 4], B[comp == 2, 4] = -z[comp == 1], y[comp == 2]
            B[comp == 0, 5], B[comp == 2, 5] = z[comp == 0], -x[comp == 2]
            self.amg[key] = (pyamg.smoothed_aggregation_solver(
                Kc.tocsr(), B=B, improve_candidates=None, max_coarse=5000, coarse_solver="splu"), I)
        ml, I = self.amg[key]
        u[I] = ml.solve(f[I], tol=1e-6, accel="cg", maxiter=2000)
        if case.squeeze:  # scale to the wanted mean inward movement of the squeezed faces
            nodes = np.unique(m.facets[:, sq])
            U = u[b0.dofs.nodal_dofs]
            dx, dy = m.p[0, nodes] - sx, m.p[1, nodes] - sy
            u *= delta / np.mean(-(U[0, nodes] * dx + U[1, nodes] * dy) / np.hypot(dx, dy))
        vm, snn = np.empty(m.t.shape[1]), np.zeros(m.t.shape[1])
        for i, cb in enumerate(self.chunks()):
            ep = sym_grad(cb.interpolate(u))
            S = 2 * self.mu * ep + self.lam * eye(trace(ep), 3)
            sl = slice(i * self.chunk, (i + 1) * self.chunk)
            vm[sl] = np.sqrt(
                0.5 * ((S[0, 0] - S[1, 1]) ** 2 + (S[1, 1] - S[2, 2]) ** 2 + (S[2, 2] - S[0, 0]) ** 2)
                + 3 * (S[0, 1] ** 2 + S[1, 2] ** 2 + S[0, 2] ** 2)).max(axis=1)
            if build is not None:
                snn[sl] = sum(build[a] * build[b] * S[a, b] for a in range(3) for b in range(3)).max(axis=1)
        return u[b0.dofs.nodal_dofs], vm, snn


def percentile(m, vm, frac=1e-3):
    """von Mises exceeded in only `frac` of the volume, and where the peak is."""
    P_ = m.p[:, m.t]
    vol = np.abs(np.linalg.det(np.stack([P_[:, i] - P_[:, 0] for i in (1, 2, 3)], 1).transpose(2, 0, 1))) / 6
    o = np.argsort(vm)[::-1]
    return vm[o][np.searchsorted(np.cumsum(vol[o]), frac * vol.sum())], P_[:, :, o[0]].mean(axis=1)


# ---- face pickers (vectorised over facets: c, n are (3, nf)) ----------------------------------

def down(z, tol=0.05):
    return lambda c, n: (np.abs(c[2] - z) < tol) & (n[2] < -0.9)


def up(z, tol=0.05):
    return lambda c, n: (np.abs(c[2] - z) < tol) & (n[2] > 0.9)


def bore_y(x, z, r, y0=-99, y1=99, tol=0.08):
    """Inward-facing cylindrical faces of radius r about an axis parallel to y through (x, z)."""
    def sel(c, n):
        d = np.hypot(c[0] - x, c[2] - z)
        inward = (n[0] * (x - c[0]) + n[2] * (z - c[2])) > 0.8 * d
        return (np.abs(d - r) < tol) & inward & (c[1] > y0) & (c[1] < y1)
    return sel


def any_of(*sels):
    return lambda c, n: np.any([s(c, n) for s in sels], axis=0)


# ---- load cases per part ----------------------------------------------------------------------

def block_cases(side):
    """The block assembled (base and cap bonded: the screws clamp them), on its two pads."""
    s = 1 if side == "L" else -1
    angle = P.layout.motor_angle_left if s > 0 else P.layout.motor_angle_right
    sx, sz = drive.seat_axis(angle)
    rb = (P.bearing.od + drive.D.bearing_fit) / 2
    ys = sorted((s * P.bearing_inner_y, s * P.bearing_outer_y))
    housing = bore_y(0, P.axle_z, rb, ys[0] - 1, ys[1] + 1)
    seat = bore_y(sx, sz, drive.seat_d() / 2)
    pads = down(P.board.top_z)
    wheel, motor = 6.0e-3, 7.0e-3  # kg: wheel + gear + axle + bearings; motor (+ sleeve)
    return "resin", [
        Case("crash", pads, [(housing, (wheel * CRASH, 0, 0)), (seat, (motor * CRASH, 0, 0))], (-CRASH, 0, 0)),
        Case("drop", pads, [(housing, (0, 0, 30.0)), (seat, (0, 0, -motor * DROP))], (0, 0, DROP)),
        # a side hit on the wheel: the tire pushes the axle (and the bearings' lips) inwards
        Case("side hit on the wheel", pads, [(housing, (0, -s * wheel * CRASH * 2, 0))], (0, 0, 0)),
    ]


def fan_mount_cases(m):
    """The yoke: held at its front foot (on the MCU) and at the two tabs on the drive caps' ears."""
    zmin, zmax = m.p[2].min(), m.p[2].max()
    cx, cy = fan.centre()
    zt, (tx, ty) = drive.fan_ear_top(), drive.D.fan_ear
    foot = lambda c, n: (c[2] < zmin + 0.05) & (n[2] < -0.9)
    tabs = lambda c, n: ((np.hypot(c[0] - tx, np.abs(c[1]) - ty) < drive.D.fan_ear_r + 0.05)
                         & (np.abs(c[2] - zt) < 0.05) & (n[2] < -0.9))
    fixed = any_of(foot, tabs)
    bore = lambda c, n: ((np.hypot(c[0] - cx, c[1] - cy) < 5.0) & (np.abs(n[2]) < 0.2) & (c[2] > zmax - 6)
                         & ((n[0] * (c[0] - cx) + n[1] * (c[1] - cy)) < 0))
    # the motor stands on the plate inside the collar (its seat)
    seat = lambda c, n: up(fan.heights()["motor"])(c, n) & (np.hypot(c[0] - cx, c[1] - cy) < P.motor.d / 2 + 0.1)
    motor = 8.0e-3
    return "resin", [
        Case("crash, fan motor on the collar", fixed, [(bore, (motor * CRASH, 0, 0))], (-CRASH, 0, 0)),
        Case("drop, fan motor on its seat", fixed, [(seat, (0, 0, -motor * DROP))], (0, 0, DROP)),
        Case("side crash, fan motor on the collar", fixed, [(bore, (0, motor * CRASH, 0))], (0, -CRASH, 0)),
    ]


def impeller_cases(m):
    cx, cy = fan.centre()
    bore = lambda c, n: (np.hypot(c[0] - cx, c[1] - cy) < 0.7) & (np.abs(n[2]) < 0.3)
    return "resin", [Case("spin 18k rpm", bore, spin=(cx, cy, 18000 * 2 * np.pi / 60), sf=3.0)]


def sensor_cap_cases(name):
    (ox, oy), ang = front.SENSORS[name[-2:]]
    a = np.radians(ang)
    look, left = np.array((np.cos(a), np.sin(a))), np.array((-np.sin(a), np.cos(a)))
    u = lambda c: (c[0] - ox) * look[0] + (c[1] - oy) * look[1]
    u_front = max(front.RECEIVER.tip, front.EMITTER.tip) + front.FP.hood
    face = lambda c, n: (u(c) > u_front - 0.05) & ((n[0] * look[0] + n[1] * look[1]) > 0.9)
    side = lambda c, n: (u(c) > u_front - 3) & ((n[0] * left[0] + n[1] * left[1]) > 0.9) & (c[2] > P.board.top_z + 4)
    base = down(P.board.top_z)
    return "resin", [
        Case("5 N push on the front", base, [(face, (-5 * look[0], -5 * look[1], 0))]),
        Case("3 N sideways at the front", base, [(side, (-3 * left[0], -3 * left[1], 0))]),
    ]


def basket_cases():
    """On its two posts (their feet on the cap bosses); the cells load the walls and floor. The straps hold
    the cells down, so an upside-down landing loads the straps, not the basket."""
    x0, x1, y0, y1 = frame.cavity()
    tray = frame.D_TRAY_TOP()
    posts = []
    for s in (1, -1):
        bx, by, bz = drive.frame_boss(s)
        posts.append(lambda c, n, bx=bx, by=s * by, bz=bz: (np.hypot(c[0] - bx, c[1] - by) < 3.0)
                     & (np.abs(c[2] - bz) < 0.1) & (n[2] < -0.9))
    fixed = any_of(*posts)
    inside = lambda c: (c[0] > x0 - 0.1) & (c[0] < x1 + 0.1) & (c[1] > y0 - 0.1) & (c[1] < y1 + 0.1)
    front_wall = lambda c, n: inside(c) & (np.abs(c[0] - x1) < 0.1) & (n[0] < -0.9)
    side_wall = lambda c, n: inside(c) & (np.abs(c[1] - y1) < 0.1) & (n[1] < -0.9)
    floor = lambda c, n: inside(c) & (c[2] < tray + 2.0) & (n[2] > 0.9)
    cells = 18e-3
    return "petg", [
        Case("crash, cells on the front wall", fixed, [(front_wall, (cells * CRASH, 0, 0))], (-CRASH, 0, 0)),
        Case("side crash, cells on the side wall", fixed, [(side_wall, (0, cells * CRASH, 0))], (0, -CRASH, 0)),
        Case("drop, cells on the floor", fixed, [(floor, (0, 0, -cells * DROP))], (0, 0, DROP)),
    ]


def bumper_cases(m):
    xmax = m.p[0].max()
    grip = lambda c, n: (c[2] > P.board.bottom_z + 0.05) & (c[2] < P.board.top_z + 0.65) & (n[0] < -0.3)
    nose = lambda c, n: (c[0] > xmax - 1.0) & (n[0] > 0.7)
    return "tpu", [Case("crash 85 N on the nose", grip, [(nose, (-85.0, 0, 0))])]


def wheel_hub_cases(name):
    s = 1 if name.endswith("_L") else -1
    y_web = s * (P.gear_y + P.gears.wheel_w)
    web = lambda c, n: (np.abs(c[1] - y_web) < 0.05) & (n[1] * s < -0.9)
    r = P.hub_d / 2
    rim = lambda c, n: (np.abs(np.hypot(c[0], c[2] - P.axle_z) - r) < 0.08) & (c[2] < P.axle_z - 0.87 * r)
    return "resin", [Case("drop, tire load on the rim", web, [(rim, (0, 0, 30.0))])]


def analyses(names):
    """(label, shape, h factor, cases or a function of the mesh) for the requested parts."""
    parts = assembly.printed()
    out = []
    for side in "LR":
        # the cap drawn split_relief above the base: tightening the screws closes that gap
        cap = Pos(0, 0, -drive.D.split_relief) * parts[f"block_cap_{side}"]
        out.append((f"block_{side}", parts[f"block_base_{side}"].fuse(cap).clean(), 1.0, block_cases(side)))
    out += [("fan_mount", parts["fan_mount"], 1.0, fan_mount_cases), ("impeller", parts["impeller"], 1.0, impeller_cases),
            ("sensor_cap_W1", parts["sensor_cap_W1"], 1.0, sensor_cap_cases("sensor_cap_W1")),
            ("sensor_cap_W2", parts["sensor_cap_W2"], 1.0, sensor_cap_cases("sensor_cap_W2")),
            ("basket", parts["basket"], 1.0, basket_cases()),
            ("bumper", parts["bumper"], 1.0, bumper_cases), ("wheel_hub_L", parts["wheel_hub_L"], 1.0, wheel_hub_cases("wheel_hub_L"))]
    return [a for a in out if not names or any(a[0].startswith(n) for n in names)]


def render(m, U, vm, path, strength):
    import pyvista as pv
    pv.OFF_SCREEN = True
    g = pv.UnstructuredGrid({pv.CellType.TETRA: m.t.T}, m.p.T.copy())
    g.cell_data["von Mises MPa"] = vm
    g.point_data["u"] = U.T
    s = g.warp_by_vector("u", factor=10).extract_surface(algorithm=None).cell_data_to_point_data()
    pl = pv.Plotter(off_screen=True, window_size=(1000, 700))
    pl.add_mesh(s, scalars="von Mises MPa", clim=(0, strength / 2), cmap="turbo")
    pl.add_text(path.stem.replace("_", " "), font_size=10)
    pl.view_vector((1, -1.2, 0.8))
    pl.reset_camera()
    pl.screenshot(str(path))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("parts", nargs="*")
    ap.add_argument("--h", type=float, default=0.5)
    ap.add_argument("--png", action="store_true")
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    t_all, rows = time.time(), []
    for label, shape, hf, cases in analyses(args.parts):
        t = time.time()
        m = mesh(shape, args.h * hf)
        mat, cases = cases(m) if callable(cases) else cases
        if args.dry:  # check that every support and load lands on faces, without solving
            for case in cases:
                n_fix = len(facets(m, case.fixed))
                n_loads = [len(facets(m, sel)) for sel, _ in case.loads]
                ok = n_fix > 0 and all(n_loads)
                rows.append((label, case.name, ok))
                print(f"  {'ok  ' if ok else 'FLAG'} {case.name:40s} supports {n_fix} facets, loads {n_loads}", flush=True)
            print(f"{label}: {m.t.shape[1]} tets, {mat}, mesh {time.time() - t:.0f} s", flush=True)
            continue
        model = Model(m, mat)
        print(f"{label}: {m.t.shape[1]} tets, {mat}, mesh + assembly {time.time() - t:.0f} s", flush=True)
        for case in cases:
            t = time.time()
            build = BUILD.get(label) if mat in LAYER else None
            U, vm, snn = model.solve(case, build)
            p999, where = percentile(m, vm)
            sf = MAT[mat][2] / p999
            layers = ""
            if build is not None:
                n999, nwhere = percentile(m, np.maximum(snn, 0))
                sf = min(sf, LAYER[mat] / max(n999, 1e-9))
                layers = f" | across layers {n999:5.1f} MPa at {nwhere.round(1)}"
            ok = sf >= case.sf
            rows.append((label, case.name, ok))
            print(f"  {'ok  ' if ok else 'FLAG'} {case.name:40s} max|u| {np.linalg.norm(U, axis=0).max():6.3f} mm | "
                  f"vM {p999:5.1f} MPa (peak {vm.max():5.1f} at {where.round(1)}){layers} | SF {sf:4.1f} / {case.sf} "
                  f"| {time.time() - t:.0f} s", flush=True)
            if args.png:
                render(m, U, vm, OUT / f"{label}_{case.name.split(',')[0].replace(' ', '_')}.png", MAT[mat][2])
        del model
    flags = [f"{a}: {b}" for a, b, ok in rows if not ok]
    print(f"\n{len(rows)} cases, {len(flags)} flagged | {time.time() - t_all:.0f} s, "
          f"peak {resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024:.0f} MB")
    for f in flags:
        print("  FLAG", f)


if __name__ == "__main__":
    main()
