"""Suction-fan study for the 10 x 23 mm, 54k rpm fan motor on a 3S pack (docs/fan_study.md).

Self-contained model, no CAD: numpy + scipy only; the board outline comes from ref/board_mech.json.

    tools/capped.sh uv run tools/fan_study.py            # full report (about a minute)
    tools/capped.sh uv run tools/fan_study.py --quick    # coarser board grid and design sweep
    tools/capped.sh uv run tools/fan_study.py --cfd build/cfd/curves_medium.json   # CFD fan curves (tools/cfd)
                                                                                   # against this 1D model

Pieces (SI units inside, mm / rpm / g in the printout):
- Motor: brushed coreless, Ke from 54k rpm no-load at 7.6 V; friction torque from the owner's 150 mA no-load at
  9 V (part Coulomb, part viscous); winding resistance R and inductance L estimated from 10 mm coreless datasheets
  scaled by Ke^2; driver: synchronous PWM (STSPIN958 in parallel mode: high side or low side on), now 100 kHz
  (TIM12 period 2749 at 275 MHz), with its ripple current, which a low-inductance coreless winding makes large; two-node thermal model (Faulhaber 1024 SR thermal resistances and time constants).
- Impeller: closed radial-bladed centrifugal fan discharging freely (no volute): Euler head with Wiesner slip,
  exit kinetic energy lost, inlet shock loss of straight radial blades, an empirical factor kappa for 3D losses;
  shaft power = Euler power on the through-flow (board leak + seal leak) + recirculation at low flow + disk friction.
- Inlet seal: the shroud runs s above the board, then a neck dips into the Ø15 hole: two thin annular gaps in
  series that leak from the tip (ambient) back to the hole; the rotating shroud's swirl opposes it a little.
- Board: the 1 mm gap under the whole board, solved as a Hele-Shaw (lubrication) flow on a 0.5 mm grid with the
  hole at the suction pressure and leaks along the edge: the film skirt (an effective contact gap, laminar +
  orifice) and, with the present skirt pattern, the open wheel notches. Gives leak flow, downforce and the
  centre of pressure.
"""

import argparse
import json
import math
from dataclasses import dataclass, replace
from functools import cache
from pathlib import Path

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from matplotlib.path import Path as MPath
from scipy.optimize import brentq

ROOT = Path(__file__).resolve().parents[1]
RHO, MU = 1.20, 1.81e-5  # air at ~25 C
RPM = 2 * math.pi / 60
G = 9.81
ROBOT_MASS = 0.087  # kg


# ----------------------------------------------------------------------------------------------------------------
# motor
@dataclass(frozen=True)
class Motor:
    n_rated: float = 54000.0  # rpm, taken as no-load at v_rated
    v_rated: float = 7.6
    i0_meas: float = 0.150  # A no-load at v_i0 (owner)
    v_i0: float = 9.0
    R: float = 0.55  # ohm winding at 25 C (estimate, 0.35-0.8; sensitivity runs go wider)
    L: float = 8e-6  # H winding (estimate, 5-15 uH)
    L_ext: float = 0.0  # H series inductor added at the motor
    R_drv: float = 0.20  # ohm: STSPIN958 on-resistance + 50 mOhm shunt + wires
    visc: float = 0.5  # share of the friction torque at the 9 V point that is viscous (rest Coulomb)
    f_pwm: float = 100e3
    # thermal (Faulhaber 1024 SR: Rth1 16 K/W, Rth2 41 K/W plastic flange, tau 6.5 s / 180 s); Rth2 a little lower
    # for the airflow of a moving robot
    rth1: float = 16.0
    rth2: float = 35.0
    tau1: float = 6.5
    tau2: float = 180.0
    t_amb: float = 30.0
    t_max: float = 85.0  # winding (NdFeB magnets, self-bonded winding of a cheap motor)
    t_run: float = 300.0  # s, burst length the limit is set for (from cold)

    @property
    def consts(self):
        """(Ke, Tc, cv): back-EMF constant (V s/rad = N m/A) and friction torque Tc + cv*w."""
        ke = self.v_rated / (self.n_rated * RPM)
        for _ in range(50):
            w9 = (self.v_i0 - self.i0_meas * self.R) / ke
            tf9 = ke * self.i0_meas
            tc, cv = tf9 * (1 - self.visc), tf9 * self.visc / w9
            w0 = self.n_rated * RPM
            i0 = (tc + cv * w0) / ke
            ke = (self.v_rated - i0 * self.R) / w0
        return ke, tc, cv

    def friction(self, w):
        _, tc, cv = self.consts
        return tc + cv * w

    def r_hot(self, t=None):
        return self.R * (1 + 0.0039 * ((self.t_max if t is None else t) - 25))

    def ripple_rms(self, v_bat, duty, r_tot):
        """RMS of the PWM ripple current (synchronous switching: the winding sees v_bat or 0)."""
        if duty >= 1 or duty <= 0:
            return 0.0
        n = np.arange(1, 400)
        vn = 2 * v_bat / (n * math.pi) * np.abs(np.sin(n * math.pi * duty))
        zn = np.hypot(r_tot, n * 2 * math.pi * self.f_pwm * (self.L + self.L_ext))
        return float(np.sqrt(np.sum(0.5 * (vn / zn) ** 2)))

    def ripple_pp(self, v_bat, duty, r_tot):
        lt = self.L + self.L_ext
        return v_bat * duty * (1 - duty) / (lt * self.f_pwm)  # small-ripple approximation

    def winding_temp(self, p_cu, p_fr, t=None):
        t = self.t_run if t is None else t
        g1 = 1 - math.exp(-t / self.tau1) if t < 1e8 else 1.0
        g2 = 1 - math.exp(-t / self.tau2) if t < 1e8 else 1.0
        # copper loss crosses winding->housing; the brush/bearing/windage loss heats the housing side
        return self.t_amb + p_cu * self.rth1 * g1 + (p_cu + p_fr) * self.rth2 * g2


# ----------------------------------------------------------------------------------------------------------------
# impeller and seal
@dataclass(frozen=True)
class Impeller:
    d2: float = 26.4e-3
    d1: float = 11.0e-3  # eye (shroud inner edge)
    b2: float = 3.0e-3  # channel height at the tip
    b1: float = 3.0e-3  # at the eye
    z: int = 12
    t: float = 0.55e-3
    beta2: float = 90.0  # outlet blade angle from tangent
    nose_d: float = 6.0e-3  # hub/nose over the pinion: blocks the middle of the eye
    shroud_t: float = 0.6e-3
    back_t: float = 0.8e-3
    # seal: face gap over the board ring, then the neck in the board hole
    s_face: float = 0.3e-3
    neck_gap: float = 0.3e-3  # radial, neck OD to the Ø15 hole
    neck_l: float = 0.9e-3  # depth into the hole
    hole_d: float = 15.0e-3
    # empirical factors (ranges in the report)
    kappa: float = 0.90  # 3D losses on the shut-off pressure (0.75-1.0)
    k_shock: float = 0.8  # straight radial blades meet the axial inflow at the eye
    k_rec: float = 0.03  # low-flow recirculation power: k_rec * rho u2^3 d2 b2 (0.015-0.06)
    phi_rec: float = 0.15  # flow coefficient where recirculation has died out
    zeta_in: float = 0.3

    @property
    def slip(self):
        return 1 - math.sqrt(math.sin(math.radians(self.beta2))) / self.z ** 0.7

    def suction(self, q, w):
        """Static suction (Pa, positive) in the board hole at impeller through-flow q (m^3/s), speed w (rad/s)."""
        u2, u1 = w * self.d2 / 2, w * self.d1 / 2
        a2 = (math.pi * self.d2 - self.z * self.t) * self.b2
        cm2 = q / a2
        ct2 = self.slip * u2 - cm2 / math.tan(math.radians(self.beta2))
        a1 = math.pi * ((self.d1 / 2) ** 2 - (self.nose_d / 2) ** 2)
        a1 = min(a1, (math.pi * self.d1 - self.z * self.t) * self.b1)
        c1 = q / a1
        dp = self.kappa * (RHO * u2 * ct2 - 0.5 * RHO * (ct2 ** 2 + cm2 ** 2) - 0.5 * RHO * self.k_shock * u1 ** 2)
        return dp - 0.5 * RHO * self.zeta_in * c1 ** 2  # inlet loss (the eye's dynamic head is recovered in the rotor)

    def shaft_power(self, q, w):
        u2 = w * self.d2 / 2
        a2 = (math.pi * self.d2 - self.z * self.t) * self.b2
        cm2 = q / a2
        ct2 = self.slip * u2 - cm2 / math.tan(math.radians(self.beta2))
        p_euler = RHO * q * u2 * ct2
        phi = cm2 / u2
        p_rec = self.k_rec * RHO * u2 ** 3 * self.d2 * self.b2 * max(0.0, 1 - phi / self.phi_rec) ** 2
        r2 = self.d2 / 2
        re = w * r2 ** 2 * RHO / MU
        cm = 3.87 / math.sqrt(re) if re < 3e5 else 0.146 * re ** -0.2  # both faces (free disk)
        p_disk = cm * 0.5 * RHO * w ** 3 * r2 ** 5
        p_rim = 0.0365 * RHO * w ** 3 * r2 ** 4 * (self.b2 + self.shroud_t + self.back_t) * math.pi  # rough
        return p_euler + p_rec + p_disk + p_rim

    def seal_leak(self, s, w):
        """Leak (m^3/s) back from the tip to the hole through the face gap and the neck gap, at suction s (Pa)."""
        rh = self.hole_d / 2
        r2 = self.d2 / 2
        p_cf = 0.5 * RHO * (0.4 * w) ** 2 * max(r2 ** 2 - rh ** 2, 0)  # swirl in the gap pumps outwards
        dp = s - p_cf
        if dp <= 0:
            return 0.0
        f = 0.08
        a_face = 2 * math.pi * rh * self.s_face
        a_neck = 2 * math.pi * (rh - self.neck_gap / 2) * self.neck_gap
        lf = max(r2 - rh, 0.5e-3)
        k = (0.5 + f * lf / (2 * self.s_face)) / a_face ** 2 + (1.5 + f * self.neck_l / (2 * self.neck_gap)) / a_neck ** 2
        return math.sqrt(2 * dp / (RHO * k))

    def mass(self, rho_resin=1150.0):
        r2, r1 = self.d2 / 2, self.d1 / 2
        v = math.pi * (r2 ** 2 - r1 ** 2) * self.shroud_t + math.pi * r2 ** 2 * self.back_t
        v += self.z * self.t * (r2 - self.nose_d / 2) * (self.b1 + self.b2) / 2
        v += math.pi * ((self.nose_d / 2) ** 2 - 1.65e-3 ** 2) * 6.0e-3  # hub/nose around the pinion
        return v * rho_resin


# ----------------------------------------------------------------------------------------------------------------
# board gap
@dataclass(frozen=True)
class Skirt:
    name: str
    delta: float  # effective gap of the skirt's contact with the floor (m)
    notches_open: bool
    contact_l: float = 1.0e-3  # contact width of the bent-down margin on the floor
    cd: float = 0.62
    gap: float = 0.9e-3  # board underside to floor (1.0, less the tires' sink under load)


SKIRTS = {
    "notches open": Skirt("skirt as cut now (0.5 mm margin at the wheel notches), floor contact 0.05", 0.05e-3, True),
    "sealed good": Skirt("skirt closed round the notches, good floor contact 0.02", 0.02e-3, False),
    "sealed": Skirt("skirt closed round the notches, typical floor contact 0.05", 0.05e-3, False),
    "sealed poor": Skirt("skirt closed round the notches, poor contact 0.10 (joints, dust)", 0.10e-3, False),
}


def _loops():
    m = json.loads((ROOT / "ref" / "board_mech.json").read_text())

    def arc(s, mid, e, n=16):
        (ax, ay), (bx, by), (cx, cy) = s, mid, e
        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
        uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d
        r = math.hypot(ax - ux, ay - uy)
        a0, a1, a2 = (math.atan2(p[1] - uy, p[0] - ux) for p in (s, mid, e))
        up = lambda a: a + 2 * math.pi * math.ceil((a0 - a) / (2 * math.pi)) if a < a0 else a
        a1u, a2u = up(a1), up(a2)
        end = a2u if a1u <= a2u else a2u - 2 * math.pi
        return [(ux + r * math.cos(t), uy + r * math.sin(t)) for t in np.linspace(a0, end, n)]

    segs = []
    for g in m["edge"]:
        if g["type"] == "line" and math.dist(g["start"], g["end"]) > 1e-6:
            segs.append([tuple(g["start"]), tuple(g["end"])])
        elif g["type"] == "arc":
            segs.append(arc(g["start"], g["mid"], g["end"]))
    loops = []
    while segs:
        cur = segs.pop(0)
        while True:
            for i, s in enumerate(segs):
                if math.dist(s[0], cur[-1]) < 1e-3:
                    cur += s[1:]
                elif math.dist(s[-1], cur[-1]) < 1e-3:
                    cur += s[::-1][1:]
                else:
                    continue
                segs.pop(i)
                break
            else:
                break
        loops.append(np.array(cur) * 1e-3)
    outer = max(loops, key=lambda a: np.ptp(a[:, 0]))
    hole = min(loops, key=lambda a: np.hypot(a[:, 0].mean() - 17.5e-3, a[:, 1].mean()) + (0 if len(a) > 20 else 1))
    return outer, hole


@cache
def board_table(skirt: Skirt, h=0.5e-3):
    """Leak flow, downforce and centre of pressure against the suction in the hole, for one skirt."""
    outer, hole = _loops()
    xs = np.arange(outer[:, 0].min() + h / 2, outer[:, 0].max(), h)
    ys = np.arange(outer[:, 1].min() + h / 2, outer[:, 1].max(), h)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    pts = np.column_stack([X.ravel(), Y.ravel()])
    in_out = MPath(outer).contains_points(pts).reshape(X.shape)
    in_hole = MPath(hole).contains_points(pts).reshape(X.shape)
    cell = in_out & ~in_hole
    nx, ny = X.shape
    idx = -np.ones(X.shape, int)
    idx[cell] = np.arange(cell.sum())
    n = int(cell.sum())
    kh = skirt.gap ** 3 / (12 * MU)
    rows, cols, vals = [], [], []
    hole_g = np.zeros(n)
    edges = []  # (cell index, is_notch)
    ii, jj = np.nonzero(cell)
    for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        i2, j2 = ii + di, jj + dj
        ok = (i2 >= 0) & (i2 < nx) & (j2 >= 0) & (j2 < ny)
        a = idx[ii, jj]
        nb_cell = np.zeros_like(ok)
        nb_hole = np.zeros_like(ok)
        nb_cell[ok] = cell[i2[ok], j2[ok]]
        nb_hole[ok] = in_hole[i2[ok], j2[ok]]
        m = nb_cell
        rows += [a[m]]; cols += [idx[i2[m], j2[m]]]; vals += [-kh * np.ones(m.sum())]
        rows += [a[m]]; cols += [a[m]]; vals += [kh * np.ones(m.sum())]
        np.add.at(hole_g, a[nb_hole], 2 * kh)
        out = ~nb_cell & ~nb_hole
        fx = X[ii[out], jj[out]] + di * h / 2
        fy = Y[ii[out], jj[out]] + dj * h / 2
        notch = (np.abs(fx) < 10.6e-3) & (np.abs(fy) > 17.0e-3)
        edges.append((a[out], notch, np.abs(fy)))
    e_idx = np.concatenate([e[0] for e in edges])
    e_notch = np.concatenate([e[1] for e in edges])
    e_y = np.concatenate([e[2] for e in edges])
    a_lap = sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(n, n))
    # edge leak per face: dp = A q + B q^2 (half cell + laminar contact + orifice)
    delta = np.full(e_idx.shape, skirt.delta)
    lcont = np.full(e_idx.shape, skirt.contact_l)
    if skirt.notches_open:
        delta[e_notch] = np.where(e_y[e_notch] < 17.8e-3, 0.5e-3, skirt.gap)  # 0.5 mm margin flap / nothing
        lcont[e_notch] = 0.0
    A = 1 / (2 * kh) + 12 * MU * lcont / (delta ** 3 * h)
    B = 0.5 * RHO / (skirt.cd * delta * h) ** 2

    def solve(s):
        g = np.full(e_idx.shape, 1 / A.mean())
        p = None
        for _ in range(60):
            diag = np.zeros(n)
            np.add.at(diag, e_idx, g)
            mat = a_lap + sp.diags(diag + hole_g)
            p_new = spla.spsolve(mat.tocsc(), hole_g * (-s))
            p = p_new if p is None else 0.5 * p + 0.5 * p_new
            dpe = np.maximum(-p[e_idx], 1e-9)
            q = (-A + np.sqrt(A ** 2 + 4 * B * dpe)) / (2 * B)
            g_new = q / dpe
            if np.max(np.abs(g_new - g) / g) < 1e-4:
                g = g_new
                break
            g = g_new
        dpe = np.maximum(-p[e_idx], 0)
        q = (-A + np.sqrt(A ** 2 + 4 * B * dpe)) / (2 * B)
        x = X[cell]
        f_board = float(np.sum(-p) * h * h)
        a_hole = float(in_hole.sum() * h * h)
        f = f_board + a_hole * s
        xcp = (float(np.sum(-p * x)) * h * h + a_hole * s * 17.5e-3) / f
        return float(q.sum()), f, xcp, float(np.mean(-p) / s), float(q[e_notch].sum())

    ss = np.geomspace(5, 8000, 22)
    res = np.array([solve(s) for s in ss])
    area = float(cell.sum() * h * h + in_hole.sum() * h * h)
    return ss, res, area


def board_at(skirt, s):
    ss, res, _ = board_table(skirt, BOARD_H)
    s = max(s, ss[0])
    out = [np.exp(np.interp(np.log(s), np.log(ss), np.log(np.maximum(res[:, k], 1e-12)))) for k in (0, 1, 2, 3)]
    return dict(q=out[0], f=out[1], xcp=out[2], uniform=out[3])


BOARD_H = 0.5e-3


# ----------------------------------------------------------------------------------------------------------------
# a fan given by its measured or CFD curve instead of the correlations above
@dataclass(frozen=True)
class CurveFan:
    """Fan curve at one speed (from CFD, tools/cfd/post.py, or a rig): the net through-flow drawn from the cavity
    (seal leak already inside it) and the shaft torque against the suction; other speeds by the fan laws
    (s ~ w^2, q ~ w, torque ~ w^2: Reynolds-number effects ignored, see docs/fan_study.md "CFD results")."""
    name: str
    rpm_ref: float
    s: tuple  # Pa, ascending
    q_net: tuple  # m^3/s from the cavity (negative past shut-off)
    torque: tuple  # N m on the impeller (aerodynamic only; the motor's friction is in Motor)
    q_seal: tuple = ()  # m^3/s recirculated through the seal, if known
    geom: Impeller = None  # for the structural speed limit

    def _ref(self, s, w):
        k = self.rpm_ref * RPM / w
        return s * k * k, 1 / k

    def flow(self, s, w):
        sr, kq = self._ref(s, w)
        xs, ys = np.array(self.s), np.array(self.q_net)
        if sr > xs[-1]:  # beyond the last point: extend the last segment
            return kq * float(ys[-1] + (ys[-1] - ys[-2]) / (xs[-1] - xs[-2]) * (sr - xs[-1]))
        return kq * float(np.interp(sr, xs, ys))

    def torque_at(self, s, w):
        sr, kq = self._ref(s, w)
        xs, ys = np.array(self.s), np.array(self.torque)
        if sr > xs[-1]:
            ys_e = ys[-1] + (ys[-1] - ys[-2]) / (xs[-1] - xs[-2]) * (sr - xs[-1])
            return kq * kq * float(ys_e)
        return kq * kq * float(np.interp(sr, xs, ys))

    def seal_at(self, s, w):
        if not self.q_seal:
            return float("nan")
        sr, kq = self._ref(s, w)
        return kq * float(np.interp(sr, np.array(self.s), np.array(self.q_seal)))

    def shutoff(self, w):
        """Suction at zero net flow (Pa) at speed w."""
        f = lambda s: self.flow(s, w)
        k = (w / (self.rpm_ref * RPM)) ** 2
        hi = self.s[-1] * k
        while f(hi) > 0:
            hi *= 1.2
        return brentq(f, 1e-6, hi) if f(1e-6) > 0 else 0.0


def load_cfd_curves(path):
    """CurveFans from the JSON written by tools/cfd/post.py ({name: {rpm, s_pa, q_net_m3s, torque_nm, ...}})."""
    data = json.loads(Path(path).read_text())
    rec = replace(Impeller(), d2=22e-3, d1=10e-3, b2=2.0e-3, b1=3.0e-3, z=12)
    out = {}
    for k, d in data["curves"].items():
        o = np.argsort(d["s_pa"])
        pick = lambda key: tuple(float(np.array(d[key])[i]) for i in o) if d.get(key) else ()
        out[k] = CurveFan(k, d["rpm"], pick("s_pa"), pick("q_net_m3s"), pick("torque_nm"), pick("q_seal_m3s"), rec)
    return out


# ----------------------------------------------------------------------------------------------------------------
# operating point
def fan_point(imp: Impeller, skirt: Skirt, w):
    """Suction, flows and shaft torque at fan speed w (rad/s)."""
    if isinstance(imp, CurveFan):
        s_hi = imp.shutoff(w)
        if s_hi <= 1:
            return None
        s = brentq(lambda s: imp.flow(s, w) - board_at(skirt, s)["q"], 1e-3, s_hi)
        b = board_at(skirt, s)
        qs = imp.seal_at(s, w)
        tq = imp.torque_at(s, w)
        return dict(w=w, rpm=w / RPM, s=s, q_leak=b["q"], q_seal=qs, q=b["q"] + qs, f=b["f"], xcp=b["xcp"],
                    uniform=b["uniform"], p_shaft=tq * w, torque=tq, p_air=s * b["q"])

    def resid(s):
        ql = board_at(skirt, s)["q"]
        qs = imp.seal_leak(s, w)
        return s - imp.suction(ql + qs, w)

    s_hi = imp.suction(0.0, w)
    if s_hi <= 1:
        return None
    s = brentq(resid, 1e-3, s_hi)
    b = board_at(skirt, s)
    qs = imp.seal_leak(s, w)
    q = b["q"] + qs
    p = imp.shaft_power(q, w)
    return dict(w=w, rpm=w / RPM, s=s, q_leak=b["q"], q_seal=qs, q=q, f=b["f"], xcp=b["xcp"], uniform=b["uniform"],
                p_shaft=p, torque=p / w, p_air=s * b["q"])


def motor_point(mot: Motor, fp, v_bat=12.3):
    """Electrical side of a fan point: current, duty, losses, temperature."""
    ke, _, _ = mot.consts
    w = fp["w"]
    r_w = mot.r_hot()
    r_tot = r_w + mot.R_drv
    i = (fp["torque"] + mot.friction(w)) / ke
    v = ke * w + i * r_tot
    duty = v / v_bat
    i_ac = mot.ripple_rms(v_bat, min(duty, 1), r_tot)
    p_cu = (i * i + i_ac * i_ac) * r_w
    p_fr = mot.friction(w) * w
    t5 = mot.winding_temp(p_cu, p_fr)
    t_cont = mot.winding_temp(p_cu, p_fr, t=1e9)
    t3 = mot.winding_temp(p_cu, p_fr, t=180)
    p_in = v_bat * duty * i + (i * i + i_ac * i_ac) * (r_tot - r_w) + i_ac ** 2 * r_w  # battery side, approx.
    return dict(i=i, i_ac=i_ac, i_peak=i + mot.ripple_pp(v_bat, min(duty, 1), r_tot) / 2, duty=duty, v=v,
                p_cu=p_cu, p_fr=p_fr, p_loss=p_cu + p_fr, t5=t5, t3=t3, t_cont=t_cont, p_in=v * i + i_ac ** 2 * r_tot,
                p_bat=p_in)


def best_speed(imp, skirt, mot, v_bat=12.3, rpm_cap=65000.0, i_peak_max=4.0, t_key="t5"):
    """Highest fan speed inside the limits (winding temperature, duty <= 1, driver peak current, rpm cap)."""

    def ok(rpm):
        fp = fan_point(imp, skirt, rpm * RPM)
        if fp is None:
            return False, None, None
        mp = motor_point(mot, fp, v_bat)
        good = mp["duty"] <= 1.0 and mp[t_key] <= mot.t_max and mp["i_peak"] <= i_peak_max
        return good, fp, mp

    rpm_cap = min(rpm_cap, safe_rpm(imp))  # SF 4 on the resin's strength
    lo, hi = 5000.0, rpm_cap
    good, fp, mp = ok(hi)
    if good:
        return fp, mp, "rpm cap"
    if not ok(lo)[0]:
        return None, None, "infeasible"
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        if ok(mid)[0]:
            lo = mid
        else:
            hi = mid
    good, fp, mp = ok(lo)
    lim = "duty" if mp["duty"] > 0.995 else ("peak current" if mp["i_peak"] > 0.99 * i_peak_max else "temperature")
    return fp, mp, lim


def at_force(imp, skirt, f):
    """Fan point that gives downforce f (N)."""
    rpm = brentq(lambda r: fan_point(imp, skirt, r * RPM)["f"] - f, 3000, 150000)
    return fan_point(imp, skirt, rpm * RPM)


def at_duty(imp, skirt, mot, duty, v_bat=12.3):
    """Fan and motor at a fixed PWM duty."""

    def resid(rpm):
        fp = fan_point(imp, skirt, rpm * RPM)
        return motor_point(mot, fp, v_bat)["duty"] - duty

    rpm = brentq(resid, 3000, 120000)
    fp = fan_point(imp, skirt, rpm * RPM)
    return fp, motor_point(mot, fp, v_bat)


# ----------------------------------------------------------------------------------------------------------------
# structure
def structure(imp: Impeller, rpm, uts=35e6, rho=1150.0, nu=0.38):
    w = rpm * RPM
    r2, r1 = imp.d2 / 2, imp.d1 / 2
    u2 = w * r2
    # annular disk, max hoop stress at the bore
    s_disk = rho * w ** 2 / 4 * ((3 + nu) * r2 ** 2 + (1 - nu) * r1 ** 2)
    # the blades hang on the shroud and backplate: their centrifugal load adds to the rings' own
    m_rings = rho * (math.pi * (r2 ** 2 - r1 ** 2) * imp.shroud_t + math.pi * (r2 ** 2 - (imp.nose_d / 2) ** 2) * imp.back_t)
    m_bl = rho * imp.z * imp.t * (r2 - imp.nose_d / 2) * (imp.b1 + imp.b2) / 2
    s_hoop = s_disk * (1 + m_bl / m_rings)
    # a backward-curved blade would also bend (span b between the disks, fixed both ends); radial: none
    q_bl = rho * imp.t * w ** 2 * r2 * abs(math.cos(math.radians(imp.beta2)))
    s_blade = q_bl * imp.b2 ** 2 / (2 * imp.t ** 2)
    return dict(u2=u2, s_hoop=s_hoop, s_blade=s_blade, sf=uts / s_hoop, rho_u2=rho * u2 ** 2)


def safe_rpm(imp, sf=4.0, uts=35e6):
    s1 = structure(imp.geom if isinstance(imp, CurveFan) else imp, 10000)["s_hoop"]
    return 10000 * math.sqrt(uts / sf / s1)


# ----------------------------------------------------------------------------------------------------------------
def fmt(fp, mp):
    return (f"{fp['rpm']/1000:5.1f}k rpm  suction {fp['s']:5.0f} Pa  F {fp['f']:4.2f} N  CoP x {fp['xcp']*1e3:4.1f}  "
            f"Q leak {fp['q_leak']*1e3:4.2f} seal {fp['q_seal']*1e3:4.2f} L/s  shaft {fp['p_shaft']:4.2f} W  "
            f"I {mp['i']:4.2f} A (ripple rms {mp['i_ac']:4.2f}, peak {mp['i_peak']:4.2f})  duty {mp['duty']*100:3.0f}%  "
            f"in {mp['p_in']:4.1f} W  loss {mp['p_cu']:4.2f}+{mp['p_fr']:4.2f} W  Tw 3/5 min/cont "
            f"{mp['t3']:3.0f}/{mp['t5']:3.0f}/{mp['t_cont']:3.0f} C")


def one_d_impellers():
    """The three Ø22 candidates as the 1D model sees them (docs/fan_study.md, 'Update: curved blade inlets')."""
    rec = replace(Impeller(), d2=22e-3, d1=10e-3, b2=2.0e-3, b1=3.0e-3, z=12)
    return {"radial": rec, "inducer": replace(rec, k_shock=0.2), "backward": replace(rec, z=7, beta2=35.0, k_shock=0.2)}


def cfd_report(path, f_target=4.0):
    sk = SKIRTS["sealed"]
    mot = Motor(f_pwm=400e3)
    curves = load_cfd_curves(path)
    ref = one_d_impellers()
    print(f"== {f_target} N and the 5-minute thermal limit (85 C), sealed skirt, 400 kHz PWM, nominal motor ==")
    print("max F: within 85 C after 5 min, the 4 A peak and duty <= 1, with the study's 65k rpm cap / with only the "
          "structural cap (SF 4)")
    print("model    impeller  | rpm    suction  Q skirt  Q seal   shaft   battery  I      Tw 5min | max F   at rpm  limit"
          "        | max F   at rpm  limit")
    rows = [("CFD", k, c) for k, c in curves.items()] + [("1D", k, ref[k]) for k in curves if k in ref]
    for lab, k, imp in rows:
        fp = at_force(imp, sk, f_target)
        mp = motor_point(mot, fp)
        b, _, lim = best_speed(imp, sk, mot)
        b2, _, lim2 = best_speed(imp, sk, mot, rpm_cap=1e6)
        print(f"{lab:8} {k:9} | {fp['rpm']:6.0f} {fp['s']:6.0f} Pa {fp['q_leak']*1e3:5.3f}   {fp['q_seal']*1e3:5.3f} L/s "
              f"{fp['p_shaft']:5.2f} W {mp['p_bat']:5.2f} W {mp['i']:5.3f} A {mp['t5']:5.1f} C | {b['f']:5.2f} N "
              f"{b['rpm']:6.0f}  {lim:12} | {b2['f']:5.2f} N {b2['rpm']:6.0f}  {lim2}")
    print("-- shut-off suction at 42.6k rpm --")
    w = 42600 * RPM
    for k, c in curves.items():
        print(f"CFD {k:9}: {c.shutoff(w):5.0f} Pa;  1D: {ref[k].suction(0.0, w):5.0f} Pa" if k in ref else k)


def main():
    global BOARD_H
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--cfd", metavar="JSON", help="only compare the CFD fan curves in JSON (tools/cfd/post.py) with "
                    "the 1D model, sealed skirt, 400 kHz PWM")
    args = ap.parse_args()
    if args.quick or args.cfd:
        BOARD_H = 1.0e-3
    if args.cfd:
        cfd_report(args.cfd)
        return

    print("== motor ==")
    for R in (0.35, 0.55, 0.8, 1.5):
        m = Motor(R=R)
        ke, tc, cv = m.consts
        n0 = (12.3 - (tc + cv * 9000) / ke * (R + m.R_drv)) / ke / RPM
        w9 = (9 - 0.15 * R) / ke
        print(f"R {R:4.2f}: Ke {ke*1e3:5.3f} mV s/rad = Kt mN m/A ({1/ke/RPM:5.0f} rpm/V)  friction {tc*1e3:5.3f}+"
              f"{cv*1e6:6.4f}e-3*w/1000 mN m  (9 V: {w9/RPM:5.0f} rpm, {tc*1e3+cv*w9*1e3:4.3f} mN m, {(tc+cv*w9)*w9:4.2f} W)"
              f"  no-load @12.3 V ~{n0:5.0f} rpm, friction {m.friction(n0*RPM)*n0*RPM:4.2f} W, stall {12.3/(R+m.R_drv):4.1f} A")
    m = Motor()
    for t, lab in ((60, "1 min"), (180, "3 min"), (300, "5 min"), (1e9, "continuous")):
        # loss budget if it were all copper / all friction
        dT = m.t_max - m.t_amb
        g1, g2 = (1 - math.exp(-t / m.tau1)), (1 - math.exp(-t / m.tau2))
        print(f"  heat budget {lab:>10}: {dT/(m.rth1*g1+m.rth2*g2):4.2f} W all in the winding, "
              f"{dT/(m.rth2*g2):4.2f} W all on the housing side")

    print("\n== board gap (Hele-Shaw) ==")
    for k, sk in SKIRTS.items():
        area = board_table(sk, BOARD_H)[2]
        for s in (300, 1000):
            b = board_at(sk, s)
            print(f"{k:13} at {s:4d} Pa: leak {b['q']*1e3:5.3f} L/s, F {b['f']:4.2f} N (= {b['f']/s*1e6:5.0f} mm2 x s), "
                  f"CoP x {b['xcp']*1e3:4.1f} mm, mean/hole pressure {b['uniform']:4.2f}")
    print(f"footprint (board + hole) {area*1e6:5.0f} mm2")

    base = Impeller()
    sealed = SKIRTS["sealed"]
    # the motor cases: L is the big unknown (PWM ripple); an inductor in the motor lead removes it
    IND = {"L_ext": 47e-6, "R_drv": 0.30}  # 47 uH, ~0.1 ohm DCR
    F400 = {"f_pwm": 400e3}  # TIM12 at 400 kHz (the STSPIN958 takes up to 500)
    cases = {"100 kHz (now)": Motor(), "400 kHz": Motor(**F400), "+47 uH": Motor(**IND)}
    print("\n== present impeller (Ø26.4, eye 11, b 3), nominal motor, sealed skirt ==")
    for d in (0.3, 0.5, 0.7):
        fp, mp = at_duty(base, sealed, Motor(), d)
        print(f"duty {d:3.1f}: " + fmt(fp, mp))
    for k, mot in cases.items():
        fp, mp, lim = best_speed(base, sealed, mot)
        print(f"{k:12} limit ({lim}): " + fmt(fp, mp))

    print("\n== design sweep: best eye/b2 per d2 at the limit (5 min burst <= 85 C, peak <= 4 A, "
          "rpm <= min(65k, SF 4)) ==")
    d2s = np.arange(14e-3, 26.5e-3, (2e-3 if args.quick else 1e-3))
    sweep_cases = {"100 kHz (now)": Motor(), "400 kHz": Motor(**F400), "400 kHz R 0.35": Motor(R=0.35, **F400),
                   "400 kHz R 0.8": Motor(R=0.8, **F400), "400 kHz R 1.5": Motor(R=1.5, **F400)}
    for k, mot in sweep_cases.items():
        print(f"-- {k} --")
        for d2 in d2s:
            row = None
            for d1 in (9e-3, 10e-3, 11e-3):
                if d1 > 0.62 * d2:
                    continue
                for b2 in (1.5e-3, 2.0e-3, 3.0e-3):
                    imp = replace(base, d2=d2, d1=d1, b2=b2, b1=max(b2, 3.0e-3))
                    fp, mp, lim = best_speed(imp, sealed, mot)
                    if fp and (row is None or fp["f"] > row[0]["f"]):
                        row = (fp, mp, lim, imp)
            if row:
                fp, mp, lim, imp = row
                print(f"d2 {d2*1e3:4.1f} eye {imp.d1*1e3:4.1f} b2 {imp.b2*1e3:3.1f} [{lim:11}] F {fp['f']:4.2f} N "
                      f"{fp['rpm']/1000:5.1f}k rpm  suction {fp['s']:5.0f} Pa  shaft {fp['p_shaft']:4.2f} W  "
                      f"I {mp['i']:4.2f} A  duty {mp['duty']*100:3.0f}%  loss {mp['p_cu']:4.2f}+{mp['p_fr']:4.2f} W")

    rec = replace(base, d2=22e-3, d1=10e-3, b2=2.0e-3, b1=3.0e-3, z=12)
    print("\n== the same downforce from each size (eye 10, b2 2.0, b1 3.0; sealed skirt, nominal "
          "motor): Tw after 5 min / continuous ==")
    for f_t in (3.0, 4.0, 5.0):
        for d2 in (18e-3, 20e-3, 22e-3, 24e-3, 26.4e-3):
            imp = replace(rec, d2=d2)
            fp = at_force(imp, sealed, f_t)
            cols = []
            for k, mot in cases.items():
                mp = motor_point(mot, fp)
                cols.append(f"{k}: I {mp['i']:4.2f}+{mp['i_ac']:4.2f} rms, {mp['t5']:3.0f}/{mp['t_cont']:3.0f} C")
            mp = motor_point(Motor(**F400), fp)
            print(f"F {f_t} N d2 {d2*1e3:4.1f}: {fp['rpm']/1000:5.1f}k rpm u2 {fp['w']*d2/2:4.1f} m/s shaft "
                  f"{fp['p_shaft']:4.2f} W, {mp['v']:4.2f} V avg (duty {mp['duty']*100:3.0f}% at 12.3 V), battery "
                  f"{mp['p_bat']:4.1f} W | " + " | ".join(cols))
    print(f"\n== recommended impeller: Ø22, eye 10, b2 2.0 (b1 3.0), 12 radial blades, seal 0.3/0.3; "
          f"safe rpm (SF 4) {safe_rpm(rec):.0f} ==")
    for name, mot in (("100 kHz (now)", Motor()), ("100 kHz L 5 uH", Motor(L=5e-6)), ("100 kHz L 15 uH", Motor(L=15e-6)),
                      ("+47 uH", Motor(**IND)), ("400 kHz", Motor(**F400)), ("400 kHz L 5 uH", Motor(L=5e-6, **F400)),
                      ("400 kHz R 0.35", Motor(R=0.35, **F400)), ("400 kHz R 0.8", Motor(R=0.8, **F400)),
                      ("400 kHz R 1.5", Motor(R=1.5, **F400)), ("400 kHz Coulomb fr.", Motor(visc=0.0, **F400)),
                      ("400 kHz fr. -30%", Motor(i0_meas=0.105, **F400)), ("400 kHz continuous", Motor(t_run=1e9, **F400)),
                      ("400 kHz 3 min", Motor(t_run=180, **F400)), ("400 kHz Tmax 100", Motor(t_max=100, **F400))):
        fp, mp, lim = best_speed(rec, sealed, mot)
        print(f"{name:19} [{lim:11}] " + fmt(fp, mp))
    print("-- skirt and fan-model ranges (400 kHz, nominal motor, at the limit; then at 4 N) --")
    for k, sk in SKIRTS.items():
        fp, mp, lim = best_speed(rec, sk, Motor(**F400))
        print(f"{k:19} [{lim:11}] " + fmt(fp, mp))
        fp = at_force(rec, sk, 4.0)
        mp = motor_point(Motor(**F400), fp)
        print(f"{k:19} [4 N        ] " + fmt(fp, mp))
    for name, imp in (("kappa 0.75", replace(rec, kappa=0.75)), ("kappa 1.0", replace(rec, kappa=1.0)),
                      ("k_rec 0.015", replace(rec, k_rec=0.015)), ("k_rec 0.06", replace(rec, k_rec=0.06)),
                      ("k_shock 0.4", replace(rec, k_shock=0.4)), ("b2 1.5", replace(rec, b2=1.5e-3)),
                      ("b2 3.0", replace(rec, b2=3.0e-3)), ("eye 9", replace(rec, d1=9e-3)),
                      ("eye 11", replace(rec, d1=11e-3)), ("z 9", replace(rec, z=9)), ("z 16", replace(rec, z=16)),
                      ("seal 0.2/0.2", replace(rec, s_face=0.2e-3, neck_gap=0.2e-3)),
                      ("seal 0.4/0.3", replace(rec, s_face=0.4e-3, neck_gap=0.3e-3))):
        fp, mp, lim = best_speed(imp, sealed, Motor(**F400))
        print(f"{name:19} [{lim:11}] " + fmt(fp, mp))
    for k, mot in (('100 kHz (now)', Motor()), ('400 kHz', Motor(**F400))):
        print(f"-- PWM table, {k}, nominal motor, sealed skirt, 12.3 V --")
        for d in (0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0):
            fp, mp = at_duty(rec, sealed, mot, d)
            print(f"duty {d:3.1f}: " + fmt(fp, mp))
    print("-- speed held by the firmware (duty from the battery voltage), 400 kHz, against R --")
    for R in (0.35, 0.55, 0.8, 1.5):
        for rpm in (40000, 50000, 60000):
            fp = fan_point(rec, sealed, rpm * RPM)
            mp = motor_point(Motor(R=R, **F400), fp)
            print(f"R {R:4.2f} " + fmt(fp, mp))
    for v in (12.3, 11.1, 10.5):
        fp = fan_point(rec, sealed, 55000 * RPM)
        mp = motor_point(Motor(**F400), fp, v_bat=v)
        print(f"55k rpm at {v} V: duty {mp['duty']*100:3.0f}%")
    for v in (12.3, 11.1):
        fp, mp = at_duty(rec, sealed, Motor(**F400), 1.0, v_bat=v)
        print(f"100% duty at {v} V: " + fmt(fp, mp))

    print("\n== traction (87 g; CoP x 7.9, front skid at x 45 carries F*7.9/45) ==")
    ke_d = 12 / (18000 * RPM)  # drive motors (firmware: 18k rpm at 12 V, 16.06 ohm with the bridge, 19.63 V)
    for f in (1, 2, 3, 4, 6, 8):
        s = f / 3.93e-3
        skid = f * 7.9 / 45
        tire = (ROBOT_MASS * G + f - skid) / 2
        flap = s * 1.0e-3 * 0.29 * 0.5
        print(f"F {f} N: suction {s:5.0f} Pa, per tire {tire:4.2f} N, skid {skid:4.2f} N, traction (mu 1) "
              f"{2*tire:4.2f} N = {2*tire/ROBOT_MASS:4.0f} m/s2, drag: rolling (c_rr 0.03) {0.03*2*tire*1e3:3.0f} mN, "
              f"skid (mu 0.25) {0.25*skid*1e3:3.0f} mN, skirt (mu 0.3) {0.3*flap*1e3:3.0f} mN, tire sink "
              f"{tire/6000*1e3:4.2f}-{tire/4000*1e3:4.2f} mm")
    for v in (0.0, 1.0, 2.0, 3.0, 4.0):
        wm = v / 0.01104 * 36 / 7
        i = (19.63 - ke_d * wm) / 16.06
        print(f"drive force at {v} m/s (both wheels, full voltage): {2 * i * ke_d * 36 / 7 / 0.01104:4.2f} N")
    print("\n== structure (ABS-like resin 1150 kg/m3, UTS 35 MPa) ==")
    for imp, lab in ((base, "Ø26.4"), (rec, "Ø22"), (replace(rec, d2=20e-3), "Ø20")):
        for rpm in (40000, 55000, 70000, 87000):
            st = structure(imp, rpm)
            print(f"{lab} {rpm:6d} rpm: u2 {st['u2']:5.1f} m/s  hoop {st['s_hoop']/1e6:5.2f} MPa (SF {st['sf']:4.1f})")
        print(f"{lab}: mass {imp.mass()*1e3:4.2f} g, safe rpm at SF 4 on 35 MPa: {safe_rpm(imp):6.0f}")
    for e in (5e-6, 10e-6, 25e-6, 50e-6):
        for rpm in (42500, 55000):
            w = rpm * RPM
            print(f"unbalance e {e*1e6:3.0f} um at {rpm} rpm (Ø22, {rec.mass()*1e3:.2f} g): {rec.mass()*e*w*w:5.3f} N "
                  f"rotating at {rpm/60:4.0f} Hz; ISO 1940 grade G{e*w*1e3:4.1f}")


if __name__ == "__main__":
    main()
