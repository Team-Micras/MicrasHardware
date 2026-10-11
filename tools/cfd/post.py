"""Post-process the fan CFD cases in build/cfd/runs/ (written by tools/cfd/run_case.sh).

    tools/capped.sh uv run tools/cfd/post.py                 # every case: stage table + convergence plots
    tools/capped.sh uv run tools/cfd/post.py --curves medium # also write build/cfd/curves_medium.json for
                                                             # tools/fan_study.py --cfd

Each case runs in stages (stages.txt: first iteration, last iteration, suction in Pa). Per stage the values are
averaged over the last AVG iterations, with
- the spread (std) over that window and the drift (mean of its second half minus its first half),
- mass balance: inlet + ambient flow (should be ~0) and the hole flow at z 1.8 against the inlet flow.
Signs: q_net = flow drawn from the cavity (m^3/s); q_eye = up through the neck; q_seal = q_eye - q_hole = leak coming
back down the neck gap; torque = -Mz on the impeller (it turns +z), so shaft power = torque * omega.
"""

import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RUNS = ROOT / "build" / "cfd" / "runs"
AVG = 400
SETTLED = 0.04e-3  # m^3/s: std and drift of q_net over the averaging window
EYE_FLOWING = 0.3e-3  # m^3/s up through the neck: above it the impeller pumps (flowing branch), below it stalls


def read_dat(case, fo, fname):
    """Concatenate postProcessing/<fo>/<start>/<fname> over restarts: (names, array sorted by time, last wins)."""
    base = case / "postProcessing" / fo
    if not base.exists():
        return None, None
    rows, names = {}, None
    for d in sorted(base.iterdir(), key=lambda p: float(p.name)):
        f = d / fname
        if not f.exists():
            continue
        hdr = None
        for line in f.read_text().splitlines():
            if line.startswith("#"):
                hdr = line[1:].strip()
                continue
            vals = line.replace("(", " ").replace(")", " ").split()
            if not vals:
                continue
            v = []
            for x in vals:
                try:
                    v.append(float(x))
                except ValueError:
                    v.append(float("nan"))  # solver names, true/false
            if np.isnan(v[0]):
                continue
            rows[v[0]] = v
        if hdr:
            names = [x.strip() for x in hdr.split("\t")] if "\t" in hdr else hdr.split()
    if not rows:
        return names, None
    t = sorted(rows)
    n = min(len(rows[k]) for k in t)
    return names, np.array([rows[k][:n] for k in t])


def series(case):
    """Per-iteration histories of the monitored quantities."""
    out = {}
    _, a = read_dat(case, "inletFlow", "surfaceFieldValue.dat")
    if a is None:
        return None
    out["t"] = a[:, 0]
    out["q_net"] = -a[:, 1]
    for key, fo in (("q_amb", "ambientFlow"), ("q_eye", "eyeFlow"), ("q_hole", "holeFlow")):
        _, b = read_dat(case, fo, "surfaceFieldValue.dat")
        out[key] = np.interp(out["t"], b[:, 0], b[:, 1]) if b is not None else np.full_like(out["t"], np.nan)
    _, b = read_dat(case, "inletPressure", "surfaceFieldValue.dat")
    out["p_inlet"] = np.interp(out["t"], b[:, 0], b[:, 1]) * 1.2 if b is not None else np.nan
    names, m = read_dat(case, "impellerForces", "moment.dat")
    # columns: Time total_x total_y total_z pressure_x pressure_y pressure_z viscous_x viscous_y viscous_z
    out["torque"] = -np.interp(out["t"], m[:, 0], m[:, 3])
    out["torque_p"] = -np.interp(out["t"], m[:, 0], m[:, 6])
    out["torque_v"] = -np.interp(out["t"], m[:, 0], m[:, 9])
    names, m = read_dat(case, "mountForces", "moment.dat")
    out["mount_mz"] = np.interp(out["t"], m[:, 0], m[:, 3]) if m is not None else np.nan
    names, r = read_dat(case, "residuals", "solverInfo.dat")
    if r is not None and names:
        for fld in ("p", "Ux", "Uy", "Uz", "k", "omega"):
            col = f"{fld}_initial"
            if col in names:
                out[f"res_{fld}"] = np.interp(out["t"], r[:, 0], r[:, names.index(col)])
    names, p = read_dat(case, "probes", "p")
    if p is not None:
        out["probes_p"] = np.array([np.interp(out["t"], p[:, 0], p[:, j]) for j in range(1, p.shape[1])]).T * 1.2
    return out


def stages(case):
    f = case / "stages.txt"
    if not f.exists():
        return []
    return [tuple(float(x) for x in l.split()) for l in f.read_text().split("\n") if l.strip()]


def summarise(case, avg=AVG):
    meta = json.loads((case / "case.json").read_text())
    s = series(case)
    res = []
    if s is None:
        return meta, res, None
    for t0, t1, pa in stages(case):
        m = (s["t"] > max(t0, t1 - avg)) & (s["t"] <= t1)
        if m.sum() < 10:
            continue
        idx = np.nonzero(m)[0]
        half = len(idx) // 2

        def stat(key):
            v = s[key][idx]
            return float(np.mean(v)), float(np.std(v)), float(np.mean(v[half:]) - np.mean(v[:half]))

        row = dict(t0=t0, t1=t1, suction=pa, n_avg=int(m.sum()), done=bool(s["t"][-1] >= t1 - 1))
        for k in ("q_net", "q_amb", "q_eye", "q_hole", "torque", "torque_p", "torque_v", "p_inlet", "mount_mz"):
            if k in s and np.ndim(s[k]):
                row[k], row[k + "_std"], row[k + "_drift"] = stat(k)
        row["q_seal"] = row["q_eye"] - row["q_hole"]
        row["p_shaft"] = row["torque"] * meta["omega"]
        for fld in ("p", "Ux", "k", "omega"):
            if f"res_{fld}" in s:
                row[f"res_{fld}"] = float(s[f"res_{fld}"][idx[-1]])
        if "probes_p" in s:
            row["probes_p"] = [float(x) for x in s["probes_p"][idx].mean(axis=0)]
        res.append(row)
    return meta, res, s


def plot(case, s, st):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
    ax[0].plot(s["t"], s["q_net"] * 1e3, label="net from cavity (inlet)")
    ax[0].plot(s["t"], s["q_hole"] * 1e3, "--", label="net up the hole (z 1.8)")
    ax[0].plot(s["t"], (s["q_eye"] - s["q_hole"]) * 1e3, label="seal leak (eye - hole)")
    ax[0].plot(s["t"], s["q_eye"] * 1e3, label="eye")
    ax[0].set_ylabel("L/s")
    ax[0].legend(fontsize=8)
    ax[0].set_ylim(-0.5, max(1.0, float(np.nanpercentile(s["q_eye"][len(s["t"]) // 4:], 99)) * 1e3 * 1.2))
    ax[1].plot(s["t"], s["torque"] * 1e3, label="total")
    ax[1].plot(s["t"], s["torque_p"] * 1e3, label="pressure")
    ax[1].plot(s["t"], s["torque_v"] * 1e3, label="viscous")
    ax[1].set_ylabel("shaft torque mN m")
    ax[1].legend(fontsize=8)
    lo = np.nanpercentile(s["torque_v"][len(s["t"]) // 4:], 1) * 1e3
    hi = np.nanpercentile(s["torque"][len(s["t"]) // 4:], 99) * 1e3
    ax[1].set_ylim(min(0, lo) * 1.3, hi * 1.3)
    ax[2].plot(s["t"], (s["q_amb"] - s["q_net"]) * 1e6, label="ambient out - inlet in (mL/s)")
    ax[2].plot(s["t"], (s["q_net"] - s["q_hole"]) * 1e6, label="inlet - hole (mL/s)")
    ax[2].set_ylim(-20, 20)
    ax[2].legend(fontsize=8)
    for fld in ("p", "Ux", "k", "omega"):
        if f"res_{fld}" in s:
            ax[3].semilogy(s["t"], s[f"res_{fld}"], label=fld, lw=0.7)
    ax[3].legend(fontsize=8)
    ax[3].set_ylabel("initial residual")
    for a in ax:
        a.grid(alpha=0.3)
        for t0, t1, pa in st:
            a.axvline(t0, color="k", lw=0.5, alpha=0.4)
    for t0, t1, pa in st:
        ax[0].text(t0, ax[0].get_ylim()[1] * 0.9, f" {pa:.0f} Pa", fontsize=8)
    ax[3].set_xlabel("iteration")
    fig.suptitle(case.name)
    fig.tight_layout()
    fig.savefig(case / "convergence.png", dpi=80)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cases", nargs="*")
    ap.add_argument("--avg", type=int, default=AVG)
    ap.add_argument("--curves", metavar="MESH", help="write build/cfd/curves_<MESH>.json from the <impeller>_<MESH> cases")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    cases = [RUNS / c for c in args.cases] if args.cases else sorted(p for p in RUNS.iterdir() if (p / "case.json").exists())
    allres = {}
    for case in cases:
        meta, res, s = summarise(case, args.avg)
        allres[case.name] = dict(meta=meta, stages=res)
        print(f"== {case.name}: {meta['impeller']} {meta['mesh']} dx {meta['dx_mm']} mm, {meta['rpm']:.0f} rpm")
        print("  suction  iters      | q_net L/s (std, drift)   | seal L/s | eye L/s | torque mN m (std, drift)   "
              "p/visc     | shaft W | bal in+amb, in-hole mL/s | res p, U")
        for r in res:
            print(f"  {r['suction']:6.0f} {r['t0']:6.0f}-{r['t1']:<6.0f}{'' if r['done'] else '*'}| "
                  f"{r['q_net']*1e3:7.4f} ({r['q_net_std']*1e3:.4f}, {r['q_net_drift']*1e3:+.4f}) | "
                  f"{r['q_seal']*1e3:7.4f}  | {r['q_eye']*1e3:6.3f} | "
                  f"{r['torque']*1e3:6.4f} ({r['torque_std']*1e3:.4f}, {r['torque_drift']*1e3:+.4f}) "
                  f"{r['torque_p']*1e3:5.3f}/{r['torque_v']*1e3:5.3f} | {r['p_shaft']:5.3f}   | "
                  f"{(r['q_amb']-r['q_net'])*1e6:+6.2f}, {(r['q_net']-r['q_hole'])*1e6:+6.2f}         | "
                  f"{r.get('res_p', float('nan')):.1e}, {r.get('res_Ux', float('nan')):.1e}")
        if s is not None and not args.no_plot:
            plot(case, s, stages(case))
    (ROOT / "build" / "cfd" / "results.json").write_text(json.dumps(allres, indent=1))
    if args.curves:
        curves = {}
        for name, d in allres.items():
            m = d["meta"]
            if m["mesh"] != args.curves or not name.startswith(m["impeller"]) or name != f"{m['impeller']}_{m['mesh']}":
                continue
            # settled stages only: a stage that switched branch part-way has a meaningless average
            pts = [r for r in d["stages"] if r["done"] and r["q_net_std"] < SETTLED and abs(r["q_net_drift"]) < SETTLED]
            flowing = [r for r in pts if r["q_eye"] > EYE_FLOWING]
            stalled = [r for r in pts if r["q_eye"] <= EYE_FLOWING]
            s_f = max((r["suction"] for r in flowing), default=-1.0)
            # the flowing branch up to its highest suction, the stalled branch above it (where both exist at the
            # same suction the flowing one is kept); the segment between them is read as a time-mix of the two
            keep = {}
            for r in flowing + [r for r in stalled if r["suction"] > s_f]:
                k = r["suction"]  # a suction run twice on one branch: keep the better settled stage
                if k not in keep or r["q_net_std"] < keep[k]["q_net_std"]:
                    keep[k] = r
            rows = [keep[k] for k in sorted(keep)]
            s_s = min((r["suction"] for r in stalled if r["suction"] > s_f), default=None)
            curves[m["impeller"]] = dict(rpm=m["rpm"], s_pa=[r["suction"] for r in rows],
                                         q_net_m3s=[r["q_net"] for r in rows],
                                         torque_nm=[r["torque"] for r in rows],
                                         q_seal_m3s=[r["q_seal"] for r in rows],
                                         q_eye_m3s=[r["q_eye"] for r in rows],
                                         q_net_std=[r["q_net_std"] for r in rows],
                                         torque_std=[r["torque_std"] for r in rows],
                                         stall_bracket_pa=[s_f, s_s])
            print(f"{m['impeller']}: flowing branch up to {s_f:.0f} Pa, stalled from {s_s} Pa")
        out = ROOT / "build" / "cfd" / f"curves_{args.curves}.json"
        out.write_text(json.dumps(dict(source="tools/cfd/post.py", curves=curves), indent=1))
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
