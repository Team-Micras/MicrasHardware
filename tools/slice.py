"""Turn the print files (build/print, from tools/export.py) into files the printers run, with no GUI.

Each plate is laid out here (parts already turned to their print orientation by export.py, copies as in
assembly.PRINT), then
  resin:  PrusaSlicer (SLA mode: supports, pad, layer images) -> .sl1 -> UVtools -> .pm4n for the Photon Mono 4
  FDM:    PrusaSlicer -> .gcode for the Ender 3 V3 SE
with the profiles in tools/slicing/. Output in build/sliced/:
  <plate>.3mf          the plate as laid out (open it in PrusaSlicer or Lychee to look at it or change it)
  <plate>.pm4n         Photon Mono 4 (copy to its USB stick)
  <plate>.gcode        Ender 3 V3 SE (copy to its SD card)

Plates: calibration, resin, sensor_caps, drive_<sleeve|nosleeve>_<split|solid> (one per drive variant),
basket. A plate that doesn't fit the build area is split in two (<plate>_1, <plate>_2).

Usage:
  uv run tools/slice.py                          # every plate
  uv run tools/slice.py calibration              # some plates
  uv run tools/slice.py resin drive_sleeve_split --copy-to E:    # and copy the files to the drive at E:
  uv run tools/slice.py calibration --resin standard              # another resin (tools/slicing/resin_<name>.ini)
  uv run tools/slice.py resin sensor_caps drive_sleeve_split --merge robot   # several plates as one print
Needs prusa-slicer and UVtoolsCmd on the PATH (tools/setup_print_tools.sh installs them).
"""
import argparse
import re
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import numpy as np
import pyvista as pv

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, ROOT.as_posix())
from micras import assembly  # noqa: E402

PRINT = ROOT / "build/print"
OUT = ROOT / "build/sliced"  # (outside build/print: export.py clears that)
PROFILES = ROOT / "tools/slicing"
# printer: (build area x, y, margin, gap between parts' outlines)
RESIN_BED = (153.4, 87.0, 5.0, 4.0)  # the gap leaves room for the pads' brims; the margin for their lifted
# borders, which reach about 4.6 mm out from a part at the pad's top
FDM_BED = (220.0, 220.0, 10.0, 8.0)
PM4N_VERSION = 517  # what Lychee 7.5 writes for the Mono 4


RESIN = "abs_pro2"  # tools/slicing/resin_<name>.ini


def plates(resin=RESIN):
    """{plate: (kind, profiles, [stl files])}"""
    out = {}
    for d in sorted(PRINT.iterdir()):
        if not d.is_dir():
            continue
        files = sorted(d.glob("*.stl"))
        if d.name == "fdm_pla":
            out["basket"] = ("fdm", ["ender3v3se.ini", "pla.ini"], files)
        else:
            out[d.name] = ("resin", ["mono4.ini", f"resin_{resin}.ini"], files)
    return out


def load(path):
    m = pv.read(path).triangulate()
    v = np.asarray(m.points, dtype=float)
    t = m.faces.reshape(-1, 4)[:, 1:]
    return v, t


def turn_flat(v):
    """Turn about z so the longer side runs along x (the beds are widest in x)."""
    w = v.max(0) - v.min(0)
    if w[1] > w[0] * 1.05:
        v = v[:, [1, 0, 2]] * (-1, 1, 1)
    return v - [*(v.min(0)[:2]), 0]


def rotate90(v):
    """Quarter turn about z (a rotation, not a mirror), back to x, y >= 0."""
    v = v[:, [1, 0, 2]] * (-1, 1, 1)
    return v - [*(v.min(0)[:2]), 0]


def lay_out(items, bed, cell=0.5):
    """Bottom-left packing on an occupancy grid, each part in its better quarter turn:
    [(name, verts, tris)] -> plates of [(name, verts, tris, (x, y))]. Parts keep `gap` between their outlines
    and `margin` from the build area's edges."""
    bx, by, margin, gap = bed
    nx, ny = int((bx - 2 * margin) / cell), int((by - 2 * margin) / cell)
    items = sorted(items, key=lambda it: -float(np.prod(it[1].max(0)[:2])))
    plates, grids = [], []

    def place(grid, w, d):
        """Lowest, then leftmost free spot for a w x d cell block (with the gap around it), or None."""
        g = int(np.ceil(gap / cell))
        W, D = int(np.ceil(w / cell)) + g, int(np.ceil(d / cell)) + g
        if W - g > nx or D - g > ny:
            return None
        # free(i, j): the block at (i, j) .. (i + W, j + D), clipped to the grid, is empty
        pad = np.pad(grid, ((0, g), (0, g)))
        s = np.pad(pad.cumsum(0).cumsum(1), ((1, 0), (1, 0)))
        I, J = pad.shape[0] - W + 1, pad.shape[1] - D + 1
        if I <= 0 or J <= 0:
            return None
        occ = s[W:W + I, D:D + J] - s[:I, D:D + J] - s[W:W + I, :J] + s[:I, :J]
        occ = occ[:nx - (W - g) + 1, :ny - (D - g) + 1]
        free = np.argwhere(occ.T == 0)  # (j, i): sorted by y, then x
        return None if len(free) == 0 else (int(free[0][1]), int(free[0][0]), W, D)

    for name, v, t in items:
        best = None
        for vv in (v, rotate90(v)):
            w, d = vv.max(0)[:2]
            for k, grid in enumerate(grids):
                spot = place(grid, w, d)
                if spot and (best is None or (k, spot[1], spot[0]) < (best[0], best[2][1], best[2][0])):
                    best = (k, vv, spot)
                if spot:
                    break
        if best is None:  # a new plate
            grids.append(np.zeros((nx, ny), dtype=np.int32))
            plates.append([])
            for vv in (v, rotate90(v)):
                spot = place(grids[-1], *vv.max(0)[:2])
                if spot:
                    best = (len(grids) - 1, vv, spot)
                    break
            if best is None:
                sys.exit(f"{name} ({v.max(0)[0]:.0f} x {v.max(0)[1]:.0f} mm) doesn't fit the build area")
        k, vv, (i, j, W, D) = best
        grids[k][i:i + W, j:j + D] = 1
        plates[k].append((name, vv, t, (margin + i * cell, margin + j * cell)))
    return plates


def centre(plate, bed):
    """Shift a plate's parts so the group sits in the middle of the bed."""
    lo = np.min([np.array(p) for *_, p in plate], 0)
    hi = np.max([np.array(p) + v.max(0)[:2] for _, v, _, p in plate], 0)
    shift = (np.array(bed[:2]) - (hi - lo)) / 2 - lo
    return [(n, v, t, tuple(np.array(p) + shift)) for n, v, t, p in plate]


def write_3mf(plate, path):
    """Plain 3MF (core spec): one object and one build item per placed part."""
    objs, items, ids = [], [], {}
    for i, (name, v, t, (x, y)) in enumerate(plate):
        name = f"{name}_{i}"  # each copy its own object (PrusaSlicer 2.9.4's SLA mode can crash on instances)
        if name not in ids:
            ids[name] = len(ids) + 1
            vs = "".join(f'<vertex x="{a:.5f}" y="{b:.5f}" z="{c:.5f}"/>' for a, b, c in v)
            ts = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in t)
            objs.append(f'<object id="{ids[name]}" name="{name}" type="model"><mesh><vertices>{vs}</vertices>'
                        f'<triangles>{ts}</triangles></mesh></object>')
        items.append(f'<item objectid="{ids[name]}" transform="1 0 0 0 1 0 0 0 1 {x:.4f} {y:.4f} 0"/>')
    model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             f'<resources>{"".join(objs)}</resources><build>{"".join(items)}</build></model>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.'
                   'openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/'
                   'vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/'
                   'vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        z.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.'
                   'openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" '
                   'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        z.writestr("3D/3dmodel.model", model)


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


def warnings(log):
    """PrusaSlicer's warnings with the lines that follow them (they say what the issue is)."""
    lines = [ln.strip() for ln in log.splitlines()]
    out = []
    for i, ln in enumerate(lines):
        if "warning" in ln.lower():
            out += [ln] + [x for x in lines[i + 1:i + 4] if x and "=>" not in x]
    return out


def slicer_args(profiles):
    return [a for prof in profiles for a in ("--load", str(PROFILES / prof))]


def slice_resin(name, profiles, src):
    sl1, pm4n = OUT / f"{name}.sl1", OUT / f"{name}.pm4n"
    sl1.unlink(missing_ok=True)
    pm4n.unlink(missing_ok=True)
    code, log = run(["prusa-slicer", "--export-sla", "--dont-arrange", *slicer_args(profiles), "-o", str(sl1), str(src)])
    if code != 0 or not sl1.exists():
        sys.exit(f"PrusaSlicer failed on {name} (exit {code}):\n" + "\n".join(log.splitlines()[-15:]))
    run(["UVtoolsCmd", "convert", str(sl1), "pm4n", str(pm4n), "-v", str(PM4N_VERSION)])  # (its exit code lies)
    if not pm4n.exists() or pm4n.stat().st_size == 0:
        sys.exit(f"UVtools didn't write {pm4n.name}")
    sl1.unlink()
    _, props = run(["UVtoolsCmd", "print-properties", str(pm4n)])
    get = lambda key: (re.search(rf"^{key}: (.+)$", props, re.M) or [None, "?"])[1]
    secs = float(get("PrintTime")) if get("PrintTime") != "?" else 0
    warn = warnings(log)
    return pm4n, f"{get('LayerCount')} layers, {secs / 3600:.1f} h, {float(get('MaterialMilliliters') or 0):.1f} ml" \
        if get("MaterialMilliliters") != "?" else f"{get('LayerCount')} layers, {secs / 3600:.1f} h", warn


def slice_fdm(name, profiles, src):
    gcode = OUT / f"{name}.gcode"
    gcode.unlink(missing_ok=True)
    code, log = run(["prusa-slicer", "--export-gcode", "--dont-arrange", *slicer_args(profiles), "-o", str(gcode), str(src)])
    if code != 0 or not gcode.exists():
        sys.exit(f"PrusaSlicer failed on {name} (exit {code}):\n" + "\n".join(log.splitlines()[-15:]))
    text = gcode.read_text(errors="ignore")
    t = re.search(r"estimated printing time \(normal mode\) = (.+)", text)
    g = re.search(r"filament used \[g\] = ([\d.]+)", text)
    warn = warnings(log)
    return gcode, f"{t[1] if t else '?'}, {g[1] if g else '?'} g", warn


def copy_to(files, target):
    """Copy to a Windows drive given as "E:" (mounted at /mnt/e, mounting it if needed) or to a directory."""
    if re.fullmatch(r"[A-Za-z]:", target):
        letter = target[0].lower()
        dest = Path(f"/mnt/{letter}")
        if subprocess.run(["mountpoint", "-q", str(dest)]).returncode != 0:
            print(f"mounting {letter.upper()}: at {dest} (sudo)")
            subprocess.run(["sudo", "mkdir", "-p", str(dest)], check=True)
            subprocess.run(["sudo", "mount", "-t", "drvfs", f"{letter.upper()}:", str(dest)], check=True)
    else:
        dest = Path(target)
    if not dest.is_dir():
        sys.exit(f"{dest} is not a directory (is the stick plugged in?)")
    for f in files:
        shutil.copy(f, dest / f.name)
        print(f"copied {f.name} -> {dest}")
    print("eject the drive in Windows before pulling it out")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("plates", nargs="*", help="plates to slice (default all)")
    ap.add_argument("--resin", default=RESIN, help=f"tools/slicing/resin_<name>.ini (default {RESIN})")
    ap.add_argument("--copy-to", default="", help="drive letter (E:) or directory to copy the sliced files to")
    ap.add_argument("--merge", default="", help="lay the given plates out together as one print of this name")
    args = ap.parse_args()
    if not (PROFILES / f"resin_{args.resin}.ini").exists():
        sys.exit(f"no tools/slicing/resin_{args.resin}.ini")
    all_plates = plates(args.resin)
    if not all_plates:
        sys.exit("no print files: run tools/export.py first")
    for tool in ("prusa-slicer", "UVtoolsCmd"):
        if not shutil.which(tool):
            sys.exit(f"{tool} not found: run tools/setup_print_tools.sh")
    unknown = [p for p in args.plates if p not in all_plates]
    if unknown:
        sys.exit(f"unknown plate(s) {unknown}; plates: {', '.join(all_plates)}")
    OUT.mkdir(parents=True, exist_ok=True)
    if args.merge:
        jobs = {(all_plates[n][0], tuple(all_plates[n][1])) for n in args.plates}
        if not args.plates or len(jobs) != 1:
            sys.exit("--merge needs plates of one printer and material")
        kind, profiles = jobs.pop()
        profiles = list(profiles)
        all_plates = {args.merge: (kind, profiles, [f for n in args.plates for f in all_plates[n][2]])}
        args.plates = [args.merge]
    done, report = [], []
    for name in args.plates or all_plates:
        kind, profiles, files = all_plates[name]
        bed = RESIN_BED if kind == "resin" else FDM_BED
        items = []
        for f in files:
            v, t = load(f)
            v = turn_flat(v)
            items += [(f.stem, v, t)] * assembly.print_pose(f.stem)[2]
        laid = lay_out(items, bed)
        for i, plate in enumerate(laid):
            pname = name if len(laid) == 1 else f"{name}_{i + 1}"
            src = OUT / f"{pname}.3mf"
            write_3mf(centre(plate, bed), src)
            print(f"{pname}: {len(plate)} parts, slicing ...", flush=True)
            out, info, warn = (slice_resin if kind == "resin" else slice_fdm)(pname, profiles, src)
            done.append(out)
            report.append((pname, out.name, info, warn))
    print()
    for pname, fname, info, warn in report:
        print(f"{fname:32s} {info}")
        for w in warn:
            print(f"    {w.strip()}")
    if args.copy_to:
        copy_to(done, args.copy_to)


if __name__ == "__main__":
    main()
