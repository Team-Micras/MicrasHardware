"""Regenerate the board reference files from the KiCad project.

Produces, in ref/:
  board.step       populated board from kicad-cli (component models are embedded in the .kicad_pcb)
  board.stl        same, as a mesh for rendering
  board_mech.json  outline, holes, slots and chassis contact zones, in the robot frame

Robot frame (same as the firmware): origin on the floor under the wheel-axle midpoint,
x forward, y left, z up, millimetres.

Usage: uv run tools/export_board.py [path/to/MicrasMainBoard.kicad_pcb]
"""

import json
import math
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PCB = Path(sys.argv[1] if len(sys.argv) > 1 else ROOT.parent / "hw_debug/kicad/MicrasMainBoard.kicad_pcb")
KICAD_CLI = Path(os.environ.get("KICAD_CLI", "/mnt/c/Users/cosme/AppData/Local/Programs/KiCad/10.0/bin/kicad-cli.exe"))
WIN_TMP = Path(os.environ.get("KICAD_TMP", "/mnt/c/Users/cosme/AppData/Local/Temp/micras_board"))

# KiCad position of the wheel axle midpoint (encoder slots / wheel notches centre line).
AXLE_KX, AXLE_KY = 148.5011, 113.5036
BOARD_BOTTOM_Z = 1.0  # floor to board bottom


def to_robot(kx, ky):
    return (round(AXLE_KY - ky, 4), round(AXLE_KX - kx, 4))


def parse_sexpr(text):
    tokens = re.findall(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+', text)
    stack = [[]]
    for t in tokens:
        if t == "(":
            stack.append([])
        elif t == ")":
            done = stack.pop()
            stack[-1].append(done)
        else:
            stack[-1].append(t.strip('"'))
    return stack[0][0]


def child(node, key):
    for n in node:
        if isinstance(n, list) and n and n[0] == key:
            return n
    return None


def xy(node, key):
    c = child(node, key)
    return (float(c[1]), float(c[2])) if c else None


def arc_center(s, m, e):
    (x1, y1), (x2, y2), (x3, y3) = s, m, e
    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    ux = ((x1**2 + y1**2) * (y2 - y3) + (x2**2 + y2**2) * (y3 - y1) + (x3**2 + y3**2) * (y1 - y2)) / d
    uy = ((x1**2 + y1**2) * (x3 - x2) + (x2**2 + y2**2) * (x1 - x3) + (x3**2 + y3**2) * (x2 - x1)) / d
    return ux, uy


def graphics(tree, layer):
    """Board-level lines/arcs/circles on a layer, converted to the robot frame."""
    out = []
    for n in tree:
        if not (isinstance(n, list) and n and n[0] in ("gr_line", "gr_arc", "gr_circle")):
            continue
        if child(n, "layer")[1] != layer:
            continue
        if n[0] == "gr_line":
            out.append({"type": "line", "start": to_robot(*xy(n, "start")), "end": to_robot(*xy(n, "end"))})
        elif n[0] == "gr_arc":
            s, m, e = xy(n, "start"), xy(n, "mid"), xy(n, "end")
            out.append({"type": "arc", "start": to_robot(*s), "mid": to_robot(*m), "end": to_robot(*e)})
        else:
            c, e = xy(n, "center"), xy(n, "end")
            out.append({"type": "circle", "center": to_robot(*c), "r": round(math.dist(c, e), 4)})
    return out


def mounting_holes(tree):
    holes = []
    for fp in tree:
        if not (isinstance(fp, list) and fp and fp[0] == "footprint"):
            continue
        at = child(fp, "at")
        fx, fy = float(at[1]), float(at[2])
        rot = math.radians(float(at[3])) if len(at) > 3 else 0.0
        for pad in fp:
            if isinstance(pad, list) and pad and pad[0] == "pad" and pad[2] == "np_thru_hole":
                drill = float(child(pad, "drill")[1])
                if drill < 2.0:
                    continue  # USB-C pegs
                px, py = xy(pad, "at")
                kx = fx + px * math.cos(rot) + py * math.sin(rot)
                ky = fy - px * math.sin(rot) + py * math.cos(rot)
                holes.append({"pos": to_robot(kx, ky), "drill": drill})
    return sorted(holes, key=lambda h: (-h["pos"][0], -h["pos"][1]))


def export_step():
    WIN_TMP.mkdir(parents=True, exist_ok=True)
    for f in (PCB, PCB.with_suffix(".kicad_pro")):
        shutil.copy(f, WIN_TMP / f.name)
    subprocess.run(
        [str(KICAD_CLI), "pcb", "export", "step", "--subst-models", "--grid-origin", "-f",
         "-o", "board.step", PCB.name],
        cwd=WIN_TMP, check=True, capture_output=True,
    )
    shutil.copy(WIN_TMP / "board.step", ROOT / "ref/board.step")
    # mesh for fast rendering (tessellating the STEP in Python is far slower)
    subprocess.run(
        [str(KICAD_CLI), "pcb", "export", "stl", "--subst-models", "--grid-origin", "-f",
         "-o", "board.stl", PCB.name],
        cwd=WIN_TMP, check=True, capture_output=True,
    )
    shutil.copy(WIN_TMP / "board.stl", ROOT / "ref/board.stl")


def main():
    tree = parse_sexpr(PCB.read_text())
    thickness = float(child(child(tree, "general"), "thickness")[1])
    mech = {
        "source": str(PCB),
        "frame": "robot: origin floor under axle midpoint, x fwd, y left, z up, mm",
        "board_bottom_z": BOARD_BOTTOM_Z,
        "thickness": thickness,
        "grid_origin_kicad": [84.3403, 180.6148],
        "axle_kicad": [AXLE_KX, AXLE_KY],
        "edge": graphics(tree, "Edge.Cuts"),
        "contact_zone_lines": [g for g in graphics(tree, "F.SilkS")
                               if g["type"] != "circle" and -19 < g["start"][0] < 6.5 and abs(g["start"][1]) > 8],
        "holes": mounting_holes(tree),
    }
    (ROOT / "ref").mkdir(exist_ok=True)
    (ROOT / "ref/board_mech.json").write_text(json.dumps(mech, indent=1))
    export_step()
    print(f"thickness {thickness}, {len(mech['holes'])} holes, "
          f"{len(mech['contact_zone_lines'])} contact-zone segments -> ref/")


if __name__ == "__main__":
    main()
