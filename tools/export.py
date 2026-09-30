"""Export print files and the assembly.

  build/stl/<part>.stl     every printed part (robot frame; orient in the slicer per build/parts.md)
  build/micras.step        printed + bought parts (open with ref/board.step in FreeCAD)
  build/skirt.dxf|svg      skirt cutting pattern, 1:1
  build/parts.md           part list with material, mass and print notes
"""
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from build123d import Compound, export_step, export_stl  # noqa: E402

from micras import assembly, body, gears, skirt  # noqa: E402
from micras.mass import DENSITY  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "build"
(OUT / "stl").mkdir(parents=True, exist_ok=True)

printed = assembly.printed()
rows = []
for name, part in printed.items():
    mat, printer, note = assembly.material(name)
    export_stl(part, str(OUT / "stl" / f"{name}.stl"), tolerance=0.01, angular_tolerance=0.1)
    rows.append((name, mat, printer, part.volume, part.volume * DENSITY[mat], note))
# the spoiler is optional: a plain lid to print instead
plain = body.lid(b=replace(body.B, spoiler=False))
export_stl(plain, str(OUT / "stl" / "lid_plain.stl"), tolerance=0.01, angular_tolerance=0.1)
mat, printer, _ = assembly.material("lid")
rows.append(("lid_plain", mat, printer, plain.volume, plain.volume * DENSITY[mat],
             "alternative to lid, without the fin and rear wing; cover on the bed, no supports"))
# printed stand-ins for the brass gears (until the bought ones arrive)
for part in (gears.pinion(), gears.wheel_gear()):
    mat, printer, note = assembly.material(part.label)
    export_stl(part, str(OUT / "stl" / f"{part.label}.stl"), tolerance=0.002, angular_tolerance=0.05)
    rows.append((part.label, mat, printer, part.volume, part.volume * DENSITY[mat], note))
skirt.export(OUT)

bought = assembly.bought()
everything = Compound(children=list(printed.values()) + list(bought.values()), label="micras")
export_step(everything, str(OUT / "micras.step"))

lines = ["| part | material | printer | volume mm³ | mass g | print notes |", "|---|---|---|---|---|---|"]
for name, mat, printer, vol, m, note in rows:
    lines.append(f"| {name} | {mat} | {printer} | {vol:.0f} | {m:.2f} | {note} |")
total = sum(r[4] for r in rows if r[0] != "lid_plain" and not r[0].startswith("gear_"))
lines.append(f"| **total printed** (with the spoiler lid) | | | | **{total:.1f}** | |")
(OUT / "parts.md").write_text("\n".join(lines) + "\n")
print("\n".join(lines))
