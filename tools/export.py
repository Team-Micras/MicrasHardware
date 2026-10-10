"""Export the print files and the assembly.

  build/print/<group>/<part>.stl   every printed part, already turned to its print orientation and standing on the
                                   plate (assembly.PRINT), grouped by printer and material:
                                     resin/        Photon Mono 4 (Anycubic ABS-Like Pro 2)
                                     sensor_caps/  Ender 3 V3 SE, black PLA: the four wall-sensor caps and the
                                                   three test caps (fit_test.led_caps)
                                     calibration/  Photon Mono 4, the fit bars (optional, to check the fits)
                                                   Photon Mono 4, optional: the cutting templates with the lip at
                                                   the trim line, for stiff films (acetate)
                                     alternatives/ Photon Mono 4, optional: the magnet cups for the Ø6x2 magnet and
                                                   the two other impeller styles
                                     fdm_pla/      Ender 3 V3 SE, PLA
                                     drive_<split|solid>/
                                                   the bearing blocks of the two drive variants: split into base +
                                                   cap or one piece. Print one of the two; the rest of the robot is
                                                   the same for both.
  build/print/parts.md             what to print: folder, file, material, copies, mass, notes
  build/micras.step                printed + bought parts in place (the default variant; open with ref/board.step)
  build/skirt.dxf|svg              skirt cutting pattern, 1:1

Usage: tools/capped.sh uv run tools/export.py
"""
import shutil
import sys
from pathlib import Path

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from build123d import Compound, export_step, export_stl  # noqa: E402

from micras import assembly, drive, fan, fit_test, gear_guide, skirt, trials  # noqa: E402
from micras.mass import DENSITY  # noqa: E402
from micras.params import P, override  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "build"
PRINT = OUT / "print"
VARIANTS = ("split", "solid")


def variant_dir(blocks):
    return f"drive_{blocks}"


def group(name, mat):
    if name.startswith("sensor_cap"):
        return "sensor_caps"
    return {"resin": "resin", "pla": "fdm_pla", "petg": "fdm_petg"}[mat]


def write(part, folder, rows, fine=False):
    mat, printer, note = assembly.material(part.label)
    _, _, copies = assembly.print_pose(part.label)
    (PRINT / folder).mkdir(parents=True, exist_ok=True)
    tol = (0.002, 0.05) if fine else (0.01, 0.1)
    export_stl(assembly.oriented(part, part.label), str(PRINT / folder / f"{part.label}.stl"),
               tolerance=tol[0], angular_tolerance=tol[1])
    rows.append((folder, part.label, mat, printer, copies, part.volume * DENSITY[mat], note))


def main():
    for old in (PRINT, OUT / "stl"):  # parts that no longer exist must not linger
        if old.exists():
            shutil.rmtree(old)
    PRINT.mkdir(parents=True)
    default = P.layout.blocks
    rows = []
    printed = assembly.printed()
    for name, part in printed.items():
        if not name.startswith(("block", "impeller")):
            write(part, group(name, assembly.material(name)[0]), rows)
    # the impeller: the default style (FanParams.style) with the robot, the other two styles in the optional
    # alternatives (to compare on the scale, docs/fan_study.md)
    for part in fan.impeller_variants().values():
        chosen = part.label == f"impeller_{fan.F.style}"
        write(part, "resin" if chosen else "alternatives", rows, fine=True)
    # the alternative magnet cups for the Ø6x2 magnet (optional: build/print/alternatives)
    for side in (1, -1):
        write(drive.magnet_cup(side, alt=True), "alternatives", rows)
    # the brass 36T's drill guide (gear_guide.py, optional, with the wheels for it below); the pinions are the
    # bought brass ones
    for part in gear_guide.parts().values():
        write(part, "alternatives", rows)
    # the wheels for the drilled brass 36T (Gears.wheel_gear = "brass", optional): three nut columns, no teeth
    default_gear = P.gears.wheel_gear
    override("gears.wheel_gear=brass")
    for side in (1, -1):
        part = drive.wheel(side)
        part.label = part.label.replace("wheel_", "wheel_brass_gear_")
        write(part, "alternatives", rows)
    override(f"gears.wheel_gear={default_gear}")
    # trial parts on a plate of their own (trials.py): plain-shaft impellers, the tire cutting guide, wheels with
    # less backlash
    for part in trials.parts().values():
        write(part, "trials", rows)
    # the sensor caps' grip in FDM (fit_test.led_caps): printed with the caps, on the same plate
    for part in fit_test.led_caps().values():
        write(part, "sensor_caps", rows)
    # calibration (optional): the fit bars, on supports like the parts
    for part in fit_test.fit_blocks().values():
        write(part, "calibration", rows)
    for blocks in VARIANTS:
        override(f"layout.blocks={blocks}")
        for part in drive.printed().values():
            write(part, variant_dir(blocks), rows)
        print(f"{variant_dir(blocks)} done")
    override(f"layout.blocks={default}")
    skirt.export(OUT)

    bought = assembly.bought()
    everything = Compound(children=list(printed.values()) + list(bought.values()), label="micras")
    export_step(everything, str(OUT / "micras.step"))

    lines = ["| folder | part | material | printer | copies | mass g (each) | print notes |",
             "|---|---|---|---|---|---|---|"]
    for folder, name, mat, printer, copies, m, note in rows:
        lines.append(f"| {folder} | {name} | {mat} | {printer} | {copies} | {m:.2f} | {note} |")
    # the robot as built with the default drive variant (the stand-in gears are left out)
    vd = variant_dir(default)
    total = sum(r[5] for r in rows if not r[1].startswith(("drill_guide", "sensor_cap_test"))
                and not r[0].startswith(("calibration", "alternatives", "trials")) and (not r[0].startswith("drive_") or r[0] == vd))
    lines.append(f"| | **total printed, {vd}** (one impeller, no fit bars, no drill guide) | | | | **{total:.1f}** | |")
    (PRINT / "parts.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
