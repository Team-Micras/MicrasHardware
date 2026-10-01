"""Export the print files and the assembly.

  build/print/<group>/<part>.stl   every printed part, already turned to its print orientation and standing on the
                                   plate (assembly.PRINT), grouped by printer and material:
                                     resin/        Photon Mono 4 (Anycubic ABS-Like Pro 2)
                                     sensor_caps/  Photon Mono 4, the four wall-sensor caps and five test variants
                                                   (painted black after printing)
                                     calibration/  Photon Mono 4, the fit-test coupon (print it first)
                                     fdm_pla/      Ender 3 V3 SE, PLA
                                     fdm_tpu/      Ender 3 V3 SE, TPU
                                     drive_<sleeve|nosleeve>_<split|solid>/
                                                   the bearing blocks (and sleeves) of the four drive variants:
                                                   with or without the eccentric motor sleeves, blocks split into
                                                   base + cap or one piece. Print one of the four; the rest of the
                                                   robot is the same for all of them.
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

from micras import assembly, drive, fit_test, gears, skirt  # noqa: E402
from micras.mass import DENSITY  # noqa: E402
from micras.params import P, override  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "build"
PRINT = OUT / "print"
VARIANTS = [(mode, blocks) for mode in ("eccentric", "fixed") for blocks in ("split", "solid")]


def variant_dir(mode, blocks):
    return f"drive_{'sleeve' if mode == 'eccentric' else 'nosleeve'}_{blocks}"


def group(name, mat):
    if name.startswith("sensor_cap"):
        return "sensor_caps"
    return {"resin": "resin", "pla": "fdm_pla", "petg": "fdm_petg", "tpu": "fdm_tpu"}[mat]


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
    default = (P.layout.backlash_mode, P.layout.blocks)
    rows = []
    printed = assembly.printed()
    for name, part in printed.items():
        if not name.startswith(("block", "sleeve")):
            write(part, group(name, assembly.material(name)[0]), rows)
    # printed stand-ins for the brass gears (until the bought ones arrive)
    for part in (gears.pinion(), gears.wheel_gear()):
        write(part, "resin", rows, fine=True)
    write(fit_test.coupon(), "calibration", rows)
    for part in fit_test.led_caps().values():
        write(part, "sensor_caps", rows)
    for mode, blocks in VARIANTS:
        override(f"layout.backlash_mode={mode}")
        override(f"layout.blocks={blocks}")
        for part in drive.printed().values():
            write(part, variant_dir(mode, blocks), rows)
        print(f"{variant_dir(mode, blocks)} done")
    override(f"layout.backlash_mode={default[0]}")
    override(f"layout.blocks={default[1]}")
    skirt.export(OUT)

    bought = assembly.bought()
    everything = Compound(children=list(printed.values()) + list(bought.values()), label="micras")
    export_step(everything, str(OUT / "micras.step"))

    lines = ["| folder | part | material | printer | copies | mass g (each) | print notes |",
             "|---|---|---|---|---|---|---|"]
    for folder, name, mat, printer, copies, m, note in rows:
        lines.append(f"| {folder} | {name} | {mat} | {printer} | {copies} | {m:.2f} | {note} |")
    # the robot as built with the default drive variant (the stand-in gears are left out)
    vd = variant_dir(*default)
    total = sum(r[5] for r in rows if not r[1].startswith(("gear_", "sensor_cap_test")) and not r[0].startswith("calibration")
                and (not r[0].startswith("drive_") or r[0] == vd))
    lines.append(f"| | **total printed, {vd}** (without the stand-in gears and test caps) | | | | **{total:.1f}** | |")
    (PRINT / "parts.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
