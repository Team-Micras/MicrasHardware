# Suction-fan CFD (OpenFOAM in docker)

Steady RANS (k-omega SST, MRF) of the Ø22 suction-fan impellers over the board hole, to get fan curves and shaft
torque for `tools/fan_study.py`. Results and discussion: `docs/fan_study.md`, section "CFD results".

OpenFOAM is not installed on the host. Everything runs in the official OpenCFD image, **OpenFOAM v2512**
(`opencfd/openfoam-default:latest`, pulled 2026-10-03; set `CFD_IMAGE` to pin another tag), started by `cfd.sh` with
hard limits (14 GB RAM, no swap, 16 CPUs; `CFD_MEM`, `CFD_CPUS` override). Run one case at a time: WSL dies if the
memory runs out.

## Files

| file | what |
| --- | --- |
| `make_case.py` | writes a case to `build/cfd/runs/<impeller>_<mesh>/` from `template/` and the exported STLs in `build/cfd/` (`impeller_*.stl`, `fan_mount.stl`, `fan_motor.stl`, `geometry.json`; made by `tools/export.py`, not here) |
| `template/` | the OpenFOAM dictionaries, with `@@KEY@@` placeholders |
| `cfd.sh` | host wrapper: `docker run` with limits, mounts `build/cfd` and this folder |
| `run_case.sh` | inside the container: `mesh`, `solve <Pa> <iterations>`, `yplus` |
| `queue.sh` | runs a list of `cfd.sh` (or `walk ...`) jobs one after another, detached (log `build/cfd/runs/queue.log`); refuses to start while another queue runs |
| `walk.sh` | steps one case up the flowing branch (`walk.sh <case> <iterations> <Pa> <Pa> ...`) and stops at the first stage that ends stalled |
| `post.py` | stage averages, convergence and mass-balance checks, plots (`convergence.png` in each case), and the fan-curve JSON for `tools/fan_study.py --cfd` |

## Model

- Domain (generated as a closed STL of revolution): a Ø40 x 1.2 plenum under the board (floor z 0, board underside
  z 1.2), fed through its rim (`inlet`) at a fixed total pressure equal to minus the suction; the Ø15 hole through the
  board; open air above the board top, Ø80 up to z 42 (`ambient`: total pressure 0 in, static 0 out). The impeller,
  the mount (collar, plate, arm) and the motor are cut out of it.
- Rotation: MRF zone (cylinder r < 11.5, z 1.25-7.3, plus r < 5.5 up to z 9.9 around the hub), +z axis
  (counter-clockwise seen from above), 42,600 rpm by default. Only axisymmetric stationary walls (board top and hole,
  plate underside) lie inside the zone (`nonRotatingPatches`).
- Air: incompressible, nu 1.5e-5 m^2/s, rho 1.2 (OpenFOAM's p is p/rho; the scripts convert).
- Mesh: snappyHexMesh on a uniform background `dx` (coarse 3.0, medium 2.4, fine 1.92 mm; ratio 1.25), impeller
  surface and passages at level 4 (dx/16), the face gap and the neck gap at level 5 (dx/32: 3.2 / 4 / 5 cells across
  the 0.3 mm gaps), no prism layers, Spalding wall function (any y+).
- Numerics: SIMPLEC (relaxation p 0.7, U k omega 0.7), linearUpwind for U, limitedLinear for k and omega.
- Monitors (every 5 iterations, `postProcessing/`): moment on the impeller about z (pressure + viscous), flow through
  the inlet and the ambient boundary, flow up through the neck and through the whole hole at z 1.8 (their
  difference is the seal leak coming back down the neck gap), probes, residuals; at write times y+ and lines across
  the gaps.

## Rerun

The study (docs/fan_study.md, "CFD results") ran these stages. Each one continues from the previous state of its case.

```sh
docker pull opencfd/openfoam-default:latest                        # once (about 1 GB)
for imp in radial inducer backward; do tools/capped.sh uv run tools/cfd/make_case.py --impeller $imp --mesh medium; done
tools/capped.sh uv run tools/cfd/make_case.py --impeller radial --mesh coarse   # mesh study
tools/capped.sh uv run tools/cfd/make_case.py --impeller radial --mesh fine

tools/cfd/queue.sh \
  "radial_medium mesh" "radial_medium solve 1000 1500" "radial_medium solve 800 800" "radial_medium solve 600 800" \
  "radial_medium solve 900 800" "radial_medium solve 1200 800" "radial_medium solve 400 800" \
  "radial_medium solve 650 800" "radial_medium solve 700 800" "radial_medium solve 750 800" \
  "radial_medium solve 800 800" "radial_medium solve 850 800" "walk radial_medium 600 900 950 1000"
# inducer_medium / backward_medium: mesh, 1000 (1500 it), 800, 600, 400, 700, 900 (800 it each), then back to the
# flowing branch with 600 (500 it) and up: 750, 800, 850 (700 it), then "walk <case> 600 900 950 1000"
# (backward: "walk backward_medium 500 600 700 775", its branch ends earlier)
# radial_coarse: mesh, 1000 (1500), 600 (800), 750 (700), 800 (700); radial_fine: mesh, 1000 (1800), 600 (1000)

tools/capped.sh uv run tools/cfd/post.py --curves medium           # tables, convergence.png per case,
                                                                    # build/cfd/results.json, curves_medium.json
tools/capped.sh uv run tools/fan_study.py --cfd build/cfd/curves_medium.json
```

Single stages by hand: `tools/cfd/cfd.sh <case> mesh`, `tools/cfd/cfd.sh <case> solve <Pa> <iterations>`,
`tools/cfd/cfd.sh <case> yplus`.

Two side cases were copied from converged states rather than generated:
- `radial_medium_55k`: the fan-law check. It is `radial_medium`'s processor directories at iteration 7900 (750 Pa,
  flowing), with `omega` in `constant/MRFProperties` and `rpm`/`omega` in `case.json` set to 55,000 rpm, and an
  empty `constant/p0Table.steps`. Then `solve 1250 800`.
- `radial_coarse_lam`: the model-form bracket. It is `radial_coarse` at iteration 2300 (600 Pa), with
  `simulationType laminar` and `k`/`omega` dropped from the residual and line monitors. Then `solve 600 800`,
  `solve 750 800`, `solve 800 700`.

Timing on the Core Ultra 7 155H (WSL2): 1.1 / 2.2 / 4.2 s per iteration on the coarse / medium / fine mesh (0.77M /
1.35M / 2.56M cells), 2.3 GB of RAM on medium and 4.4 GB on fine. A medium impeller with its whole curve takes about
6 h.

**Two branches.** Near shut-off these impellers have two steady RANS solutions: a *flowing* branch (the impeller
pumps through the eye) and a *stalled* branch (almost no eye flow; the seal leak runs back down the hole into the
cavity). Which one a stage lands on depends on where it starts, so walk the suction down from 1000 Pa (stalled
side), and back up from 400-600 Pa (flowing side) in steps of 50 Pa (larger steps stall early) to bracket the stall. `post.py` classifies each stage by its eye
flow (above 0.3 L/s: flowing) and drops stages that switched branch part-way (std or drift of the flow over 0.04 L/s).
It builds the curve from the flowing branch up to its highest suction plus the stalled branch above it; `tools/fan_study.py` interpolates linearly across the gap, i.e. as a time-mix of the two
states.

`make_case.py --rpm` and `--suction` change the defaults (42,600 rpm, 1000 Pa from `geometry.json`); `--np` the MPI
ranks (default 12; this laptop is memory-bandwidth bound, 8-16 ranks run at the same speed). Each stage appends a
step to `constant/p0Table` (inlet total pressure against the iteration count) and a line to `stages.txt`;
`post.py` averages each stage's last 400 iterations.
