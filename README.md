# Micras chassis

A parametric chassis for the Micras micromouse, written as [build123d](https://github.com/gumyr/build123d) code. Every dimension lives in `micras/params.py` and in the `*Params` dataclasses at the top of each part module. Change a value, then rerun the checks and the exports.

The old SolidWorks files in the repository root belong to the previous design and are kept only for reference.

## Workflow

```sh
uv sync                                   # Python 3.12 environment
uv run tools/export_board.py              # re-import the board after it changes in ../hw_debug (needs Windows KiCad)
uv run pytest -q                          # design rules: clashes, board contact zones, LED fit, CoM over the axle
uv run tools/show.py                      # send the model to the OCP CAD Viewer (open the viewer panel in VS Code first)
uv run tools/export.py                    # build/stl/*.stl, build/micras.step, build/skirt.dxf|svg, build/parts.md
uv run tools/mass_report.py               # mass, centre of mass, yaw inertia, battery position for balance
uv run tools/battery_study.py             # compare battery arrangements
```

Run heavy jobs through `tools/capped.sh`, for example `tools/capped.sh uv run tools/export.py`. It caps memory at 6 GB so that a runaway job gets killed instead of crashing WSL.

**Robot frame.** The same frame as the firmware: the origin is on the floor under the midpoint of the wheel axle, x points forward, y points left and z points up, in millimetres.

## Parts

| Module | Parts | Material |
|---|---|---|
| `drive.py` | bearing-block base + cap per side, eccentric motor sleeves, magnet cups, race spacers, wheel hubs | resin |
| `fan.py` | closed radial impeller (Ø26.4), symmetric fan mount (two feet, two ears) | resin |
| `frame.py` | frame geometry: walled battery box, cap posts, fan "airbox" tube, lid bosses | PETG |
| `body.py` | one-piece top body (frame + engine-cover fairing + nose cone + front wing with skid) and the lid with gills, fin and rear wing | PETG |
| `front.py` | four slide-on sensor caps | resin |
| `skirt.py` | skirt cutting pattern (0.05–0.1 mm PET or Kapton film) | film |

`build/parts.md` lists each part's mass and print orientation. The printed parts come to about 12 g, and the whole robot to about 82 g.

## Key design decisions

- **Board contact.** Only the two L-shaped silkscreen zones carry the bearing blocks, with two countersunk M2 screws per side from below the board. Only a few other parts touch the board: the fan mount's three feet (outside the zones, as agreed) and the nose, which bears on the board's front edge. `check_layout.py` enforces this.
- **Encoder alignment.** The axle sits 9.00 mm above the board top, so it lines up with the AS5047U. The magnet (Ø6×2) is 1.0 mm from the package face, giving about 53 mT for N35 and 57 mT for N42; the chip's window is 35–70 mT.
- **Axial stack.** Along each axle: magnet cup, housing shoulder, two bearings, housing lip, race spacer, brass gear, hub. Both bearings are retained in both directions, so the magnet cannot be pushed into the chip.
- **Backlash.** Each motor sits in an eccentric sleeve with 0.3 mm eccentricity, giving ±0.3 mm of center-distance adjustment. After adjusting, the left sleeve is clamped by its cap and the right one by the ring clamp screw. To print the fixed-bore variant instead, set `Layout.backlash_mode = "fixed"`; `Gears.center_adjust` then tunes the center distance.
- **Battery.** Three cells on edge, one behind the other, in a PETG box with walls on all four sides and a screwed lid, so the cells cannot fly out in a spin. The edge and pyramid arrangements have the same yaw inertia; edge is 2 mm lower (`battery_study.py`). `Battery.x` puts the center of mass over the axle (`mass_report.py`).
- **Motor layout.** Both motors sit behind the axle. Moving the raised motor in front of the axle was evaluated: it lowers the pack by about 10 mm but pushes the battery 4 mm further back and raises yaw inertia by 5.5 %.
- **Car body.** The top is one PETG part: frame, battery box, a fairing down to the fan airbox, and a hollow nose cone ending in the front wing and skid. The only other top part is the lid, which carries the fin and rear wing. The wing bears on the board's front edge, so crash loads go into the board.
- **Fan.** The closed impeller's front shroud runs 0.3 mm above the clean Ø27 ring, and that gap is the inlet seal. The skirt is what makes suction work: without it the 1 mm gap limits downforce to well under 1 N.

## Fasteners (M2×5 countersunk + M2×2 inserts only)

Every joint uses the same screw and insert: 15 of each in total. Each head seat is placed so the 5 mm screw engages the full 2 mm of its insert.

| Joint | Qty | Insert in | Notes |
|---|---|---|---|
| board → bearing-block bases | 4 | base pads (resin, glued) | from under the board, heads in the board's countersinks |
| caps → bases | 4 | bases (resin, glued) | heads counterbored into the cap bosses |
| right motor ring clamp | 1 | lower clamp ear (resin, glued) | locks the right eccentric sleeve |
| frame → caps | 2 | cap bosses (resin, glued) | left one down the post, through the box floor |
| airbox tube → fan mount | 2 | mount ears (resin, glued) | presses the mount's feet onto the board |
| lid → body | 2 | body bosses (PETG, heat-set) | |

No screw holds these; they're pressed, glued or clamped instead:
- motor pinions and brass wheel gears on their shafts: press fit
- magnets: glued in their cups
- hubs: glued to the gear faces
- tires: stretched onto the hubs
- impeller: pressed onto the shaft and glued
- sensor caps: slide-on friction fit
- fan motor: held in the collar and captured by the airbox lugs

## Assembly

1. **Board.** Trim the THT leads flush on the underside: the board is only 1 mm off the floor. Tape the skirt to the underside: the inner 3 mm band gets tape, and the outer margin is bent down.
2. **Inserts.** Glue M2 inserts into the resin parts with CA or epoxy. Heat-setting does not work in resin. The inserts go in:
   - the bases: board and cap screws
   - the caps: frame boss and right ring clamp
   - the frame's nose boss (PETG, so this one can be heat-set)
3. **Bases.** Screw each base to the board from below with M2×5 countersunk screws through the countersunk holes.
4. **Wheel cartridges** (build on the bench):
   1. magnet in its cup
   2. cup on the axle
   3. two bearings
   4. race spacer
   5. brass gear (press fit)
   6. hub, glued to the gear face
   7. tire, stretched onto the hub

   Drop each cartridge into the base's half-bores.
5. **Left motor.** Put it in its sleeve, with the pinion pressed flush with the shaft end. Lay it in the left base seat, then fit the left cap with two M2×5.
6. **Right motor.** Put it in its sleeve and slide it into the right cap's ring. Fit the cap.
7. **Mesh.** Turn each sleeve with tweezers in its notches until the gears mesh without play but still turn freely. Then lock it: the left sleeve with the cap screws, the right with the ring clamp screw.
8. **Fan.** Press the impeller onto the fan motor shaft, with the bore reamed to 0.97–0.98 mm. Put the motor into the fan mount collar, with its terminal tabs pointing left and right, then stand the mount on its feet.
9. **Sensor caps.** Slide each one onto its LED pair, along the direction that sensor looks.
10. **Body.** Lower it on. The airbox tube slides over the fan motor until its lugs sit on the motor's rear face, and the front wing comes to rest against the board's front edge. Screw it to both cap bosses and to the two fan-mount ears.
11. **Battery.** Put the cells in the box, run the wires out of the end-wall slots, and screw the lid on.

## Still to measure

These values are estimates in the parameters, and the design should be updated once they're measured:

- Motor diameter. The drawing says Ø10; `components.md` says 9.61.
- Motor front boss diameter and length.
- How far the solder tabs stick out behind the motor.
- Inner race diameter of the bearings. The race spacer and magnet-cup boss are Ø2.7.
- Populated board mass, estimated at 15 g.
- Real downforce, to be measured on the scale rig. See the fan research notes in the commit history or ask Claude.
