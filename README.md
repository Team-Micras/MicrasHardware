# Micras chassis

A parametric chassis for the Micras micromouse, written as [build123d](https://github.com/gumyr/build123d) code. Every dimension lives in `micras/params.py` and in the `*Params` dataclasses at the top of each part module. Change a value, then rerun the checks and the exports.

The old SolidWorks files in the repository root belong to the previous design and are kept only for reference.

## Workflow

```sh
uv sync                                   # Python 3.12 environment
uv run tools/export_board.py              # re-import the board after it changes in ../hw_debug (needs Windows KiCad); the wall sensors get the datasheet LED models of micras/leds.py (--original-sensors keeps the embedded model)
uv run pytest -q                          # design rules: clashes, board contact zones, LED fit, CoM over the axle
uv run tools/show.py                      # send the model to the OCP CAD Viewer, grouped, with the real board (--simple: boxes)
uv run tools/export.py                    # build/stl/*.stl, build/micras.step, build/skirt.dxf|svg, build/parts.md
uv run tools/mass_report.py               # mass, centre of mass, yaw inertia, battery position for balance
uv run tools/battery_study.py             # compare battery arrangements
uv run tools/check_gears.py [--png f]    # printed gear pair: mesh through a tooth pitch, backlash, binding distance, tooth stress
uv run tools/fea.py [part ...] [--png]    # structural check of the printed parts (about 13 min for all, 1-3 min per part)
```

Run heavy jobs through `tools/capped.sh`, for example `tools/capped.sh uv run tools/export.py`. It caps memory at 6 GB so that a runaway job gets killed instead of crashing WSL.

**Structural check.** `tools/fea.py` meshes each printed part with gmsh and solves linear elasticity with scikit-fem (quadratic tets, pyamg solver) under rough worst-case loads: a 1 m/s crash into a wall (100 g), a 30 cm drop onto the wheels (300 g), side hits, handling pushes and the impeller's spin. It reports the stress exceeded in only 0.1 % of the volume (the raw peak sits in sharp corners, where linear FEA does not converge) against printed-material strengths (resin 35 MPa; PETG 35 MPa along its layers and 12 MPa across them, using the print orientation; TPU 8.6 MPa), and flags anything under a safety factor of 2 (3 for sustained loads). A flag means "look here": the loads are estimates. It runs on the CPU (about 5 GB at most); run it through `tools/capped.sh` with `MEM=5G`. `--dry` only meshes the parts and checks that every support and load finds its faces (seconds).

**Robot frame.** The same frame as the firmware: the origin is on the floor under the midpoint of the wheel axle, x points forward, y points left and z points up, in millimetres.

## Parts

| Module | Parts | Material |
|---|---|---|
| `drive.py` | bearing-block base + cap per side, eccentric motor sleeves, magnet cups, race spacers, wheel hubs | resin |
| `fan.py` | closed radial impeller (Ø26.4, eye 11, 12 blades, neck into the board hole); the fan mount, a yoke hung from the drive caps with a front leg on the MCU and a split collar clamp | resin |
| `frame.py` | battery basket on two posts onto the drive caps, held shut by two velcro straps | PETG |
| `leds.py` | SFH 4550 and TPS601A models from their datasheets (they replace the board model's hand-drawn LEDs) and the numbers the caps are built from | – |
| `gears.py` | printed stand-ins for the brass gears (0.5M 7T pinion, 36T wheel gear), generated with py_gearworks | resin |
| `front.py` | four wall-sensor caps that stand on the footprint outlines and set the LEDs' aim; the press-fit front bumper | black resin; bumper TPU |
| `skirt.py` | skirt cutting pattern (0.05–0.1 mm PET or Kapton film) | film |

`build/parts.md` lists each part's mass and print orientation. The printed parts come to about 16 g, and the whole robot to about 86 g (motors 9.61 × 20.3, as measured in `components.md`).

## Key design decisions

- **Board contact.** Only the two L-shaped silkscreen zones carry the bearing blocks, with two countersunk M2 screws per side from below the board. Only a few other parts touch the board: the sensor caps on the casing outlines their footprints draw, and the bumper on the board's front edges and a thin strip behind the front edge. The fan mount hangs from the drive caps and rests its front foot on top of the MCU, so it never touches the board itself. `check_layout.py` enforces this.
- **Encoder alignment.** The axle sits 9.00 mm above the board top, so it lines up with the AS5047U. The magnet (Ø6×2) is 0.5 mm from the package face, giving about 60 mT for N35 and 66 mT for N42 at the Hall elements when centred; the chip's window is 35–70 mT, so with an N42 magnet it must stay centred (0.3 mm off-centre adds about 20 mT).
- **Printed gears.** Until the brass gears arrive, `gears.py` makes a printable pair with the same module, tooth counts, widths and centre distance. A standard 7-tooth pinion would be deeply undercut, so the pair is profile shifted (+0.45 pinion, −0.45 wheel: the centre distance stays 10.75) and the pinion's tip is cut back 0.2 module (0.24 mm tip land, contact ratio 1.25). They are generated with [py_gearworks](https://github.com/GarryBGoode/py_gearworks) (Apache-2.0, build123d-native: true involutes with the generated undercut, profile shift, backlash, fillets; bd_warehouse's gears have no profile shift). 0.06 mm of backlash; the eccentric sleeves can close the centre distance by about 0.09 mm before the teeth bind (`check_gears.py`). Print them in resin, axis vertical, on supports; drill the bores and glue. Use them as a pair, not mixed with a brass gear. The layout's clearances cover both the brass and the printed tips.
- **Bearing blocks.** Each side is one solid block, like the v1 bearing blocks: an outer plate next to the gear, whose outline is the hull of the bearing housing, the motor seat, the two cap-screw bosses and (on the left) the frame boss, with the housing and the seat running inboard from it; the frame boss is blended into the motor seat's ring. Each cap has an ear in front of its front screw for the fan mount's arm, with an M2 nut trapped underneath. Shallow pockets on the plate's hidden inboard face save a little resin without adding supports. It is split at the axle plane into base and cap.
- **Axial stack.** Along each axle: magnet cup, housing shoulder, bearing, a 0.5 mm housing ridge, bearing, housing lip, race spacer, brass gear, hub. The ridge spreads the two bearings (3.0 mm between centres) for a stiffer support of the overhung wheel. Both bearings are retained in both directions, so the magnet cannot be pushed into the chip.
- **Backlash.** Each motor sits in an eccentric sleeve with 0.3 mm eccentricity, giving ±0.3 mm of center-distance adjustment. After adjusting, the left sleeve is clamped by its cap (the cap's split face is relieved by 0.15 mm, so tightening it squeezes the bearings and the sleeve) and the right one by the ring clamp screw. To print the fixed-bore variant instead, set `Layout.backlash_mode = "fixed"`; `Gears.center_adjust` then tunes the center distance.
- **Battery.** Three cells on edge, one behind the other, in an open PETG basket on two posts screwed to the drive caps' frame bosses. Two velcro straps run over the pack and through a window in the front and rear walls, and close on themselves, so the cells can't lift out in a spin or a flip, and swapping the pack takes no tools. Two posts in a line across the robot would let the basket pitch in a crash, so a saddle rib under the floor on each side rests along the drive cap's top edge. `fea.py` shaped the rest for a 100 g crash, a 100 g side crash and a 300 g drop: full-height walls on all sides, the front and rear walls thickened at the foot (outside, 45° top edge), and a floor of ribs with solid bands under the posts and along all four walls; the left post flares into it. It prints upside down without supports (the floor bridges between the end walls). The edge and pyramid arrangements have the same yaw inertia; edge is 2 mm lower (`battery_study.py`). `Battery.x` puts the center of mass over the axle (`mass_report.py`).
- **Motor layout.** Both motors sit behind the axle. Moving the raised motor in front of the axle was evaluated: it lowers the pack by about 10 mm but pushes the battery 4 mm further back and raises yaw inertia by 5.5 %.
- **No car body.** An earlier version had a one-piece PETG body (battery box with a lid, a faceted nose over the fan motor, fin and rear wing) whose airbox clamped the fan motor. The open layout is about 3.5 g lighter and simpler to print and service; the car body is kept at the git tag `car-body`.
- **Bumper.** A TPU band hugs the board's front edge and the first 5 mm of both diagonal edges. The board's nose widens backwards at about 27°, so pushing the band on wedges it tight (0.1 mm interference); a lip over the free strip behind the front edge sets its height. Crash loads go into the board edge, and the band also closes the skirt across the front. Its lower outer edge is rounded (R1.5), so it rides up over floor seams and tile edges instead of catching; the top stays flat, and the bumper prints upside down on it.
- **Wall sensors.** The SFH 4550 emitter (5 mm epoxy, ±3°) and TPS601A receiver (a TO-18 metal can with a lens and a key tab, ±10°) are modelled from their datasheets in `leds.py`; the board's own sensor model had hand-drawn LEDs, with a receiver copied from the emitter. Their beams are already narrow, so aiming them matters far more than shaping the beams: a 1.5° pitch error changes the reading by 10–40 % up close, while an aperture in front of the lenses only cuts the signal (a Ø3 aperture loses about 70 %). Each black-resin cap stands on the casing outline the footprint draws, and fixes the LEDs' height and pitch from the board. Crush ribs grip each LED at its flange and body (each bore sized for its own part), a fork around the four soldered legs sets the sideways position and yaw, a keyway along the receiver's sleeve takes its flange tab (which fixes its roll), and the front stays open at full lens width behind a 1 mm hood. `FrontParams.emitter_tilt` pitches the emitter towards the receiver: an optics model predicts 15–50 % more signal at 10–40 mm for 1–2°, so print 0°, 1° and 2° caps and compare on the bench. The sensors can't see a wall closer than about 8 mm from the caps whatever the cap does, because the emitter sits 6.5 mm above the receiver.
- **Fan.** The motor runs 18k rpm with no load at 12 V, so it is speed-limited, and the fan study says the impeller should be as large as possible with a small eye. The Ø26.4 impeller is the largest that fits: a raised Ø33 would hit the encoder daughterboards and the raised drive motor. Its front shroud runs 0.3 mm above the clean Ø27 ring (the inlet seal) and a short neck dips into the board hole. The study predicts about 1 N with the skirt, not the firmware's 3 N; the skirt is what makes suction work, so seal it well and tape over the encoder slots. The fan mount is a yoke: the motor sits in a collar, clamped by a split at its top with a crosswise M2 screw into a trapped nut; two arms reach back to the ears on the drive caps (a screw into a trapped nut each), and a front leg rests on the MCU.

## Fasteners (M2×5 countersunk, M2×2 inserts, M2 nuts)

14 screws, 11 inserts and 3 nuts in total. Each head seat is placed so the 5 mm screw engages the full 2 mm of its insert, or a whole nut.

| Joint | Qty | Insert in | Notes |
|---|---|---|---|
| board → bearing-block bases | 4 | base pads (resin, glued) | from under the board, heads in the board's countersinks |
| caps → bases | 4 | bases (resin, glued) | heads counterbored into the cap bosses |
| right motor ring clamp | 1 | lower clamp ear (resin, glued) | locks the right eccentric sleeve |
| basket → caps | 2 | cap bosses (resin, glued) | left one down the post, through the basket floor |
| fan mount → caps | 2 | M2 nut trapped under each cap's ear | down through the arm's tab |
| fan motor clamp | 1 | M2 nut trapped in the clamp ear | crosswise through the split collar |

No screw holds these; they're pressed, glued or clamped instead:
- motor pinions and brass wheel gears on their shafts: press fit
- magnets: glued in their cups
- hubs: glued to the gear faces
- tires: stretched onto the hubs
- impeller: pressed onto the shaft and glued
- sensor caps: crush ribs on the LEDs, a drop of glue on the base
- bumper: press fit on the board's nose
- cells: two velcro straps

## Assembly

1. **Board.** Trim the THT leads flush on the underside: the board is only 1 mm off the floor. Tape the skirt to the underside: the inner 3 mm band gets tape, and the outer margin is bent down.
2. **Inserts.** Glue M2 inserts into the resin parts with CA or epoxy. Heat-setting does not work in resin. The inserts go in:
   - the bases: board and cap screws
   - the caps: frame boss and right ring clamp

   Put an M2 nut in the trap under each cap's fan ear, and one in the fan mount's clamp ear.
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
8. **Fan.** Press the impeller onto the fan motor shaft, with the bore reamed to 0.97–0.98 mm. Put the motor into the fan mount collar, with its terminal tabs pointing left and right, and tighten the clamp screw. Lower the mount in: its front foot lands on the MCU and its tabs on the caps' ears; screw both tabs down.
9. **Sensor caps.** The caps assume the LEDs' legs are bent where `leds.py` puts them: the receiver's 2.0 mm behind its flange (the datasheet's minimum; the old model had 0.85), the emitter's 2.55 mm, with the axes 9.75 and 3.25 mm above the board. Slide each cap onto its LED pair from the front, along the direction that sensor looks, until the fork takes the legs and the bores seat on the LED flanges. Press the base flat on the board and fix it with a drop of glue.
10. **Basket.** Screw it to both cap bosses (the left screw goes down its post).
11. **Battery.** Thread the two velcro straps through the windows. Put the cells in the basket, run the wires out of the slots at the rear of the side walls, and close the straps over the pack.
12. **Bumper.** Push the TPU bumper straight back onto the board's nose, its lip over the board top, until it sits tight on the diagonal edges.

## Still to measure

These values are estimates in the parameters, and the design should be updated once they're measured:

- Motor front boss diameter and length.
- How far the solder tabs stick out behind the motor.
- Inner race diameter of the bearings. The race spacer and magnet-cup boss are Ø2.7.
- Populated board mass, estimated at 15 g.
- Fits to tune with test prints (all are parameters): `bearing_fit`, `sleeve_fit` and `split_relief` (drive), `insert_d` (3.35 glued in resin), the nut traps' `nut_af`, the sensor caps' `rib_interf` and `slot_w` (and check on a real TPS601A which way its tab points: `Tps601a.tab_angle`, read from the datasheet's outline as up and towards pin 1), the bumper's `grip`.
- Real downforce, to be measured on the scale rig. See the fan research notes in the commit history or ask Claude.
