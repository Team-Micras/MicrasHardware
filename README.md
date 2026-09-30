# Micras chassis

A parametric chassis for the Micras micromouse, written as [build123d](https://github.com/gumyr/build123d) code. Every dimension lives in `micras/params.py` and in the `*Params` dataclasses at the top of each part module. Change a value, then rerun the checks and the exports.

The old SolidWorks files in the repository root belong to the previous design and are kept only for reference.

## Workflow

```sh
uv sync                                   # Python 3.12 environment
uv run tools/export_board.py              # re-import the board after it changes in ../hw_debug (needs Windows KiCad); drops the old sensor casing (--keep-casing keeps it)
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
| `fan.py` | closed radial impeller (Ø26.4, eye 11, 12 blades, neck into the board hole), symmetric fan mount (two legs rooted high on the collar, collet collar) | resin |
| `frame.py` | frame geometry: walled battery box with the lid screw boss, cap posts, fan "airbox" tube | PETG |
| `body.py` | top body (frame + short faceted cowl over the fan motor) and the lid with gills, fin and rear wing | PETG |
| `front.py` | four wall-sensor caps that stand on the footprint outlines and set the LEDs' aim; the press-fit front bumper | black resin; bumper TPU |
| `skirt.py` | skirt cutting pattern (0.05–0.1 mm PET or Kapton film) | film |

`build/parts.md` lists each part's mass and print orientation. The printed parts come to about 18 g, and the whole robot to about 88 g (motors 9.61 × 20.3, as measured in `components.md`).

## Key design decisions

- **Board contact.** Only the two L-shaped silkscreen zones carry the bearing blocks, with two countersunk M2 screws per side from below the board. Only a few other parts touch the board: the fan mount's two feet (outside the zones, as agreed), the sensor caps on the casing outlines their footprints draw, and the bumper on the board's front edges and a thin strip behind the front edge. `check_layout.py` enforces this.
- **Encoder alignment.** The axle sits 9.00 mm above the board top, so it lines up with the AS5047U. The magnet (Ø6×2) is 0.5 mm from the package face, giving about 60 mT for N35 and 66 mT for N42 at the Hall elements when centred; the chip's window is 35–70 mT, so with an N42 magnet it must stay centred (0.3 mm off-centre adds about 20 mT).
- **Axial stack.** Along each axle: magnet cup, housing shoulder, bearing, a 0.5 mm housing ridge, bearing, housing lip, race spacer, brass gear, hub. The ridge spreads the two bearings (3.0 mm between centres) for a stiffer support of the overhung wheel. Both bearings are retained in both directions, so the magnet cannot be pushed into the chip.
- **Backlash.** Each motor sits in an eccentric sleeve with 0.3 mm eccentricity, giving ±0.3 mm of center-distance adjustment. After adjusting, the left sleeve is clamped by its cap (the cap's split face is relieved by 0.15 mm, so tightening it squeezes the bearings and the sleeve) and the right one by the ring clamp screw. To print the fixed-bore variant instead, set `Layout.backlash_mode = "fixed"`; `Gears.center_adjust` then tunes the center distance.
- **Battery.** Three cells on edge, one behind the other, in a PETG box with walls on all four sides and a lid, so the cells cannot fly out in a spin. The walls hold the cells sideways, so the lid only has to stop them lifting: two screws on the centre line hold it, in bosses blended into the front and rear walls; the front lug lies flush in a pocket in the nose top, like a hood pin. The edge and pyramid arrangements have the same yaw inertia; edge is 2 mm lower (`battery_study.py`). `Battery.x` puts the center of mass over the axle (`mass_report.py`).
- **Motor layout.** Both motors sit behind the axle. Moving the raised motor in front of the axle was evaluated: it lowers the pack by about 10 mm but pushes the battery 4 mm further back and raises yaw inertia by 5.5 %.
- **Car body.** The top is a short faceted cowl: level with the lid's top where they meet (the lid sits on the walls), then one flat 33° plane down over the fan motor, which pokes through it; the cowl ends just in front of the motor. The body prints lying on that plane, so the visible cowl comes out smooth (`tools/overhang.py body --down 0.647,0,1`); only the battery-box wall tops (under the lid), the lid lip and the floor ribs need supports. The lid carries the fin and rear wing.
- **Bumper.** A TPU band hugs the board's front edge and the first 5 mm of both diagonal edges. The board's nose widens backwards at about 27°, so pushing the band on wedges it tight (0.1 mm interference); a lip over the free strip behind the front edge sets its height. Crash loads go into the board edge, and the band also closes the skirt across the front.
- **Wall sensors.** The SFH 4550 emitter (±3°) and TPS601A receiver (±10°) are already narrow, so aiming them matters far more than shaping their beams: a 1.5° pitch error changes the reading by 10–40 % up close, while an aperture in front of the lenses only cuts the signal (a Ø3 aperture loses about 70 %). Each black-resin cap stands on the casing outline the footprint draws, and fixes the LEDs' height and pitch from the board. Crush ribs grip each LED at its flange and body, a fork around the four soldered legs sets the sideways position and yaw, and the front stays open at full lens width behind a 1 mm hood. `FrontParams.emitter_tilt` pitches the emitter towards the receiver: an optics model predicts 15–50 % more signal at 10–40 mm for 1–2°, so print 0°, 1° and 2° caps and compare on the bench. The sensors can't see a wall closer than about 8 mm from the caps whatever the cap does, because the emitter sits 6.5 mm above the receiver.
- **Fan.** The motor runs 18k rpm with no load at 12 V, so it is speed-limited, and the fan study says the impeller should be as large as possible with a small eye. The Ø26.4 impeller is the largest that fits: a raised Ø33 would hit the encoder daughterboards and the raised drive motor. Its front shroud runs 0.3 mm above the clean Ø27 ring (the inlet seal) and a short neck dips into the board hole. The study predicts about 1 N with the skirt, not the firmware's 3 N; the skirt is what makes suction work, so seal it well and tape over the encoder slots. The body's airbox presses the fan mount down and squeezes its slotted collar onto the motor, with no screws.

## Fasteners (M2×5 countersunk + M2×2 inserts only)

Every joint uses the same screw and insert: 13 of each in total. Each head seat is placed so the 5 mm screw engages the full 2 mm of its insert.

| Joint | Qty | Insert in | Notes |
|---|---|---|---|
| board → bearing-block bases | 4 | base pads (resin, glued) | from under the board, heads in the board's countersinks |
| caps → bases | 4 | bases (resin, glued) | heads counterbored into the cap bosses |
| right motor ring clamp | 1 | lower clamp ear (resin, glued) | locks the right eccentric sleeve |
| frame → caps | 2 | cap bosses (resin, glued) | left one down the post, through the box floor |
| lid → body | 2 | bosses on the centre line, in front of and behind the battery box (PETG, heat-set, Ø3.1 holes) | thick lugs; the front one sits in a pocket in the nose top |

No screw holds these; they're pressed, glued or clamped instead:
- motor pinions and brass wheel gears on their shafts: press fit
- magnets: glued in their cups
- hubs: glued to the gear faces
- tires: stretched onto the hubs
- impeller: pressed onto the shaft and glued
- sensor caps: crush ribs on the LEDs, a drop of glue on the base
- bumper: press fit on the board's nose
- fan motor: held in the fan mount's collar, squeezed by the body's airbox (collet)

## Assembly

1. **Board.** Trim the THT leads flush on the underside: the board is only 1 mm off the floor. Tape the skirt to the underside: the inner 3 mm band gets tape, and the outer margin is bent down.
2. **Inserts.** Glue M2 inserts into the resin parts with CA or epoxy. Heat-setting does not work in resin. The inserts go in:
   - the bases: board and cap screws
   - the caps: frame boss and right ring clamp
   - the body's lid boss (PETG, so this one can be heat-set)
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
9. **Sensor caps.** Slide each one onto its LED pair from the front, along the direction that sensor looks, until the fork takes the legs and the bores seat on the LED flanges. Press the base flat on the board and fix it with a drop of glue.
10. **Body.** Lower it on. The airbox slides over the fan motor and its tapered bottom seats on the fan mount's collar. Screwing the body to both cap bosses presses the mount onto the board and clamps the motor.
11. **Battery.** Put the cells in the box and run the wires out of the end-wall slots. Lay the lid on the walls, its front lug in the nose pocket, and screw both lugs.
12. **Bumper.** Push the TPU bumper straight back onto the board's nose, its lip over the board top, until it sits tight on the diagonal edges.

## Still to measure

These values are estimates in the parameters, and the design should be updated once they're measured:

- Motor front boss diameter and length.
- How far the solder tabs stick out behind the motor.
- Inner race diameter of the bearings. The race spacer and magnet-cup boss are Ø2.7.
- Populated board mass, estimated at 15 g.
- Fits to tune with test prints (all are parameters): `bearing_fit`, `sleeve_fit` and `split_relief` (drive), `insert_d` (3.35 glued in resin, 3.1 heat-set in PETG), `collet_squeeze`, the sensor caps' `rib_interf` and `slot_w`, the bumper's `grip`.
- Real downforce, to be measured on the scale rig. See the fan research notes in the commit history or ask Claude.
