# Printing the Micras chassis

Everything goes through two commands: `tools/export.py` writes every part, turned to its print orientation, and
`tools/slice.py` lays out the plates and slices them into files the printers run (`.pm4n` for the Photon Mono 4,
`.gcode` for the Ender 3 V3 SE), with the profiles in `tools/slicing/`. The tools are open source and run on
Linux: PrusaSlicer slices both printers (supports for the resin parts), and UVtools converts the resin files to
the Mono 4's format. `tools/setup_print_tools.sh` installs them.

```sh
tools/capped.sh uv run tools/export.py              # build/print/<group>/*.stl
tools/capped.sh uv run tools/slice.py               # build/sliced/*.pm4n, *.gcode (and the plates as .3mf)
tools/capped.sh uv run tools/slice.py calibration --copy-to E:   # one plate, copied to the USB stick at E:
```

## 0. Exposure test (first, and whenever the resin changes)

The first parts came out with every hole too tight, much tighter than the flat calibration coupon predicted: the
exposure was too long for parts this thick. `tools/rerf.py` makes `R_E_R_F.pm4n` (about 45 min, 21 ml): with exactly
that file name, the Mono 4 runs Anycubic's exposure range finder and exposes 8 zones of the screen at 2.0, 2.25 …
3.75 s. Each zone gets one coupon (`fit_test.exposure_coupon`), printed on supports like the parts: a 14 × 38 × 3
block with holes at the exact sizes of the bought parts (motor Ø10, magnet Ø6, bearing Ø5, M2 nut, insert Ø3.2,
axle Ø2, motor shaft Ø1), a comb of 0.2–0.5 slots, and its number on top.

```sh
tools/capped.sh uv run tools/rerf.py --copy-to E:
```

Prepare, wash and cure it like the calibration print below. Then, for each coupon, try the real parts in its holes,
caliper its outside (14.00 × 38.00) and see which slots are open, and fill in the exposure table in
`docs/print_check.md`. The best exposure has the outside closest to size and its holes the least undersize while
the slots and edges are still sharp; the fits are then set at that exposure.

**Result (2026-10-03):** the outsides printed true at every exposure (37.94-38.02 x 13.93-14.04); the slots closed as
the exposure rose (all four open at 2.0 s, one at 3.75 s); at 2.0 s the bearing, motor, magnet, insert and motor shaft
each pressed into its nominal hole with some force (the axle hardest: its cut ends were rough). The parts now print
at 2.0 s and the fits in the code are set from that block.

**Second round (2026-10-03):** at 2.0 s the fits were good, but the parts were weak: some layers parted, the
impellers warped, the sensor caps and magnet cups felt fragile. The parts now print at 2.5 s, with every hole in the
code 0.05 bigger than the fits that worked at 2.0 s (at 3.0 s the holes had closed by about 0.1 more than at 2.0 s).

## 1. Calibration print (optional: to check the fits)

The fits are set from the exposure test above. The fit bars (`fit_test.fit_blocks`: one bar per fit, the bought
part tried in holes at several clearances, printed on supports like the parts; `build/sliced/calibration.pm4n`,
1.5 h, 38 ml, with the five sensor-cap variants) are there to check them if a fit turns out wrong. The older flat
coupon described below misled: printed flat with thin walls, its holes came out much looser than the parts'.

### The first, flat coupon (superseded)

## 1. Calibration print (once, before any part)

One resin print, `build/sliced/calibration.pm4n` (about 30 min, 7 ml), flat on the build plate with no supports. It holds every fit the robot uses at a
range of clearances, so the parts can be printed right the first time. The fits it measures belong to this resin
at this exposure (Anycubic ABS-Like Pro 2, 3.0 s): change either and it has to be printed again.

### Before printing

1. Room and resin at 25-30 °C (below 20 °C the resin needs about 30 % more exposure). Shake the bottle for
   a minute.
2. Vat: clean film, no cured flakes. After any failed print, run the printer's tank clean (it cures a layer over
   the whole film that you peel off with the debris) or sieve the resin: a cured flake under the plate can
   puncture the film. Fill to well above the minimum line.
3. Level the plate the way Anycubic describes for the Mono 4 (paper under the plate, home, tighten).
4. Copy `calibration.pm4n` to the USB stick and print it. Don't change any setting on the printer.

### After printing

1. Let it drip for 10 min, then slide the spatula under the coupon's bars from one end and lift it off the plate
   gently (it has no supports).
2. Wash in IPA: two baths of 2-3 min (no more than 6 min in all; long soaks swell the resin).
3. Let it dry completely: 30 min, no IPA smell left.
4. Cure briefly: about 2 min per side (or 3 min on the Wash & Cure's turntable). Over-curing makes it brittle
   and shrinks it.
5. Let it rest a day before measuring (or an hour at 50 °C): resin keeps shrinking a little after the cure.

### Reading it

Every tube and slot stands on a tab whose number is its clearance over the nominal size, in
hundredths of a mm (the size is the diameter, or across the flats for the nuts). The digits should read normally
(not mirrored). Measure 1 mm or more above the bottom, and push the test parts in from the top: the first layers
are over-cured on the plate and a little tight.

Try the real parts and note, for each row:

| Row | Test | Note |
| --- | --- | --- |
| PIN Ø2, Ø5, Ø8 | calipers on each pin, at half height, twice at 90° | the three diameters |
| BRG (bearing Ø5) | push a bearing in by hand | the smallest that takes it with a firm thumb press, and the smallest it just drops into |
| MOT (motor Ø9.61) | slide a motor in | the smallest it slides into with light friction, and the first that is loose |
| INS (insert Ø3.2) | drop an M2 insert in | the smallest it goes into without forcing (it will be glued) |
| AXL (axle Ø2) | push the 2 mm axle through | the smallest it slides through freely, and the smallest it pushes through at all |
| SHF (motor shaft Ø1) | push a motor's shaft (or a 1 mm pin) in | the smallest it presses into firmly, and the smallest it slides into |
| MG6, MG4 (magnets) | press each magnet in | the smallest it presses into with a finger |
| NUT (M2 nut) | press a nut in | the smallest it presses into and stays in |
| GAP (slots) | look through each slot at a light | the narrowest slot that is open all the way |

Also note anything that went wrong: a tube missing or torn off, warping, a soft or sticky surface (under-exposed),
lost detail or closed slots (over-exposed). Send the table back: the fits go into the parameters, and every part
is sliced again with them.

## 2. Set the fits

Results of the first coupon in ABS-Like Pro 2 at 3.0 s (2026-10-01): pins Ø2 / Ø5 / Ø8 measured 2.05 / 4.97 / 8.05
(the scale is right; outsides grow about 0.02-0.03 per side); the bearing presses in at +0.06 and drops in at +0.08;
the insert goes in at +0.10; the axle pushes through at +0.06 and slides at +0.10; the Ø4 magnet presses in at
+0.04; slots open from 0.30. So holes print about 0.06 small. The fits now in the parameters:

| Fit | Value | From |
| --- | --- | --- |
| bearing in a one-piece block (`solid_bearing_fit`) | +0.06 | measured: firm press |
| motor in a ring or the fan collar | +0.09 | like the bearing's drop-in |
| insert hole (`insert_d`) | 3.30 | measured |
| magnet cup, hub, race spacer on the axle (`axle_fit`) | +0.10 | measured: slides (glued) |
| printed pinion / wheel gear bores | 1.05 / 2.08 | press on the shaft / push on the axle |
| nut traps (`nut_af`) | 4.05 | hole -0.06 |
| magnet pockets | +0.04 | measured |

The rows not measured yet (MOT, SHF, NUT, MG6) are set from the same hole and outside offsets; check them on the
coupon when you can, and on the parts.

## 3. Resin parts (Photon Mono 4, Anycubic ABS-Like Pro 2)

Everything fits in one print, with the split drive blocks as an optional second (same preparation and
post-processing as the calibration print):

| File | What | Time, resin |
| --- | --- | --- |
| `robot.pm4n` | the whole robot's resin parts with the default drive (one-piece blocks): blocks, wheels (with their printed 36T), magnet cups (Ø4), fan mount, radial impeller. No pinions: the motors carry the brass ones (the brass 36T's wheels and drill guide are in `alternatives/`). The fan mount prints upside down | 2.1 h, 18 ml |
| `drive_split.pm4n` | the other drive variant (base + cap blocks) | 2.1 h, 11 ml |
| `trials.pm4n` | trial parts (`trials.py`): two impellers for the plain Ø1.5 shaft motor (`impeller_d155`, `_d158`: one and two dots), the tire cutting guide (base and ring), four wheel pairs with less backlash (`wheel_bl20/16/12/08`: 0.20 to 0.08 mm with the brass pinion, the value engraved on the gear's face) | 1.8 h, 27 ml |
| `skirt_guide_1.pm4n`, `skirt_guide_2.pm4n` | the skirt's placing jig and nose template, then its ring template (`skirt_guide_trim.pm4n`: the templates for acetate, optional) | 0.9 + 0.8 h, 43 ml |

```sh
tools/capped.sh uv run tools/slice.py resin drive_solid --merge robot
tools/capped.sh uv run tools/slice.py sensor_caps basket   # FDM
tools/capped.sh uv run tools/slice.py drive_split
tools/capped.sh uv run tools/slice.py trials
tools/capped.sh uv run tools/slice.py skirt_guide
```

The parts stand on braced supports (they come off as one piece: cut the tips with flush cutters, don't twist the
parts off) on a pad whose border is lifted off the plate, so the spatula slides under its edge.

### The two drive variants

The rest of the robot is the same for both. The motors sit straight in their rings (the backlash is what the
print gives).

| Variant | Blocks | Motors | Screws |
| --- | --- | --- | --- |
| `drive_solid` (default) | one piece per side; the bearings press in from either end against the ridge | each in a slit ring clamp | 2 ring clamps |
| `drive_split` | base + cap per side; the cap clamps the bearings and the left motor | the right one in a slit ring clamp | 4 cap screws + 1 ring clamp |

### Finishing the resin parts

Every part is turned so that its bores are vertical and its supports land on faces that don't matter; the few
working faces that must face down are flattened by hand (a minute each: fine sandpaper, 600-1000 grit, on glass,
figure-of-eight strokes):

- Cut the supports at their tips with flush cutters while the parts are soft, before the cure; the braced supports
  come away as one piece. Clip any nub flush with a blade.
- Bearing blocks: look into each bearing bore before fitting a bearing and scrape the lip's and the tube step's
  faces flat (a nub there stops the bearing seating); deburr the housing's inboard end (it runs 0.25 mm from the
  magnet cup).
- Motor rings: clear any nub off the ledge at the bore's outer end (the motor's front face sits on it).
- Magnet cups: clean the cone's tip (it bears on the inner race); deburr the flange face.
- Wheels: clean the cone on the end web's inner side (it bears on the outer bearing's inner race).
- Fan mount: it prints upside down, so the tabs' undersides (they seat on the blocks' ears and set the impeller's tilt and its 0.3 mm seal gap) come out clean; don't sand them. Clear the nubs off the tabs' tops so the screw heads seat.
- Inserts: their holes print sideways in the blocks; dry-fit an insert first, and run a 3.3 mm drill through if
  tight, then glue it with a drop of CA or epoxy (resin is a thermoset: heat-setting cracks it).
- Impeller: its bore is the 9T pinion's outline with 0.06 mm all round: it slides onto the pinion, the teeth key
  it; wick in thin CA. If it's tight, clean the bore by twisting the pinion in and out (don't drill it). Balance it
  afterwards (README, assembly).
- Pinions: two bores (the number on the file is the bore in hundredths): use the one that slides on the shaft with
  the least play, and glue it.

## 4. FDM parts (Ender 3 V3 SE)

| File | What | Notes |
| --- | --- | --- |
| `basket.gcode` | battery basket, PLA, upside down, the whole floor on supports from the bed (with a solid interface under it; round 7's left the bridged parts unsupported), 31 min, 4.8 g | the supports sit inside the box and come out through its open side |
| `sensor_caps.gcode` | the four wall-sensor caps and three test caps (1-3 dots: crush ribs 0 / 0.05 / 0.10), black PLA, front face down, no supports, solid (100 % infill), 0.08 mm layers (`pla_caps.ini`) | trim the brim flush with the hood's mouth; push a spare emitter and receiver into the test caps and keep the grip that holds them firmly without forcing (`FrontParams.rib_interf`) |

**Sensor caps (the resin ones, painted, did not work):** black PLA, and check the filament first. Some black
pigments let infrared through: print a 0.8 mm plate (the caps' wall), shine the emitter at the receiver through
it, and the reading should be the same as with the path blocked by a finger. If it isn't, try another black
filament (carbon-black pigmented).

Before the first print: run the printer's auto-levelling and set the Z offset on a first-layer test (the stock
start G-code loads the stored mesh with `M420 S1`). Clean the PEI sheet with IPA.

## How the settings were chosen

Resin (`tools/slicing/mono4.ini`, `resin_abs_pro2.ini`):

- **Orientation:** every critical bore and pin has its axis vertical (each layer then holds the whole circle at
  the 17 µm pixel resolution; tilted bores come out stepped, oval and skewed by the light that cures through to the
  layer below). Supports go on hidden faces; blind pockets open towards the plate or are vented (a cavity opening
  towards the film becomes a suction cup). The impeller is the exception: tilted 45° so its shroud's inner
  overhangs can be supported; its bore is shaped like the pinion, with a small clearance, and glued.
- **Supports:** 0.4 mm tips (0.3 is borderline on an FEP film), dense, 3 mm above a pad (the bottom layers'
  over-cure stays in the pad).
- **Exposure:** 2.5 s, from the exposure test and the parts printed at 2.0 s (section 0; Anycubic's table says 3.0 s, which closed every hole too far, and 2.0 s left the layers weak; the bottom layers at 45 s x 6, harder than its 35 s x 5, for the pad's grip), at
  25-30 °C. ABS-like resin is tough enough for the press fits, slit clamps and the 0.5 module gears; Standard is brittle.
  Anti-aliasing off (its grey edge pixels mostly don't cure and shift the edges; at 17 µm it gains nothing here).
- **Motion:** slow lifts (1 mm/s up, 2 mm/s down) so the soft fresh layers and the support tips bend less, and a
  2 s rest before each exposure so the resin film settles (even layers).

FDM (`ender3v3se.ini`, `pla.ini`): the V3 SE's machine values and start G-code from Creality's
OrcaSlicer profile; Arachne perimeters; PLA at 210 °C with full cooling and 25 mm/s bridges.

Sources: [cross-layer curing and layer bulging](https://blog.honzamrazek.cz/2022/11/cross-layer-curing-and-layer-bulging-on-resin-printers-enemy-of-overall-dimensional-accuracy-and-printed-threads/),
[resin shrinkage and exposure bleeding](https://blog.honzamrazek.cz/2022/06/getting-perfectly-crisp-and-dimensionally-accurate-3d-prints-on-a-resin-printer-fighting-resin-shrinkage-and-exposure-bleeding/),
[Formlabs: model orientation](https://formlabs.com/eu/support/Model-Orientation/),
[Formlabs: suction cups](https://formlabs.com/support/Preventing-suction-cups-in-PreForm/),
[AmeraLabs: support experiment](https://ameralabs.com/blog/experiment-3d-printing-supports/),
[Liqcreate: anti-aliasing and blur](https://www.liqcreate.com/supportarticles/explained-tested-anti-aliasing-aa-and-blur-in-resin-3d-printing/),
[UVtools: rest times](https://github.com/sn4k3/UVtools/wiki/Rest-times-and-TSMC),
[Anycubic: resin settings](https://store.anycubic.com/blogs/news/resin-settings-for-anycubic-3d-printers),
[PrusaSlicer CLI](https://help.prusa3d.com/article/command-line-interface_1694),
[UVtools](https://github.com/sn4k3/UVtools).

## If something goes wrong

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| part missing, pad on the film | supports tore off (peel force) | check the plate level and the film; more support density |
| layers split, soft or sticky surface | under-exposed or cold resin | warm the room/resin to 25 °C; then +0.3 s exposure (reprint the coupon) |
| slots closed, holes small, details bloated | over-exposed | -0.3 s exposure (reprint the coupon) |
| printer won't list the file | the converted .pm4n | open the plate's `.3mf` in Lychee and slice it there with the settings above |
| Ender stops mid-print and beeps (thermal-runaway protection) | the nozzle cooled below its target: the part fan's air on the heater block, or a loose thermistor or heater cartridge | check the heater block's silicone sock and that the thermistor and heater cartridge are tight; test: heat the nozzle to 215, fan at 100 % from the screen, watch the temperature for 3 min (it should stay within 2-3 °C); `pla_caps.ini` runs the fan at 50-70 % |
| basket floor sags or strings | bridges | check cooling (100 %) and that the supports printed; lower `bridge_speed` in `pla.ini` |
