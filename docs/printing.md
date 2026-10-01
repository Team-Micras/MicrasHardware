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

## 1. Calibration print (once, before any part)

One resin print, `build/sliced/calibration.pm4n` (about 50 min, 19 ml). It holds every fit the robot uses at a
range of clearances, so the parts can be printed right the first time. The fits it measures belong to this resin
at this exposure (Anycubic ABS-Like Pro 2, 3.0 s): change either and it has to be printed again. (A first
coupon in Anycubic Standard, `--resin standard`, tests the pipeline but its fits don't carry over.)

### Before printing

1. Room and resin at 25-30 °C (below 20 °C the resin needs about 30 % more exposure). Shake the bottle for
   a minute.
2. Vat: clean film, no cured flakes (run the printer's tank clean if unsure). Fill to well above the minimum line.
3. Level the plate the way Anycubic describes for the Mono 4 (paper under the plate, home, tighten).
4. Copy `calibration.pm4n` to the USB stick and print it. Don't change any setting on the printer.

### After printing

1. Let it drip for 10 min, then lift the pad off the plate with the spatula.
2. Wash in IPA: two baths of 2-3 min (no more than 6 min in all; long soaks swell the resin).
3. Cut the supports off at their tips with flush cutters while the part is still soft (don't pull or twist).
4. Let it dry completely: 30 min, no IPA smell left.
5. Cure briefly: about 2 min per side (or 3 min on the Wash & Cure's turntable). Over-curing makes it brittle
   and shrinks it.
6. Let it rest a day before measuring (or an hour at 50 °C): resin keeps shrinking a little after the cure.

### Reading it

Every tube and slot stands on a tab whose number is its clearance over the nominal size, in
hundredths of a mm (the size is the diameter, or across the flats for the nuts). Measure 1 mm or more above the
bottom: the first layers sit on the supports and are a little off.

Try the real parts and note, for each row:

| Row | Test | Note |
| --- | --- | --- |
| PIN Ø2, Ø5, Ø8 | calipers on each pin, at half height, twice at 90° | the three diameters |
| BRG (bearing Ø5) | push a bearing in by hand | the smallest that takes it with a firm thumb press, and the smallest it just drops into |
| MOT (motor Ø9.61) | slide a motor in | the smallest it slides into with light friction, and the first that is loose |
| SLV (sleeve seat) | turn the "P" peg in each ring | the smallest it turns in smoothly with no play |
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

From the table, the clearances go into the parameters (`DriveParams.solid_bearing_fit`, `motor_fit`, `sleeve_fit`,
`insert_d`, `nut_af`, `WheelParams.axle_fit`, `PrintedGears.pinion_bore`, `FanParams.motor_fit`, the magnet pocket),
and the pins and bores give the light bleed. Then export and slice again; the layout checks rerun on the way
(`tools/check_layout.py` for each drive variant).

## 3. Resin parts (Photon Mono 4, Anycubic ABS-Like Pro 2)

Same preparation and post-processing as the calibration print. The plates, in `build/sliced/`:

| File | What | Notes |
| --- | --- | --- |
| `resin.pm4n` | magnet cups, race spacers, wheel hubs, impeller, fan mount, 2 printed gear pairs | the gears and spacers are tiny: count them off the supports |
| `sensor_caps.pm4n` | the 4 wall-sensor caps and 5 test caps (1-5 dots: three crush-rib grips, the emitter pitched 1° and 2°) | paint them black (below) |
| `drive_<variant>.pm4n` | one drive variant's bearing blocks (and sleeves) | print the variant you want to try (below) |

### The four drive variants

The rest of the robot is the same for all of them.

| Variant | Blocks | Motors | Screws |
| --- | --- | --- | --- |
| `drive_sleeve_split` (default) | base + cap per side; the cap clamps the bearings and the left sleeve | eccentric sleeves: turn them to set the gear backlash | 4 cap screws + 1 ring clamp |
| `drive_sleeve_solid` | one piece per side; the bearings press in from either end against the ridge | eccentric sleeves, each locked by a ring clamp | 2 ring clamps |
| `drive_nosleeve_split` | base + cap | straight in the blocks (the backlash is what the print gives) | 4 cap screws + 1 ring clamp |
| `drive_nosleeve_solid` | one piece | straight in their ring clamps | 2 ring clamps |

### Finishing the resin parts

- Glue the M2 inserts with a drop of CA or epoxy (resin is a thermoset: heat-setting cracks it).
- Magnet cups: the boss that bears on the bearing's inner race is the supported face; lap it flat on fine
  sandpaper on glass.
- Race spacers: they're 0.55 mm thick (11 layers, a 0.33 mm wall: cut them off the supports gently). Measure them; if one is thicker, lap it on fine sandpaper on
  glass. A 2 mm shim washer that clears the outer race works too.
- Impeller: it prints tilted, so its bore is undersize on purpose: ream it to 0.97-0.98 mm and balance it.
- Gears: drill the bores to size (1.0 pinion, 2.0 wheel) if the calibration didn't give a direct fit, and glue.
- **Sensor caps:** most black paints let infrared through. Use a carbon-black paint (matte black acrylic or
  enamel with carbon/lamp black pigment, or a black permanent marker for the bores), two thin coats outside and in
  the bores, and check it before fitting: shine the emitter at the receiver through a painted cap wall. The
  reading should not change from that with the cap removed and the path blocked by a finger. The crush ribs take
  the paint's thickness.

## 4. FDM parts (Ender 3 V3 SE)

| File | What | Notes |
| --- | --- | --- |
| `basket.gcode` | battery basket, PLA, upside down, no supports | about 30 min; the floor bridges between the end walls |
| `bumper.gcode` | bumper, TPU 95A, 100 % infill | about 6 min; dry the TPU first (4 h at 50 °C) |

Before the first print: run the printer's auto-levelling and set the Z offset on a first-layer test (the stock
start G-code loads the stored mesh with `M420 S1`). Clean the PEI sheet with IPA. For TPU, loosen the extruder's
tension a little if it skips, and print with the spool where the filament runs freely.

## How the settings were chosen

Resin (`tools/slicing/mono4.ini`, `resin_abs_pro2.ini`):

- **Orientation:** every critical bore and pin has its axis vertical (each layer then holds the whole circle at
  the 17 µm pixel resolution; tilted bores come out stepped, oval and skewed by the light that cures through to the
  layer below). Supports go on hidden faces; blind pockets open towards the plate or are vented (a cavity opening
  towards the film becomes a suction cup). The impeller is the exception: tilted 45° so its shroud's inner
  overhangs can be supported; its bore is reamed.
- **Supports:** 0.4 mm tips (0.3 is borderline on an FEP film), dense, 3 mm above a pad (the bottom layers'
  over-cure stays in the pad).
- **Exposure:** Anycubic's settings table for ABS-Like Pro 2 on the Mono 4 (3.0 s, 5 bottom layers at 35 s), at
  25-30 °C. ABS-like resin is tough enough for the press fits, slit clamps and the 0.5 module gears; Standard is brittle.
  Anti-aliasing off (its grey edge pixels mostly don't cure and shift the edges; at 17 µm it gains nothing here).
- **Motion:** slow lifts (1 mm/s up, 2 mm/s down) so the soft fresh layers and the support tips bend less, and a
  2 s rest before each exposure so the resin film settles (even layers).

FDM (`ender3v3se.ini`, `pla.ini`, `tpu.ini`): the V3 SE's machine values and start G-code from Creality's
OrcaSlicer profile; Arachne perimeters; PLA at 210 °C with full cooling and 25 mm/s bridges; TPU at 228 °C,
slow (3.5 mm³/s), short retraction.

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
| basket floor sags or strings | bridges | check cooling (100 %), lower `bridge_speed` in `pla.ini` |
| TPU skips or tangles | extruder pressure, wet filament | dry it; slower `filament_max_volumetric_speed` in `tpu.ini` |
