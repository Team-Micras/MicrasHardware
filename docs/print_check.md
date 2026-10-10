# Print check sheet

Fill in the **Result** column: `ok`, `tight`, `loose`, `broken`, or a short note. Measurements in mm. Write `L: … R: …`
when the sides differ. Leave a row empty if you didn't check it.

## Prints

| Print | Failed / missing parts, other problems |
| --- | --- |
| print1_robot (resin) | |
| print2_drive_variants (resin) | |
| skids (resin) | not used: mouse skates instead |
| basket (PLA) | floor bridges failed without supports (rest fine); needs ~3 mm more length |

## Exposure test (R_E_R_F)

For each coupon (its number is on top): does the real part go in its hole? Write `in` (slides), `press`, or `no`.
Then the outside with calipers, and the narrowest slot that's open.

| # | Motor Ø10 | Magnet Ø6 | Bearing Ø5 | Nut | Insert Ø3.2 | Axle Ø2 | Shaft Ø1 | Outside (14 × 38) | Slots open from | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | press | press | press | | press | press (much force) | press | 37.98x13.95 | 4 | |
| 2 | | | | | | | | 37.94x13.93 | 3 | |
| 3 | | | | | | | | 37.95x13.94 | 3 | |
| 4 | | | | | | | | 37.98x14.04 | 3 | |
| 5 | | | | | | | | 38.02x14.00 | 3 | |
| 6 | | | | | | | | 37.97x13.98 | 2 | |
| 7 | | | | in | | | | 38.00x13.99 | 3 | |
| 8 | | | | press | | | | 38.00x13.98 | 1 | |

## Calibration coupon: the rows not measured yet

| Row | Which cell | Result |
| --- | --- | --- |
| MOT | the motor slides in with no play | this part did not fit to the plate |
| SHF | the motor shaft presses in | no one |
| NUT | an M2 nut presses in and stays | 0 |
| MG6 | the Ø6 magnet presses in and stays | 4 |

## Drive (variant used: ______________)

| Part | Check | Design | Result |
| --- | --- | --- | --- |
| Blocks | printed complete, faces clean | | |
| Blocks | bearings: held, no play | Ø5.10 split, Ø5.06 one-piece | don't go in, even with a lot of force |
| Blocks | sleeve turns in its seat (or motor slides in, no-sleeve) | Ø12.05 / Ø9.70 | |
| Blocks | inserts go in; nuts stay in their traps | Ø3.30 / 4.05 AF | inserts ok; nuts need too much force |
| Sleeves | motor slides in to the lip, no play | Ø9.70 | |
| Sleeves | turning them sets the gear mesh | | |
| Magnet cups | magnet fits; slides on the axle | Ø6.04 / Ø2.10 | magnet fits well; the boss around the axle is too thin, broke |
| Race spacers | thickness | 0.55 | too small to handle, didn't survive |
| Wheel hubs | tire sits straight in the channel; wheel runs true | tire Ø22.08 mounted | |
| Gears | teeth complete; bores fit (pinion press, wheel push-on) | Ø1.05 / Ø2.08 | shaft into pinion: extreme force; axle into wheel gear: too much force |
| Gears | mesh: turns freely, no play, no tight spots | | |

## Fan

| Part | Check | Design | Result |
| --- | --- | --- | --- |
| Impeller | blades and shroud complete; bore reamed | Ø0.97–0.98 | |
| Impeller | vibration at full speed (none / some / bad) | | |
| Fan mount | motor slides into the collar; clamp holds | Ø9.70 | |
| Fan mount | tabs sit in the ear recesses, foot on the MCU | | |
| Fan mount | impeller spins without touching | 0.3 gap at the board | |

## Sensor caps

| Part | Check | Result |
| --- | --- | --- |
| W1–W4 | printed complete | |
| W1–W4 | slide on; the fork takes the legs, the tab goes in the keyway | too tight to fit |
| W1–W4 | grip on the LEDs (none / light / firm / too tight) | |
| W1–W4 | base flat on the board, LEDs look straight | |
| W1–W4 | paint blocks IR (phone camera shows no glow through it) | |
| Test caps | best grip (1–3 dots) and best signal (3, 4 or 5 dots) | |

## Skid pads

| Notches | 1 | 2 | 3 | 4 | 5 |
| --- | --- | --- | --- | --- | --- |
| Design thickness | 0.85 | 0.80 | 0.75 | 0.70 | 0.65 |
| Measured | | | | | |

Pair glued: ___ notches. Paper check result: ______________

## Battery basket

| Check | Result |
| --- | --- |
| floor bridges clean, not warped | |
| cells fit (not tight, no rattle) | |
| pegs, feet and screws fit on the caps | |
| strap goes through the lugs | |

## Assembled robot

| Check | Design | Result |
| --- | --- | --- |
| mass with battery | 84 g | |
| board height above the floor, fan off / fan on | 1.2 / ~0.75 | |
| anything rubbing or touching | | |

## Other problems

- Drive motor is Ø10.0 (as its datasheet says), not 9.61.
- Fan motor is Ø9.97 and has a 9T M0.3 pinion on its shaft, 4.6 long.
- Most holes print much smaller than the coupon predicted: exposure (and the coupon method) to be redone.
- Exposure test done: parts now print at 2.0 s, fits set from block 1 (no new calibration print). The axle's cut ends were rough: deburr them.
- Second round at 2.0 s: fits mostly good, but weak layers and warped impellers. Now 2.5 s with every hole +0.05; magnet
  pocket a little tighter; bearings measure 2.57 wide (pockets were 2.5: now 2.67); impeller bore shaped like the
  pinion; sensor caps and magnet cups thicker; the opening under the receiver is straight-sided now.
- Third round (2.5 s): much better. Changes: fan mount without the front leg; split bearing pockets 5.20 (5.13 stopped
  the cap) and a bearing fit bar (fit_BRG, 5.10-5.30) in the robot print; basket peg sockets 2.4; motor in sleeve back
  to 0.10; axle bores 2.08 (loose at 2.15); magnet pocket 6.02; two pinion bores (1.07, 1.03); sensor caps with both
  legs on the board and a loose flat for the emitter's flange; new axle stack: one-piece wheel (gear, drum, end web)
  turning round a tube that carries the outer bearing inside the wheel, bearings 5.9 mm apart, magnet cup with a
  3.4 mm sleeve; no test caps (which dots were best?).
- Test caps: 2 dots (ribs 0.10) gripped best and the 2 deg pitch was good: the caps now use both.
- Bearing fit bar (2.5 s): the bearing slid into the 5.10 hole with no force. Tube and one-piece seats now 5.05.
- Round 3 blocks: split pocket 5.20 (cap closes, no rattle): kept. Tube 5.15: bearing loose, now 5.05.
- Magnet cup on the axle a little loose at 2.08: now 2.04 (the wheel stays 2.08).
- Round 4 feedback: fan clamp nut turned with the screw (its trap broke out of the 4 mm ears): ears now 6 mm, nut
  flats up and down. Right ring clamp had no way to fit its insert (the pocket opened only into the slit): it now goes
  in from under the lower ear. Motor fits: sleeve 0.06 (was 0.10), split no-sleeve 0.06, one-piece no-sleeve 0.15.
  Default drive: split blocks without sleeves.
- Ring clamps: an M2 nut trapped under the lower ear instead of the insert. Fan motor collar 0.10 (0.15 was loose).
- Wheel grip on the axle 1.4 mm (could tilt): outer bearing moved 1.5 mm in, the wheel's boss now grips 2.9 mm; bearings 4.4 mm apart.
- Alternative magnet cups for the Ø4x2 magnet (magnet_cup4_L/R), its face 0.5 mm from the chip like the Ø6 (the field estimate there is over the chip's 70 mT window: check the encoder's readings).
- Board screws into the bases: M2 nuts in side slots (a boss round each hole above the 1.8 mm lift, nut 1.9-3.5 mm above the board) instead of glued inserts.
- Emitters wrapped in electrical tape: the emitter's body bore and ribs 0.2 mm bigger all round (one wrap of ~0.18 mm tape; not over the flange).
- Board-screw nuts lowered to 0.9 mm above the board (like the v1 blocks); still in side slots (a well from above would cut the bearing housing over the front holes).
- Wheel bore on the axle 2.04 (2.08 a little loose). Ø4 magnet cup: 0.75 mm wall round the magnet (head Ø5.5), was the Ø6 cup's outside.
- Cap-screw insert pockets in the split bases 2.3 deep (2.0 was too shallow).
- Upper (right) motor ring, split no-sleeve: 0.10 (0.06 a little tight); the left motor stays 0.06.
- Upper (right) motor ring: 0.15 (0.10 from round 4 should be bigger; the 0.06 in between was my mistake, applying the left seat's fit to both).
- Round 5 feedback: the motor rings were thicker than needed (drawn for the sleeves) and the motors needed more support. Eccentric sleeves dropped (two drive variants: `drive_split`, `drive_solid`); motor rings 1.5 mm wall, centred on the motor, running inboard to 1 mm short of the centre line (14 mm of grip, was 7.4) with a slot round each encoder board; the motor's front face stops on a 1 mm flat ledge before the cone (it rested on the cone's rim). Battery tray 1 mm lower (28.0). The Ø4x2 magnet is now the default cup (`magnet_cup`); the Ø6x2 cup (`magnet_cup6`) is the alternative.
- The Ø6 cup clipped the caps (0.14 / 0.27 mm³ at the room's outer end): the frame boss's blend into the sleeve-sized ring was added after the cup's room was cut. The room (and the encoder slot) is now re-cut after the blends, and check_layout checks the cups (the Ø6 one too) and the wheels against the blocks: it allowed any contact between them as a running gap. The right frame boss is blended into its ring from the ring's inboard end (it stood clear of the thin ring and would have started in mid-air).
- Clamp screws closer to their motors, 0.6 mm of resin between the nut trap and the bore: the upper drive motor's 1.1 mm (7.65 from its axis; the trap runs on down through the ring's outside), the fan motor's 0.5 mm (7.94; the ears end the same 2.6 past the screw). The upper ring's clamp moved inboard to y 8.9 (the middle of the longer ring): further out, the housing's blend into the ring lay under its nut trap and blocked the nut going in from below.
- Round 6 viewer feedback: the cap screws' countersinks left a 0.25 mm skin on the caps' outer face over the heads (in r5 too: it prints as a ceiling and broke into points and fins): the head's clearance now opens out through the face. The right cap's front screw column was raised to a near-tangent join with the thin ring and the clamp's slit cut a flake off it: a column joins the ring only where they overlap by more than the ring's wall. Left cap: the rear basket peg moved onto the frame boss's flat top (its socket at x -15 ran into the rear screw's countersink), and the front socket has a flat landing across the plate (it opened lopsided on the slope), with no lightening pocket under it. One-piece left clamp: the ear runs a wall past its nut both ways (0.1 mm before).
- Seen from behind: a wedge of the boss's blend into the ring refilled the rear cap screw's countersink (the heads are now cut again after the blends); the head clearance clipped a lens off each motor ring (the rear cap screws moved back, to x -19.2 left and -13.0 right, so the clearance stays 0.3 off the ring); the right frame boss showed its bottom 0.3 below its blend and later a 0.05 strip down its back (it now stands from the blend's floor, inside the blend and the web, which are drawn 0.05 wider) and moved in to y 10.28, 0.6 clear of the rear screw's head.
- Round 6 built: the left wheel turned heavy on every tooth (not freed by loosening the cap), the right one free but with almost no backlash; the left pair had none. The geometry is the same both sides (every running gap measured equal), so the print closed both meshes by about 0.06-0.09 mm against the 0.06 mm designed. Backlash now 0.12 mm in the pair (0.10 on the pinion, 0.14 on the wheel, whose teeth have the wider tips): the teeth bind only after the centre distance closes by about 0.15-0.19 mm. For the round-6 robot: pinions with all the extra on them (`gear_r6_bl12_*`, `gear_r6_bl14_*`, both bores), in the robot print.
- Round 6: the sensor caps did not work in resin (painted). They are now black PLA on the Ender, front face down with the bores vertical, no supports, solid (100 % infill), 0.08 mm layers (`tools/slicing/pla_caps.ini`): bores 0.25 mm radial clearance (0.175), crush ribs 0.6 wide (0.5) and 0.05 interference until the three FDM test caps (0 / 0.05 / 0.10) choose, keyway cover 0.8 (0.6) and tab clearance 0.15 (0.125). Check the filament blocks IR first (docs/printing.md).
- The brass gears arrived: the motors carry the brass 7T pinions (no printed pinions, nor the round-6 retrofit ones), and the default wheel is for the bought 36T (`Gears.wheel_gear = "brass"`): no printed teeth; the gear is drilled in the printed guide (`gear_guide.py`: bore to Ø7.8, three Ø2.2 holes on Ø12.3, countersunk by hand) and screwed on with three M2×5 flat heads (the guide is square outside for a vise, its pocket has the gear's tooth outline so the gear can't turn, and the square lid can't turn in its recess), flush on its inboard face, into nuts in hex pockets in the wheel's outer face. The drum stays hollow: each pocket is in a column from the end web to the gear, and the gear touches only the three column ends (the owner's request; 0.66 cm³ a wheel, was 0.96 with the drum filled). The printed-gear wheels are in `alternatives` (they run with the brass pinion at about 0.25 mm of backlash). One-piece blocks are now the default (`drive_solid` in the robot print; `drive_split.pm4n` is the option). check_layout: 0 issues for both variants (the flush heads keep the gear's 0.25 mm running gap to the block). Sensor caps print one at a time (`complete_objects`, 46 mm apart for the head).
- Round 7 built and worked well. Of the two wheel gears tried with the brass pinions, the printed 36T was preferred over the drilled brass one: `Gears.wheel_gear = "printed"` is the default again (the brass 36T's wheels and drill guide move to `alternatives/`). The fan mount sits tilted a little upward (front up) instead of hanging level, because of how its tabs seat on the blocks' ears; a new way of holding it is being worked out.
- Round 7: the fan mount now prints upside down (collar down). Collar up, the tabs' undersides that seat on the blocks' ears were on supports, and their nubs tilted the fan's front up by more than 1 mm. Upside down they print as clean top faces; the slice shows no supports inside the motor bore (the seat prints as an unsupported 1.6 mm ledge). The basket's floor sagged in its thin parts: PrusaSlicer left out supports under what it took for bridges (almost all of the 0.4 mm skin, only patches round the posts were supported); `pla.ini` now has `dont_support_bridges = 0`, 1.5 mm support spacing and 3 interface layers, so the whole floor sits on supports.
- Round 7: the sensor caps' print (Ender) stopped mid-way with an alarm, the thermal-runaway protection (the nozzle fell below its target). `pla_caps.ini` now runs the part fan at 50-70 %, ramped over the first 4 layers (pla.ini's 100 % from layer 2), and one nozzle temperature, 215.
- Trial plate (`trials.py`, `trials.pm4n`), the default design untouched: impellers for a fan motor with a plain Ø1.5 shaft, 6 mm out (it fits the same mount; the shaft isn't cut: the bore's bottom stops it where the pinion put the hub), bores 1.55 and 1.58 (they print about 0.06 small: a hard press and a snug push; ream with a Ø1.5 drill by hand if too tight); a tire cutting guide (cut at the channel's 3.2 mm); wheel pairs with 0.20, 0.16, 0.12 and 0.08 mm of design backlash with the brass pinion (printed 36T unshifted, 0.85 / 1.40 addendum / dedendum for 0.2 mm of tip clearance each way: `check_gears.py` shows them free until the centre distance closes by 0.15-0.20). To report: which pair runs smoothest without binding, which impeller bore holds, how the tires cut.
