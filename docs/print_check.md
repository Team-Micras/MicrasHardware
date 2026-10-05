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
