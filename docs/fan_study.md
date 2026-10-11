# Fan study: the 54k rpm fan motor on 3S

This study replaces the earlier one, which sized the fan for the 18k rpm motor. It is a model, not a measurement:
`tools/capped.sh uv run tools/fan_study.py` reproduces every number here in about 30 s (`--quick` runs on a coarser
grid). The script needs only numpy and scipy, and reads the board outline from `ref/board_mech.json`.

## Summary

- **Impeller: Ø22, eye 10, 12 straight radial blades, channel 3.0 mm at the eye tapering to 2.0 at the tip.**
  The shroud seal (0.3 face gap, 0.3 neck gap) is unchanged. The impeller mounts on the pinion by a plain bore on
  the tip circle, glued, with at least 3 mm of engagement (about 4 mm preferred).
- **Run it at about 4 N, not at its limit.** About 6.3 V average (duty = 6.3 V / V_bat) gives about 1000 Pa of
  suction, 42.5k rpm, 0.46 A and 3.0 W from the battery. The winding sits at 60 °C after 5 minutes and 66 °C
  continuously.
- **The motor could give 7–8 N (5–9 N over the parameter ranges), but the robot can't use it.** At 4 N each tire
  already carries 5× its no-fan load and sinks about 0.35–0.5 mm. The body is only 1 mm off the floor. Traction
  then exceeds the drive motors' force above about 3 m/s.
- **Two conditions for those numbers. Each matters more than the impeller design:**
  1. **The PWM ripple.** The fan PWM runs at 100 kHz now. A coreless winding of about 8 µH and 0.6 Ω then carries
     about 1.1 A rms of ripple on top of 0.45 A mean, and the ripple heats the winding more than the load does.
     As is, the motor would reach 93 °C at 4 N, and the most it can give within 85 °C is about 2.8 N (0.9 N if
     L = 5 µH). The fix is 300–400 kHz on TIM12 (the STSPIN958 takes up to 500 kHz), or a 22–47 µH inductor in
     the motor lead.
  2. **The skirt must close round the wheel notches.** The present pattern leaves the 1 mm gap open along both
     notches (about 50 mm² of leak). That limits the fan to about 3.5 N at the thermal limit, and 4 N would take
     96 °C.
- **Never 100 % duty.** On 12.3 V the Ø22 impeller would reach 79k rpm (hoop safety factor 3.2), 14 N and 17 W,
  and the winding would pass 140 °C in 5 minutes.
- A smaller impeller is not better for downforce with this motor. Its no-load friction (1.3 W at 64k rpm) grows
  with speed, so a slower, larger impeller makes the same downforce a little cooler. The difference is small,
  though: at 4 N, Ø26.4 runs 4 °C cooler than Ø22, and both reach about 8 N at the thermal limit. Ø22 frees a
  2.2 mm ring round the fan, is 28 % lighter (less unbalance) and has more structural margin (safe to 71k rpm
  against 59k). Only if the PWM ripple can't be fixed is the Ø26.4 worth keeping (3.35 N against 2.84 N at the
  limit), re-bladed as below.

## 1. Motor model (at 12.3 V)

| quantity | value | basis |
| --- | --- | --- |
| Ke = Kt | 1.33 mV·s/rad = 1.33 mN·m/A (7180 rpm/V) | 54k rpm no-load at 7.6 V, less the no-load I·R |
| friction torque | 0.20 mN·m at 64k rpm (1.34 W) | 150 mA no-load at 9 V (owner) × Kt |
| friction model | 0.10 mN·m + 0.015 µN·m per rad/s (half Coulomb, half viscous at the 9 V point) | split assumed; an all-Coulomb split changes the results by about 3 % |
| winding R (25 °C) | **0.35–0.8 Ω, nominal 0.55** (+23 % at 85 °C) | datasheets scaled by Ke², below |
| winding L | 5–15 µH, nominal 8 (estimate) | Faulhaber 1024 SR scaled by Ke² gives 5 µH |
| driver + shunt + wires | 0.2 Ω | STSPIN958 0.165 Ω per FET with two halves in parallel, R31 50 mΩ |
| no-load at 12.3 V | about 87k rpm, with 2.2 W of friction | so never run the bare motor at full voltage for long |
| stall at 12.3 V | 12–22 A | the driver's 5 A limit (TOFF) takes over: ramp the duty up |
| loaded, 100 % duty, Ø22 | 79k rpm at 12.3 V, 72k at 11.1 V | |

**Resistance from similar motors** (each one's R scaled by (1.33 / its Ke)², since R ∝ turns² for the same
size):

| motor | nominal V, no-load | stated R | Ke (mV·s/rad) | R scaled to this motor |
| --- | --- | --- | --- | --- |
| RIC-1020DT-074400 | 7.4 V, 39.5k rpm | 0.83 Ω (7.4 V / 8.9 A stall) | 1.79 | 0.46 Ω |
| RIC-1020DT-037450 | 3.7 V, 45k rpm | 0.23 Ω (3.7 V / 16.4 A) | 0.79 | 0.61 Ω |
| NFP-D1020T-1437 | 7.4 V, 51.5k rpm | 0.5 Ω (15.7 A stall) | 1.37 | 0.47 Ω |
| Kegu 1020R-V3 / V2 | 7.4 V, 39.1k / 33.6k rpm | 1.4 / 1.8 Ω | 1.81 / 2.10 | 0.71 / 0.68 Ω |
| Faulhaber 1024 K 003 SR (10 × 24) | 3 V, 12.2k rpm | 1.36 Ω | 2.33 | 0.44 Ω |

This motor is 23 mm long, longer than the 10 × 20 parts, which favours the low end. Kegu's own table is
internally inconsistent (its stall currents don't match V/R), and the NFP figures come from a search snippet,
because the page refused the fetch.

**Sensitivity to R** (Ø22, sealed skirt, 400 kHz PWM):
- At the thermal limit: 8.9 / 7.7 / 6.7 / 4.9 N for R = 0.35 / 0.55 / 0.8 / 1.5 Ω.
- At a fixed 40k rpm (3.5 N): the current is 0.40–0.41 A for any R. The winding reaches 54 / 57 / 60 / 69 °C
  after 5 minutes.
- So R hardly matters at the recommended point. It matters at 100 kHz, where the ripple loss scales with R/L².

**Safe power.** The thermal model uses the Faulhaber 1024 SR values: winding to housing 16 K/W (6.5 s), housing
to air 41 K/W on a plastic flange (180 s), taken as 35 K/W for a moving robot. The winding may reach 85 °C (a
cheap motor's NdFeB magnets and bonded winding) in 30 °C air. The friction heat goes in on the housing side.

| loss the motor can take | 1 min | 3 min | 5 min | continuous |
| --- | --- | --- | --- | --- |
| all in the winding (I²R) | 2.1 W | 1.44 W | 1.24 W | 1.08 W |
| all friction | 5.5 W | 2.5 W | 1.9 W | 1.6 W |

A mixed load of about 1.4–1.6 W is the budget for bursts of a few minutes. That is about 0.8–1.0 A rms in a hot
0.68 Ω winding, once the friction (0.7–1.2 W at 40–60k rpm) is counted. This agrees with RIC's "rated" 1.0 A for
the 7.4 V 1020, which is meant for prop-cooled aircraft. Bursts of a few minutes are about one housing time
constant, so they gain little over continuous running: 5 minutes from cold allows about 15 % more than
continuous, 3 minutes about 30 %.

With the ripple fixed, the mechanical output at the limit is about 5.5 W at 59k rpm (Ø22) for 5-minute bursts,
4.2 W at 54k continuously. The textbook maximum, V²/4R ≈ 45 W, means nothing here.

The PWM ripple decides everything else. The bridge switches synchronously: in parallel mode PWM1 = 1 turns the
high sides on and PWM1 = 0 the low sides (DS14341 table 11), so the winding sees 12.3 V or 0. Ripple at 50 %
duty:

| | 100 kHz, 8 µH | 100 kHz, 5 µH | 400 kHz, 8 µH | 100 kHz + 47 µH |
| --- | --- | --- | --- | --- |
| ripple, rms | 1.09 A | 1.71 A | 0.28 A | 0.16 A |
| copper loss it adds | 0.8 W | 2.0 W | 0.05 W | 0.02 W |

**Measure L before relying on any of this.** With a scope across R31 at 50 % duty, ΔI_pp ≈ 12.3 V · 0.25 / (L · f).
At 400 kHz the result barely depends on L: 7.4 N at 5 µH against 7.7 N at 8 µH, both at the limit.

## 2. Impeller and operating point

### Model

The model is in `tools/fan_study.py`. The ranges below are its sensitivity runs.

- **Head.** Euler head with Wiesner slip (σ = 0.82 for 12 radial blades). The tip's kinetic energy is lost (no
  volute), and straight radial blades take a shock loss of 0.8 · ½ρu₁² at the eye. On top of that, a 3D factor
  κ = 0.9 (range 0.75–1.0). Shut-off suction comes out at 0.72–0.75 · ½ρu₂².
- **Power.** Shaft power is the Euler power on the through-flow, plus low-flow recirculation
  (0.03 · ρu₂³D₂b₂, range 0.015–0.06) and laminar disk friction.
- **Inlet seal.** The 0.3 mm face gap over the Ø27 ring and the 0.3 mm neck gap in the hole act as two thin
  annular orifices in series. They leak from the tip back to the hole, and the shroud's swirl opposes the leak a
  little. That leak is 0.3–0.5 L/s, two to three times the skirt's, so it is most of the impeller's flow.
- **Board.** The gap under the board is solved as 2D lubrication (Hele-Shaw) flow on a 0.5 mm grid, with the
  hole at the suction pressure. The gap is taken as 0.9 mm, the 1.0 mm less the tires' sink.
- **Skirt.** The bent-down film margin leaks like an effective contact gap δ over a 1 mm contact width (laminar
  plus orifice):

| case | δ | notches | leak at 1000 Pa | downforce / suction | centre of pressure |
| --- | --- | --- | --- | --- | --- |
| sealed, good floor | 0.02 mm | closed | 0.01 L/s | 3970 mm² | 7.8 mm ahead of the axle |
| **sealed, typical (design case)** | 0.05 mm | closed | 0.15 L/s | 3930 mm² | 7.9 mm |
| sealed, poor (joints, dust) | 0.10 mm | closed | 0.59 L/s | 3800 mm² | 8.3 mm |
| present pattern | 0.05 mm | open: 0.5 mm margin on the inner face, none on the end faces | 1.33 L/s | 3630 mm² | 9.5 mm |

With the skirt closed, the pressure under the board is uniform to within 1–5 %. The downforce is then just the
suction times the 3970 mm² footprint (board plus hole): 1 N per 254 Pa.

**The notches can be closed.** The 36T wheel gear's lowest point is 1.54 mm above the floor, which is above the
board's underside (1.0). The tire's inner face is at |y| = 21.0. At x = ±8.5 the tire is 4.0 mm off the floor. So
a 2 mm margin bent down along the notch's inner and end faces touches nothing. This change belongs to the skirt,
not the fan: drop the notch cut in `skirt.pattern` so the 2 mm margin runs all the way round, and let the FLOOR
check confirm it.

### Recommendation

**Ø22 closed impeller:**

| part | value |
| --- | --- |
| eye | 10 |
| hub and nose | Ø6 (over the pinion) |
| blades | 12 straight radial (β₁ = β₂ = 90°), 0.55 thick |
| channel | 3.0 at the eye, 2.0 at the tip (cone-shaped backplate) |
| shroud | 0.6, flat, 0.3 above the Ø27 ring |
| backplate | 0.8 |
| neck | as now (ID 13.4, wall 0.5, 0.3 radial gap in the Ø15 hole) |
| mass | about 0.85 g (1.18 g now) |

Sensitivities at the thermal limit, against 7.7 N nominal:

| change | downforce |
| --- | --- |
| channel at the tip 1.5 / 3.0 | 7.9 / 7.2 N |
| eye 9 / 11 | 7.8 / 7.5 N |
| 9 or 16 blades | ±0.1 % |
| seal gaps 0.2/0.2 | 8.4 N |
| seal gaps 0.4/0.3 | 7.5 N |
| twisted inducer at the eye (half the shock loss) | 8.3 N, not worth the printing trouble |

Keep the 0.3 mm neck gap. The mount is located by the drive caps, not by the board hole, so a tighter gap may
rub.

**Operating points** (Ø22, sealed typical skirt, 12.3 V, nominal motor):

| downforce | suction | rpm (u₂) | through-flow: skirt + seal | shaft | V avg (duty) | battery | mean I | Tw 5 min / cont., 400 kHz | Tw 5 min, 100 kHz now |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 3 N | 763 Pa | 36.9k (42 m/s) | 0.12 + 0.29 L/s | 1.3 W | 5.5 V (44 %) | 2.1 W | 0.37 A | 54 / 58 °C | 86 °C |
| **4 N** | 1019 Pa | 42.6k (49 m/s) | 0.16 + 0.34 L/s | 2.0 W | **6.3 V (52 %)** | 3.0 W | 0.46 A | **60 / 66 °C** | 93 °C |
| 5 N | 1270 Pa | 47.6k (55 m/s) | 0.19 + 0.37 L/s | 2.8 W | 7.1 V (58 %) | 4.1 W | 0.55 A | 66 / 74 °C | 98 °C |
| limit, 5 min | 1958 Pa | 59.1k (68 m/s) | 0.27 + 0.46 L/s | 5.5 W | 9.0 V (73 %) | 7.3 W | 0.81 A | 85 °C | |
| limit, continuous | 1635 Pa | 54.0k | | 4.2 W | | | | 85 °C | |

The thermal limit is 7.7 N for 5-minute bursts and 6.4 N continuous. Over the ranges: 8.9 N at R = 0.35, 4.9 N at
R = 1.5, 6.7–8.4 N for κ = 0.75–1.0, 5.6 N with the poor skirt, 3.5 N with the present notches. At 4 N with the
notches open, the fan needs 49.6k rpm and 1.1 A, and the winding reaches 96 °C.

### PWM limit

**Yes, limit the fan with PWM, for traction and sink more than heat.**

- Command an average voltage, not a duty: duty = V_cmd / V_bat. The pack sags from 12.3 V to about 10.5 V under
  load, and the same duty would lose about 25 % of the downforce over a run. The fan has no tachometer, so this
  is open loop.
- V_cmd = 6.3 V for about 4 N. Calibrate it on the scale rig.
- Hard cap at about 7.5 V: 52k rpm, about 6 N.
- Ramp up over 0.2 s or more. Stall would draw 12–22 A, against the driver's 5 A limit.
- At 100 kHz, keep V_cmd at 5.5 V or below (about 3 N, 86 °C after 5 minutes) until the ripple is fixed.

### What more than 3–4 N means for an 87 g robot

The centre of pressure is 7.9 mm ahead of the axle, so the nose rests on the front skid (x = 45), which carries
F · 7.9/45 ≈ 18 % of the downforce. Tire sink is the firmware model's 4000–6000 N/m per tire, taken as linear.
Real rubber stiffens, so it is an upper bound.

| F | per tire (0.43 N without fan) | traction at μ = 1 | sink | drag: rolling (c_rr 0.03) + skid (μ 0.25) + skirt |
| --- | --- | --- | --- | --- |
| 2 N | 1.25 N | 2.5 N, 29 m/s² | 0.21–0.31 mm | 0.19 N |
| 4 N | 2.08 N | 4.2 N, 48 m/s² | 0.35–0.52 mm | 0.35 N |
| 6 N | 2.90 N | 5.8 N, 67 m/s² | 0.48–0.73 mm | 0.50 N |
| 8 N | 3.72 N | 7.5 N, 86 m/s² | 0.62–0.93 mm | 0.66 N |

The drive motors, at full 19.6 V, give 7.3 N stalled, 5.1 N at 2 m/s, 4.0 N at 3 m/s and 2.9 N at 4 m/s. So:

- **Beyond about 4 N, straight-line acceleration is motor-limited** above roughly 2.5–3 m/s. Only cornering and
  low-speed braking still gain.
- **Beyond about 5–6 N the 1 mm gap runs out.** The sink approaches the clearance, and the skids would take load
  off the tires, which is the opposite of what they are for.
- **Drag grows with downforce.** The front skid is the biggest share: a PTFE patch halves it.
- **The skid set needs recutting.** `skids.py` sizes the five-pad set for 0.13–0.23 mm of sink at 1 N. At 4 N the
  sink is 0.35–0.5 mm.
- **Odometry shifts.** The rolling radius drops by about 0.15 mm (77 µm/N per tire), so calibrate the wheel
  radius with the fan on.
- Board bending under 1000 Pa is about 0.01–0.03 mm: negligible.

## 3. Structure (Anycubic ABS-Like Pro 2)

Material properties from the TDS: UTS 35–45 MPa, flexural modulus 1.4–1.8 GPa, 35–40 % elongation, HDT
60–65 °C.

**Hoop stress.** The shroud and backplate are annular disks, and their peak stress is at the bore. The blades'
centrifugal load is added to them.

| | 42.6k rpm | 55k rpm | 70k rpm | 79k rpm (100 % duty) |
| --- | --- | --- | --- | --- |
| Ø22 | 3.1 MPa (SF 11) | 5.2 MPa (SF 6.7) | 8.4 MPa (SF 4.2) | 10.9 MPa (SF 3.2) |
| Ø26.4 | 3.8 MPa | 7.5 MPa (SF 4.7) | 12.2 MPa (SF 2.9) | |

The safe speed is taken at SF 4 on 35 MPa, which covers stress concentration of about 2 at blade roots and a
warm, possibly voided print. That gives **71k rpm for Ø22** and 59k for Ø26.4. Resin weakens near its HDT, so
hold more margin than that whenever the motor runs hot.

**Blades.**
- Straight radial blades take no centrifugal bending.
- The aerodynamic load is about 4 mN per blade at 4 N, which is negligible.
- A backward-curved blade would add plate bending, about 0.25 MPa · cos β per unit span at 55k rpm, still
  small. That is not a reason to curve them.

**Hub on the pinion.**
- A diametral interference of 0.03 mm in a Ø6 hub gives about 10 MPa hoop at the bore (Lamé, E = 1.6 GPa).
  Keep the interference at 0–0.03 and let glue do the rest.
- The glue (CA, about 5 MPa over about 34 mm²) carries about 0.2 N·m. That is 500 times the 0.45 mN·m running torque, and
  about 30 times the 6.7 mN·m the driver's 5 A limit can produce.
- The pinion runs next to a 60–85 °C motor, while the resin's HDT is 60–65 °C. Keep the motor near the
  recommended point (≤ 66 °C), and expect a press fit to relax, which is why the joint should be glued.

**Axial pull.** The suction pulls the impeller towards the board with about 0.25 N at 1000 Pa (0.5 N at
2000 Pa). This is like a small propeller's thrust on these motors. It takes up the rotor's axial play (typically
0.05–0.15 mm), so set the 0.3 mm shroud gap with the shaft pulled towards the board.

**Balance.** The force is m·e·ω². At 42.6k rpm (710 Hz), 0.85 g:

| eccentricity | rotating force | ISO 1940 grade |
| --- | --- | --- |
| 5 µm | 0.08 N | G22 |
| 10 µm | 0.17 N | G45 |
| 25 µm | 0.42 N | G110 |

At 25 µm the rotating force is half the robot's weight, and it lands at 710 Hz, where the IMU listens. Printed
and glued parts land around 10–30 µm, so balance it. The robot's own IMU can do it: run at the working speed,
read the acceleration amplitude and phase at the rotation frequency, add a trial drop of CA or paint (about 1 mg
at r = 10 corrects 10 µm) and repeat (one-plane trial-weight method). Or sand the shroud rim. Aim for 5 µm or
less.

## 4. Mounting on the 9T m0.3 pinion

The pinion: tip Ø3.30, pitch Ø2.70, root about Ø1.95, 4.6 mm long, pressed on the shaft.

- **Recommended: a plain bore on the tip circle, glued.**
  - The nine tips are ground concentric with the shaft, so they centre the impeller better than a printed spline
    could. The torque is tiny, so it needs no positive drive.
  - The impeller prints tilted 45°, so a printed bore comes out stepped and oval. Model it undersize (Ø3.15), then
    open it with a Ø3.2–3.3 drill in a pin vise. The bore should end up 0–0.03 mm under the tip diameter, so the
    tips bite slightly.
  - Then wick thin CA in.
  - Confirm the bite on a calibration row with bores 3.15 / 3.20 / 3.25 / 3.30, printed tilted like the impeller.
- **Engagement: at least 3 mm (L/D ≈ 1), about 4 mm preferred.** With 0.03 mm of clearance over 4 mm, the rim
  wobbles about ±0.08 mm axially.
  - Today's stack puts a 1.2 mm mount plate and a 0.4 mm gap between the motor face and the hub. That leaves only
    about 3.0 mm of a 4.6 mm pinion that starts at the face. Measure where the pinion starts.
  - To get about 4 mm, let the hub rise through the plate (plate hole ≥ hub Ø + 0.8), or drop the plate under
    the hub. The collar clamp already locates the motor.
  - Then end the hub 0.3–0.5 mm below the motor's front boss.
- **Bore end.** Make it blind, so air doesn't leak through the tooth spaces from the backplate side to the eye.
  End it 0.3 mm or more past the pinion's end, and keep 1 mm or more of nose wall around the bore all the way
  down. Today's cone narrows to 0.6 × hub radius at its tip, so the nose needs to be a Ø6 cylinder over the bore,
  then the cone.
- **Assembly.** Clamp the motor in the mount and stand the mount on a flat plate with the impeller under it,
  shroud down on 0.3 mm shims, and the shaft pulled out to the end of its play. Then wick the CA. The mount sets
  both the gap and the squareness.
- **Alternatives considered.**
  - An internal 9T spline (0.47 mm teeth): it can't be printed tilted to ±0.03, needs clearance anyway, and
    centres no better.
  - A loose bore filled with epoxy: no centring.
  - A two-piece impeller, with the backplate, blades and hub printed axis-vertical (round, concentric bore) and a
    flat shroud ring glued on, would give the best bore. It is worth it if balancing proves hard.

## 5. FanParams changes (micras/fan.py)

| parameter | now | recommended | note |
| --- | --- | --- | --- |
| `d2` | 26.4 | **22.0** | the rim ends 2.5 mm short of the nearest part (SOT-89 at 13.54) |
| `eye_d` | 11.0 | **10.0** | stays inside `neck_id` 13.4 |
| `h_tip` | 3.0 | **2.0** | `h_eye` stays 3.0 (cone-shaped backplate) |
| `hub_d` | 4.5 | **6.0** | 1.35 mm wall round the Ø3.3 pinion tips |
| `bore_d` | 0.9 | **3.15** | printed undersize; finish to 3.27–3.30, glued (section 4) |
| `hub_above`, `plate_gap`, `plate_t` (mount) | 1.5, 0.4, 1.2 | engage ≥ 3 mm (about 4) of the 4.6 mm pinion | the hub ends 0.3–0.5 below the motor face; the plate hole clears the hub, or the plate goes |
| nose (the code in `impeller()`) | cone from 0.6·rh | Ø6 cylinder over the bore, cone below | keep 1 mm or more of wall round the blind bore |

Unchanged: `blades` 12, `blade_t` 0.55, `blade_angle` 90, `h_eye` 3.0, `shroud_t` 0.6, `back_t` 0.8, `shroud_z`
0.3, `neck_id` 13.4, `neck_t` 0.5, `neck_l` 1.2, `nose_d` 6.0.

Outside FanParams:
- Skirt: margin round the notches, as in section 2.
- Skid set: recut for 0.35–0.5 mm of sink.
- Assembly notes (`assembly.PRINT`, `docs/printing.md`): the impeller bore is now Ø3.3 on the pinion, no longer
  reamed to 0.97.
- Firmware: these are recommendations only, nothing was edited.
  - TIM12 at 300–400 kHz (ARR 687 gives 400 kHz).
  - The fan command as an average voltage, about 6.3 V with a 7.5 V cap.
  - `fan_downforce` to be set from the scale measurement. 1.0 N is now an underestimate if the fan runs at
    6.3 V.

## Uncertainties and what to measure

- **Fan model.** The shut-off coefficient and losses are textbook correlations, not measured on this impeller:
  ±15 % on suction. The skirt's contact gap is a guess (0.02–0.1 mm). The 1000 Pa operating point should be
  confirmed on the scale rig, and the voltage adjusted to suit.
- **Motor.** Measure these, and the script takes them directly (`Motor(R=..., L=..., i0_meas=..., visc=...)`):
  - R, with a 4-wire measurement on a held rotor, averaged over a few positions.
  - L, with an LCR meter or from the ripple on R31.
  - The no-load current at 6, 9 and 12 V, which fixes the friction split. It often drops by 20–30 % after
    run-in.
  - The can temperature after a 5-minute run at the chosen voltage.
- **Thermal limits.** The thermal resistances are Faulhaber's, applied to a cheap motor, and its real limit may
  be lower than 85 °C. Brush wear at 40–60k rpm sets its life (likely tens of hours). Lower speed helps that too.
- **Tires.** The sink figures use a linear tire stiffness, so they are upper bounds.

## Sources

- Faulhaber DC-Micromotors Series 1024 … SR datasheet (R, kM, L, thermal resistances and time constants):
  https://www.faulhaber.com/fileadmin/Import/Media/EN_1024_SR_FMM.pdf
- RIC Motor RIC-1020DT table (no-load, rated and stall figures): https://www.ricmotor.com/details/1020-coreless-motor
- Kegu 1020R table: https://kegumotor.com/en/product/10mm-coreless-dc-motor-model-1020r-7-4v-dc-24500rpm.html
- NFP-D1020T-1437 (0.5 Ω, 51.5k rpm, 180 mA at 7.4 V; search summary only, the page refused the fetch):
  https://nfpshop.com/product/10mm-dc-motor-20-8mm-length-7-4v-dc-model-nfp-d1020t-1437-c
- ST STSPIN958 DS14341 (parallel-mode truth table, f_PWM ≤ 500 kHz, R_DS(on)): `../hw_debug/datasheets/stspin958_lcsc.pdf`.
  Board: R31 = 50 mΩ, motor between the tied OUTs and the sense node (`../hw_debug/kicad/Drivers.kicad_sch`).
  Firmware: TIM12 period 2749 at 275 MHz = 100 kHz (`../MicrasFirmware/cube/micras_v1.ioc`).
- Anycubic ABS-Like Resin Pro 2 TDS: https://get3d.pl/wp-content/uploads/2026/02/ABS_Like_Resin_Pro_2_TDS_EN.pdf
- Tire compliance and contact stiffness: `../MicrasFirmware/config/targets/v1/robot.hpp`, `sim/robot.toml`.
- Methods: Wiesner slip factor; Pfleiderer/Stepanoff shut-off head of impellers without a volute; free-disk
  friction C_M = 3.87/√Re (laminar); Hele-Shaw flow; Lamé thick cylinder; ISO 1940 balance grades.

## Update: curved blade inlets (after the study)

The study's model, run at 4 N, shows the outlet blade angle hardly matters this close to shut-off (60° and 30°
backward-curved outlets give the same result), but the inlet does: the air reaches the eye at about 24° from the
tangent, and straight radial blades meeting it edge-on cost about 0.3 W of shaft power. With the inlet shock loss
cut from 0.8 to 0.2 (`k_shock`; an estimate, the real gain may be anywhere from about 5 to 20 %):

| blades | rpm at 4 N | shaft | battery | winding, 5 min | max force (85 °C) |
| --- | --- | --- | --- | --- | --- |
| straight radial | 42.6k | 2.0 W | 3.0 W | 60 °C | 7.7 N |
| radial tip, curved inlet | 39.6k | 1.7 W | 2.6 W | 57 °C | 8.6 N |

So the impeller (micras/fan.py) keeps radial tips, which don't bend spinning, but its six main blades start inside
the eye at 26° and ease to radial by r = 8 mm, leaning into the turn; six straight radial splitters from r = 7 mm
keep 12 blades at the tip (12 curved inlets would block about 60 % of the eye). It only works one way round: the
arrow on its top shows the direction (counter-clockwise seen from the motor's side).

## Comparing the three impellers on the bench

`fan.impeller_variants()` makes three Ø22 impellers that differ only in their blades: A straight radial (12),
B curved inlets with splitters (6 + 6, the default), C backward-curved (7, 26° at the inlet to 35° at the tip,
like a vacuum-cleaner fan). Run each at the same PWM duty and compare downforce and current.

Measuring downforce: weighing the robot doesn't work (the suction pulls the robot down and the floor up by the same
amount). Lay a flat, stiff plate a little smaller than the board on the scale, rest the wheels on two blocks beside
it so the board sits at its height over the plate with the skirt touching it, tare, and run the fan: the plate is
pulled up, and the scale's drop is the downforce.

## CFD results (steady RANS of the three impellers)

The three Ø22 bench variants were run in OpenFOAM to check the 1D model above:
- A: straight radial, 12 blades;
- B: curved inlets with splitters, 6 + 6;
- C: backward-curved, 7 blades.

All three turn counter-clockwise seen from above, at 42,600 rpm. How to rerun: `tools/cfd/README.md`. The cases are
in `build/cfd/runs/` (not committed). The tables come from `build/cfd/post_report.txt`, and the fan curves are in
`build/cfd/curves_medium.json`.

**Setup.**
- **Solver.** OpenFOAM v2512 (docker `opencfd/openfoam-default`), steady `simpleFoam` (SIMPLEC), an MRF zone round
  the impeller, k-omega SST, incompressible air (ρ 1.2, ν 1.5e-5).
- **Domain.** A Ø40 × 1.2 cavity under the board, fed through its rim at a fixed total pressure (−suction). Then the
  Ø15 hole, and Ø80 × 40 of open air above the board. The mount (collar, plate, arm) and the motor are walls.
- **Seal.** The 0.3 mm face gap and neck gap get 3.2 / 4 / 5 cells across on the coarse / medium / fine meshes
  (0.77M / 1.35M / 2.56M cells). There are no prism layers and the wall function is Spalding's. y+ on the impeller
  averages 4–7, with a maximum of 18–29.
- **Measured.** The net flow drawn from the cavity (the useful flow), the flow up the neck (their difference is the
  seal leak coming back down the neck gap), and the shaft torque (the moment on the impeller about z, pressure plus
  viscous).
- **Stages.** Each suction is a stage of 500–1500 iterations, continuing from the previous one and averaged over its
  last 400.

### What the CFD shows

1. **Two branches near shut-off, with hysteresis.**
   - **Flowing branch.** The impeller pumps 0.5–1.1 L/s through the eye, and the seal leak is small: 0.06–0.16 L/s.
   - **Stalled branch.** Almost no eye flow. The seal leak (0.2–0.3 L/s) comes back down the hole with the shroud's
     swirl, and a vortex in the hole holds the eye about 300 Pa below the cavity. The net flow then runs *into* the
     cavity.
   - **Where it switches.** Stepping the suction up from the flowing side, A and B keep flowing up to 850 Pa and stall
     at 900 Pa. C keeps flowing to 775 Pa and stalls at 800 Pa. Coming down from 1000 Pa, all three stay stalled at
     800 Pa and recover by 600 Pa.
   - **Where the robot runs.** It needs only about 0.15 L/s of useful flow (the sealed skirt's leak). That is far
     below the lowest flow of the flowing branch (0.45–0.6 L/s), so the fan works in the stall gap.
2. **Less suction per rpm than the 1D model.** At 42.6k rpm, net flow from the cavity, in L/s:

   | suction | A, CFD | B, CFD | C, CFD | A, 1D | B, 1D | C, 1D |
   | --- | --- | --- | --- | --- | --- | --- |
   | 400 Pa | 1.12 | 1.12 | 0.89 | 2.28 | 2.57 | 1.69 |
   | 600 Pa | 0.99 | 0.94 | 0.74 | 1.81 | 2.15 | 1.34 |
   | 750 Pa | 0.86 | 0.72 | 0.53 | 1.39 | 1.80 | 1.05 |
   | 800 Pa | 0.69 (stalled from above: −0.13) | 0.66 (−0.12) | −0.10 (775 Pa: 0.47) | | | |
   | 850 Pa | 0.58 | 0.54 | −0.13 | 1.06 | 1.53 | 0.84 |
   | 900 Pa | −0.22 | −0.21 | −0.18 | | | |
   | 1000 Pa | −0.28 | −0.26 | −0.25 | 0.32 | 1.07 | 0.47 |
   | net-flow shut-off | 886 Pa | 886 Pa | 796 Pa | 1033 Pa | 1192 Pa | 1136 Pa |

   The CFD "shut-off" lies inside the stall gap. The 1D model puts it 15–35 % higher and predicts about twice the flow
   below it. It also ranks B and C above A, while the CFD puts A = B > C.
3. **Torque.** At 42.6k rpm, in mN·m:

   | suction | A | B | C |
   | --- | --- | --- | --- |
   | 400 / 600 / 750 Pa | 0.67 / 0.64 / 0.60 | 0.85 / 0.74 / 0.55 | 0.39 / 0.36 / 0.30 |
   | 850 Pa (flowing) | 0.44 | 0.44 | 0.28 at 775 Pa |
   | stalled (900–1000 Pa) | 0.10–0.11 | 0.11–0.13 | 0.17–0.18 |

   - **The 1D model's torque** at 750 Pa is 1.07 / 1.28 / 0.61 mN·m for A / B / C.
   - **Viscous share.** Pressure torque dominates; the viscous part is 0.03–0.08 mN·m.
   - **C** draws 45 % less torque than A on the flowing branch.
   - **B** draws 27 % more than A at 400 Pa, 15 % more at 600 Pa, 8 % less at 750 Pa and the same at 800–850 Pa.
     Its curved inlets shed periodically (±4 % torque oscillation in the steady solution). The likely reason: the air entering the eye already swirls with the rotor
     (the seal leak and the hub drive it), so the 24° inflow the curved inlets were drawn for isn't there.
4. **Operating points.** The CFD curves were run through the skirt (sealed, typical) and motor (400 kHz PWM, nominal)
   models (`tools/fan_study.py --cfd build/cfd/curves_medium.json`). Across the stall gap, flow and torque are
   interpolated between the last flowing and the first stalled point, i.e. as a time-mix of the two states (row "CFD").
   The row "CFD, no stall" instead extends the flowing branch smoothly past its last point: an optimistic bound if the
   real fan doesn't stall.

   | impeller | model | rpm at 4 N | shaft | battery | current | winding, 5 min | max force: 65k rpm cap / thermal only |
   | --- | --- | --- | --- | --- | --- | --- | --- |
   | A radial | CFD | 45.9k | 1.44 W | 2.46 W | 0.35 A | 59 °C | 8.0 N (cap) / 8.5 N at 67k |
   | A radial | CFD, no stall | 42.0k | 0.61 W | 1.46 W | 0.23 A | 55 °C | 9.4 N (cap) / 11.3 N at 71k (SF 4) |
   | A radial | 1D | 42.6k | 1.99 W | 3.03 W | 0.46 A | 60 °C | 7.7 N at 59k |
   | B curved inlets | CFD | 45.9k | 1.50 W | 2.53 W | 0.36 A | 60 °C | 8.0 N (cap) / 8.4 N at 67k |
   | B curved inlets | CFD, no stall | 42.7k | 0.91 W | 1.80 W | 0.28 A | 56 °C | 9.1 N (cap) / 10.3 N at 69k |
   | B curved inlets | 1D | 39.6k | 1.68 W | 2.61 W | 0.43 A | 57 °C | 8.6 N at 58k |
   | C backward | CFD | 48.4k | 1.35 W | 2.42 W | 0.33 A | 61 °C | 7.2 N (cap) / 8.4 N at 70k |
   | C backward | CFD, no stall | 45.2k | 1.02 W | 1.98 W | 0.29 A | 58 °C | 8.1 N (cap) / 9.6 N at 71k |
   | C backward | 1D | 41.1k | 1.65 W | 2.61 W | 0.41 A | 57 °C | 8.6 N at 61k |

   - **Seal recirculation at 4 N** (time-mix): 0.19–0.21 L/s. The 1D model has 0.34 L/s.
   - **The 65k cap.** The study's speed cap (65k rpm) now binds before the 85 °C limit for all three. The
     "thermal only" column lifts it, keeping the SF 4 structural limit (71k).

### Numerical checks and uncertainty

| check | result |
| --- | --- |
| mesh, A at 600 Pa (flowing), coarse / medium / fine | net 0.980 / 0.986 / 0.982 L/s, torque 0.641 / 0.642 / 0.655 mN·m, seal 0.091 / 0.085 / 0.087 L/s |
| mesh, A at 1000 Pa (stalled) | net −0.285 / −0.277 / −0.256 L/s (the backflow is the seal leak: 0.265 / 0.256 / 0.246), torque 0.098 / 0.100 / 0.098 |
| mesh near stall, A at 750 / 800 Pa, coarse vs medium | 0.862 / 0.745 (still falling) against 0.863 / 0.688 L/s |
| model: laminar instead of SST (coarse), A at 600 / 750 / 800 Pa | net 0.985 / 0.874 / 0.822 L/s against 0.980 / 0.862 / 0.745, torque 0.638 / 0.606 / 0.585 against 0.641 / 0.621 / 0.555 mN·m |
| fan laws: A at 55k rpm and 1250 Pa (= 750 Pa at 42.6k) | 1.141 L/s, 1.016 mN·m against 1.114 and 1.003 scaled: +2.4 % and +1.3 % |
| convergence | flowing stages away from stall settle to < 0.5 % std in flow and < 1.5 % in torque (B: 3 %, its inlet shedding), 1–3 % near stall (800–850 Pa); stalled stages 2–7 %; flow balance (inlet against ambient) within 0.6 %; initial residuals p 1e-5 to 3e-4, U 1e-3 to 2e-2, with the monitored integrals flat |

**What these mean:**
- **Discretisation.** About 2 % on torque and under 1 % on the flowing branch's flow. About 10 % on the stalled
  backflow, which is the seal leak and still falls with refinement.
- **Turbulence model.** The flow is transitional:
  - The gap Reynolds numbers are about 800–1000, and the disk's ωr²/ν is 4e4.
  - In the 0.3 mm gaps the SST eddy viscosity is only 0.3–2 times the molecular viscosity.
  - On the flowing branch away from stall, laminar and SST agree within 2.5 %, so the turbulence model is not what
    sets the numbers there.
  - Near stall the laminar run keeps more flow: at 800 Pa it still flows 0.82 L/s, against 0.69–0.75 with SST. So the
    stall point itself is uncertain by about ±50 Pa (mesh and model), which is about ±3 % in rpm.
- **The stall gap dominates.** Steady RANS can't tell what the fan does between its two branches. The time-mix and
  no-stall rows above bracket it:
  - for A at 4 N: 42–46k rpm, 0.6–1.4 W shaft, 55–59 °C;
  - for the maximum force: 8.0–9.4 N under the 65k cap.

  Real stall (rotating stall, surge of the small cavity) usually lands nearer the pessimistic end. Settling this
  needs an unsteady (sliding-mesh) run, or the bench.
- **Not in the uncertainty.** The geometry idealisations:
  - the 1.2 mm Ø40 cavity, where the real one is about 0.9 mm and the board's size;
  - a stationary motor shaft stub;
  - the motor model's own spread (R, L, friction; section 1), which moves the absolute temperatures more than the
    impeller choice does.

### Ranking

| | suction per rpm (stall point at 42.6k) | torque on the flowing branch | at 4 N | max force (65k cap) |
| --- | --- | --- | --- | --- |
| A straight radial | 850–900 Pa (best, with B) | reference | 45.9k rpm, 1.44 W, 59 °C | 8.0 N |
| B curved inlets | 850–900 Pa | −8 to +27 % against A | 45.9k rpm, 1.50 W, 60 °C | 8.0 N |
| C backward | 775–800 Pa (about 11 % lower) | 45 % less than A | 48.4k rpm, 1.35 W, 61 °C | 7.2 N |

- **Clear differences.** The stall points (A = B > C by about 90 Pa, against a ±50 Pa uncertainty), so C needs about
  5 % more rpm for the same force. The torque levels on the flowing branch also differ by more than the numerical
  error.
- **Not resolved.** Shaft and battery power at 4 N (within ±5 % of each other, against a stall-gap uncertainty of
  25–60 % that moves them all the same way) and the winding temperature (within 1.3 °C).
- **Ranking: A ≥ B > C.**
  - A makes as much suction as B for the same or less torque. It is the simplest to print and works in either
    direction.
  - The curved inlets (B) buy nothing in the CFD; the 1D model's 0.3 W gain doesn't appear.
  - C has the least torque but stalls earlier, so it runs faster for the same force and reaches less force under the
    speed cap.
- **What it means for the design.** The 1D model's recommendation (Ø22, 4 N at about 6 V) still holds, but expect
  about 46k rpm rather than 42.6k at 4 N, and recalibrate the voltage on the scale rig.
- **The stall gap is the main open question.** On the rig, look for:
  - suction that doesn't rise smoothly with voltage;
  - pressure ripple or noise from surge;
  - a jump in current between two duty settings.

  The CFD points to the inlet swirl from the seal leak as the trigger. Anti-swirl ribs in the board hole, or a
  tighter or labyrinth face seal, are candidates to delay it. They are untested; CFD or the bench would have to
  confirm them before any change to `micras/fan.py`.

Compute: about 23 h on 12 MPI ranks of the laptop's Core Ultra 7 155H, in docker capped at 14 GB and 16 CPUs (WSL).
The peak was 4.4 GB on the fine mesh. Speed was 1.1 / 2.2 / 4.2 s per iteration on the coarse / medium / fine mesh,
and the runs total 6.8 GB of case data.
