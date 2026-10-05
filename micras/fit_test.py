"""Calibration prints: every fit the robot uses, at a range of clearances, printed the way the parts are printed.
Print them once the exposure is set (docs/printing.md), try the real parts in them, and set the parameters from
the best fit.

coupon() (grey resin): short vertical tubes, each with a tab below it giving its diametral clearance in
hundredths of a mm (raised digits), joined by thin bars (small layer areas: low peel forces). Rows:
  BRG  bearing Ø5          -2..12   one-piece blocks: press fit (DriveParams.solid_bearing_fit); split blocks
                                    clamp theirs (bearing_fit 0.1 with split_relief)
  MOT  motor Ø9.61         0..15    the motor in the blocks and the fan collar
                                    (DriveParams.ring_motor_fit_*, FanParams.motor_fit)
  INS  insert Ø3.2         5..25    the glued M2 inserts (DriveParams.insert_d = 3.2 + this)
  AXL  axle Ø2             0..10    magnet cup and hub bores (WheelParams.axle_fit); also the basket's peg sockets
  SHF  motor shaft Ø1     -10..10   the printed pinion's bore (PrintedGears.pinion_bore); the impeller prints
                                    tilted, so its bore stays undersize and is reamed
  PIN  pins Ø2, Ø5, Ø8               outsides of known size: with the bores they separate the scale from the
                                    light bleed (outside = s d + 2e, bore = s d - 2e)
  MG6  magnet Ø6 pocket    0..12    blind, 2.1 deep, opening up as in the magnet cup (Stack / magnet_cup)
  MG4  magnet Ø4 pocket    0..12    the same for the Ø4x2 magnet
  NUT  M2 nut trap 4.0 AF -10..20   DriveParams.nut_af = 4.0 + this
  GAP  slots              10..40    which gaps print open (running clearances like Stack.holder_gap)
Tubes and pins are 5 mm tall: measure them above their first 1 mm (the layers on the supports are distorted),
after the post-cure and a day's rest (or an hour at 50 °C).

exposure_coupon(n): the exposure test (tools/rerf.py): a 14 x 38 x 3 block, like a part's thick walls, with
holes at the exact nominal sizes of the bought parts (motor Ø10, magnet Ø6, bearing Ø5, M2 nut 4.0 AF, insert
Ø3.2, axle Ø2, motor shaft Ø1), a comb of slots 0.2-0.5 on one side and its number n engraved on top. Printed
on supports in the printer's exposure-test mode, one per zone at increasing exposure.

led_caps(): a wall-sensor cap per variant of the LED grip and the emitter's pitch, marked with
1-5 dots on top: 1-3 crush ribs 0.05 / 0.10 / 0.15 (FrontParams.rib_interf), 4-5 emitter pitched 1 and 2 deg
(FrontParams.emitter_tilt, ribs 0.10; compare the readings on the bench).
"""

from dataclasses import replace

from build123d import Align, Axis, Box, Cylinder, FontStyle, Pos, RegularPolygon, Text, extrude

from .params import P

MIN = (Align.MIN, Align.MIN, Align.MIN)
CMIN = (Align.CENTER, Align.CENTER, Align.MIN)
WALL = 1.2  # tube walls
TAB = (2.8, 1.0)  # label tab depth, thickness (digits stand 0.4 on it)
BAR = (1.0, 1.0)  # bars joining the cells: width, height
GAP_X = 1.5  # between cells
H = 5.0  # tubes and pins: measure them above their first 1 mm (the supported layers are distorted)
BIG = ("MOT", "PIN")  # rows in the right-hand column


def rows():
    return [  # (code, nominal, clearances, kind, height)
        ("BRG", P.bearing.od, (-0.02, 0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.12), "tube", H),
        ("MOT", P.motor.d, (0.0, 0.03, 0.05), "tube", H),
        ("MOT", P.motor.d, (0.08, 0.10, 0.15), "tube", H),
        ("INS", 3.2, (0.05, 0.10, 0.15, 0.20, 0.25), "tube", H),
        ("AXL", P.wheel.axle_d, (0.0, 0.03, 0.06, 0.10), "tube", H),
        ("SHF", P.motor.shaft_d, (-0.10, -0.05, 0.0, 0.03, 0.06, 0.10), "tube", H),
        ("PIN", 0.0, (2.0, 5.0, 8.0), "pin", H),
        ("MG6", 6.0, (0.0, 0.04, 0.08, 0.12), "cup", 2.7),
        ("MG4", 4.0, (0.0, 0.04, 0.08, 0.12), "cup", 2.7),
        ("NUT", 4.0, (-0.10, 0.0, 0.10, 0.20), "hex", 2.0),
        ("GAP", 0.0, (0.10, 0.15, 0.20, 0.25, 0.30, 0.40), "slot", 2.5),
    ]


def _text(s, x, y, z, size=1.9):
    """Raised text standing on z."""
    return Pos(x, y, z - 0.01) * extrude(Text(s, size, font_style=FontStyle.BOLD, align=(Align.CENTER, Align.CENTER)), 0.41)


def _cell(kind, nominal, c, h):
    """One test feature and its outer size (w, d), its bottom-left corner at the origin."""
    if kind == "pin":  # an outside of known size (c is its diameter), for the scale vs the light bleed
        return Pos(c / 2, c / 2, 0) * Cylinder(c / 2, h, align=CMIN), c, c
    if kind in ("tube", "cup", "hex"):
        hole = nominal + c if kind != "hex" else (nominal + c) / 3 ** 0.5 * 2
        od = hole + 2 * WALL
        part = Pos(od / 2, od / 2, 0) * Cylinder(od / 2, h, align=CMIN)
        if kind == "hex":
            part -= Pos(od / 2, od / 2, -1) * extrude(RegularPolygon((nominal + c) / 3 ** 0.5, 6), h + 2)
        else:
            z0 = -1 if kind == "tube" else h - 2.1  # the cup: a blind pocket opening up
            part -= Pos(od / 2, od / 2, z0) * Cylinder(hole / 2, h + 2, align=CMIN)
        return part, od, od
    # slot: two walls 4 long on a 0.6 floor, the gap between them
    w = 2 * WALL + c
    return Box(w, 4.0, h, align=MIN) - Pos(WALL, -1, 0.6) * Box(c, 6.0, h, align=MIN), w, 4.0


def _row(code, nominal, fits, kind, h):
    """A row of cells, each on a tab with its clearance, along a bar; the row's code on a tab at its left.
    Returns (part, width, depth) with the bar along y = 0."""
    part = Pos(-9.0, 0, 0) * Box(8.6, TAB[0], TAB[1], align=MIN) + _text(code, -4.7, TAB[0] / 2, TAB[1], 2.0)
    part += Pos(-0.5, 0, 0) * Box(0.6, BAR[0], BAR[1], align=MIN)
    x, depth = 0.0, 0.0
    for c in fits:
        cell, w, d = _cell(kind, nominal, c, h)
        tw = max(w, 4.6)
        label = "P" if c == "peg" else f"{c:g}" if kind == "pin" else f"{round(c * 100):d}"
        part += Pos(x, 0, 0) * Box(tw, TAB[0], TAB[1], align=MIN) + _text(label, x + tw / 2, TAB[0] / 2, TAB[1])
        part += Pos(x + (tw - w) / 2, TAB[0] - 0.4, 0) * cell
        x += tw + GAP_X
        depth = max(depth, d)
    part += Box(x - GAP_X, BAR[0], BAR[1], align=MIN)
    return part, x - GAP_X + 9.0, TAB[0] - 0.4 + depth


def coupon():
    body, spans, col_x = None, [], 9.0
    for col in ([r for r in rows() if r[0] not in BIG], [r for r in rows() if r[0] in BIG]):
        y, width = 0.0, 0.0
        for r in col:
            part, w, d = _row(*r)
            body = Pos(col_x, y, 0) * part if body is None else body + Pos(col_x, y, 0) * part
            width = max(width, w)
            last = y
            y += d + 1.5
        body += Pos(col_x - 9.6, 0, 0) * Box(BAR[0], last + BAR[0], BAR[1], align=MIN)  # spine
        spans.append(col_x - 9.6)
        col_x += width + 3.0
    body += Pos(spans[0], 0, 0) * Box(spans[1] - spans[0] + 1.0, BAR[0], BAR[1], align=MIN)
    body = body.clean()
    body.label = "fit_test"
    return body


EXPOSURE_HOLES = (  # (label, kind, nominal, x, y) on the 14 x 38 block
    ("MOT", "round", 10.0, 7.0, 7.5),
    ("MG6", "round", 6.0, 3.8, 18.0), ("BRG", "round", 5.0, 10.2, 18.0),
    ("NUT", "hex", 4.0, 3.8, 25.0), ("INS", "round", 3.2, 10.2, 25.0),
    ("AXL", "round", 2.0, 3.8, 30.5), ("SHF", "round", 1.0, 10.2, 30.5),
)
EXPOSURE_SLOTS = (0.2, 0.3, 0.4, 0.5)  # in the block's right side, 2 deep, from y 28.5 up


def exposure_coupon(n):
    """The exposure-test block for zone n (see the module doc)."""
    w, d, h = 14.0, 38.0, 3.0
    body = Box(w, d, h, align=MIN)
    for _, kind, nominal, x, y in EXPOSURE_HOLES:
        if kind == "hex":
            body -= Pos(x, y, -1) * extrude(RegularPolygon(nominal / 3 ** 0.5, 6), h + 2)
        else:
            body -= Pos(x, y, -1) * Cylinder(nominal / 2, h + 2, align=CMIN)
    y = 28.5
    for g in EXPOSURE_SLOTS:
        body -= Pos(w - 2.0, y, -1) * Box(3.0, g, h + 2, align=MIN)
        y += g + 1.6
    body -= Pos(6.0, d - 2.6, h - 0.4) * extrude(Text(str(n), 3.2, font_style=FontStyle.BOLD,
                                                        align=(Align.CENTER, Align.CENTER)), 1.0)
    body = body.clean()
    body.label = f"exposure_{n}"
    return body


FIT_ROWS = (  # (code, nominal, kind, diametral offsets) for fit_blocks(); the bought part is tried in each hole.
    # The exposure test (2.0 s) still needed force at nominal in every hole, so the rows start above nominal.
    ("BRG", 5.0, "round", (0.0, 0.03, 0.05, 0.08, 0.10)),  # bearing (5.10 at 2.5 s: slid in with no force)
    ("MOT", 10.0, "round", (0.05, 0.10, 0.15, 0.20, 0.25)),  # drive motor (Ø10) and fan motor (Ø9.97)
    ("MG6", 6.0, "round", (0.04, 0.08, 0.12, 0.16)),  # magnet
    ("NUT", 4.0, "hex", (0.05, 0.10, 0.15, 0.20, 0.25)),  # M2 nut, across flats
    ("INS", 3.2, "round", (0.05, 0.10, 0.15, 0.20, 0.25)),  # M2 insert (glued)
    ("PIN", 3.3, "round", (-0.05, 0.0, 0.05, 0.10)),  # the fan motor's 9T pinion tips (the impeller's bore, glued)
    ("AXL", 2.0, "round", (0.05, 0.10, 0.15, 0.20, 0.25)),  # axle: magnet cup, hub, wheel gear (glued)
    ("SHF", 1.0, "round", (0.03, 0.06, 0.10, 0.14, 0.18)),  # drive motor shaft: the printed pinion (press, glued)
)
FIT_WALL = 2.0  # round each hole: thick like the parts' walls (thin tubes printed looser, and misled the first coupon)
FIT_H = 3.0
FIT_LABEL = 3.0  # strip along the front with each hole's offset engraved, in hundredths


def fit_blocks():
    """{label: block}: the calibration print (printed on supports like the parts, at the chosen exposure): one bar
    per fit, its holes at the nominal size plus each offset (engraved below the hole, in hundredths of a mm; "m"
    marks a negative one), its code engraved at the left end."""
    out = {}
    for code, nominal, kind, offsets in FIT_ROWS:
        pitch = nominal + max(offsets) + 2 * FIT_WALL
        lead = 7.0  # the code's end
        w, d = lead + pitch * len(offsets), pitch + FIT_LABEL
        body = Box(w, d, FIT_H, align=MIN)
        for i, c in enumerate(offsets):
            x, y = lead + pitch * (i + 0.5), FIT_LABEL + pitch / 2
            size = nominal + c
            if kind == "hex":
                body -= Pos(x, y, -1) * extrude(RegularPolygon(size / 3 ** 0.5, 6), FIT_H + 2)
            else:
                body -= Pos(x, y, -1) * Cylinder(size / 2, FIT_H + 2, align=CMIN)
            label = f"{'m' if c < 0 else ''}{abs(round(c * 100)):d}"
            body -= Pos(x, FIT_LABEL / 2 + 0.2, FIT_H - 0.35) * extrude(
                Text(label, min(2.2, pitch * 0.5), font_style=FontStyle.BOLD, align=(Align.CENTER, Align.CENTER)), 1.0)
        body -= Pos(lead / 2, d / 2, FIT_H - 0.35) * extrude(
            Text(code, 2.0, font_style=FontStyle.BOLD, align=(Align.CENTER, Align.CENTER)), 1.0).rotate(Axis.Z, 90)
        body = body.clean()
        body.label = f"fit_{code}"
        out[body.label] = body
    return out


def led_caps():
    """{label: cap}: W1 caps in the five variants (see the module doc), marked with dots on top."""
    from . import front
    variants = [(0.05, 0.0), (0.10, 0.0), (0.15, 0.0), (0.10, 1.0), (0.10, 2.0)]
    out = {}
    (_, y), _ = front.SENSORS["W1"]
    for i, (rib, tilt) in enumerate(variants, 1):
        cap = front.sensor_cap("W1", fp=replace(front.FP, rib_interf=rib, emitter_tilt=tilt))
        bb = cap.bounding_box()
        for k in range(i):  # dots along the top of the emitter's sleeve
            cap -= Pos(bb.center().X + (k - (i - 1) / 2) * 1.0, y, bb.max.Z - 0.35) * Cylinder(0.3, 2, align=CMIN)
        cap.label = f"sensor_cap_test{i}"
        out[cap.label] = cap
    return out
