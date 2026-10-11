"""Wall-sensor LEDs from their datasheets, replacing the hand-drawn bodies of the KiCad model
WALL_SENSOR_STACKED.STEP (which had a 5.5 mm emitter body and a receiver copied from it: the TPS601A is a TO-18 can).

- emitter: OSRAM SFH 4550, 5 mm (T-1 3/4) clear epoxy, half angle 3 deg. Datasheet v1.9 (2022-08-09),
  https://look.ams-osram.com/m/7d214b223a9adb85/original/SFH-4550.pdf, "Dimensional Drawing" p.10; values
  without a tolerance there are +-0.1 (note 9, p.14).
- receiver: Toshiba TPS601A(F), phototransistor in a TO-18 metal can with a lens (package 0-5A1), half angle
  10 deg. Datasheet 2007-10-01: outline p.1, precautions p.2.

Part frame (emitter(), receiver()): x along the optical axis (where the lens looks), y left, z up, origin on the
axis at the seating plane (back face of the flange / header). The leads leave it at y = +-pitch/2, run back
along -x and bend down (-z). Pin 1 (the SFH's cathode, the TPS's emitter) is at -y: the footprint's pads 2 and 4
(v = -1.27) carry the emitter's cathode (to Q2's drain) and the receiver's emitter (the ADC net; collector on 3V3).

sensor_model() places both 1:1 in the old model's frame; SEATS has the numbers the sensor cap (front.py) needs,
in its frame (u along the look direction from the footprint origin, v left, z above the board top).
"""

from dataclasses import dataclass
from math import cos, radians, sin

from build123d import (Axis, Box, Circle, Compound, Location, Plane, Polyline, Pos, Rectangle, Rot, ThreePointArc,
                       extrude, make_face, revolve)


@dataclass(frozen=True)
class Sfh4550:
    length: float = 8.6  # seating plane to lens tip: 8.2..9.0 (p.10)
    front_to_tip: float = 7.65  # flange front to lens tip: 7.5..7.8 (p.10)
    body_d: float = 4.95  # 4.8..5.1 (p.10)
    body_d_max: float = 5.1
    flange_d: float = 5.7  # 5.5..5.9 (p.10, front view)
    flange_d_max: float = 5.9
    flat: float = 2.475  # flange flat on the cathode side, from the axis: not dimensioned, drawn flush with the body
    chip: float = 5.4  # chip below the lens tip: 5.1..5.7 (p.10, "chip position")
    lead: float = 0.5  # square: 0.4..0.6 (p.10)
    lead_max: float = 0.6
    pitch: float = 2.54  # p.10
    stub: tuple = (1.0, 2.0)  # wider lead section (dambar) behind the seating plane: drawn to scale, not dimensioned
    stub_w: float = 0.65  # its width in the lead plane: 0.5..0.8 (p.10)
    # no bending rule in the datasheet: bend past the stub, the same 2 mm Toshiba asks for
    bend_min: float = 2.0
    bend_r: float = 0.5  # inner bend radius: one lead thickness (IPC-A-610 minimum for leads under 0.8 mm)

    @property
    def flange_t(self):  # 0.95 (0.4..1.5 by the stack); the drawing scales to 0.96-0.99
        return self.length - self.front_to_tip

    @property
    def dome(self):  # the lens is a hemisphere of the body's radius (p.10, drawn so)
        return self.length - self.body_d / 2


@dataclass(frozen=True)
class Tps601a:
    length: float = 6.5  # seating plane to lens tip: +-0.5 (p.1)
    can_h: float = 4.5  # seating plane to the can's top edge, where the lens starts: +-0.3 (p.1)
    can_d: float = 4.7  # +0.1 -0.15 (p.1)
    can_d_max: float = 4.8
    flange_d: float = 5.6  # 5.8 max (p.1); nominal assumed mid JEDEC TO-18 (5.31..5.84)
    flange_d_max: float = 5.8
    flange_t: float = 0.5  # +-0.2 (p.1)
    tab_w: float = 1.0  # key tab width: +-0.2 (p.1)
    tab_reach: float = 1.0  # beyond the flange: +-0.2 (p.1)
    # from the lead line, beside pin 1 (emitter): +-5 (p.1). The circle view is read as a bottom view (JIS third
    # angle, leads drawn solid): mounted, the tab points up at 45 deg on the -v side, towards the emitter LED.
    tab_angle: float = 45.0
    lead: float = 0.45  # round: +-0.1 (p.1)
    lead_max: float = 0.55
    pitch: float = 2.54  # +-0.3 (p.1)
    bend_min: float = 2.0  # "formed at a distance of 2mm from the body" (p.2, precaution 2)
    bend_r: float = 0.45  # inner bend radius: one lead diameter (as the SFH)

    @property
    def lens_r(self):  # spherical cap on the can: 2.0 high on a 4.7 base -> r 2.38, nearly a hemisphere
        h, a = self.length - self.can_h, self.can_d / 2
        return (a * a + h * h) / (2 * h)


SFH4550, TPS601A = Sfh4550(), Tps601a()

# Placement in the footprint (WallSensor): fixed by the footprint and the assembly, kept from the old model
LEG_V = 1.27  # pads at v = +-1.27
LEG_END = -1.45  # leg tips below the board top (1.04 board, 0.4 through)
EMITTER_Z, EMITTER_LEGS_U = 9.75, -5.62
RECEIVER_Z, RECEIVER_LEGS_U = 3.25, -3.12
# Bend distance behind the seating plane. The emitter keeps the old model's seat (u -2.32), where the cap holds
# it: its leads then bend 2.55 from the body, legal. The old receiver bent 0.85 from its body (2.0 minimum): at
# the minimum its seat moves 0.875 forward.
EMITTER_BEND = -2.32 - EMITTER_LEGS_U - (SFH4550.bend_r + SFH4550.lead / 2)
RECEIVER_BEND = TPS601A.bend_min
COLORS = {"SFH4550": (0.78, 0.84, 0.92), "TPS601A": (0.72, 0.72, 0.75)}


def _lathe(pts, arc, tip):
    """Solid of revolution about +x from an (x, r) outline: pts run from the axis at the back to the start of the
    lens, arc = (mid, end) of the lens profile, ending on the axis at the tip."""
    p3 = [(x, 0, r) for x, r in pts]
    mid = (arc[0], 0, arc[1])
    edges = Polyline(*p3) + ThreePointArc(p3[-1], mid, (tip, 0, 0)) + Polyline((tip, 0, 0), p3[0])
    return revolve(make_face(edges), Axis.X, 360)


def _leads(section, bend, rc, drop, pitch, overlap):
    """Two leads at y = +-pitch/2: back along -x from overlap inside the body to the bend, a 90 deg bend of
    centreline radius rc, and down to z = -drop. section(plane) is the lead's cross-section on that plane."""
    out = None
    for y in (-pitch / 2, pitch / 2):
        leg = extrude(section(Plane((overlap, y, 0), x_dir=(0, 1, 0), z_dir=(-1, 0, 0))), bend + overlap)
        face = section(Plane((-bend, y, 0), x_dir=(0, 1, 0), z_dir=(-1, 0, 0)))
        leg += revolve(face, Axis((-bend, y, -rc), (0, 1, 0)), -90)
        leg += extrude(section(Plane((-bend - rc, y, -rc), x_dir=(0, 1, 0), z_dir=(0, 0, -1))), drop - rc)
        out = leg if out is None else out + leg
    return out


def _single(part, label):
    solids = part.solids()
    assert len(solids) == 1 and solids[0].is_valid, (label, len(solids))
    s = solids[0]
    s.label, s.color = label, COLORS[label]
    return s


def emitter(e: Sfh4550 = SFH4550, bend=EMITTER_BEND, drop=EMITTER_Z - LEG_END):
    """SFH 4550 with its leads bent down bend behind the seating plane, ending drop below the axis."""
    assert bend >= e.bend_min - 1e-9, bend
    rf, rb, t = e.flange_d / 2, e.body_d / 2, e.flange_t
    a = radians(45)
    body = _lathe([(0, 0), (0, rf), (t, rf), (t, rb), (e.dome, rb)], (e.dome + rb * sin(a), rb * cos(a)), e.length)
    # flange flat on the cathode side (-y)
    body -= Pos(t / 2, -e.flat - rf, 0) * Box(t + 0.02, 2 * rf, 2 * rf)
    rc = e.bend_r + e.lead / 2
    body += _leads(lambda pl: pl * Rectangle(e.lead, e.lead), bend, rc, drop, e.pitch, t / 2)
    # wider lead section next to the body
    for y in (-e.pitch / 2, e.pitch / 2):
        body += Pos(-sum(e.stub) / 2, y, 0) * Box(e.stub[1] - e.stub[0], e.stub_w, e.lead)
    return _single(body, "SFH4550")


def receiver(r: Tps601a = TPS601A, bend=RECEIVER_BEND, drop=RECEIVER_Z - LEG_END):
    """TPS601A with its leads bent down bend behind the seating plane, ending drop below the axis."""
    assert bend >= r.bend_min - 1e-9, bend
    rf, rb, t, R = r.flange_d / 2, r.can_d / 2, r.flange_t, r.lens_r
    cx = r.length - R  # lens sphere centre
    a = radians(45)
    body = _lathe([(0, 0), (0, rf), (t, rf), (t, rb), (r.can_h, rb)], (cx + R * sin(a), R * cos(a)), r.length)
    # key tab on the flange, beside pin 1 (-y), tab_angle from the lead line towards +z
    reach = r.tab_reach + 0.3
    body += Rot(180 - r.tab_angle, 0, 0) * Pos(t / 2, rf + r.tab_reach - reach / 2, 0) * Box(t, reach, r.tab_w)
    rc = r.bend_r + r.lead / 2
    body += _leads(lambda pl: pl * Circle(r.lead / 2), bend, rc, drop, r.pitch, t / 2)
    return _single(body, "TPS601A")


@dataclass(frozen=True)
class Seat:
    """One LED as mounted, in the cap's frame (u along the look direction from the footprint origin, v left,
    z above the board top). Field names follow front.Led."""
    z: float  # optical axis
    flange: tuple  # (u back = seating plane, u front)
    flange_r: float  # nominal
    flange_r_max: float
    body_r: float  # nominal (epoxy body / can)
    body_r_max: float
    dome: float  # u where the lens starts
    tip: float  # u of the lens tip
    legs_u: float  # u of the vertical leg run (pads)
    bend_u: float  # u where the leads start to bend (they run straight from the body to here)
    bend_z: float  # z where the vertical leg run starts (below it the leads are straight)
    leg_v: float  # legs at v = +-leg_v
    leg_w: float  # lead width / diameter, nominal
    leg_w_max: float
    chip: float  # u of the chip (emitter) / sensitive area (receiver: on the header, assumed)


@dataclass(frozen=True)
class Key:
    """A flange feature that breaks the round flange, in the cap's frame."""
    dir: tuple  # (v, z) unit vector from the axis
    dist: float  # flat: its distance from the axis; tab: its outer end from the axis (nominal)
    dist_max: float
    w: float  # tab width (0 for a flat)
    w_max: float


def _seats():
    e, r = SFH4550, TPS601A
    eu = EMITTER_LEGS_U + e.bend_r + e.lead / 2 + EMITTER_BEND
    ru = RECEIVER_LEGS_U + r.bend_r + r.lead / 2 + RECEIVER_BEND
    em = Seat(z=EMITTER_Z, flange=(eu, eu + e.flange_t), flange_r=e.flange_d / 2, flange_r_max=e.flange_d_max / 2,
              body_r=e.body_d / 2, body_r_max=e.body_d_max / 2, dome=eu + e.dome, tip=eu + e.length,
              legs_u=EMITTER_LEGS_U, bend_u=eu - EMITTER_BEND, bend_z=EMITTER_Z - e.bend_r - e.lead / 2,
              leg_v=LEG_V, leg_w=e.lead, leg_w_max=e.lead_max, chip=eu + e.length - e.chip)
    rc = Seat(z=RECEIVER_Z, flange=(ru, ru + r.flange_t), flange_r=r.flange_d / 2, flange_r_max=r.flange_d_max / 2,
              body_r=r.can_d / 2, body_r_max=r.can_d_max / 2, dome=ru + r.can_h, tip=ru + r.length,
              legs_u=RECEIVER_LEGS_U, bend_u=ru - RECEIVER_BEND, bend_z=RECEIVER_Z - r.bend_r - r.lead / 2,
              leg_v=LEG_V, leg_w=r.lead, leg_w_max=r.lead_max, chip=ru + r.flange_t)
    a = radians(r.tab_angle)
    flat = Key(dir=(-1.0, 0.0), dist=e.flat, dist_max=e.flat, w=0.0, w_max=0.0)
    tab = Key(dir=(-cos(a), sin(a)), dist=r.flange_d / 2 + r.tab_reach, dist_max=r.flange_d_max / 2 + r.tab_reach + 0.2,
              w=r.tab_w, w_max=r.tab_w + 0.2)
    em, rc, flat, tab = (type(k)(**{f: _round(v) for f, v in vars(k).items()}) for k in (em, rc, flat, tab))
    return em, rc, flat, tab


def _round(v):
    return tuple(round(x, 4) for x in v) if isinstance(v, tuple) else round(v, 4)


EMITTER, RECEIVER, EMITTER_FLAT, RECEIVER_TAB = _seats()
SEATS = {"SFH4550": EMITTER, "TPS601A": RECEIVER}

# The footprint's model frame (KiCad rotates the STEP by -90 about x): model (x, y, z) = (-v, z, -u)
TO_MODEL = Location(Plane((0, 0, 0), x_dir=(0, 0, -1), z_dir=(0, 1, 0)))


def placed():
    """Both parts in the cap's frame."""
    return [Pos(EMITTER.flange[0], 0, EMITTER.z) * emitter(), Pos(RECEIVER.flange[0], 0, RECEIVER.z) * receiver()]


def sensor_model():
    """Drop-in for WALL_SENSOR_STACKED.STEP (without its casing): same frame, axis heights and leg positions."""
    kids = []
    for p in placed():
        k = TO_MODEL * p
        k.label, k.color = p.label, p.color
        kids.append(k)
    return Compound(children=kids, label="WALL_SENSOR_STACKED")
