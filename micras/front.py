"""Front: sensor caps (resin). The nose, front wing and skid are part of the top body (body.py).

Each sensor cap slides onto its emitter/detector pair along the sensor's look direction, keeps both
LEDs parallel, separates them optically and narrows their beams with a front aperture. The caps do not
touch the board.
"""

from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Pos, Rot

from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)

# Wall sensors: footprint origin (x, y) in the robot frame and look direction (deg from +x), from the
# KiCad footprints W1..W4. Local frame: u along the look direction, v to its left.
SENSORS = {
    "W1": ((36.7198, 21.5), 0.0),
    "W2": ((47.5698, 10.1858), 45.0),
    "W3": ((47.5700, -10.1860), -45.0),
    "W4": ((36.7198, -21.5), 0.0),
}
EMITTER_H, DETECTOR_H = 9.75, 3.25  # optical axes above the board top
LENS_U = {"emitter": 6.68, "detector": 7.68}  # lens tips along u
HOUSING_U = (-1.12, 6.88)  # envelope of the KiCad sensor model's housing


@dataclass(frozen=True)
class FrontParams:
    led_r: float = 2.95  # LED flange radius (body 2.75)
    led_fit: float = 0.05  # radial clearance (snug slide-on fit)
    cap_wall: float = 0.6
    cap_back: float = -1.0  # u where the cap starts (legs leave behind it)
    snout: float = 2.5  # beyond the housing front
    aperture_d: float = 3.0
    board_gap: float = 0.7  # clears the 0402 parts next to the sensors
    tube_wall: float = 0.6

FP = FrontParams()


def _local(part, sensor):
    (x, y), ang = SENSORS[sensor]
    return Pos(x, y, 0) * Rot(0, 0, ang) * part


def sensor_cap(sensor, p: Params = P, fp: FrontParams = FP):
    top = p.board.top_z
    r = fp.led_r + fp.led_fit
    u0, u1 = fp.cap_back, HOUSING_U[1] + fp.snout
    ze, zd = top + EMITTER_H, top + DETECTOR_H
    z0, z1 = top + fp.board_gap, ze + r + fp.cap_wall
    w = 2 * (r + fp.cap_wall)
    body = None
    for zc, name in ((ze, "emitter"), (zd, "detector")):
        lens = LENS_U[name]
        # sleeve around the LED up to its lens, then a thin aperture tube
        sleeve = Pos(u0, 0, zc) * Rot(0, 90, 0) * Cylinder(r + fp.cap_wall, lens + 0.2 - u0, align=MIN)
        tube = Pos(lens, 0, zc) * Rot(0, 90, 0) * Cylinder(fp.aperture_d / 2 + fp.tube_wall, u1 - lens, align=MIN)
        part = sleeve + tube
        body = part if body is None else body + part
    # web joining the two sleeves (also the optical septum)
    body += Pos((u0 + HOUSING_U[1]) / 2, 0, zd) * Box(HOUSING_U[1] - u0, fp.cap_wall * 2, ze - zd, align=MIN)
    for zc, name in ((ze, "emitter"), (zd, "detector")):
        lens = LENS_U[name]
        body -= Pos(u0 - 1, 0, zc) * Rot(0, 90, 0) * Cylinder(r, lens + 0.2 - u0 + 1, align=MIN)
        body -= Pos(lens, 0, zc) * Rot(0, 90, 0) * Cylinder(fp.aperture_d / 2, u1 - lens + 1, align=MIN)
    # stay off the board
    body -= Pos(0, 0, z0) * Box(100, 100, 20, align=(Align.CENTER, Align.CENTER, Align.MAX))
    body = _local(body, sensor)
    body.label, body.color = f"sensor_cap_{sensor}", (0.95, 0.85, 0.3)
    return body


def parts(p: Params = P):
    out = {}
    for s in SENSORS:
        cap = sensor_cap(s, p)
        out[cap.label] = cap
    return out
