"""Front: sensor caps (resin) and the nose bumper with the floor skid (PETG).

Each sensor cap slides onto its emitter/detector pair along the sensor's look direction, keeps both
LEDs parallel, separates them optically and narrows their beams with a front aperture. The caps do not
touch the board. The nose bumper hangs from the frame's fan tube, bears on the board's front edge and
carries the skid that meets the floor when the fan pulls the nose down.
"""

from dataclasses import dataclass
from math import atan2, degrees, hypot

from build123d import Align, Box, Cylinder, Pos, Rot, Solid

from . import fan, frame
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
    # nose bumper
    nose_w: float = 9.0  # across y
    nose_t: float = 2.0  # along x, in front of the board edge
    nose_top_h: float = 8.0  # above the board top
    skid_z: float = 0.3  # skid bottom above the floor at rest
    skid_r: float = 1.0  # rounded skid edge
    edge_x: float = 53.5  # board front edge
    arm_w: float = 5.0
    arm_t: float = 1.2
    arm_h: float = 7.0  # web height (stiffness)
    plate_t: float = 1.6  # mounting plate against the frame boss


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


def nose(p: Params = P, fp: FrontParams = FP):
    top = p.board.top_z
    # nose bar in front of the board edge, down to the skid
    x0 = fp.edge_x
    bar = Pos(x0 + fp.nose_t / 2, 0, fp.skid_z) * Box(fp.nose_t, fp.nose_w, top + fp.nose_top_h - fp.skid_z, align=MIN)
    # rounded skid: a cylinder along y at the bottom front
    bar += Pos(x0 + fp.nose_t / 2, 0, fp.skid_z + fp.skid_r) * Rot(90, 0, 0) * Cylinder(fp.skid_r, fp.nose_w)
    # arm from the frame's nose boss to the top of the nose bar
    mx, mz = frame.nose_mount(p)
    plate_h = frame.FR.nose_boss[2]
    plate = Pos(mx, 0, mz) * Box(fp.plate_t, fp.arm_w + 1.0, plate_h, align=(Align.MIN, Align.CENTER, Align.CENTER))
    plate -= Pos(mx, 0, mz) * Rot(0, 90, 0) * Cylinder(1.1, fp.plate_t, align=MIN)
    plate -= Pos(mx + fp.plate_t, 0, mz) * Rot(0, -90, 0) * Cylinder(2.0, (4.0 - 2.2) / 2, align=MIN)
    a0 = (mx + fp.plate_t, mz + plate_h / 2)
    a1 = (x0 + fp.nose_t / 2, top + fp.nose_top_h)
    length = hypot(a1[0] - a0[0], a1[1] - a0[1])
    ang = degrees(atan2(a1[1] - a0[1], a1[0] - a0[0]))
    # T-section: flange (arm_w x arm_t) on top of a web (arm_t x arm_h)
    arm = Box(length, fp.arm_w, fp.arm_t, align=(Align.MIN, Align.CENTER, Align.MAX))
    arm += Box(length, fp.arm_t, fp.arm_h, align=(Align.MIN, Align.CENTER, Align.MAX))
    arm = Pos(a0[0], 0, a0[1]) * Rot(0, -ang, 0) * arm
    arm -= Pos(mx, 0, 0) * Box(50, 50, 100, align=(Align.MAX, Align.CENTER, Align.MIN))
    arm += plate
    body = bar + arm
    # keep clear of the board: nothing below the board top behind the edge
    body -= Pos(x0, 0, 0) * Box(100, 100, top + 0.3, align=(Align.MAX, Align.CENTER, Align.MIN))
    body.label, body.color = "nose", (0.3, 0.8, 0.5)
    return body


def parts(p: Params = P):
    out = {"nose": nose(p)}
    for s in SENSORS:
        cap = sensor_cap(s, p)
        out[cap.label] = cap
    return out
