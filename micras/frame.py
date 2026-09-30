"""Top frame (FDM, PETG): battery box, posts onto the bearing-block caps, the fan "airbox" tube, and
the mounts for the halo and the battery-box lid (see body.py).

The frame screws to the frame bosses on both caps. The airbox tube screws to the fan mount's two ears
and its lugs sit on the fan motor's rear face, so the motor is captured between the mount plate and
the tube. The battery box walls hold the cells on every side; the lid holds them down.
"""

from dataclasses import dataclass
from math import sqrt

from build123d import Align, Box, Cylinder, Pos

from . import drive, fan
from .mass import battery_cells
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class FrameParams:
    floor_t: float = 1.2  # box floor (FDM: 6 layers at 0.2)
    wall_t: float = 0.9  # box walls (2 perimeters at 0.45)
    pack_clear: float = 0.3
    post_d: float = 5.6
    boss_d: float = 5.6  # around an M2 insert
    rib: float = 1.6  # ribs left between floor windows
    window: float = 7.0
    gill_w: float = 1.8  # vertical gill slots in the long walls
    gill_pitch: float = 3.6
    louvre_margin: float = 2.5  # solid wall kept at the top and bottom of the louvres
    wire_slot: tuple = (6.0, 5.0)  # (width, height) at the bottom of each end wall
    tube_wall: float = 1.2
    tube_clear: float = 0.3  # around the fan motor
    flange_t: float = 1.5  # tube flange screwed to the fan mount ears
    lug_t: float = 1.0  # lugs on the motor's rear face
    lug_w: float = 3.0
    lug_gap: float = 0.1  # axial play of the fan motor
    spine_h: float = 3.3  # rails stay 0.5 above the raised motor
    halo_r: float = 11.0  # halo hoop radius around the fan axis (feet sit on the spine rails)


FR = FrameParams()


def D_TRAY_TOP(p: Params = P, d=drive.D, fr: "FrameParams" = None):
    """Top of the box floor."""
    return d.tray_bottom_z + (fr or FR).floor_t


def pack_size(p: Params = P):
    """(x, y, z) size of the battery pack."""
    _, (sx, sz) = battery_cells(p.battery.arrangement, p)
    L, W, T = p.battery.cell
    sy = W if p.battery.arrangement == "standing" else L
    return sx, sy, sz


def tray_box(p: Params = P, fr: FrameParams = FR):
    """(x0, x1, y0, y1) of the box outline."""
    sx, sy, _ = pack_size(p)
    m = fr.pack_clear + fr.wall_t
    return p.battery.x - sx / 2 - m, p.battery.x + sx / 2 + m, -sy / 2 - m, sy / 2 + m


def box_top(p: Params = P, fr: FrameParams = FR, d=drive.D):
    return D_TRAY_TOP(p, d, fr) + pack_size(p)[2] + fr.pack_clear


def rail_y(p: Params = P, fr: FrameParams = FR):
    return p.motor.d / 2 + fr.tube_clear + fr.tube_wall - fr.wall_t / 2


def halo_feet(p: Params = P, fr: FrameParams = FR):
    """(x, |y|) of the two halo feet: where the hoop crosses the spine rails."""
    fx, _ = fan.centre(p)
    ry = rail_y(p, fr)
    return fx - sqrt(fr.halo_r**2 - ry**2), ry


def lid_bosses(p: Params = P, fr: FrameParams = FR):
    """(x, y) of the two lid screws: outside the middle of the front and rear walls."""
    x0, x1, _, _ = tray_box(p, fr)
    r = fr.boss_d / 2
    return (x1 + r - 0.3, 0.0), (x0 - r + 0.3, 0.0)


def frame(p: Params = P, fr: FrameParams = FR, d=drive.D, f=fan.F):
    z0 = d.tray_bottom_z
    z1 = z0 + fr.floor_t
    zt = box_top(p, fr, d)
    x0, x1, y0, y1 = tray_box(p, fr)
    cx = (x0 + x1) / 2

    # box floor with windows
    body = Pos(cx, 0, z0) * Box(x1 - x0, y1 - y0, fr.floor_t, align=MIN)
    nx = max(1, int((x1 - x0 - fr.rib) // (fr.window + fr.rib)))
    ny = max(1, int((y1 - y0 - fr.rib) // (fr.window + fr.rib)))
    wx = (x1 - x0 - fr.rib) / nx - fr.rib
    wy = (y1 - y0 - fr.rib) / ny - fr.rib
    for i in range(nx):
        for j in range(ny):
            body -= Pos(x0 + fr.rib + wx / 2 + i * (wx + fr.rib), y0 + fr.rib + wy / 2 + j * (wy + fr.rib), z0) * Box(
                wx, wy, fr.floor_t, align=MIN)
    # full-height walls on all four sides
    outer = Pos(cx, 0, z1) * Box(x1 - x0, y1 - y0, zt - z1, align=MIN)
    inner = Pos(cx, 0, z1) * Box(x1 - x0 - 2 * fr.wall_t, y1 - y0 - 2 * fr.wall_t, zt - z1, align=MIN)
    body += outer - inner
    # gills in the front and rear walls (lighter, and the race-car look)
    lh = zt - z1 - 2 * fr.louvre_margin
    n = int((y1 - y0 - 6) // fr.gill_pitch)
    for i in range(n):
        yc = (i - (n - 1) / 2) * fr.gill_pitch
        if abs(yc) < fr.boss_d / 2 + fr.gill_w:  # keep the wall solid behind the lid bosses
            continue
        for x in (x0, x1):
            body -= Pos(x, yc, z1 + fr.louvre_margin) * Box(3 * fr.wall_t, fr.gill_w, lh, align=MIN)
    # wire slots at the bottom of both end walls
    sw, sh = fr.wire_slot
    for y in (y0, y1):
        body -= Pos(x0 + sw / 2 + fr.wall_t + 1, y, z1) * Box(sw, 3 * fr.wall_t, sh, align=MIN)
    # lid bosses outside the front and rear walls, insert at the top
    for bx, by in lid_bosses(p, fr):
        body += Pos(bx, by, zt - 6) * Cylinder(fr.boss_d / 2, 6, align=MIN)
        body -= Pos(bx, by, zt) * Cylinder(d.insert_d / 2, d.insert_l, align=MAX)
        body -= Pos(bx, by, zt) * Cylinder(d.screw_clear_d / 2, d.screw_l - 0.9 + 0.5, align=MAX)  # lid 0.9 thick

    # posts onto the two cap bosses
    for side in (1, -1):
        bx, by, btop = drive.frame_boss(side, p, d)
        by *= side
        body += Pos(bx, by, z0) * Cylinder(fr.post_d / 2 + 0.6, fr.floor_t, align=MIN)
        if btop < z0:  # post down to a lower boss, screwed through its floor
            body += Pos(bx, by, btop) * Cylinder(fr.post_d / 2, z0 - btop, align=MIN)
            body -= Pos(bx, by, btop + fr.floor_t) * drive.countersunk(d, depth=5, up=40)
        else:
            body -= Pos(bx, by, z1) * drive.countersunk(d, depth=5, up=40)

    # airbox tube: flange screwed to the fan mount ears, lugs on the motor's rear face
    h = fan.heights(p, f)
    fx, fy = fan.centre(p)
    collar_top = h["plate"] + f.plate_t + f.collar_h
    motor_rear = h["motor"] + p.motor.body_l + fr.lug_gap
    r_in = p.motor.d / 2 + fr.tube_clear
    r_out = r_in + fr.tube_wall
    body += Pos(fx, fy, collar_top) * Cylinder(r_out, motor_rear - collar_top, align=MIN)
    er = f.ear_r
    body += Pos(fx, fy, collar_top) * Box(fr.boss_d, 2 * er + fr.boss_d, fr.flange_t, align=MIN)
    for sy in (1, -1):
        body += Pos(fx, sy * er, collar_top) * Cylinder(fr.boss_d / 2, fr.flange_t, align=MIN)
        body -= Pos(fx, sy * er, collar_top + fr.flange_t) * drive.countersunk(d, depth=5)
    body -= Pos(fx, fy, collar_top - 1) * Cylinder(r_in, 40, align=MIN)
    # two lugs across the tube top, 90 deg from the motor's terminal tabs (tabs lie along y)
    for sx in (1, -1):
        body += Pos(fx + sx * (r_in - fr.lug_w / 2 + fr.tube_wall / 2), fy, motor_rear) * Box(
            fr.lug_w + fr.tube_wall, fr.lug_w, fr.lug_t, align=MIN)

    # spine from the box front to the tube: two rails and a floor bridge
    ry = rail_y(p, fr)
    for sy in (1, -1):
        body += Pos((x1 + fx) / 2, sy * ry, z1 - fr.spine_h) * Box(fx - x1, fr.wall_t, fr.spine_h, align=MIN)
    body += Pos((x1 + fx) / 2, 0, z0) * Box(fx - x1, 2 * r_out, fr.floor_t, align=MIN)
    # halo feet: bosses on the rails with inserts
    hx, hy = halo_feet(p, fr)
    for sy in (1, -1):
        body += Pos(hx, sy * hy, z1 - fr.spine_h) * Cylinder(fr.boss_d / 2, fr.spine_h, align=MIN)
        body -= Pos(hx, sy * hy, z1) * Cylinder(d.insert_d / 2, d.insert_l, align=MAX)
        body -= Pos(hx, sy * hy, z1) * Cylinder(d.screw_clear_d / 2, fr.spine_h, align=MAX)
    body -= Pos(fx, fy, z0 - 1) * Cylinder(r_in, fr.floor_t + 2, align=MIN)
    body.label, body.color = "frame", (0.25, 0.25, 0.28)
    return body


def parts(p: Params = P):
    return {"frame": frame(p)}
