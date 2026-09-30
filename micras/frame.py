"""Top frame (FDM, PETG): battery tray, posts onto the bearing-block caps, and the fan tube.

The frame is screwed to the frame bosses on both caps and its fan tube presses the fan mount onto
its feet. The front bumper hangs from the fan tube (see front.py).
"""

from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Pos, Rot

from . import drive, fan
from .mass import battery_cells
from .params import P, Params

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
MAX = (Align.CENTER, Align.CENTER, Align.MAX)


@dataclass(frozen=True)
class FrameParams:
    floor_t: float = 1.2  # tray floor (FDM: 6 layers at 0.2)
    wall_t: float = 1.2  # 3 perimeters at 0.4
    wall_h: float = 3.0  # low walls locate the pack; a TPU strap holds it down
    pack_clear: float = 0.3
    post_d: float = 5.6
    screw_head_d: float = 4.4  # access bore for the M2 flat heads
    csk_d: float = 4.0
    rib: float = 1.6  # ribs left between floor lightening windows
    window: float = 7.0
    tube_wall: float = 1.2
    tube_clear: float = 0.3  # around the fan motor
    spine_w: float = 6.0
    spine_h: float = 4.0
    nose_boss: tuple = (3.0, 6.0, 7.0)  # (x depth, y width, z height) on the tube front, for the nose screw


FR = FrameParams()


def D_TRAY_TOP(p: Params = P, d=drive.D, fr: "FrameParams" = None):
    """Top of the tray floor (= top of the fan tube)."""
    return d.tray_bottom_z + (fr or FR).floor_t


def pack_size(p: Params = P):
    """(x, y, z) size of the battery pack."""
    _, (sx, sz) = battery_cells(p.battery.arrangement, p)
    L, W, T = p.battery.cell
    sy = W if p.battery.arrangement == "standing" else L
    return sx, sy, sz


def tray_box(p: Params = P, fr: FrameParams = FR):
    """(x0, x1, y0, y1) of the tray outline."""
    sx, sy, _ = pack_size(p)
    m = fr.pack_clear + fr.wall_t
    return p.battery.x - sx / 2 - m, p.battery.x + sx / 2 + m, -sy / 2 - m, sy / 2 + m


def frame(p: Params = P, fr: FrameParams = FR, d=drive.D, f=fan.F):
    z0 = d.tray_bottom_z
    z1 = z0 + fr.floor_t
    x0, x1, y0, y1 = tray_box(p, fr)
    # tray floor with lightening windows, and low walls
    body = Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(x1 - x0, y1 - y0, fr.floor_t, align=MIN)
    nx = max(1, int((x1 - x0 - fr.rib) // (fr.window + fr.rib)))
    ny = max(1, int((y1 - y0 - fr.rib) // (fr.window + fr.rib)))
    wx = (x1 - x0 - fr.rib) / nx - fr.rib
    wy = (y1 - y0 - fr.rib) / ny - fr.rib
    for i in range(nx):
        for j in range(ny):
            cx = x0 + fr.rib + wx / 2 + i * (wx + fr.rib)
            cy = y0 + fr.rib + wy / 2 + j * (wy + fr.rib)
            body -= Pos(cx, cy, z0) * Box(wx, wy, fr.floor_t, align=MIN)
    outer = Pos((x0 + x1) / 2, (y0 + y1) / 2, z1) * Box(x1 - x0, y1 - y0, fr.wall_h, align=MIN)
    inner = Pos((x0 + x1) / 2, (y0 + y1) / 2, z1) * Box(x1 - x0 - 2 * fr.wall_t, y1 - y0 - 2 * fr.wall_t, fr.wall_h, align=MIN)
    body += outer - inner
    # wire exits in the rear corners
    for sy in (1, -1):
        body -= Pos(x0, sy * (y1 - 6), z1) * Box(4 * fr.wall_t, 6, fr.wall_h, align=MIN)

    # screw pads over the two cap bosses (solid floor around each)
    for side in (1, -1):
        bx, by, btop = drive.frame_boss(side, p, d)
        by *= side
        body += Pos(bx, by, z0) * Cylinder(fr.post_d / 2 + 0.6, fr.floor_t, align=MIN)
        if btop < z0:  # post down to a lower boss
            body += Pos(bx, by, btop) * Cylinder(fr.post_d / 2, z0 - btop, align=MIN)
            body -= Pos(bx, by, btop + fr.floor_t) * Cylinder(fr.screw_head_d / 2, 40, align=MIN)
            body -= Pos(bx, by, btop) * Cylinder(d.screw_clear_d / 2, fr.floor_t, align=MIN)
        else:
            body -= Pos(bx, by, z0) * Cylinder(d.screw_clear_d / 2, fr.floor_t, align=MIN)
            body -= Pos(bx, by, z1) * Cylinder(fr.csk_d / 2, (fr.csk_d - d.screw_clear_d) / 2, align=MAX)

    # fan tube: presses the fan mount collar down, guides the motor
    h = fan.heights(p, f)
    fx, fy = fan.centre(p)
    collar_top = h["plate"] + f.plate_t + f.collar_h
    r_in = p.motor.d / 2 + fr.tube_clear
    r_out = r_in + fr.tube_wall
    tube = Pos(fx, fy, collar_top) * Cylinder(r_out, z1 - collar_top, align=MIN)
    tube -= Pos(fx, fy, collar_top) * Cylinder(r_in, 40, align=MIN)
    body += tube
    # spine from the tray front to the tube (two rails, T-shaped)
    for sy in (1, -1):
        rail = Pos((x1 + fx) / 2, sy * (r_out - fr.wall_t / 2), z1 - fr.spine_h) * Box(
            fx - x1, fr.wall_t, fr.spine_h, align=MIN)
        body += rail
    body += Pos((x1 + fx) / 2, 0, z0) * Box(fx - x1, 2 * r_out, fr.floor_t, align=MIN)
    body -= Pos(fx, fy, z0 - 1) * Cylinder(r_in, 40, align=MIN)
    # nose mounting boss on the tube front, horizontal insert facing forward
    bx, bw, bh = fr.nose_boss
    face = fx + r_out - 0.6
    body += Pos(face, fy, z1 - bh) * Box(bx + 0.6, bw, bh, align=(Align.MIN, Align.CENTER, Align.MIN))
    zc = z1 - bh / 2
    body -= Pos(face + bx + 0.6, fy, zc) * Rot(0, -90, 0) * Cylinder(d.insert_d / 2, d.insert_l, align=MIN)
    body -= Pos(face + bx + 0.6, fy, zc) * Rot(0, -90, 0) * Cylinder(d.screw_clear_d / 2, d.screw_l, align=MIN)
    body -= Pos(fx, fy, z0 - 1) * Cylinder(r_in, 40, align=MIN)
    body.label, body.color = "frame", (0.3, 0.8, 0.5)
    return body


def nose_mount(p: Params = P, fr: FrameParams = FR):
    """(x of the boss front face, z of the screw axis) for the nose."""
    fx, _ = fan.centre(p)
    r_out = p.motor.d / 2 + fr.tube_clear + fr.tube_wall
    return fx + r_out + fr.nose_boss[0], D_TRAY_TOP(p) - fr.nose_boss[2] / 2


def parts(p: Params = P):
    return {"frame": frame(p)}
