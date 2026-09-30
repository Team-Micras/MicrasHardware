"""Every dimension of the chassis lives here.

Robot frame (same as the firmware): origin on the floor under the wheel-axle midpoint,
x forward, y left, z up, millimetres. Left-side values are given; the right side mirrors them
unless noted.
"""

from dataclasses import dataclass, field
from math import cos, radians, sin


@dataclass(frozen=True)
class Board:
    bottom_z: float = 1.0  # floor gap under the board
    thickness: float = 1.0412
    fan_hole_x: float = 17.5
    fan_hole_d: float = 15.0
    notch_inner_y: float = 17.25  # wheel notch inner face
    notch_x: tuple = (-8.5, 8.5)
    front_hole: tuple = (2.5, 11.75)
    rear_hole: tuple = (-15.5, 19.25)
    hole_d: float = 2.4
    countersink_d: float = 4.4
    encoder_slot_y: float = 6.9  # slot centre
    encoder_pcb_t: float = 0.77
    encoder_chip_h: float = 9.0  # package centre above board top
    encoder_top_h: float = 13.1  # encoder daughterboard top above board top
    hall_depth: float = 0.306  # Hall plane below package top
    package_h: float = 1.10  # TSSOP14 nominal (1.20 max)

    @property
    def top_z(self):
        return self.bottom_z + self.thickness


@dataclass(frozen=True)
class Motor:
    """Coreless 1020, per motor.png (components.md measured 9.61 diameter: check)."""
    d: float = 10.0
    body_l: float = 20.0
    shaft_d: float = 1.0
    shaft_l: float = 6.0
    rear_l: float = 2.0  # solder tabs behind the can (rear shaft is cut off; measure)
    wire_space: float = 2.0  # room for soldered wires behind the tabs
    boss_d: float = 3.0  # front bearing boss (not dimensioned on drawing: measure)
    boss_l: float = 0.5
    terminal_pitch: float = 7.4
    terminal_w: float = 1.5
    terminal_t: float = 0.32


@dataclass(frozen=True)
class Gears:
    module: float = 0.5
    pinion_z: int = 7
    wheel_z: int = 36
    pinion_w: float = 5.0
    wheel_w: float = 2.0
    pinion_bore: float = 0.98
    wheel_bore: float = 1.98
    center_adjust: float = 0.0  # added to the nominal centre distance (fixed-bore tuning)

    @property
    def center_distance(self):
        return self.module * (self.pinion_z + self.wheel_z) / 2 + self.center_adjust

    def tip_d(self, z):
        return self.module * (z + 2)

    def pitch_d(self, z):
        return self.module * z


@dataclass(frozen=True)
class Bearing:
    od: float = 5.0
    id: float = 2.0
    w: float = 2.5


@dataclass(frozen=True)
class Magnet:
    d: float = 6.0
    t: float = 2.0
    gap: float = 1.0  # package top to magnet face (magpylib: ~53 mT N35 / ~57 mT N42, window 35-70)


@dataclass(frozen=True)
class Wheel:
    tire_id: float = 16.0
    tire_t: float = 2.0
    tire_len: float = 10.0  # as bought, cut to width
    recess: float = 0.5  # tire outer face inside the board edge
    gear_gap_min: float = 0.3  # wheel gear to notch inner face, minimum
    axle_d: float = 2.0

    def stretched_t(self, hub_d):
        """Tire wall after stretching onto the hub (incompressible, equal biaxial thinning)."""
        mean0 = self.tire_id / 2 + self.tire_t / 2
        t = self.tire_t
        for _ in range(20):
            lam = (hub_d / 2 + t / 2) / mean0
            t = self.tire_t / lam**0.5
        return t


@dataclass(frozen=True)
class AxialStack:
    """Axial stack along the axle, from the magnet outwards (all along y)."""
    holder_wall: float = 0.5  # magnet cup wall behind the magnet
    holder_gap: float = 0.25  # magnet cup to housing shoulder (rotating vs static)
    shoulder: float = 0.4  # housing shoulder in front of the inner bearing
    shoulder_id: float = 3.6  # clears the inner race
    boss_d: float = 2.7  # bosses that touch only the inner races (MR52 inner race OD ~2.9: measure)
    lip: float = 0.3  # housing lip outside the outer bearing
    lip_gap: float = 0.25  # lip to wheel gear


@dataclass(frozen=True)
class Battery:
    cell: tuple = (47.16, 11.32, 6.45)  # length, width, thickness
    cell_mass: float = 6.0
    arrangement: str = "stack"  # "pyramid" | "edge" | "flat" | "stack" (tools/battery_study.py)
    gap: float = 0.3
    x: float = -9.7  # pack centre (tools/mass_report.py: CoM over the axle)
    floor_z: float = 30.0  # bottom of the cells (tray floor top)


@dataclass(frozen=True)
class Layout:
    # Motor axis angle around its wheel axle, measured from +x (forward) towards +z (up).
    motor_angle_left: float = 180.0  # low, straight behind the axle
    motor_angle_right: float = 112.0  # above the left motor, clears the encoder boards
    backlash_mode: str = "eccentric"  # "eccentric" | "fixed"
    eccentricity: float = 0.3
    clearance: float = 0.3  # minimum air gap to board components


@dataclass(frozen=True)
class Params:
    board: Board = field(default_factory=Board)
    motor: Motor = field(default_factory=Motor)
    gears: Gears = field(default_factory=Gears)
    bearing: Bearing = field(default_factory=Bearing)
    magnet: Magnet = field(default_factory=Magnet)
    wheel: Wheel = field(default_factory=Wheel)
    battery: Battery = field(default_factory=Battery)
    stack: AxialStack = field(default_factory=AxialStack)
    layout: Layout = field(default_factory=Layout)

    # ---- derived geometry -------------------------------------------------------------
    @property
    def axle_z(self):
        return self.board.top_z + self.board.encoder_chip_h

    @property
    def wheel_d(self):
        return 2 * self.axle_z

    @property
    def hub_d(self):
        """Hub diameter that makes the stretched tire reach the floor."""
        lo, hi = self.wheel.tire_id, self.wheel_d
        for _ in range(50):
            mid = (lo + hi) / 2
            if mid + 2 * self.wheel.stretched_t(mid) > self.wheel_d:
                hi = mid
            else:
                lo = mid
        return lo

    @property
    def hall_y(self):
        b = self.board
        return b.encoder_slot_y + b.encoder_pcb_t / 2 + b.package_h - b.hall_depth

    @property
    def package_face_y(self):
        b = self.board
        return b.encoder_slot_y + b.encoder_pcb_t / 2 + b.package_h

    @property
    def magnet_y(self):
        """Inner face of the magnet."""
        return self.package_face_y + self.magnet.gap

    @property
    def shoulder_y(self):
        """Inner face of the housing shoulder."""
        return self.magnet_y + self.magnet.t + self.stack.holder_wall + self.stack.holder_gap

    @property
    def bearing_inner_y(self):
        """Inner face of the inner bearing."""
        return self.shoulder_y + self.stack.shoulder

    @property
    def bearing_outer_y(self):
        """Outer face of the outer bearing."""
        return self.bearing_inner_y + 2 * self.bearing.w

    @property
    def gear_y(self):
        """Inner face of the wheel gear."""
        return max(self.bearing_outer_y + self.stack.lip + self.stack.lip_gap,
                   self.board.notch_inner_y + self.wheel.gear_gap_min)

    @property
    def tire_outer_y(self):
        return 25.0 - self.wheel.recess

    @property
    def tire_w(self):
        return self.tire_outer_y - (self.gear_y + self.gears.wheel_w)

    @property
    def track(self):
        return 2 * (self.tire_outer_y - self.tire_w / 2)

    def motor_axis(self, side):
        """(x, z) of a motor axis; side = +1 left, -1 right."""
        a = radians(self.layout.motor_angle_left if side > 0 else self.layout.motor_angle_right)
        cd = self.gears.center_distance
        return cd * cos(a), self.axle_z + cd * sin(a)

    @property
    def pinion_y(self):
        """Pinion outer face: flush with the wheel gear's outer face so it never reaches the hub."""
        return self.gear_y + self.gears.wheel_w

    @property
    def motor_front_y(self):
        """Motor front face; the shaft end is flush with the pinion outer face."""
        return self.pinion_y - self.motor.shaft_l


P = Params()
