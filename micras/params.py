"""Every dimension of the chassis lives here.

Robot frame (same as the firmware): origin on the floor under the wheel-axle midpoint,
x forward, y left, z up, millimetres. Left-side values are given; the right side mirrors them
unless noted.
"""

from dataclasses import dataclass, field
from math import cos, radians, sin


@dataclass(frozen=True)
class Board:
    # floor gap under the board: the axle sits at the encoder chip's height above the board, so this sets the
    # wheel size (Ø 2 x (bottom_z + thickness + encoder_chip_h)). 1.2 (was 1.0, Ø22.08 wheels) leaves room for
    # the tires' 0.35-0.5 mm squash under the fan's 4 N, with the 0.7 mm skates just clear (skids.py). The heights
    # given above the floor elsewhere (Battery.floor_z, DriveParams.tray_bottom_z and frame_boss_top_left,
    # FanParams.arm_z0 and leg_knee_z) were raised with it.
    bottom_z: float = 1.2
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
    """Coreless 1020 drive motor: Ø10 x 20 with a Ø1 x 6 shaft (motor.png, the datasheet; the owner confirmed the can
    is Ø10). The fan motor is another size (Params.fan_motor)."""
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
    # printed stand-in pair (gears.py): profile shift +shift on the pinion, -shift on the wheel (same centre
    # distance), pinion addendum cut by tip_cut (module)
    shift: float = 0.45
    tip_cut: float = 0.2

    @property
    def center_distance(self):
        return self.module * (self.pinion_z + self.wheel_z) / 2 + self.center_adjust

    def tip_d(self, z):
        """Tip diameter, the larger of the brass gear (unshifted) and the printed one."""
        x, cut = (self.shift, self.tip_cut) if z == self.pinion_z else (-self.shift, 0.0)
        return self.module * max(z + 2, z + 2 * (1 - cut + x))

    def pitch_d(self, z):
        return self.module * z


@dataclass(frozen=True)
class Bearing:
    od: float = 5.0
    id: float = 2.0
    w: float = 2.57  # 2.5 nominal; the bought ones measure 2.57 (a 2.5 pocket would not take them)


@dataclass(frozen=True)
class Magnet:
    d: float = 6.0
    t: float = 2.0
    gap: float = 0.5  # package top to magnet face (magpylib, Bz on the Hall circle: ~60 mT N35 / ~66 mT N42
    # centred, window 35-70; keep it centred). Closer than 1.0 to make room for the bearing ridge


@dataclass(frozen=True)
class Wheel:
    tire_id: float = 16.0
    tire_t: float = 2.0
    tire_len: float = 10.0  # as bought, cut to width
    recess: float = 0.5  # tire outer face inside the board edge
    gear_gap_min: float = 0.3  # wheel gear to notch inner face, minimum
    axle_d: float = 2.0
    # the hub holds the tire in a channel (no glue): a flange each side, flange_h above the seat (about half the
    # stretched tire, so the tread stays proud of them); the outer one ends at the board edge
    flange_h: float = 0.8
    flange_in: float = 0.6
    flange_out: float = 0.5
    # the pinion's outer face is flush with the gear's: the hub's drum and flanges stand this far off the gear
    # face (only its glued web, inside the pinion's tip circle, touches the gear)
    hub_relief: float = 0.35

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
    """Axial stack along the axle, from the magnet outwards (all along y).

    magnet cup (its sleeve runs in a room in the housing) | shoulder | inner bearing | lip = the block's outer face |
    tube, out through the wheel's gear into its hollow drum | outer bearing at the tube's end | the wheel's end web.
    The wheel (gear, drum and end web, one piece) turns round the tube; the two bearings sit about 6 mm apart.
    """
    holder_wall: float = 0.5  # magnet cup wall behind the magnet
    cup_wall: float = 0.75  # magnet cup wall round the magnet (0.5 looked fragile)
    holder_gap: float = 0.25  # magnet cup to housing (rotating vs static), axial and radial
    sleeve_d: float = 5.3  # the magnet cup's sleeve along the axle (it glues the cup to the axle)
    sleeve_l: float = 3.0  # the housing's room for the sleeve, from the housing's inboard face to the shoulder
    shoulder: float = 0.4  # housing shoulder in front of the inner bearing
    shoulder_id: float = 3.6  # clears the inner race
    bearing_play: float = 0.1  # axial room in each bearing pocket (the race cones push the bearings against the
    # lip and the tube's step)
    # the parts that bear on a bearing's inner race (the magnet cup and the wheel) end in a cone: boss_d where it
    # touches the race (inner race OD 2.9 measured; the shields are metal), cone_d where it leaves the housing's
    # bore, then a 45 deg flare to the part behind (a plain Ø2.7 boss left a 0.3 mm wall round the axle, which broke)
    boss_d: float = 2.8
    cone_d: float = 3.3
    lip: float = 0.3  # housing lip outside the inner bearing
    lip_gap: float = 0.25  # block's outer face to the wheel's gear
    tube_od: float = 7.2  # the outer bearing's tube (1 mm wall over the bearing)
    tube_gap: float = 0.3  # radial, tube to the wheel's bore
    tube_step: float = 0.6  # the step the outer bearing is pressed against (from the tube's inboard side)
    end_web: float = 0.8  # the wheel's outer web (it carries the axle boss and the cone onto the outer bearing)
    end_gap: float = 2.0  # tube's end to the end web: the wheel's axle boss fills it (2.9 mm of grip on the axle;
    # at 0.5 it gripped 1.4 mm and the wheel could tilt)


@dataclass(frozen=True)
class Battery:
    cell: tuple = (51.0, 11.32, 6.45)  # length (the 47.16 body and its leads), width, thickness
    cell_mass: float = 6.0
    arrangement: str = "edge"  # "pyramid" | "edge" | "flat" | "stack" (tools/battery_study.py)
    gap: float = 0.3
    x: float = -7.8  # pack centre (tools/mass_report.py: CoM over the axle)
    floor_z: float = 30.2  # bottom of the cells (tray floor top)


@dataclass(frozen=True)
class Layout:
    # Motor axis angle around its wheel axle, measured from +x (forward) towards +z (up).
    motor_angle_left: float = 180.0  # low, straight behind the axle
    motor_angle_right: float = 112.0  # above the left motor, clears the encoder boards
    backlash_mode: str = "fixed"  # "eccentric" | "fixed" (the owner's choice: split blocks without sleeves)
    blocks: str = "split"  # "split" (base + cap) | "solid" (one-piece bearing blocks)
    eccentricity: float = 0.3
    clearance: float = 0.3  # minimum air gap to board components


@dataclass(frozen=True)
class Params:
    board: Board = field(default_factory=Board)
    motor: Motor = field(default_factory=Motor)
    # fan motor (measured): Ø9.97 x 23.08, 54k rpm at 7.6 V; its shaft carries a pressed-on 9T module 0.3 pinion
    # (4.6 long) that doesn't come off, so the "shaft" here is that pinion (tip Ø3.3) and the impeller mounts on it
    fan_motor: Motor = field(default_factory=lambda: Motor(d=9.97, body_l=23.08, shaft_d=3.3, shaft_l=4.6))
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
        """Inner face of the inner bearing (against the lip, bearing_play from the shoulder)."""
        st = self.stack
        return self.shoulder_y + st.sleeve_l + st.shoulder + st.bearing_play

    @property
    def housing_end_y(self):
        """The block's outer face: the lip outside the inner bearing (the tube starts here)."""
        st = self.stack
        return self.bearing_inner_y + self.bearing.w + st.bearing_play + st.lip

    @property
    def gear_y(self):
        """Inner face of the wheel's gear."""
        return max(self.housing_end_y + self.stack.lip_gap, self.board.notch_inner_y + self.wheel.gear_gap_min)

    @property
    def wheel_outer_y(self):
        """Outer face of the wheel (its outer flange and end web)."""
        return self.tire_outer_y + self.wheel.flange_out

    @property
    def tube_end_y(self):
        return self.wheel_outer_y - self.stack.end_web - self.stack.end_gap

    @property
    def bearing_outer_y(self):
        """Outer face of the outer bearing (bearing_play inside the tube's end)."""
        return self.tube_end_y - self.stack.bearing_play

    @property
    def wheel_hole_d(self):
        """The wheel's bore round the tube."""
        return self.stack.tube_od + 2 * self.stack.tube_gap

    @property
    def tire_outer_y(self):
        return 25.0 - self.wheel.recess

    @property
    def tire_w(self):
        return self.tire_outer_y - (self.gear_y + self.gears.wheel_w) - self.wheel.hub_relief - self.wheel.flange_in

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


def override(assignment: str, p: Params = P):
    """Change a parameter of the shared P in place, e.g. "layout.backlash_mode=fixed", for what-if renders
    and checks (the part functions take P as their default). Values are parsed as numbers when possible."""
    path, value = assignment.split("=", 1)
    *parents, name = path.split(".")
    obj = p
    for a in parents:
        obj = getattr(obj, a)
    old = getattr(obj, name)
    try:
        value = type(old)(value) if not isinstance(old, str) else value
    except (TypeError, ValueError):
        pass
    object.__setattr__(obj, name, value)
