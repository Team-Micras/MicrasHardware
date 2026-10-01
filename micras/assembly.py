"""Every designed part, with its material and print notes, for rendering, viewing and export."""

from . import drive, fan, frame, front

# part-name prefix -> (material, printer, orientation / notes)
MATERIALS = {
    "block_base": ("resin", "Photon Mono 4", "flat pad face down on the plate; light supports under the lifted body (about 150 mm2, 1.8-3.2 mm tall) and the rounded top; clear the insert holes of burn-in, glue the M2 inserts"),
    "block_cap": ("resin", "Photon Mono 4", "split face up (bores open upward), supports on the outside; glue inserts; an M2 nut goes in the fan ear's trap before the cap is fitted"),
    "block_": ("resin", "Photon Mono 4", "one-piece block: inboard face down (all bores vertical: round), supports on that hidden face; glue the inserts; press the bearings in from either end up to the ridge; an M2 nut goes in the fan ear's trap"),
    "sleeve": ("resin", "Photon Mono 4", "axis vertical, notched rim up; ream the motor bore if tight"),
    "magnet_cup": ("resin", "Photon Mono 4", "axis vertical, magnet pocket up; glue the magnet with the correct pole direction"),
    "race_spacer": ("resin", "Photon Mono 4", "axis vertical; tiny (11 layers): measure 0.55 thick and lap it on fine sandpaper on glass if thicker; a 2 mm shim washer that clears the outer race also works"),
    "wheel_hub": ("resin", "Photon Mono 4", "axis vertical, web down (its vents keep the drum from acting as a suction cup); glue to the gear face; the tire (cut to the channel's width) is stretched over the outer flange into its channel, no glue"),
    "impeller": ("resin", "Photon Mono 4", "tilted 45 deg, hub side towards the plate, supports on the backplate and hub only (the flat shroud face is the inlet seal: keep it support-free); ream the bore to 0.97-0.98, balance"),
    "fan_mount": ("resin", "Photon Mono 4", "collar up, supports under the foot, the arms' undersides and the plate; put an M2 nut in the clamp ear's trap"),
    "sensor_cap": ("resin", "Photon Mono 4", "front face down (bores vertical, fork up), on supports; paint it black with an IR-opaque (carbon-black) paint, outside and in the bores, and check it on the sensor (docs/printing.md)"),
    "bumper": ("tpu", "Ender 3 V3 SE", "upside down, flat top on the bed (the rounded lower edge then needs no supports); 100 % infill"),
    "gear_": ("resin", "Photon Mono 4", "stand-ins for the brass gears, print 2 of each: axis vertical, lifted on supports (on the plate the first layers flare the teeth); tough / ABS-like resin if available; drill the bore (1.0 pinion, 2.0 wheel) and glue; a pair only, don't mix with a brass gear"),
    "fit_test": ("resin", "Photon Mono 4", "calibration coupon: flat, on supports like the parts; print it first (docs/printing.md)"),
    "basket": ("pla", "Ender 3 V3 SE", "upside down (wall tops on the bed), no supports: the floor bridges ~21 mm between the front and rear walls (bridge settings on); the feet and posts print as short columns, the strap lugs are 45 deg wedges"),
}


# how each part goes on the printer: part-name prefix -> (robot-frame direction that faces the build plate, written
# for the left (_L) part (mirrored for _R), or "look" for a sensor cap's look direction; tilt about the printer's
# x after that, deg; copies to print)
PRINT = {
    "block_base": ((0, 0, -1), 0, 1),  # pads down
    "block_cap": ((0, 0, 1), 0, 1),  # split face up
    "block_": ((0, -1, 0), 0, 1),  # one-piece block: inboard face down, all bores vertical
    "sleeve": ((0, 1, 0), 0, 1),  # notched rim up
    "magnet_cup": ((0, 1, 0), 0, 1),  # magnet pocket up
    "race_spacer": ((0, 1, 0), 0, 2),  # tiny: a spare
    "wheel_hub": ((0, -1, 0), 0, 1),  # web down
    "impeller": ((0, 0, 1), 45, 1),  # hub side towards the plate, tilted (the shroud's inside supportable)
    "fan_mount": ((0, 0, -1), 0, 1),  # collar up
    "sensor_cap_test": ((1, 0, 0), 0, 1),  # calibration caps (W1's shape)
    "sensor_cap": ("look", 0, 1),  # front face down
    "bumper": ((0, 0, 1), 0, 1),  # upside down
    "gear_": ((0, 0, -1), 0, 2),  # axis vertical (gears.py's frame: outer face up)
    "basket": ((0, 0, 1), 0, 1),  # upside down
    "fit_test": ((0, 0, -1), 0, 1),
}


def print_pose(name):
    """(down, tilt, copies) for a part, `down` already for its side."""
    from . import front
    down, tilt, copies = next(v for k, v in PRINT.items() if name.startswith(k))
    if down == "look":
        from math import cos, radians, sin
        a = radians(front.SENSORS[name.rsplit("_", 1)[1]][1])
        down = (cos(a), sin(a), 0)
    elif name.endswith("_R"):
        down = (down[0], -down[1], down[2])
    return down, tilt, copies


def oriented(part, name):
    """The part turned into its print orientation, centred on the plate, standing on z = 0."""
    from build123d import Plane, Pos, Rot
    down, tilt, _ = print_pose(name)
    up = tuple(-c for c in down)
    x_dir = (1, 0, 0) if abs(up[0]) < 0.9 else (0, 1, 0)
    out = Plane(origin=(0, 0, 0), x_dir=x_dir, z_dir=up).to_local_coords(part)
    if tilt:
        out = Rot(tilt, 0, 0) * out
    bb = out.bounding_box()
    out = Pos(-(bb.min.X + bb.max.X) / 2, -(bb.min.Y + bb.max.Y) / 2, -bb.min.Z) * out
    out.label = name
    return out


def material(name):
    for prefix, info in MATERIALS.items():
        if name.startswith(prefix):
            return info
    raise KeyError(name)


def printed():
    fan_parts = {k: v for k, v in fan.parts().items() if k != "fan_motor"}
    return {**drive.all_parts(), **fan_parts, **front.parts(), **frame.parts()}


GROUPS = {  # viewer groups: part-name prefix -> group (checked in order; per-side parts get _L/_R groups)
    "cell": "battery",
    "basket": "battery", "velcro": "battery",
    "impeller": "fan", "fan_": "fan",
    "sensor_cap": "front", "bumper": "front",
    "encoder_": "encoders",
}


def groups():
    """{group: {part: shape}}: every bought and printed part, grouped for the viewer (drive parts by side)."""
    out = {}
    for name, shape in {**bought(), **printed()}.items():
        group = next((g for prefix, g in GROUPS.items() if name.startswith(prefix)), None)
        if group is None:
            group = "drive_" + name[-1] if name[-2:] in ("_L", "_R") else "other"
        elif name[-2:] in ("_L", "_R") and group == "encoders":
            group = "drive_" + name[-1]
        out.setdefault(group, {})[name] = shape
    return out


def display():
    """Printed parts plus the real (meshed) gears in place of the bought gears' tip circles: for pictures."""
    from . import gears
    return {**printed(), **gears.placed(1), **gears.placed(-1)}


def bought():
    from .layout import reference
    return {**reference(with_board=False), "fan_motor": fan.fan_motor(), **frame.straps()}
