"""Every designed part, with its material and print notes, for rendering, viewing and export."""

from . import drive, fan, frame, front, skids

# part-name prefix -> (material, printer, orientation / notes)
MATERIALS = {
    "block_cap": ("resin", "Photon Mono 4", "inboard (hidden) face down on supports: bores vertical, the split face, fan ear and basket seats come out as clean walls; glue the inserts; an M2 nut goes in the fan ear's trap before the cap is fitted"),
    "block_base": ("resin", "Photon Mono 4", "inboard (hidden) face down on supports: bores vertical, the pads and the split face come out as clean walls; glue the M2 inserts; press the outer bearing into the tube's end up to its step"),
    "block_": ("resin", "Photon Mono 4", "one-piece block: inboard face down (all bores vertical: round), supports on that hidden face; glue the inserts; press the inner bearing in from the inboard end up to the lip, the outer one into the tube's end up to its step; an M2 nut goes in the fan ear's trap"),
    "magnet_cup": ("resin", "Photon Mono 4", "axis vertical, magnet pocket up; glue the magnet with the correct pole direction, and the sleeve to the axle"),
    "wheel": ("resin", "Photon Mono 4", "drum, tire channel and end web in one piece; the drilled brass 36T screws onto its web face (three M2x5 countersunk from the gear's inboard face into nuts dropped into the pockets in the wheel's outer face; it sits on the three nut columns' ends); wheel_printed_gear_X carries its own printed teeth instead: axis vertical, end web down (the nut pockets or the vents keep the bore from acting as a suction cup), the web's face (or the teeth) on top, away from the supports; glue it to the axle; the tire (cut to the channel's width) is stretched over the outer flange into its channel, no glue"),
    "impeller": ("resin", "Photon Mono 4", "tilted 45 deg, hub side towards the plate, supports on the backplate and hub only (the flat shroud face is the inlet seal: keep it support-free); its bore is the pinion's shape: slide it on, wick in thin CA, balance it"),
    "fan_mount": ("resin", "Photon Mono 4", "collar up, supports under the arms' undersides and the plate; put an M2 nut in the clamp ear's trap"),
    "sensor_cap": ("pla", "Ender 3 V3 SE", "black PLA, front face down (bores vertical), no supports; check that the filament blocks IR before printing them (docs/printing.md); the test caps (1-3 dots) choose the crush ribs"),
    "gear_": ("resin", "Photon Mono 4", "printed pinions, 2 of each bore (the number is the bore in hundredths): axis vertical, lifted on supports (on the plate the first layers flare the teeth); glue the one that fits best onto the motor shaft; they mesh only with the wheel's printed gear"),
    "drill_guide": ("resin", "Photon Mono 4", "the brass 36T's drill guide (gear_guide.py, square outside for a vise): flat on supports, the base's gear-shaped pocket and the lid's holes vertical; drill the gear in it, then countersink its three holes by hand"),
    "fit_": ("resin", "Photon Mono 4", "calibration bar: holes vertical, on supports like the parts; try the bought part in each hole (docs/printing.md)"),
    "fit_test": ("resin", "Photon Mono 4", "calibration coupon: flat, on supports like the parts; print it first (docs/printing.md)"),
    "basket": ("pla", "Ender 3 V3 SE", "upside down (wall tops on the bed), the floor on supports from the bed (the bridges alone sagged); the feet and posts print as short columns, the strap lugs are 45 deg wedges"),
}


# how each part goes on the printer: part-name prefix -> (robot-frame direction that faces the build plate, written
# for the left (_L) part (mirrored for _R), or "look" for a sensor cap's look direction; tilt about the printer's
# x after that, deg; copies to print)
PRINT = {
    # the blocks stand on their hidden inboard face: every bore vertical, and the faces that seat (the pads on the
    # board, the split, the fan ear and the basket seats) are walls, not support-scarred down faces
    "block_base": ((0, -1, 0), 0, 1),
    "block_cap": ((0, -1, 0), 0, 1),
    "block_": ((0, -1, 0), 0, 1),  # one-piece block: inboard face down, all bores vertical
    "magnet_cup": ((0, 1, 0), 0, 1),  # magnet pocket up
    "wheel": ((0, 1, 0), 0, 1),  # end web down, the gear's seat (or the printed gear) on top
    "impeller": ((0, 0, 1), 45, 1),  # hub side towards the plate, tilted (the shroud's inside supportable)
    "fan_mount": ((0, 0, -1), 0, 1),  # collar up
    "sensor_cap_test": ((1, 0, 0), 0, 1),  # calibration caps (W1's shape)
    "sensor_cap": ("look", 0, 1),  # front face down
    "gear_": ((0, 0, -1), 0, 2),  # axis vertical (gears.py's frame: outer face up)
    "drill_guide": ((0, 0, -1), 0, 1),  # base: pocket up; lid: flat, its holes vertical
    "basket": ((0, 0, 1), 0, 1),  # upside down
    "fit_test": ((0, 0, -1), 0, 1),
    "fit_": ((0, 0, -1), 0, 1),  # engraved top up, supports under the bottom
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
    "sensor_cap": "front", "skid": "front",
    "encoder_": "encoders",
    "screw_": "fasteners", "nut_": "fasteners", "insert_": "fasteners",
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
    from . import fasteners
    return {**reference(with_board=False), "fan_motor": fan.fan_motor(), **frame.straps(), **skids.parts(),
            **fasteners.parts()}
