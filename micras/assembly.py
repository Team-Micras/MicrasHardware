"""Every designed part, with its material and print notes, for rendering, viewing and export."""

from . import drive, fan, frame, front

# part-name prefix -> (material, printer, orientation / notes)
MATERIALS = {
    "block_base": ("resin", "Photon Mono 4", "flat pad face down on the plate; light supports under the lifted body (about 150 mm2, 1.8-3.2 mm tall) and the rounded top; clear the insert holes of burn-in, glue the M2 inserts"),
    "block_cap": ("resin", "Photon Mono 4", "split face up (bores open upward), supports on the outside; glue inserts; an M2 nut goes in the fan ear's trap before the cap is fitted"),
    "sleeve": ("resin", "Photon Mono 4", "axis vertical, notched rim up; ream the motor bore if tight"),
    "magnet_cup": ("resin", "Photon Mono 4", "axis vertical, magnet pocket up; glue the magnet with the correct pole direction"),
    "race_spacer": ("resin", "Photon Mono 4", "axis vertical; tiny, print several"),
    "wheel_hub": ("resin", "Photon Mono 4", "axis vertical, web down; glue to the brass gear face"),
    "impeller": ("resin", "Photon Mono 4", "tilted 30-45 deg, hub side towards the plate, supports on the backplate and hub only (the flat shroud face is the inlet seal: keep it support-free); ream the bore to 0.97-0.98, balance"),
    "fan_mount": ("resin", "Photon Mono 4", "collar up, supports under the foot, the arms' undersides and the plate; put an M2 nut in the clamp ear's trap"),
    "sensor_cap": ("resin", "Photon Mono 4", "BLACK resin (IR-opaque); front face down (bores vertical, fork up), no supports"),
    "bumper": ("tpu", "Ender 3 V3 SE", "upside down, flat top on the bed (the rounded lower edge then needs no supports); 100 % infill"),
    "gear_": ("resin", "Photon Mono 4", "stand-ins for the brass gears, print 2 of each: axis vertical, lifted on supports (on the plate the first layers flare the teeth); tough / ABS-like resin if available; drill the bore (1.0 pinion, 2.0 wheel) and glue; a pair only, don't mix with a brass gear"),
    "basket": ("petg", "Ender 3 V3 SE", "upside down (wall tops on the bed), no supports: the floor and the low side walls bridge ~21 mm between the front and rear walls (bridge settings on), the strap windows 10.6 mm"),
}


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


def bought():
    from .layout import reference
    return {**reference(with_board=False), "fan_motor": fan.fan_motor(), **frame.straps()}
