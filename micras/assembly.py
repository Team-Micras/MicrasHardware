"""Every designed part, with its material and print notes, for rendering, viewing and export."""

from . import body, drive, fan, front

# part-name prefix -> (material, printer, orientation / notes)
MATERIALS = {
    "block_base": ("resin", "Photon Mono 4", "flat pad face down on the plate; light supports under the lifted body (about 150 mm2, 1.8-3.2 mm tall) and the rounded top; clear the insert holes of burn-in, glue the M2 inserts"),
    "block_cap": ("resin", "Photon Mono 4", "split face up (bores open upward), supports on the outside; glue inserts"),
    "sleeve": ("resin", "Photon Mono 4", "axis vertical, notched rim up; ream the motor bore if tight"),
    "magnet_cup": ("resin", "Photon Mono 4", "axis vertical, magnet pocket up; glue the magnet with the correct pole direction"),
    "race_spacer": ("resin", "Photon Mono 4", "axis vertical; tiny, print several"),
    "wheel_hub": ("resin", "Photon Mono 4", "axis vertical, web down; glue to the brass gear face"),
    "impeller": ("resin", "Photon Mono 4", "tilted 30-45 deg, hub side towards the plate, supports on the backplate and hub only (the flat shroud face is the inlet seal: keep it support-free); ream the bore to 0.97-0.98, balance"),
    "fan_mount": ("resin", "Photon Mono 4", "collar up, supports under the feet and the flat underside of the plate; the collet fingers must flex, do not over-cure"),
    "sensor_cap": ("resin", "Photon Mono 4", "BLACK resin (IR-opaque); front face down (bores vertical, fork up), no supports"),
    "bumper": ("tpu", "Ender 3 V3 SE", "flat on its bottom, no supports; 100 % infill"),
    "body": ("petg", "Ender 3 V3 SE", "lying on the nose plane (the visible face comes out smooth); tree supports under the battery-box wall tops (sand the lid seat flat after), the floor and the lid bosses; heat-set the two lid inserts"),
    "lid": ("petg", "Ender 3 V3 SE", "cover on the bed; the rear wing bridges between fin and endplates, no supports"),
}


def material(name):
    for prefix, info in MATERIALS.items():
        if name.startswith(prefix):
            return info
    raise KeyError(name)


def printed():
    fan_parts = {k: v for k, v in fan.parts().items() if k != "fan_motor"}
    return {**drive.all_parts(), **fan_parts, **front.parts(), **body.parts()}


GROUPS = {  # viewer groups: part-name prefix -> group (checked in order; per-side parts get _L/_R groups)
    "cell": "battery",
    "body": "body", "lid": "body",
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
    return {**reference(with_board=False), "fan_motor": fan.fan_motor()}
