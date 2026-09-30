"""Every designed part, with its material and print notes, for rendering, viewing and export."""

from . import drive, fan, frame, front

# part-name prefix -> (material, printer, orientation / notes)
MATERIALS = {
    "block_base": ("resin", "Photon Mono 4", "flat pad face down on the plate, supports on the rounded top only; press + glue the M2 inserts"),
    "block_cap": ("resin", "Photon Mono 4", "split face up (bores open upward), supports on the outside; glue inserts"),
    "sleeve": ("resin", "Photon Mono 4", "axis vertical, notched rim up; ream the motor bore if tight"),
    "magnet_cup": ("resin", "Photon Mono 4", "axis vertical, magnet pocket up; glue the magnet with the correct pole direction"),
    "race_spacer": ("resin", "Photon Mono 4", "axis vertical; tiny, print several"),
    "wheel_hub": ("resin", "Photon Mono 4", "axis vertical, web down; glue to the brass gear face"),
    "impeller": ("resin", "Photon Mono 4", "axis vertical, shroud (eye) down; ream the bore to 0.97-0.98, balance"),
    "fan_mount": ("resin", "Photon Mono 4", "collar up, supports under the feet"),
    "sensor_cap": ("resin", "Photon Mono 4", "apertures up (axis vertical)"),
    "frame": ("petg", "Ender 3 V3 SE", "tray walls down on the bed, tree supports under the floor; heat-set the nose insert"),
    "nose": ("petg", "Ender 3 V3 SE", "on its side (arm web flat), skid edge smooth"),
    "strap": ("tpu", "Ender 3 V3 SE", "print strap_print.stl flat (loop lying on the bed)"),
}


def material(name):
    for prefix, info in MATERIALS.items():
        if name.startswith(prefix):
            return info
    raise KeyError(name)


def printed():
    fan_parts = {k: v for k, v in fan.parts().items() if k != "fan_motor"}
    return {**drive.all_parts(), **fan_parts, **frame.parts(), **front.parts()}


def bought():
    from .layout import reference
    return {**reference(with_board=False), "fan_motor": fan.fan_motor()}
