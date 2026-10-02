"""Print interferences in the reference layout, including against every board component.

Board components are checked as their bounding boxes (conservative), from the ref/board_boxes.json cache.

Usage: uv run tools/check_layout.py [layout.backlash_mode=fixed] [layout.blocks=solid]
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from build123d import Align, Box, Cylinder, Plane, Pos, Rot, extrude, mirror  # noqa: E402
from micras import checks, drive, fan, frame, front, layout  # noqa: E402
from micras.params import P, override  # noqa: E402

for a in sys.argv[1:]:
    override(a)
t = time.time()
parts = layout.reference(with_board=False)
printed = drive.all_parts()
parts.update(printed)
fan_parts = fan.parts()
parts.update(fan_parts)
front_parts = {**front.parts(), **frame.parts()}
parts.update(frame.straps())
parts.update(front_parts)
# every printed part obeys the zone rule; the agreed extra contact areas are added as zones below
ZONE_EXEMPT = set()
t_parts = time.time()
others = layout.board_keepouts()
# the encoder daughterboards are fixed hardware, like the board
others.update({k: parts.pop(k) for k in list(parts) if k.startswith("encoder_")})
t_board = time.time()

# fits that are meant to touch
allowed = set()
for s in "LR":
    for a, b in [("axle", "bearing_inner"), ("axle", "bearing_outer"), ("axle", "wheel_gear"),
                 ("bearing_inner", "bearing_outer"), ("axle", "tire"), ("wheel_gear", "tire"),
                 ("encoder_pcb", "encoder_chip"), ("wheel_gear", "pinion"), ("motor", "pinion")]:
        allowed.add(frozenset((f"{a}_{s}", f"{b}_{s}")))
    # parts that seat in each other by design
    for a, b in [("block_base", "block_cap"), ("block_base", "bearing_inner"), ("block_base", "bearing_outer"),
                 ("block_cap", "bearing_inner"), ("block_cap", "bearing_outer"), ("sleeve", "block_base"),
                 ("sleeve", "block_cap"), ("sleeve", "motor"),
                 # one-piece blocks (Layout.blocks = "solid")
                 ("block", "bearing_inner"), ("block", "bearing_outer"), ("sleeve", "block"),
                 # fixed-bore variant (Layout.backlash_mode = "fixed"): the motor sits in the blocks
                 *((("motor", "block_base"), ("motor", "block_cap"), ("motor", "block"))
                   if P.layout.backlash_mode == "fixed" else ()),
                 # running gaps set by params (stack.lip_gap)
                 ("wheel_gear", "block_base"), ("wheel_gear", "block_cap"), ("wheel_gear", "block"),
                 # assembled on the axle
                 ("magnet_cup", "magnet"), ("magnet_cup", "axle"), ("magnet_cup", "bearing_inner"),
                 ("race_spacer", "axle"), ("race_spacer", "bearing_outer"), ("race_spacer", "wheel_gear"),
                 ("wheel_hub", "axle"), ("wheel_hub", "wheel_gear"), ("wheel_hub", "tire"),
                 ("magnet", "axle"),
                 # running gap set by params (stack.holder_gap)
                 ("magnet_cup", "block_base"), ("magnet_cup", "block_cap"), ("magnet_cup", "block")]:
        allowed.add(frozenset((f"{a}_{s}", f"{b}_{s}")))
allowed |= {frozenset(("impeller", "fan_motor")), frozenset(("fan_mount", "fan_motor")),
            frozenset(("fan_mount", "block_cap_L")), frozenset(("fan_mount", "block_cap_R")),  # arm tabs on the ears
            frozenset(("basket", "block_cap_L")), frozenset(("basket", "block_cap_R")),  # posts on the bosses
            frozenset(("fan_mount", "block_L")), frozenset(("fan_mount", "block_R")),
            frozenset(("basket", "block_L")), frozenset(("basket", "block_R"))}
# the cells rest on the box floor ribs (touching); real overlaps are caught below
allowed |= {frozenset(("basket", f"cell{i}")) for i in range(3)}
# the straps lie on the cells and pass through the basket's windows
allowed |= {frozenset((f"velcro_{j}", k)) for j in range(2) for k in ("basket", "cell0", "cell1", "cell2")}
# screwed / seated joints
# the fan mount's front foot rests on the MCU
allowed |= {frozenset(("fan_mount", k)) for k in others if k.startswith("brd:STM32")}
# the bumper passes under the diagonal sensors; the board model's sensor box reaches down to the legs,
# so the bumper is checked against the LED bodies in check_sensors.py instead
allowed |= {frozenset(("bumper", k)) for k in others if k.startswith("brd:WALL_SENSOR")}
# the caps wrap the LEDs; the board model only offers the sensor bounding box here (exact check below)
# (check_sensors.py checks each cap against every sensor's LEDs exactly)
allowed |= {frozenset((f"sensor_cap_{w}", k)) for w in front.SENSORS for k in others if k.startswith("brd:WALL_SENSOR")}

res = checks.clashes(parts, others, allowed, margin=P.layout.clearance)
for r in res:
    print("CLASH" if r[3] > 0 else "close", r)

# board contact: printed material within 0.3 mm of the board top must lie inside the contact zones
zone = Pos(0, 0, P.board.top_z) * extrude(drive.contact_zone(P, 0.0), 0.3)
zones = zone + mirror(zone, Plane.XZ)
# the sensor caps stand on the casing outlines the sensor footprints draw
for w in front.SENSORS:
    zones += front.outline(w)
# the bumper's lip rests on the free strip behind the board's front edge
# (the strip is taken from the board: from 0.2 past the last component to the straight front edge)
x_edge = max(v.X for v in layout.pcb_face().vertices())
edge_y = max(abs(v.Y) for v in layout.pcb_face().vertices() if abs(v.X - x_edge) < 1e-6)
x_free = max(b["max"][0] for b in layout.board_boxes()
             if not b["label"].startswith("WALL_SENSOR") and abs(b["min"][1] + b["max"][1]) / 2 < edge_y) + 0.2
zones += Pos((x_free + x_edge) / 2, 0, P.board.top_z) * Box(x_edge - x_free, 2 * (edge_y - 1.0), 0.3,
                                                             align=(Align.CENTER, Align.CENTER, Align.MIN))
# the fan mount's feet, on the free board spots chosen for them
fcx, fcy = fan.centre(P)
for ang, r, z_foot in fan.F.feet:
    if z_foot > 0:  # rests on a part, not on the board
        continue
    zones += Pos(fcx, fcy, P.board.top_z) * Rot(0, 0, ang) * Pos(r, 0, 0) * Box(
        fan.F.foot_d + 0.02, fan.F.arm_w + 0.02, 0.3, align=(Align.CENTER, Align.CENTER, Align.MIN))
# only where the board exists: the real PCB outline, 0.3 mm thick on top of the board
pcb, _ = layout.board_simple()
slab = Pos(0, 0, P.board.thickness) * pcb
slab = slab & Pos(0, 0, P.board.top_z) * Box(200, 200, 0.3, align=(Align.CENTER, Align.CENTER, Align.MIN))
for name, part in {**printed, **front_parts,
                   **{k: v for k, v in fan_parts.items() if k != "fan_motor"}}.items():
    if name in ZONE_EXEMPT:
        continue
    near = part & slab
    if near is None or near.volume < 1e-6:
        continue
    outside = near - zones
    v = outside.volume if outside is not None else 0.0
    if v > 1e-3:
        res.append((name, "outside contact zone", 0.0, round(v, 3)))
        print("ZONE ", name, f"{v:.3f} mm3 near the board top outside the contact zone")
# the cells must fit their box (their contact with the floor ribs is allowed above)
for a, b in [("wheel_hub_L", "tire_L"), ("wheel_hub_R", "tire_R")]:  # the tire sits on the hub, not in it
    common = parts[a] & parts[b]
    v = common.volume if common is not None else 0.0
    if v > 1e-3:
        res.append((a, b, 0.0, round(v, 3)))
        print("FIT  ", f"{a} overlaps {b} by {v:.3f} mm3")
# every printed part must be one valid solid (a broken boolean can leave an invalid shape that still renders)
for name, part in {**printed, **front_parts, **{k: v for k, v in fan_parts.items() if k != "fan_motor"}}.items():
    if not part.is_valid or len(part.solids()) != 1:
        res.append((name, "invalid shape", 0.0, len(part.solids())))
        print("SHAPE", f"{name}: valid {part.is_valid}, {len(part.solids())} solids")
# parts seated in the blocks touch them by design (allowed above) but must not sink into them: the bearings, the
# sleeves, and the motors in the fixed-bore variant
for sd in "LR":
    blocks = [k for k in parts if k.startswith("block") and k.endswith(f"_{sd}")]
    for k in (f"bearing_inner_{sd}", f"bearing_outer_{sd}", f"sleeve_{sd}", f"motor_{sd}"):
        if k not in parts:
            continue
        for b in blocks:
            common = parts[k] & parts[b]
            v = common.volume if common is not None else 0.0
            if v > 1e-3:
                res.append((k, b, 0.0, round(v, 3)))
                print("SEAT ", f"{k} sinks into {b} by {v:.3f} mm3")
# the blocks reach in to the motor seats' inboard end (a union that silently drops the seat ring shows up here)
for sd in "LR":
    for k in [k for k in parts if k.startswith("block") and k.endswith(f"_{sd}") and "base" not in k]:
        bb = parts[k].bounding_box()
        inner = bb.min.Y if sd == "L" else -bb.max.Y
        if inner > drive.D.seat_min_y + 0.01:
            res.append((k, "seat ring missing", 0.0, round(inner, 2)))
            print("RING ", f"{k} starts at |y| {inner:.2f}, not at the seat's {drive.D.seat_min_y}")
# base and cap of each block touch at the split by design (allowed above) but must not overlap
for sd in "LR" if P.layout.blocks == "split" else "":
    common = parts[f"block_base_{sd}"] & parts[f"block_cap_{sd}"]
    v = common.volume if common is not None else 0.0
    if v > 1e-3:
        res.append((f"block_base_{sd}", f"block_cap_{sd}", 0.0, round(v, 3)))
        print("SPLIT", f"block_cap_{sd} overlaps its base by {v:.3f} mm3")
for i in range(3):
    common = parts[f"cell{i}"] & parts["basket"]
    v = common.volume if common is not None else 0.0
    if v > 1e-3:
        res.append((f"cell{i}", "basket", 0.0, round(v, 3)))
        print("CELLS", f"cell{i} overlaps the battery box by {v:.3f} mm3")
print(f"{len(res)} issues | parts {t_parts - t:.1f}s, board {t_board - t_parts:.1f}s, "
      f"checks {time.time() - t_board:.1f}s")
