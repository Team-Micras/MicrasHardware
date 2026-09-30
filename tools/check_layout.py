"""Print interferences in the reference layout, including against every board component.

Board components are checked as their bounding boxes (conservative), from the ref/board_boxes.json cache.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from build123d import Align, Box, Cylinder, Plane, Pos, Rot, extrude, mirror  # noqa: E402
from micras import body, checks, drive, fan, front, layout  # noqa: E402
from micras.params import P  # noqa: E402

t = time.time()
parts = layout.reference(with_board=False)
printed = drive.all_parts()
parts.update(printed)
fan_parts = fan.parts()
parts.update(fan_parts)
front_parts = {**front.parts(), **body.parts()}
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
                 # fixed-bore variant (Layout.backlash_mode = "fixed"): the motor sits in the blocks
                 *((("motor", "block_base"), ("motor", "block_cap")) if P.layout.backlash_mode == "fixed" else ()),
                 # running gaps set by params (stack.lip_gap)
                 ("wheel_gear", "block_base"), ("wheel_gear", "block_cap"),
                 # assembled on the axle
                 ("magnet_cup", "magnet"), ("magnet_cup", "axle"), ("magnet_cup", "bearing_inner"),
                 ("race_spacer", "axle"), ("race_spacer", "bearing_outer"), ("race_spacer", "wheel_gear"),
                 ("wheel_hub", "axle"), ("wheel_hub", "wheel_gear"), ("wheel_hub", "tire"),
                 ("magnet", "axle"),
                 # running gap set by params (stack.holder_gap)
                 ("magnet_cup", "block_base"), ("magnet_cup", "block_cap")]:
        allowed.add(frozenset((f"{a}_{s}", f"{b}_{s}")))
allowed |= {frozenset(("impeller", "fan_motor")), frozenset(("fan_mount", "fan_motor")),
            frozenset(("body", "fan_mount")), frozenset(("body", "block_cap_L")), frozenset(("body", "block_cap_R"))}
# the cells rest on the box floor ribs (touching); real overlaps are caught below
allowed |= {frozenset(("body", f"cell{i}")) for i in range(3)}
# screwed / seated joints
allowed |= {frozenset(("lid", "body"))}
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
for i in range(3):
    common = parts[f"cell{i}"] & parts["body"]
    v = common.volume if common is not None else 0.0
    if v > 1e-3:
        res.append((f"cell{i}", "body", 0.0, round(v, 3)))
        print("CELLS", f"cell{i} overlaps the battery box by {v:.3f} mm3")
print(f"{len(res)} issues | parts {t_parts - t:.1f}s, board {t_board - t_parts:.1f}s, "
      f"checks {time.time() - t_board:.1f}s")
