"""Print interferences in the reference layout, including against every board component.

Board components are checked as their bounding boxes (conservative), from the ref/board_boxes.json cache.

Usage: uv run tools/check_layout.py [layout.blocks=solid]
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from build123d import Align, Box, Cylinder, Plane, Pos, Rot, extrude, mirror  # noqa: E402
from micras import checks, drive, fan, fasteners, frame, front, layout, skids  # noqa: E402
from micras.params import P, override  # noqa: E402

for a in sys.argv[1:]:
    override(a)
t = time.time()
parts = layout.reference(with_board=False)
printed = drive.all_parts()
parts.update(printed)
fan_parts = fan.parts()
parts.update(fan_parts)
front_parts = {**front.parts(), **frame.parts(), **skids.parts()}
parts.update(frame.straps())
parts.update(front_parts)
fastener_parts = fasteners.parts()
parts.update(fastener_parts)
fastener_hosts = fasteners.hosts()
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
                 ("block_cap", "bearing_inner"), ("block_cap", "bearing_outer"),
                 # one-piece blocks (Layout.blocks = "solid")
                 ("block", "bearing_inner"), ("block", "bearing_outer"),
                 # the motors sit in the blocks
                 ("motor", "block_base"), ("motor", "block_cap"), ("motor", "block"),
                 # running gaps set by params (stack.lip_gap)
                 ("wheel_gear", "block_base"), ("wheel_gear", "block_cap"), ("wheel_gear", "block"),
                 ("wheel", "block_base"), ("wheel", "block_cap"), ("wheel", "block"),
                 # assembled on the axle
                 ("magnet_cup", "magnet"), ("magnet_cup", "axle"), ("magnet_cup", "bearing_inner"),
                 ("wheel", "axle"), ("wheel", "bearing_outer"), ("wheel", "wheel_gear"), ("wheel", "tire"),
                 ("wheel", "pinion"),  # the wheel's teeth reach into the pinion's tip circle (check_gears.py meshes them)
                 ("magnet", "axle"),
                 # running gap set by params (stack.holder_gap)
                 ("magnet_cup", "block_base"), ("magnet_cup", "block_cap"), ("magnet_cup", "block")]:
        allowed.add(frozenset((f"{a}_{s}", f"{b}_{s}")))
allowed |= {frozenset(("impeller", "fan_motor")), frozenset(("fan_mount", "fan_motor")),
            frozenset(("fan_mount", "block_cap_L")), frozenset(("fan_mount", "block_cap_R")),  # arm tabs on the ears
            frozenset(("basket", "block_cap_L")), frozenset(("basket", "block_cap_R")),  # posts on the bosses
            frozenset(("fan_mount", "block_L")), frozenset(("fan_mount", "block_R")),
            frozenset(("basket", "block_L")), frozenset(("basket", "block_R"))}
# the fasteners sit in their holes, traps and inserts (touching them: they mustn't overlap them, checked below)
allowed |= {frozenset((k, h)) for k, hs in fastener_hosts.items() for h in hs}
# the cells rest on the box floor ribs (touching); real overlaps are caught below
allowed |= {frozenset(("basket", f"cell{i}")) for i in range(3)}
# the straps lie on the cells and pass through the basket's windows
allowed |= {frozenset((f"velcro_{j}", k)) for j in range(2) for k in ("basket", "cell0", "cell1", "cell2")}
# screwed / seated joints
# the fan mount's front foot rests on the MCU
allowed |= {frozenset(("fan_mount", k)) for k in others if k.startswith("brd:STM32")}
# the caps wrap the LEDs; the board model only offers the sensor bounding box here (exact check below)
# (check_sensors.py checks each cap against every sensor's LEDs exactly)
allowed |= {frozenset((f"sensor_cap_{w}", k)) for w in front.SENSORS for k in others if k.startswith("brd:WALL_SENSOR")}
# the USB receptacle's box reaches down into the PCB; the rear skid is glued under the PCB, between its shell tabs
allowed |= {frozenset(("skid_rear", k)) for k in others if k.startswith("brd:USB4105")}

res = checks.clashes(parts, others, allowed, margin=P.layout.clearance)
for r in res:
    print("CLASH" if r[3] > 0 else "close", r)

# board contact: printed material within 0.3 mm of the board top must lie inside the contact zones
zone = Pos(0, 0, P.board.top_z) * extrude(drive.contact_zone(P, 0.0), 0.3)
zones = zone + mirror(zone, Plane.XZ)
# the sensor caps stand on the casing outlines the sensor footprints draw
for w in front.SENSORS:
    zones += front.outline(w)
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
for a, b in [("wheel_L", "tire_L"), ("wheel_R", "tire_R")]:  # the tire sits on the hub, not in it
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
# parts seated in the blocks, and the rotating parts with a running gap to them, may touch them (allowed above) but
# must not sink into them: the bearings, the motors, the magnet cups (the alternative Ø6 cup too: it isn't in the
# assembly) and the wheels
alt_cups = {c.label: c for c in (drive.magnet_cup(s, alt=True) for s in (1, -1))}
for sd in "LR":
    blocks = [k for k in parts if k.startswith("block") and k.endswith(f"_{sd}")]
    for k in (f"bearing_inner_{sd}", f"bearing_outer_{sd}", f"motor_{sd}", f"magnet_cup_{sd}", f"magnet_cup6_{sd}",
              f"wheel_{sd}"):
        part = parts.get(k) or alt_cups.get(k)
        if part is None:
            continue
        for b in blocks:
            common = part & parts[b]
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
# the fasteners must not sink into the parts they sit in (a screw too long for its hole, a trap too small)
for k, hs in fastener_hosts.items():
    for h in hs:
        if h in parts:
            common = parts[k] & parts[h]
            v = common.volume if common is not None else 0.0
            if v > 1e-3:
                res.append((k, h, 0.0, round(v, 3)))
                print("SEAT ", f"{k} sinks into {h} by {v:.3f} mm3")
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
# pitch: the body tips about the tires' contact line (x = 0) until a skate touches the floor; nothing else may come
# within FLOOR_GAP of the floor (the tires' sink lowers everything alike). The skirt's margin brushes the floor by design; the THT leads under the board are trimmed flush.
FLOOR_GAP = 0.05
low = [(f"board edge ({v.X:.1f}, {v.Y:.1f})", v.X, P.board.bottom_z) for v in layout.pcb_face().vertices()]
for name, part in parts.items():
    if name.startswith(("tire", "skid")) or part.bounding_box().min.Z > 3.0:
        continue
    low += [(name, v.X, v.Z) for v in part.tessellate(0.02)[0] if v.Z < 3.0]
z_c = skids.contact_z()
for side, (x_c, _) in (("front", skids.SK.front), ("rear", skids.SK.rear)):
    name, x, z = min(((n, x, z) for n, x, z in low if x * x_c > 0), key=lambda q: q[2] - z_c * q[1] / x_c)
    g = z - z_c * x / x_c
    print("FLOOR", f"tipped onto the {side} skid (contact {z_c:.2f}): lowest other point {name} at x {x:.1f}, "
                   f"{g:.3f} above the floor")
    if g < FLOOR_GAP:
        res.append((name, f"floor, tipped onto the {side} skid", 0.0, round(g, 3)))
print(f"{len(res)} issues | parts {t_parts - t:.1f}s, board {t_board - t_parts:.1f}s, "
      f"checks {time.time() - t_board:.1f}s")
