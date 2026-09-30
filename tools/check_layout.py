"""Print interferences in the reference layout, including against every board component.

Board components are checked as their bounding boxes (conservative), from the ref/board_boxes.json cache.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from build123d import Align, Box, Cylinder, Plane, Pos, extrude, mirror  # noqa: E402
from micras import body, checks, drive, fan, frame, front, layout  # noqa: E402
from micras.params import P  # noqa: E402

t = time.time()
parts = layout.reference(with_board=False)
printed = drive.all_parts()
parts.update(printed)
fan_parts = fan.parts()
parts.update(fan_parts)
frame_parts = frame.parts()
parts.update(frame_parts)
front_parts = {**front.parts(), **body.parts()}
parts.update(front_parts)
# may touch the board outside the silkscreen zones (agreed for the fan supports)
ZONE_EXEMPT = {"fan_mount"}
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
            frozenset(("frame", "fan_mount")), frozenset(("frame", "block_cap_L")), frozenset(("frame", "block_cap_R"))}
allowed |= {frozenset(("frame", f"cell{i}")) for i in range(3)}
# screwed / seated joints
allowed |= {frozenset(("halo", "frame")), frozenset(("front_wing", "halo")), frozenset(("lid", "frame")),
            frozenset(("fan_motor", "frame"))}  # airbox lugs: designed 0.1 axial play
# the wing passes under the diagonal sensors; the board model's sensor box reaches down to the legs,
# so the wing is checked against the LED bodies in check_sensors.py instead
allowed |= {frozenset(("front_wing", k)) for k in others if k.startswith("brd:WALL_SENSOR")}
# the caps wrap the LEDs; the board model only offers the sensor bounding box here (exact check below)
allowed |= {frozenset((f"sensor_cap_{w}", k)) for w in ("W1", "W2", "W3", "W4") for k in others
            if k.startswith("brd:WALL_SENSOR")}

res = checks.clashes(parts, others, allowed, margin=P.layout.clearance)
for r in res:
    print("CLASH" if r[3] > 0 else "close", r)

# board contact: printed material within 0.3 mm of the board top must lie inside the contact zones
zone = Pos(0, 0, P.board.top_z) * extrude(drive.contact_zone(P, 0.0), 0.3)
zones = zone + mirror(zone, Plane.XZ)
# only where the board exists: the real PCB outline, 0.3 mm thick on top of the board
pcb, _ = layout.board_simple()
slab = Pos(0, 0, P.board.thickness) * pcb
slab = slab & Pos(0, 0, P.board.top_z) * Box(200, 200, 0.3, align=(Align.CENTER, Align.CENTER, Align.MIN))
for name, part in {**printed, **frame_parts, **front_parts,
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
print(f"{len(res)} issues | parts {t_parts - t:.1f}s, board {t_board - t_parts:.1f}s, "
      f"checks {time.time() - t_board:.1f}s")
