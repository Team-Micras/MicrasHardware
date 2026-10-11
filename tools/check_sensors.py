"""Exact check of the sensor caps against the LED bodies in the KiCad sensor models."""
import sys
from pathlib import Path

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from build123d import Pos, Rot, import_step  # noqa: E402

from micras import front, layout  # noqa: E402
from micras.leds import EMITTER, EMITTER_FLAT, RECEIVER, RECEIVER_TAB  # noqa: E402

raw = import_step(layout.REF / "board.step")
to_robot = Pos(-67.1112, 64.1608, layout.P.board.bottom_z) * Rot(0, 0, -90)
sensors = [c for c in list(raw.children) if c.label.startswith("WALL_SENSOR")]
for c in sensors:
    c.parent = None  # else moving deep-copies the whole board with each sensor
sensors = [to_robot * c for c in sensors]
# ribs built just short of the LEDs: anything else touching an LED or its legs shows as an overlap
caps = front.parts(fp=front.FrontParams(rib_interf=-0.01))
TILT = front.FP.emitter_tilt


def emitter_tilt(sensor, angle):
    """The emitter's pitch about its flange (FrontParams.emitter_tilt: its legs bend to it), as a robot-frame
    transform for one sensor."""
    from build123d import Location
    (x, y), ang = front.SENSORS[sensor]
    to_robot = Location(Pos(x, y, layout.P.board.top_z) * Rot(0, 0, ang))
    pivot = (sum(EMITTER.flange) / 2, 0, EMITTER.z)
    tilt = Location(Pos(*pivot) * Rot(0, angle, 0) * Pos(-pivot[0], 0, -pivot[2]))
    return to_robot * tilt * to_robot.inverse()


bad = 0
for name, cap in caps.items():
    # the sensor whose bounding box overlaps this cap the most
    cb = cap.bounding_box()
    sensor = max(sensors, key=lambda s: -abs(s.bounding_box().center().X - cb.center().X)
                 - abs(s.bounding_box().center().Y - cb.center().Y))
    leds = sorted(sensor.solids(), key=lambda s: s.volume)[:2]  # the two LEDs, with their legs
    top = max(leds, key=lambda s: s.bounding_box().max.Z)  # the emitter
    for led in leds:
        if led is top and TILT:  # pitched like its bore: compare the bore turned back with the model as drawn
            common = (emitter_tilt(name.rsplit("_", 1)[1], -TILT) * cap) & led
            v = common.volume if common is not None else 0.0
            print(f"{name}: LED z{led.bounding_box().max.Z:5.1f} (pitched {TILT:g} deg) overlap {v:.3f} mm3")
            bad += v > 1e-3
            continue
        common = cap & led
        v = common.volume if common is not None else 0.0
        d = cap.distance_to(led)
        print(f"{name}: LED z{led.bounding_box().max.Z:5.1f} overlap {v:.3f} mm3, gap {d:.3f} mm")
        bad += v > 1e-3
    # and clear of every other sensor's LEDs
    for other in sensors:
        if other is sensor:
            continue
        for led in sorted(other.solids(), key=lambda s: s.volume)[:2]:
            d = cap.distance_to(led)
            if d < 0.3:
                print(f"{name} too close to another sensor's LED: {d:.3f} mm")
                bad += 1

# the cap slides on from the front: in its frame each LED travels forward from far behind into its seat, so the
# space its flange sweeps (behind the flange's front face), its body sweeps (behind its tip) and the receiver's tab
# sweeps must be free of the cap (the ribs are left out: they crush)
from math import atan2, degrees  # noqa: E402

from build123d import Align, Box, Pos, Rot  # noqa: E402

BEHIND = -30.0
for name, cap in caps.items():
    sensor = name.rsplit("_", 1)[1]
    swept = None
    for led in (EMITTER, RECEIVER):
        fl = front._along_u(led.flange_r, BEHIND, led.flange[1], led.z)
        if led is EMITTER:  # the emitter's flange has a flat (cathode side, -v)
            fl -= Pos(0, -EMITTER_FLAT.dist - 5, led.z) * Box(100, 10, 20)
        sw = fl + front._along_u(led.body_r, BEHIND, led.tip, led.z)
        if led is EMITTER and TILT:  # pitched like its bore, it slides along its own axis
            pivot = (sum(EMITTER.flange) / 2, 0, EMITTER.z)
            sw = Pos(*pivot) * Rot(0, TILT, 0) * Pos(-pivot[0], 0, -pivot[2]) * sw
        swept = sw if swept is None else swept + sw
    dv, dz = RECEIVER_TAB.dir
    swept += Pos(BEHIND, 0, RECEIVER.z) * Rot(degrees(atan2(-dv, dz)), 0, 0) * Box(
        RECEIVER.flange[1] - BEHIND, RECEIVER_TAB.w, RECEIVER_TAB.dist, align=(Align.MIN, Align.CENTER, Align.MIN))
    common = cap & front._local(Pos(0, 0, layout.P.board.top_z) * swept, sensor)  # (the cap's z starts on the board top)
    v = common.volume if common is not None else 0.0
    print(f"{name}: slide-on {'clear' if v < 1e-3 else f'BLOCKED, {v:.3f} mm3 in the LEDs path'}")
    bad += v > 1e-3
print("OK" if not bad else f"{bad} problems")
