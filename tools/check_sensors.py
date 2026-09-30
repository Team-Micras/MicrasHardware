"""Exact check of the sensor caps against the LED bodies in the KiCad sensor models."""
import sys
from pathlib import Path

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from build123d import Pos, Rot, import_step  # noqa: E402

from micras import body, front, layout  # noqa: E402

raw = import_step(layout.REF / "board.step")
to_robot = Pos(-67.1112, 64.1608, layout.P.board.bottom_z) * Rot(0, 0, -90)
sensors = [c for c in list(raw.children) if c.label.startswith("WALL_SENSOR")]
for c in sensors:
    c.parent = None  # else moving deep-copies the whole board with each sensor
sensors = [to_robot * c for c in sensors]
# ribs built just short of the LEDs: anything else touching an LED or its legs shows as an overlap
caps = {**front.parts(fp=front.FrontParams(rib_interf=-0.01)), "bumper": front.bumper()}
bad = 0
for name, cap in caps.items():
    if name == "bumper":
        for sensor in sensors:
            for led in sorted(sensor.solids(), key=lambda s: s.volume)[:2]:
                if cap.distance_to(led) < 0.3:
                    print(f"bumper too close to an LED: {cap.distance_to(led):.3f}")
                    bad += 1
        continue
    # the sensor whose bounding box overlaps this cap the most
    cb = cap.bounding_box()
    sensor = max(sensors, key=lambda s: -abs(s.bounding_box().center().X - cb.center().X)
                 - abs(s.bounding_box().center().Y - cb.center().Y))
    leds = sorted(sensor.solids(), key=lambda s: s.volume)[:2]  # the two LEDs, with their legs
    for led in leds:
        common = cap & led
        v = common.volume if common is not None else 0.0
        d = cap.distance_to(led)
        print(f"{name}: LED z{led.bounding_box().max.Z:5.1f} overlap {v:.3f} mm3, gap {d:.3f} mm")
        bad += v > 1e-3
print("OK" if not bad else f"{bad} overlaps")
