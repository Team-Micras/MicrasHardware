"""Mass, centre of mass and yaw inertia of the current design; suggests battery.x for CoM over the axle."""
import sys
from pathlib import Path

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from micras import assembly, mass  # noqa: E402
from micras.params import P  # noqa: E402

printed = assembly.printed()
items = mass.fixed_items(P, printed, placeholders=False, material=lambda n: assembly.material(n)[0])
bat = mass.battery_items(P.battery.arrangement, P.battery.x, P.battery.floor_z, P)
s = mass.summarize(items + bat)
print(f"total {s['mass']:.1f} g | CoM x={s['com'][0]:+.2f} y={s['com'][1]:+.2f} z={s['com'][2]:.1f} mm | "
      f"Izz {s['izz']:.0f} g*mm^2 about the axle midpoint")
# battery x that puts the CoM over the axle
m_b = sum(i.mass for i in bat)
rest = mass.summarize(items)
x_needed = -rest["com"][0] * rest["mass"] / m_b - (sum(i.com[0] * i.mass for i in bat) / m_b - P.battery.x)
print(f"battery.x for CoM over the axle: {x_needed:.2f} (now {P.battery.x})")
print("\nlargest yaw-inertia contributions:")
contrib = sorted(((i.izz + i.mass * (i.com[0] ** 2 + i.com[1] ** 2), i.name, i.mass) for i in items + bat), reverse=True)
for izz, name, m in contrib[:10]:
    print(f"  {name:16s} {m:5.2f} g  {izz:7.0f}")
