"""Design-rule regression tests: run the check tools and require a clean result."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run(tool):
    out = subprocess.run([sys.executable, str(ROOT / "tools" / tool)], cwd=ROOT, capture_output=True,
                         text=True, timeout=600)
    assert out.returncode == 0, out.stderr
    return out.stdout


def test_no_clashes_and_board_contact_only_in_zones():
    out = run("check_layout.py")
    assert out.strip().splitlines()[-1].startswith("0 issues"), out


def test_sensor_caps_clear_the_leds():
    out = run("check_sensors.py")
    assert out.strip().splitlines()[-1] == "OK", out


def test_centre_of_mass_over_the_axle():
    out = run("mass_report.py")
    first = out.splitlines()[0]
    com_x = float(first.split("CoM x=")[1].split()[0])
    assert abs(com_x) < 0.5, first
