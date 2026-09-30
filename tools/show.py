"""Send the current model to the OCP CAD Viewer (VS Code: open the viewer panel first).

Usage: uv run tools/show.py [--simple] [layout.backlash_mode=fixed ...]
  --simple  show the board's components as boxes (faster)
"""
import sys
from pathlib import Path

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
import json  # noqa: E402
import os  # noqa: E402
import socket  # noqa: E402


def viewer_running():
    """True if an OCP CAD Viewer is reachable (same discovery as ocp_vscode: OCP_PORT, ~/.ocpvscode, 3939)."""
    ports = []
    if os.environ.get("OCP_PORT"):
        ports.append(int(os.environ["OCP_PORT"]))
    state = Path.home() / ".ocpvscode"
    if state.exists():
        try:
            ports += [int(p) for p in json.loads(state.read_text()).get("services", {})]
        except (ValueError, OSError):
            pass
    ports.append(3939)
    for port in ports:
        with socket.socket() as s:
            s.settimeout(0.3)
            if s.connect_ex(("127.0.0.1", port)) == 0:
                return True
    return False


if not viewer_running():
    sys.exit("No OCP CAD Viewer is running. In VS Code (connected to WSL, with this folder open): "
             "select .venv/bin/python as the interpreter, then run 'OCP CAD Viewer: Open Viewer' "
             "(Ctrl+K V) and rerun this script.")

from ocp_vscode import show  # noqa: E402

from micras import assembly, layout, params  # noqa: E402

from build123d import Compound  # noqa: E402

# optional parameter overrides, e.g. layout.backlash_mode=fixed
for kv in sys.argv[1:]:
    if not kv.startswith("--"):
        params.override(kv)
if "--simple" in sys.argv:  # components as boxes (faster)
    pcb, comps = layout.board_simple()
    board = Compound(children=[pcb, comps], label="board")
else:  # the real populated board from ref/board.step, every component as modelled (~0.6 GB, ~25 s)
    board = layout.board()
# one group per assembly (drive left/right, fan, body, front, battery), like the board's components
grouped = []
for group, parts in assembly.groups().items():
    for name, shape in parts.items():
        shape.label = name
    grouped.append(Compound(children=list(parts.values()), label=group))
show(board, *grouped, names=["board", *[g.label for g in grouped]])
