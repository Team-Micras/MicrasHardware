"""Send the current model to the OCP CAD Viewer (VS Code: open the viewer panel first).

Usage: uv run tools/show.py
"""
import sys
from pathlib import Path

sys.path.insert(0, Path(__file__).resolve().parents[1].as_posix())
from ocp_vscode import show  # noqa: E402

from micras import drive, layout  # noqa: E402

pcb, comps = layout.board_simple()
parts = {**layout.reference(with_board=False), **drive.all_parts()}
show(pcb, comps, *parts.values(), names=["pcb", "components", *parts.keys()])
