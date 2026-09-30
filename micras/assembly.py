"""Every designed part, for rendering, viewing and export."""

from . import drive, fan, frame, front


def printed():
    return {**drive.all_parts(), **fan.parts(), **frame.parts(), **front.parts()}
