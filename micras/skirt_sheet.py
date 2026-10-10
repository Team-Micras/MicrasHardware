"""The skirt's cutting sheet: the skirt and the nose doubler at 1:1 with scale references, to trace from a screen.

The owner lays the film on a screen showing the sheet, zoomed until the rulers and the ID-1 card outline match a real
ruler and a real card, and marks the lines through it. Units are millimetres throughout: the SVG's viewBox matches
its width and height in mm, the PDF page is the sheet's size, and the PNG has 10 px per mm.
"""

import math
from xml.sax.saxutils import escape

from build123d import GeomType

from .params import P, Params

W, H = 270.0, 168.0  # the sheet

LINE = 0.3  # traced lines: cut and fold
THIN = 0.15  # ruler ticks, leaders
INK = "#000000"
FOLD = "#1f4fbf"
NOTE = "#555555"
TRIM = "#c4570a"
FONT = "DejaVu Sans, Arial, Helvetica, sans-serif"

SKIRT_AT = (68.5, 59.0)  # sheet position of the robot origin, for the skirt and for the doubler
DOUBLER_AT = (102.7, 59.0)
RULER_H = (25.0, 15.0)  # left end of the horizontal ruler's baseline, 100 mm long, ticks up
RULER_V = (16.0, 25.0)  # top end of the vertical ruler's baseline, 100 mm long, ticks left
CARD = (30.0, 104.0, 85.60, 53.98, 3.18)  # ID-1 (ISO/IEC 7810): x, y, width, height, corner radius
SQUARE = (128.0, 104.0, 50.0)
TEXT_X = 186.0


class Sheet:
    """Drawing primitives in sheet millimetres (y down), written out as SVG and drawn by matplotlib."""

    def __init__(self):
        self.items = []

    def path(self, pts, width=LINE, color=INK, dash=None, fill=None, cls=None):
        self.items.append(("path", [tuple(map(float, q)) for q in pts], width, color, dash, fill, cls))

    def text(self, x, y, s, size=2.6, anchor="start", color=INK, bold=False):
        self.items.append(("text", float(x), float(y), s, size, anchor, color, bold))

    def svg(self):
        out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:g}mm" height="{H:g}mm" viewBox="0 0 {W:g} {H:g}">',
               f'<rect x="0" y="0" width="{W:g}" height="{H:g}" fill="#ffffff"/>']
        for it in self.items:
            if it[0] == "path":
                _, pts, width, color, dash, fill, cls = it
                closed = len(pts) > 2 and math.dist(pts[0], pts[-1]) < 1e-3
                d = "M " + " L ".join(f"{x:.3f} {y:.3f}" for x, y in (pts[:-1] if closed else pts))
                d += " Z" if closed else ""
                attrs = [f'd="{d}"', f'fill="{fill or "none"}"', f'stroke="{color}"', f'stroke-width="{width:g}"',
                         'stroke-linecap="round"', 'stroke-linejoin="round"']
                if dash:
                    attrs.append(f'stroke-dasharray="{dash[0]:g} {dash[1]:g}"')
                if cls:
                    attrs.append(f'class="{cls}"')
                out.append(f'<path {" ".join(attrs)}/>')
            else:
                _, x, y, s, size, anchor, color, bold = it
                weight = ' font-weight="bold"' if bold else ""
                out.append(f'<text x="{x:.3f}" y="{y:.3f}" font-family="{FONT}" font-size="{size:g}" '
                           f'text-anchor="{anchor}" fill="{color}"{weight}>{escape(s)}</text>')
        out.append("</svg>")
        return "\n".join(out) + "\n"

    def figure(self):
        from matplotlib.figure import Figure
        from matplotlib.patches import Polygon

        pt = 72 / 25.4
        fig = Figure(figsize=(W / 25.4, H / 25.4), facecolor="white")
        ax = fig.add_axes((0, 0, 1, 1))
        ax.set_xlim(0, W)
        ax.set_ylim(H, 0)
        ax.axis("off")
        ha = {"start": "left", "middle": "center", "end": "right"}
        for it in self.items:
            if it[0] == "path":
                _, pts, width, color, dash, fill, _ = it
                closed = len(pts) > 2 and math.dist(pts[0], pts[-1]) < 1e-3
                xs, ys = zip(*pts)
                if fill:
                    ax.fill(xs, ys, color=fill, lw=0)
                if closed and not dash and not fill:
                    ax.add_patch(Polygon(pts, closed=True, fill=False, ec=color, lw=width * pt, joinstyle="round"))
                    continue
                (line,) = ax.plot(xs, ys, color=color, lw=width * pt, solid_capstyle="round",
                                  solid_joinstyle="round", dash_capstyle="round")
                if dash:
                    line.set_dashes((dash[0] * pt, dash[1] * pt))
            else:
                _, x, y, s, size, anchor, color, bold = it
                ax.text(x, y, s, fontsize=size * pt, ha=ha[anchor], va="baseline", color=color,
                        fontweight="bold" if bold else "normal", family="DejaVu Sans")
        return fig


def edge_points(edge, step=0.1):
    if edge.geom_type == GeomType.LINE:
        ends = (edge.position_at(0), edge.position_at(1))
        return [(v.X, v.Y) for v in ends]
    n = max(2, math.ceil(edge.length / step))
    return [(v.X, v.Y) for v in (edge.position_at(i / n) for i in range(n + 1))]


def chain(polys, tol=1e-3):
    """Join polylines that share end points into as few as possible (a dashed line stays one pattern)."""
    polys = [list(q) for q in polys]
    out = []
    near = lambda a, b: abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol  # noqa: E731
    while polys:
        cur = polys.pop(0)
        grown = True
        while grown:
            grown = False
            for i, q in enumerate(polys):
                if near(cur[-1], q[0]):
                    cur += q[1:]
                elif near(cur[-1], q[-1]):
                    cur += q[-2::-1]
                elif near(cur[0], q[-1]):
                    cur = q[:-1] + cur
                elif near(cur[0], q[0]):
                    cur = q[:0:-1] + cur
                else:
                    continue
                polys.pop(i)
                grown = True
                break
        out.append(cur)
    return out


def outline(shape):
    return chain(edge_points(e) for e in shape.edges())


def clip_x(polys, x0):
    """The parts of the polylines at x >= x0."""
    out = []
    for q in polys:
        cur = []
        for a, b in zip(q, q[1:]):
            ina, inb = a[0] >= x0, b[0] >= x0
            if ina and not cur:
                cur = [a]
            if ina != inb:
                t = (x0 - a[0]) / (b[0] - a[0])
                c = (x0, a[1] + t * (b[1] - a[1]))
                if ina:
                    out.append(cur + [c])
                    cur = []
                else:
                    cur = [c]
            if inb:
                cur.append(b)
        if len(cur) > 1:
            out.append(cur)
    return out


def place(polys, at):
    return [[(at[0] + x, at[1] - y) for x, y in q] for q in polys]


def leader(sh, label_xy, target, color=NOTE):
    """A thin leader from a label to a dot on the line it names."""
    x, y = label_xy
    sh.path([(x, y + 0.8), target], width=THIN, color=color)
    r = 0.35
    sh.path([(target[0] + r * math.cos(a), target[1] + r * math.sin(a)) for a in
             (k * math.pi / 8 for k in range(17))], width=THIN, color=color, fill=color)


def arrow(sh, x, y, length, label):
    sh.path([(x, y), (x + length - 2.0, y)], width=0.4, color=NOTE)
    sh.path([(x + length, y), (x + length - 2.4, y - 1.1), (x + length - 2.4, y + 1.1), (x + length, y)],
            width=0.1, color=NOTE, fill=NOTE)
    sh.text(x + length / 2, y - 1.4, label, size=2.4, anchor="middle", color=NOTE, bold=True)


def rulers(sh):
    x0, y0 = RULER_H
    sh.path([(x0, y0), (x0 + 100, y0)], width=THIN, cls="ruler-h-base")
    for i in range(101):
        h = 5.0 if i % 10 == 0 else 3.5 if i % 5 == 0 else 2.0
        sh.path([(x0 + i, y0), (x0 + i, y0 - h)], width=THIN, cls="tick-h")
        if i % 10 == 0:
            sh.text(x0 + i, y0 - 6.0, str(i), size=2.4, anchor="middle")
    sh.text(x0 + 104, y0 - 0.3, "100 mm", size=2.6, bold=True)
    x0, y0 = RULER_V
    sh.path([(x0, y0), (x0, y0 + 100)], width=THIN, cls="ruler-v-base")
    for i in range(101):
        h = 5.0 if i % 10 == 0 else 3.5 if i % 5 == 0 else 2.0
        sh.path([(x0, y0 + i), (x0 - h, y0 + i)], width=THIN, cls="tick-v")
        if i % 10 == 0:
            sh.text(x0 - 5.6, y0 + i + 0.85, str(i), size=2.4, anchor="end")
    sh.text(x0 - 4.0, y0 + 105.0, "100 mm", size=2.6, anchor="middle", bold=True)


def references(sh):
    x, y, w, h, r = CARD
    pts = []
    for cx, cy, a0 in ((x + w - r, y + r, -90), (x + w - r, y + h - r, 0), (x + r, y + h - r, 90), (x + r, y + r, 180)):
        pts += [(cx + r * math.cos(math.radians(a0 + k * 10)), cy + r * math.sin(math.radians(a0 + k * 10)))
                for k in range(10)]
    start = [(x + w / 2, y)]
    sh.path(start + pts + start, width=LINE, color=NOTE, cls="card")
    sh.text(x + w / 2, y + h / 2 - 2.0, "ID-1 card outline: 85.60 x 53.98 mm", size=2.8, anchor="middle", bold=True)
    sh.text(x + w / 2, y + h / 2 + 2.5, "lay a real bank card here: its edges", size=2.4, anchor="middle")
    sh.text(x + w / 2, y + h / 2 + 5.8, "must cover this line all the way round", size=2.4, anchor="middle")
    x, y, s = SQUARE
    sh.path([(x, y), (x + s, y), (x + s, y + s), (x, y + s), (x, y)], width=LINE, color=NOTE, cls="square")
    sh.text(x + s / 2, y + s / 2 - 1.0, "50 x 50 mm", size=2.8, anchor="middle", bold=True)
    sh.text(x + s / 2, y + s / 2 + 3.0, "measure both sides", size=2.4, anchor="middle")


def parts(sh, p: Params, sk):
    from build123d import offset

    from .skirt import board_outline, doubler, pattern

    board = outline(board_outline(p))
    skirt, nose = pattern(p, sk), doubler(p, sk)
    trim = outline(offset(board_outline(p), sk.trim))

    for q in place(outline(skirt), SKIRT_AT):
        sh.path(q, cls="cut-skirt")
    for q in place(board, SKIRT_AT):
        sh.path(q, color=FOLD, dash=(1.5, 1.0), cls="fold-skirt")
    for q in place(trim, SKIRT_AT):
        sh.path(q, width=0.25, color=TRIM, dash=(0.01, 0.75), cls="trim-skirt")
    for q in place(clip_x(trim, sk.doubler_x), DOUBLER_AT):
        sh.path(q, width=0.25, color=TRIM, dash=(0.01, 0.75), cls="trim-doubler")
    for q in place(outline(nose), DOUBLER_AT):
        sh.path(q, cls="cut-doubler")
    for q in place(clip_x(board, sk.doubler_x), DOUBLER_AT):
        sh.path(q, color=FOLD, dash=(1.5, 1.0), cls="fold-doubler")

    sx, sy = SKIRT_AT
    sh.text(sx - 5, sy + 0.5, "MAIN SKIRT", size=3.4, anchor="middle", bold=True)
    sh.text(sx - 5, sy + 4.5, "cut 1, on the outer and the inner line", size=2.3, anchor="middle")
    arrow(sh, sx + 18, sy + 1.0, 20, "FRONT")
    band_y = 25 - sk.tape_w / 2
    sh.text(sx - 31.5, sy - 25 + sk.tape_w + 4.5, f"tape band {sk.tape_w:g} mm", size=2.1, color=NOTE)
    leader(sh, (sx - 25, sy - 25 + sk.tape_w + 1.6), (sx - 25, sy - band_y))
    sh.text(sx - 34, sy - 32.0, f"lip {sk.margin:g} mm", size=2.3, color=NOTE)
    leader(sh, (sx - 31, sy - 31.4), (sx - 31, sy - 25 - sk.margin / 2))
    sh.text(sx + 13, sy - 32.0, "fold here (board edge)", size=2.3, color=FOLD)
    leader(sh, (sx + 22, sy - 31.4), (sx + 22, sy - 25))

    sh.text(sx - 14, sy + 34.0, f"trim here for stiff films (acetate), {sk.trim:g} mm past the fold", size=2.3,
            color=TRIM)
    leader(sh, (sx + 22, sy + 30.8), (sx + 22, sy + 25 + sk.trim), color=TRIM)

    dx, dy = DOUBLER_AT
    sh.text(dx + 44, dy + 1.0, "FRONT", size=2.4, anchor="middle", color=NOTE, bold=True)
    arrow(sh, dx + 36, dy + 3.0, 14, "")
    bottom = dy + 25 + sk.margin
    sh.text(dx + 44, bottom + 5.0, "NOSE DOUBLER", size=3.4, anchor="middle", bold=True)
    sh.text(dx + 44, bottom + 9.0, "cut 1; laminate under the main skirt", size=2.3, anchor="middle")
    sh.text(dx + 44, bottom + 12.2, "at the nose, fold lines on top of each other", size=2.3, anchor="middle")


NOTES = [
    ("Micras suction skirt, 1:1 cutting sheet", 3.4, True),
    ("lip {m:g} mm, tape band {t:g} mm; 10 um polyethylene,", 2.4, False),
    ("40 um cellulose acetate or TPU film", 2.4, False),
    ("", 2.0, False),
    ("1. Zoom the screen until the 100 mm ruler", 2.6, True),
    ("   matches a real ruler and a real bank card", 2.6, True),
    ("   fits the card outline.", 2.6, True),
    ("2. Check BOTH rulers and the 50 mm square:", 2.6, True),
    ("   screens can scale x and y differently.", 2.6, False),
    ("3. Lay the film flat on the screen and trace.", 2.6, True),
    ("   The centre of each line is the true edge.", 2.6, False),
    ("   Mark the dashed fold line with a few dots.", 2.6, False),
    ("4. Cut on the solid lines.", 2.6, True),
    ("", 2.0, False),
    ("Solid black: cut. Dashed blue: the board edge,", 2.4, False),
    ("where the film folds down. Inside it the band", 2.4, False),
    ("is taped to the board (polyethylene needs LSE", 2.4, False),
    ("tape such as 3M 9472LE); outside it the lip", 2.4, False),
    ("trails outward on the floor.", 2.4, False),
    ("Dotted orange: trim line, {tr:g} mm past the fold.", 2.4, False),
    ("A stiff film (acetate) would carry the robot on", 2.4, False),
    ("its lip: cut it there instead, so it just reaches", 2.4, False),
    ("the floor. Soft films keep the full lip.", 2.4, False),
    ("", 2.0, False),
    ("Shown from above, front to the right. The film", 2.4, False),
    ("has no top or bottom side: flip it if needed.", 2.4, False),
    ("", 2.0, False),
    ("The doubler goes under the skirt (floor side)", 2.4, False),
    ("from the nose back to where the sides turn", 2.4, False),
    ("straight; it keeps the lip from folding under", 2.4, False),
    ("at the maze's board joints.", 2.4, False),
]


def build(p: Params = P, sk=None):
    from .skirt import SK

    sk = sk or SK
    sh = Sheet()
    rulers(sh)
    references(sh)
    parts(sh, p, sk)
    y = 27.0
    for s, size, bold in NOTES:
        if s:
            sh.text(TEXT_X, y, s.format(m=sk.margin, t=sk.tape_w, tr=sk.trim), size=size, bold=bold)
        y += size * 1.45
    return sh


def export(out_dir, p: Params = P, sk=None):
    sh = build(p, sk)
    (out_dir / "skirt_sheet.svg").write_text(sh.svg())
    fig = sh.figure()
    fig.savefig(out_dir / "skirt_sheet.pdf", facecolor="white")
    fig.savefig(out_dir / "skirt_sheet.png", dpi=254, facecolor="white")
    return sh
