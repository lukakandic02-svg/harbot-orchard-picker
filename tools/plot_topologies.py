#!/usr/bin/env python3
"""Draw the five workspace topologies side by side.

    PYTHONPATH=src tools/plot_topologies.py docs/media/topologies.png
"""
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from harbot_orchard_picker import orchard_topology as ot  # noqa: E402

ORDER = [("line", "line"), ("m1", "M1"), ("m2", "M2"), ("m3", "M3"), ("full", "full")]
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
TREE = "#9bbf8a"
EDGE = "#2a78d6"
SHORTCUT = "#eb6834"


def position(loc):
    kind = ot.parse_loc(loc)
    if kind[0] == "I":
        return kind[1], kind[2]
    passage, gap, neighbour = kind[1], kind[2], kind[3]
    return passage + 0.22 * (neighbour - passage), gap + 0.5


def draw(ax, name, label):
    locs, edges = ot.build_topology(name)
    for row in range(4):                       # tree rows sit between passages
        ax.scatter([row + 0.5] * 8, range(1, 9), marker="^", s=34, color=TREE, zorder=1)
    seen = set()
    for a, b in edges:
        key = tuple(sorted((a, b)))
        if key in seen:
            continue
        seen.add(key)
        (xa, ya), (xb, yb) = position(a), position(b)
        is_shortcut = "_M" in a or "_M" in b
        ax.plot([xa, xb], [ya, yb], color=SHORTCUT if is_shortcut else EDGE,
                linewidth=1.1 if is_shortcut else 1.8, zorder=2, solid_capstyle="round")
    xs, ys = zip(*(position(s) for s in ot.base_locations()))
    ax.scatter(xs, ys, s=16, color=INK, zorder=3)
    for depot, txt in (("P0_I1", "A"), ("P4_I8", "B")):
        x, y = position(depot)
        ax.scatter([x], [y], s=110, facecolor=SURFACE, edgecolor=INK, linewidth=1.5, zorder=4)
        ax.annotate(txt, (x, y), ha="center", va="center", fontsize=7, fontweight="bold",
                    zorder=5)
    ax.set_title("%s\n%d nodes · %d edges" % (label, len(locs), len(edges)),
                 fontsize=10, color=INK)
    ax.set_xlim(-0.5, 4.5)
    ax.set_ylim(0.3, 8.7)
    ax.set_aspect(0.62)
    ax.axis("off")


def main(out):
    fig, axes = plt.subplots(1, 5, figsize=(13, 4.6), facecolor=SURFACE)
    for ax, (name, label) in zip(axes, ORDER):
        draw(ax, name, label)
    handles = [
        Line2D([], [], color=EDGE, linewidth=1.8, label="passage / row-end transfer"),
        Line2D([], [], color=SHORTCUT, linewidth=1.1, label="mid-row shortcut"),
        Line2D([], [], marker="^", linestyle="", color=TREE, label="tree row"),
        Line2D([], [], marker="o", linestyle="", markerfacecolor=SURFACE,
               markeredgecolor=INK, label="depot of robot A / B"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False, fontsize=9,
               labelcolor=INK_2)
    fig.suptitle("Workspace topologies: passages P0–P4 (columns) × stations I1–I8",
                 fontsize=12, color=INK, fontweight="bold", x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0.07, 1, 0.95))
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print("wrote", out)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    main(sys.argv[1])
