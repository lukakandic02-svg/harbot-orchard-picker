#!/usr/bin/env python3
"""Plot the topology-matrix results (results/topology_matrix.csv).

    tools/analysis/plot_results.py results/topology_matrix.csv docs/media/results.png
"""
import csv
import statistics
import sys
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

TOPOLOGIES = ["line", "m1", "m2", "m3", "full"]
LABELS = ["line", "M1", "M2", "M3", "full"]

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
SERIES = "#2a78d6"


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def style(ax, title, ylabel):
    ax.set_title(title, loc="left", fontsize=11, color=INK, pad=10, fontweight="bold")
    ax.set_ylabel(ylabel, color=INK_2, fontsize=9)
    ax.set_facecolor(SURFACE)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(INK_2)
    ax.tick_params(colors=INK_2, labelsize=9, length=0)
    ax.set_xticks(range(len(TOPOLOGIES)))
    ax.set_xticklabels(LABELS)
    ax.set_xlabel("workspace topology (least → most connected)", color=INK_2, fontsize=9)


def bar_panel(ax, values, labels, title, ylabel, ymax=None):
    bars = ax.bar(range(len(values)), values, width=0.55, color=SERIES, zorder=2)
    for b, text in zip(bars, labels):
        ax.annotate(text, (b.get_x() + b.get_width() / 2, b.get_height()),
                    xytext=(0, 4), textcoords="offset points", ha="center",
                    va="bottom", fontsize=9, color=INK)
    style(ax, title, ylabel)
    if ymax:
        ax.set_ylim(0, ymax)


def strip_panel(ax, groups, title, ylabel):
    for i, vals in enumerate(groups):
        n = len(vals)
        xs = [i + (k - (n - 1) / 2) * 0.035 for k in range(n)]
        ax.scatter(xs, vals, s=22, color=SERIES, alpha=0.75, edgecolor=SURFACE,
                   linewidth=0.8, zorder=3)
        if vals:
            med = statistics.median(vals)
            ax.hlines(med, i - 0.28, i + 0.28, color=INK, linewidth=2, zorder=4)
            ax.annotate("%.0f" % med, (i + 0.3, med), va="center", fontsize=8.5, color=INK)
    style(ax, title, ylabel)
    ax.set_xlim(-0.5, len(groups) - 0.3)


def main(csv_path, out_path):
    rows = load(csv_path)
    by = defaultdict(list)
    for r in rows:
        by[r["topology"]].append(r)

    succ, succ_lab, paths, makespans, deadlocks = [], [], [], [], []
    for t in TOPOLOGIES:
        rs = by[t]
        ok = [r for r in rs if r["mission_success"] == "True"]
        succ.append(100.0 * len(ok) / len(rs))
        succ_lab.append("%d/%d" % (len(ok), len(rs)))
        paths.append([float(r["path_total_m"]) for r in ok])
        makespans.append([float(r["makespan_s"]) for r in ok])
        deadlocks.append(sum(int(r["deadlock_events_a"]) + int(r["deadlock_events_b"]) for r in rs))

    fig, axes = plt.subplots(2, 2, figsize=(11, 7.6), facecolor=SURFACE)
    bar_panel(axes[0][0], succ, succ_lab, "Mission success (both robots finish)",
              "% of 15 missions", ymax=108)
    bar_panel(axes[0][1], deadlocks, [str(d) for d in deadlocks],
              "Deadlock-handling events (all 75 missions)", "events per topology",
              ymax=max(deadlocks) * 1.12)
    strip_panel(axes[1][0], paths, "Distance driven, both robots (successful missions)",
                "metres  (dot = mission, bar = median)")
    strip_panel(axes[1][1], makespans, "Makespan (successful missions)",
                "simulated seconds  (bar = median)")
    fig.suptitle("Topology matrix: 5 topologies × 5 target splits × 3 repetitions"
                 " = 75 two-robot missions", x=0.01, ha="left", fontsize=12.5,
                 color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.96), h_pad=2.5, w_pad=2.5)
    fig.savefig(out_path, dpi=150, facecolor=SURFACE)
    print("wrote", out_path)


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    main(sys.argv[1], sys.argv[2])
