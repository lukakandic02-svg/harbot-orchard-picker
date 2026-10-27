#!/usr/bin/env python3
"""Summarise a topology-matrix batch into one CSV row per mission.

Reads the per-mission summary lines that experiments/run_topology_matrix.sh
writes to overnight_summary.txt (one or more files, e.g. one per pass),
classifies every failed mission by cause from the robots' logs, prints the
topology x seed success table and writes a tidy CSV.

    tools/analysis/summarize_matrix.py \
        --summary pass1/overnight_summary.txt pass23/overnight_summary.txt \
        --logs pass1 pass23 \
        --out results/topology_matrix.csv

Per-mission logs are looked up as <logs dir>/<run label>/auto_*.txt.
Read-only with respect to the inputs.
"""
import argparse
import csv
import re
from collections import Counter, defaultdict
from pathlib import Path

TOPOLOGIES = ["line", "m1", "m2", "m3", "full"]

RUN_LINE = re.compile(
    r"\[(?P<run>dual_topology_(?P<topology>\w+?)_seed(?P<seed>\d+)_rep(?P<rep>\d+))\]"
    r" [\d:]+ wall=(?P<wall>\d+)s (?P<fields>.*)$")
TIMEOUT_LINE = re.compile(
    r"\[(?P<run>dual_topology_\w+?_seed\d+_rep\d+)\] TIMEOUT after (?P<wall>\d+)s")

# Failure causes, in priority order: the first pattern found in the logs wins.
FAILURE_CAUSES = [
    ("bottleneck_deadlock", r"DEADLOCK|deadlock retries exhausted|RETRY_EXHAUSTED"),
    ("segment_timeout", r"PASSAGE SEGMENT aborted: timeout|absolute_timeout"),
    ("cross_track_exceeded", r"cross_track"),
    ("tilt_abort", r"TILT PERSISTENT|TILT DETECTED"),
    ("simulator_crash", r"unable to connect to service|failed to contact, will keep trying"),
    ("pick_attempts_exhausted", r"max_pick_attempts|PICK FINAL FAIL"),
]

CSV_COLUMNS = [
    "run", "topology", "seed", "rep", "wall_s", "hit_timeout",
    "success_a", "success_b", "picked_a", "picked_b", "mission_success",
    "makespan_s", "path_a_m", "path_b_m", "path_total_m",
    "deadlock_events_a", "deadlock_events_b",
    "tilt_detections_a", "tilt_detections_b", "tilt_aborts_a", "tilt_aborts_b",
    "min_peer_distance_m", "failure_cause",
]


def number_or_blank(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return ""


def parse_summaries(paths):
    runs = {}
    timeouts = set()
    for path in paths:
        for line in Path(path).read_text(errors="replace").splitlines():
            m = TIMEOUT_LINE.search(line)
            if m:
                timeouts.add(m.group("run"))
                continue
            m = RUN_LINE.search(line)
            if not m:
                continue
            f = dict(kv.split("=", 1) for kv in m.group("fields").split())
            status = {k: f.get(k, "NONE").replace("success=", "") for k in ("A_status", "B_status")}
            path_a = number_or_blank(f.get("planar_A_m"))
            path_b = number_or_blank(f.get("planar_B_m"))
            runs[m.group("run")] = {
                "run": m.group("run"),
                "topology": m.group("topology"),
                "seed": int(m.group("seed")),
                "rep": int(m.group("rep")),
                "wall_s": int(m.group("wall")),
                "success_a": status["A_status"],
                "success_b": status["B_status"],
                "picked_a": f.get("A_picked", "NONE"),
                "picked_b": f.get("B_picked", "NONE"),
                "mission_success": status["A_status"] == "True" and status["B_status"] == "True",
                "makespan_s": number_or_blank(f.get("makespan_s")),
                "path_a_m": path_a,
                "path_b_m": path_b,
                "path_total_m": path_a + path_b if path_a != "" and path_b != "" else "",
                "deadlock_events_a": int(f.get("A_deadlock", 0)),
                "deadlock_events_b": int(f.get("B_deadlock", 0)),
                "tilt_detections_a": int(f.get("A_tiltdet", 0)),
                "tilt_detections_b": int(f.get("B_tiltdet", 0)),
                "tilt_aborts_a": int(f.get("A_tilt", 0)),
                "tilt_aborts_b": int(f.get("B_tilt", 0)),
                "min_peer_distance_m": number_or_blank(f.get("min_peer_dist")),
            }
    for run in runs.values():
        run["hit_timeout"] = run["run"] in timeouts
    return runs


def failure_cause(run, log_dirs):
    text = ""
    for base in log_dirs:
        d = Path(base) / run["run"]
        for p in list(d.glob("auto_*.txt")) + list(d.glob("full_report/auto_*.txt")):
            text += p.read_text(errors="replace")
    if not text:
        return "unknown_no_logs"
    for name, pattern in FAILURE_CAUSES:
        if re.search(pattern, text):
            return name
    return "timeout_without_trace" if run["hit_timeout"] else "unknown"


def print_success_table(runs):
    seeds = sorted({r["seed"] for r in runs})
    cells = defaultdict(list)
    for r in runs:
        cells[(r["topology"], r["seed"])].append(r["mission_success"])
    header = "  %-6s" % "" + "".join("%10s" % ("seed %d" % s) for s in seeds) + "%14s" % "total"
    print(header)
    print("  " + "-" * (len(header) - 2))
    for t in TOPOLOGIES:
        row, ok, n = "  %-6s" % t, 0, 0
        for s in seeds:
            v = cells.get((t, s), [])
            ok, n = ok + sum(v), n + len(v)
            row += "%10s" % ("%d/%d" % (sum(v), len(v)) if v else "-")
        if n:
            row += "%9s %3.0f%%" % ("%d/%d" % (ok, n), 100.0 * ok / n)
        print(row)
    total_ok = sum(r["mission_success"] for r in runs)
    total = "%d/%d" % (total_ok, len(runs))
    print("  %-6s%s%9s %3.0f%%" % ("all", " " * (10 * len(seeds)), total,
                                   100.0 * total_ok / len(runs)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--summary", nargs="+", required=True, help="overnight_summary.txt file(s)")
    ap.add_argument("--logs", nargs="*", default=None,
                    help="directories holding <run>/auto_*.txt (default: the summaries' dirs)")
    ap.add_argument("--out", type=Path, help="CSV to write")
    args = ap.parse_args()

    log_dirs = args.logs if args.logs is not None else [str(Path(s).parent) for s in args.summary]
    runs = sorted(parse_summaries(args.summary).values(),
                  key=lambda r: (TOPOLOGIES.index(r["topology"]) if r["topology"] in TOPOLOGIES
                                 else 99, r["seed"], r["rep"]))
    if not runs:
        raise SystemExit("no mission summary lines found")
    for r in runs:
        r["failure_cause"] = "" if r["mission_success"] else failure_cause(r, log_dirs)

    print("Missions: %d\n" % len(runs))
    print("Mission success (both robots finished all targets):")
    print_success_table(runs)

    failed = [r for r in runs if not r["mission_success"]]
    print("\nFailure causes (%d failed missions):" % len(failed))
    for cause, n in Counter(r["failure_cause"] for r in failed).most_common():
        print("  %3d  %s" % (n, cause))

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
            w.writeheader()
            for r in runs:
                w.writerow({k: r[k] for k in CSV_COLUMNS})
        print("\nwrote %s" % args.out)


if __name__ == "__main__":
    main()
