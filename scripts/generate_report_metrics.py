#!/usr/bin/env python3

import csv
import json
import math
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_trajectory(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def count(pattern, text):
    return len(re.findall(pattern, text, flags=re.MULTILINE))


def main(session_dir):
    session = Path(session_dir)
    rows = read_trajectory(session / "trajectory.csv")
    if len(rows) < 2:
        raise RuntimeError("Too few odometry samples to compute metrics")

    sim = [float(row["sim_time_s"]) for row in rows]
    wall = [float(row["wall_time_s"]) for row in rows]
    xs = [float(row["x_m"]) for row in rows]
    ys = [float(row["y_m"]) for row in rows]
    speeds = [float(row["linear_speed_mps"]) for row in rows]
    rolls = [abs(float(row["roll_deg"])) for row in rows]
    pitches = [abs(float(row["pitch_deg"])) for row in rows]
    distance = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
                   for i in range(1, len(rows)))

    # Use the log of any profile; skip the per-robot _robot/_robot_b files.
    log_files = sorted(p for p in session.glob("auto_*.txt")
                       if "_robot" not in p.name)
    log_text = log_files[-1].read_text(errors="replace") if log_files else ""
    picked_match = re.search(r"Picked target count: (\d+) / (\d+)", log_text)
    physical_match = re.search(r"Physical fruits picked: (\d+)", log_text)
    status_match = re.search(r"ONLINE RUN STATUS success=(True|False)", log_text)

    metrics = {
        "mission_success": status_match.group(1) == "True" if status_match else None,
        "targets_picked": int(picked_match.group(1)) if picked_match else None,
        "targets_total": int(picked_match.group(2)) if picked_match else None,
        "physical_fruits_picked": int(physical_match.group(1)) if physical_match else None,
        "simulation_duration_s": sim[-1] - sim[0],
        "wall_duration_s": wall[-1] - wall[0],
        "planar_distance_m": distance,
        "average_planar_speed_while_moving_mps": (
            sum(v for v in speeds if v > 0.02) /
            max(1, sum(1 for v in speeds if v > 0.02))
        ),
        "maximum_planar_speed_mps": max(speeds),
        "maximum_abs_roll_deg": max(rolls),
        "maximum_abs_pitch_deg": max(pitches),
        "pick_attempts": count(r"ONLINE PHYSICAL PICK", log_text),
        "successful_target_picks": count(r"ONLINE PICK FINAL SUCCESS", log_text),
        "failed_target_picks": count(r"ONLINE PICK FINAL FAIL", log_text),
        "drop_events": count(r"ONLINE DROP at", log_text),
        "route_replans": count(r"GLOBAL ROUTE PLAN", log_text),
        "tilt_events": count(r"TILT DETECTED", log_text),
        "fruit_pose_invalid_events": count(r"FRUIT POSE INVALID", log_text),
        "odometry_samples": len(rows),
    }
    (session / "metrics_summary.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    )
    with (session / "metrics_summary.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["metric", "value"])
        writer.writerows(metrics.items())

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(xs, ys, linewidth=1.2, color="#d97706", label="putanja robota")
    ax.scatter([xs[0]], [ys[0]], color="green", s=45, label="start")
    ax.scatter([xs[-1]], [ys[-1]], color="red", s=45, label="kraj")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Putanja robota tokom full_global testa")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(session / "trajectory.png", dpi=180)
    plt.close(fig)

    elapsed = [value - sim[0] for value in sim]
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(elapsed, speeds, linewidth=0.8)
    axes[0].set_ylabel("brzina [m/s]")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(elapsed, rolls, label="|roll|")
    axes[1].plot(elapsed, pitches, label="|pitch|")
    axes[1].set_ylabel("nagib [deg]")
    axes[1].set_xlabel("simulaciono vreme [s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.suptitle("Brzina i stabilnost robota")
    fig.tight_layout()
    fig.savefig(session / "speed_and_tilt.png", dpi=180)
    plt.close(fig)

    print(json.dumps(metrics, indent=2, sort_keys=True))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: generate_report_metrics.py SESSION_DIR")
    main(sys.argv[1])
