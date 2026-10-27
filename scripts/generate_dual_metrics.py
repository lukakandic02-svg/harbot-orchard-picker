#!/usr/bin/env python3

"""Create per-robot and fleet-level metrics for a dual harvesting run."""

import csv
import json
import math
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROBOT_COLORS = {"robot": "#d97706", "robot_b": "#2563eb"}
ROBOT_LABELS = {"robot": "Robot A", "robot_b": "Robot B"}


def read_trajectory(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def trajectory_metrics(path):
    rows = read_trajectory(path)
    if len(rows) < 2:
        raise RuntimeError("Premalo uzoraka u %s" % path)
    sim = [float(r["sim_time_s"]) for r in rows]
    wall = [float(r["wall_time_s"]) for r in rows]
    xy = [(float(r["x_m"]), float(r["y_m"])) for r in rows]
    speeds = [float(r["linear_speed_mps"]) for r in rows]
    return {
        "simulation_duration_s": sim[-1] - sim[0],
        "wall_duration_s": wall[-1] - wall[0],
        "planar_distance_m": sum(math.hypot(xy[i][0] - xy[i-1][0], xy[i][1] - xy[i-1][1]) for i in range(1, len(xy))),
        "average_moving_speed_mps": sum(v for v in speeds if v > 0.02) / max(1, sum(v > 0.02 for v in speeds)),
        "maximum_speed_mps": max(speeds),
        "maximum_abs_roll_deg": max(abs(float(r["roll_deg"])) for r in rows),
        "maximum_abs_pitch_deg": max(abs(float(r["pitch_deg"])) for r in rows),
        "odometry_samples": len(rows),
    }


def plot_trajectories(root, profile):
    fig, ax = plt.subplots(figsize=(8, 8))
    for robot in ("robot", "robot_b"):
        path = root / ("trajectory_%s.csv" % robot)
        if not path.exists():
            continue
        rows = read_trajectory(path)
        xs = [float(r["x_m"]) for r in rows]
        ys = [float(r["y_m"]) for r in rows]
        color = ROBOT_COLORS[robot]
        ax.plot(xs, ys, linewidth=1.2, color=color, label="putanja %s" % ROBOT_LABELS[robot])
        ax.scatter([xs[0]], [ys[0]], color=color, marker="o", s=55,
                   edgecolors="green", linewidths=1.5, zorder=5)
        ax.scatter([xs[-1]], [ys[-1]], color=color, marker="s", s=55,
                   edgecolors="red", linewidths=1.5, zorder=5)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Putanje oba robota - %s (kruzic=start, kvadrat=kraj)" % profile)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(root / "trajectory_dual.png", dpi=180)
    plt.close(fig)


def plot_speed_and_tilt(root, profile):
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for robot in ("robot", "robot_b"):
        path = root / ("trajectory_%s.csv" % robot)
        if not path.exists():
            continue
        rows = read_trajectory(path)
        sim = [float(r["sim_time_s"]) for r in rows]
        elapsed = [value - sim[0] for value in sim]
        speeds = [float(r["linear_speed_mps"]) for r in rows]
        tilt = [max(abs(float(r["roll_deg"])), abs(float(r["pitch_deg"]))) for r in rows]
        color = ROBOT_COLORS[robot]
        axes[0].plot(elapsed, speeds, linewidth=0.8, color=color, label=ROBOT_LABELS[robot])
        axes[1].plot(elapsed, tilt, linewidth=0.8, color=color, label=ROBOT_LABELS[robot])
    axes[0].set_ylabel("brzina [m/s]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].set_ylabel("max(|roll|,|pitch|) [deg]")
    axes[1].set_xlabel("simulaciono vreme [s]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.suptitle("Brzina i stabilnost oba robota - %s" % profile)
    fig.tight_layout()
    fig.savefig(root / "speed_and_tilt_dual.png", dpi=180)
    plt.close(fig)


def log_metrics(path):
    text = path.read_text(errors="replace")
    picked = re.search(r"Picked target count: (\d+) / (\d+)", text)
    physical = re.search(r"Physical fruits picked: (\d+)", text)
    status = re.search(r"ONLINE RUN STATUS success=(True|False)", text)
    return {
        "mission_success": status.group(1) == "True" if status else False,
        "targets_picked": int(picked.group(1)) if picked else 0,
        "targets_total": int(picked.group(2)) if picked else 0,
        "physical_fruits_picked": int(physical.group(1)) if physical else 0,
        "pick_attempts": len(re.findall(r"ONLINE PHYSICAL PICK", text)),
        "failed_target_picks": len(re.findall(r"ONLINE PICK FINAL FAIL", text)),
        "drop_events": len(re.findall(r"ONLINE DROP at", text)),
        "route_replans": len(re.findall(r"GLOBAL ROUTE PLAN", text)),
        "tilt_events": len(re.findall(r"TILT DETECTED", text)),
    }


def main(session):
    root = Path(session)
    result = {"robots": {}}
    for robot in ("robot", "robot_b"):
        logs = sorted(root.glob("*_robot_b.txt" if robot == "robot_b" else "*_robot.txt"))
        metrics = trajectory_metrics(root / ("trajectory_%s.csv" % robot))
        metrics.update(log_metrics(logs[-1]))
        metrics["productivity_targets_per_sim_hour"] = 3600.0 * metrics["targets_picked"] / max(metrics["simulation_duration_s"], 1e-9)
        result["robots"][robot] = metrics

    robots = result["robots"]
    result["fleet"] = {
        "mission_success": all(v["mission_success"] for v in robots.values()),
        "makespan_simulation_s": max(v["simulation_duration_s"] for v in robots.values()),
        "makespan_wall_s": max(v["wall_duration_s"] for v in robots.values()),
        "total_planar_distance_m": sum(v["planar_distance_m"] for v in robots.values()),
        "targets_picked": sum(v["targets_picked"] for v in robots.values()),
        "targets_total": sum(v["targets_total"] for v in robots.values()),
        "physical_fruits_picked": sum(v["physical_fruits_picked"] for v in robots.values()),
    }
    result["fleet"]["productivity_targets_per_sim_hour"] = 3600.0 * result["fleet"]["targets_picked"] / max(result["fleet"]["makespan_simulation_s"], 1e-9)
    (root / "dual_metrics.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    with (root / "dual_metrics.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["scope", "metric", "value"])
        for robot, values in result["robots"].items():
            for key, value in values.items():
                writer.writerow([robot, key, value])
        for key, value in result["fleet"].items():
            writer.writerow(["fleet", key, value])

    profile = root.name
    plot_trajectories(root, profile)
    plot_speed_and_tilt(root, profile)

    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: generate_dual_metrics.py SESSION_DIR")
    main(sys.argv[1])
