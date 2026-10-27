#!/usr/bin/env bash
#
# Put the package's configuration where the ROS nodes expect it at runtime:
#   ~/harbot_calibration/tree_points.jsonl       orchard calibration (3 reference trees)
#   ~/harbot_rddl/orchard_targets_meta.csv        target -> station table
#   $PROST_DIR/testbed/benchmarks/orchard_targets  RDDL domains (instances are generated)
#
# Usage: tools/setup_workspace.sh            (PROST_DIR defaults to ~/prost)

set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROST_DIR="${PROST_DIR:-$HOME/prost}"
CALIB_DIR="${HARBOT_CALIBRATION_DIR:-$HOME/harbot_calibration}"
RDDL_DIR="$HOME/harbot_rddl"
BENCH_DIR="$PROST_DIR/testbed/benchmarks/orchard_targets"

if [[ ! -f "$PROST_DIR/prost.py" ]]; then
  echo "PROST not found in $PROST_DIR (set PROST_DIR); see the README setup steps." >&2
  exit 1
fi

mkdir -p "$CALIB_DIR" "$RDDL_DIR" "$BENCH_DIR"
cp "$PKG_DIR/config/tree_points.jsonl" "$CALIB_DIR/"
cp "$PKG_DIR/config/orchard_targets_meta.csv" "$RDDL_DIR/"
cp "$PKG_DIR"/rddl/orchard_targets_mdp.rddl "$PKG_DIR"/rddl/orchard_targets_line_mdp.rddl "$BENCH_DIR/"

echo "calibration -> $CALIB_DIR"
echo "targets     -> $RDDL_DIR/orchard_targets_meta.csv"
echo "RDDL domain -> $BENCH_DIR"
