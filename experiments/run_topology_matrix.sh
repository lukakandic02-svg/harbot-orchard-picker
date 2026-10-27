#!/usr/bin/env bash
#
# Run the topology experiment matrix unattended:
#   topologies x target seeds x repetitions  (default 5 x 5 x 3 = 75 missions)
#
# Each mission is one call of run_experiment.sh with profile
# dual_topology_<topology>_seed<seed>. Between missions the whole ROS/Gazebo
# process tree is torn down (gracefully first, then forcibly), memory is
# checked, and one summary line per mission is appended to
# $OUT_DIR/overnight_summary.txt - the input of tools/analysis/.
#
# Usage:
#   experiments/run_topology_matrix.sh                 # full 75-run matrix
#   REPS="2 3" experiments/run_topology_matrix.sh      # continue with reps 2-3
#   FREEZE_FILE=out/code_md5.txt REPS="2 3" ...        # refuse to run if the
#                                                      # code changed since then
#
# Environment: HARBOT_WS (default ~/catkin_ws), OUT_DIR, TOPOLOGIES, SEEDS,
# REPS, MISSION_TIMEOUT_S (default 2500), FREEZE_FILE.
#
# Repetitions run as complete passes (all cells of rep 1, then rep 2, ...),
# so an interruption leaves a balanced matrix instead of half the cells with
# n=3. Within a pass the seed is the outer loop, keeping comparable
# five-topology groups per seed together.

set -u

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${HARBOT_WS:-$HOME/catkin_ws}"
TOPOLOGIES="${TOPOLOGIES:-line m1 m2 m3 full}"
SEEDS="${SEEDS:-37 41 45 54 162}"
REPS="${REPS:-1 2 3}"
MISSION_TIMEOUT_S="${MISSION_TIMEOUT_S:-2500}"
OUT_DIR="${OUT_DIR:-$WORKSPACE/test_logs/topology_matrix_$(date +%Y%m%d_%H%M%S)}"
SUMMARY="$OUT_DIR/overnight_summary.txt"
RUNNER="$PKG_DIR/experiments/run_experiment.sh"
export PYTHONUNBUFFERED=1

mkdir -p "$OUT_DIR"
cd "$WORKSPACE" || exit 1

log() { echo "$*" | tee -a "$SUMMARY"; }

# --------------------------------------------------------------------------
# Frozen code: the matrix must measure ONE version of the system. The md5 of
# every file that defines behaviour is recorded at the start; with
# FREEZE_FILE set (e.g. the code_md5.txt of the first pass) the run refuses
# to start if anything differs.
# --------------------------------------------------------------------------
FROZEN_FILES=(
  "$PKG_DIR/scripts/online_orchard_manager.py"
  "$PKG_DIR/scripts/orchard_map_picker.py"
  "$PKG_DIR/scripts/target_pick_executor.py"
  "$PKG_DIR/src/harbot_orchard_picker/fleet_arbiter.py"
  "$PKG_DIR/src/harbot_orchard_picker/orchard_topology.py"
  "$RUNNER"
)
md5sum "${FROZEN_FILES[@]}" | sed "s|$PKG_DIR/||" > "$OUT_DIR/code_md5.txt"
if [[ -n "${FREEZE_FILE:-}" ]]; then
  if ! diff -q "$FREEZE_FILE" "$OUT_DIR/code_md5.txt" >/dev/null; then
    log "ABORTED: the code differs from $FREEZE_FILE -- this pass would"
    log "measure a different system than the earlier pass(es)."
    diff "$FREEZE_FILE" "$OUT_DIR/code_md5.txt" | tee -a "$SUMMARY"
    exit 1
  fi
fi

# --------------------------------------------------------------------------
# Process hygiene. A bare `pkill -9 -f roslaunch` once orphaned ~2000 child
# processes (move_group, controller spawners, ...) over a long batch, ate all
# RAM and made every later launch fail. Shut down in stages instead:
# process-group SIGINT to roslaunch first, then escalate, then sweep the
# nodes that are known to survive as orphans.
# --------------------------------------------------------------------------
ORPHAN_NODE_PATTERNS=(
  "moveit_ros_move_group/move_group"
  "robot_state_publisher/robot_state_publisher"
  "twist_mux/twist_mux"
  "twist_mux/twist_marker"
  "controller_manager/spawner"
  "joy/joy_node"
  "summit_xl_pad"
  "tf2_ros/static_transform_publisher"
  "orchard_map_visualizer"
  "map_server/map_server"
  "rosout/rosout"
)

kill_pattern_hard() {
  # Resolve to PIDs first: `pkill -f` silently matched nothing for some of
  # these long command lines.
  local pids pid
  pids=$(pgrep -f -- "$1" 2>/dev/null)
  [[ -z "$pids" ]] && return 0
  for pid in $pids; do
    [[ "$pid" == "$$" ]] && continue
    kill -9 "$pid" 2>/dev/null
  done
}

hard_cleanup() {
  local rl_pids pid pat
  rl_pids=$(pgrep -f "roslaunch rbkairos_sim_bringup" 2>/dev/null)
  for pid in $rl_pids; do
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  done
  if [[ -n "$rl_pids" ]]; then
    for _ in $(seq 1 40); do
      pgrep -f "roslaunch rbkairos_sim_bringup" >/dev/null 2>&1 || break
      sleep 0.5
    done
  fi
  for pat in "online_orchard_manager.py" "gzserver" "gzclient" "rosmaster" \
             "roslaunch" "run_experiment.sh" "rosbag" "gst-launch" \
             "ximagesrc" "report_metrics_recorder.py" "generate_dual_metrics.py"; do
    kill_pattern_hard "$pat"
  done
  for pat in "${ORPHAN_NODE_PATTERNS[@]}"; do
    kill_pattern_hard "$pat"
  done
  sleep 12
  rm -f "$WORKSPACE/.run_experiment.lock" "$WORKSPACE/.fresh_world.pid"
}

orphan_count() {
  ps aux | grep -E "/opt/ros/noetic/lib/|harbot_orchard_picker|summit_xl_pad" \
    | grep -v grep | wc -l
}

avail_mb() { free -m | awk '/^Mem:/{print $7}'; }

# Refuse to launch into a low-memory machine; try one deep sweep first.
memory_guard() {
  local label="$1" avail
  avail=$(avail_mb)
  if (( avail < 4000 )); then
    log "[$label] LOW MEMORY ${avail}MB (orphans=$(orphan_count)) -- deep sweep"
    hard_cleanup
    avail=$(avail_mb)
    log "[$label] after sweep: ${avail}MB available, orphans=$(orphan_count)"
  fi
  if (( avail < 2500 )); then
    log "[$label] ABORTING RUN -- only ${avail}MB available after sweep"
    return 1
  fi
}

count_in() { grep -c -- "$1" "$2" 2>/dev/null || true; }

run_one() {
  local profile="$1" rep="$2"
  local label="${profile}_rep${rep}"
  local outdir="$OUT_DIR/$label"
  hard_cleanup
  memory_guard "$label" || return 1
  mkdir -p "$outdir"
  export REPORT_SESSION_DIR="$outdir/full_report"
  log "=== $label: starting $(date +%H:%M:%S) (avail=$(avail_mb)MB orphans=$(orphan_count)) ==="

  # Launch, and confirm that robot_b's node actually came up (two attempts).
  local launch_ok=0 wrapper_pid="" waited attempt
  for attempt in 1 2; do
    nohup bash "$RUNNER" "$profile" > "$outdir/launch.log" 2>&1 &
    wrapper_pid=$!
    waited=0
    while ! pgrep -f "online_orchard_manager.py _robot_id:=robot_b" >/dev/null 2>&1; do
      sleep 2; waited=$((waited + 2))
      if (( waited >= 150 )); then
        log "[$label] launch attempt $attempt TIMEOUT"
        hard_cleanup
        break
      fi
      kill -0 "$wrapper_pid" 2>/dev/null || break
    done
    if pgrep -f "online_orchard_manager.py _robot_id:=robot_b" >/dev/null 2>&1; then
      launch_ok=1
      log "[$label] launch confirmed after ${waited}s (attempt $attempt)"
      break
    fi
    hard_cleanup
  done
  if (( launch_ok == 0 )); then
    log "[$label] FAILED TO LAUNCH after 2 attempts (avail=$(avail_mb)MB orphans=$(orphan_count)) -- skipping"
    hard_cleanup
    unset REPORT_SESSION_DIR
    return 1
  fi

  waited=0
  while kill -0 "$wrapper_pid" 2>/dev/null; do
    sleep 10; waited=$((waited + 10))
    if (( waited >= MISSION_TIMEOUT_S )); then
      log "[$label] TIMEOUT after ${MISSION_TIMEOUT_S}s -- forcing stop"
      break
    fi
  done
  log "[$label] run_experiment.sh exited (or timeout hit) after ${waited}s"

  # One summary line per mission, parsed from both robots' logs.
  local log_a log_b status_a status_b picked_a picked_b
  local tiltdet_a tiltdet_b tilt_a tilt_b dl_a dl_b ret_a ret_b min_dist
  local makespan planar_a planar_b metrics="$outdir/full_report/dual_metrics.json"
  log_a=$(ls -t test_logs/auto_${profile}_*_robot.txt 2>/dev/null | head -1)
  log_b=$(ls -t test_logs/auto_${profile}_*_robot_b.txt 2>/dev/null | head -1)
  status_a=$(grep "ONLINE RUN STATUS" "$log_a" 2>/dev/null | grep -oE "success=(True|False)" | tail -1)
  status_b=$(grep "ONLINE RUN STATUS" "$log_b" 2>/dev/null | grep -oE "success=(True|False)" | tail -1)
  picked_a=$(grep "Picked target count:" "$log_a" 2>/dev/null | tail -1 | grep -oE "[0-9]+ / [0-9]+" | tr -d ' ')
  picked_b=$(grep "Picked target count:" "$log_b" 2>/dev/null | tail -1 | grep -oE "[0-9]+ / [0-9]+" | tr -d ' ')
  tiltdet_a=$(count_in "TILT DETECTED" "$log_a"); tiltdet_b=$(count_in "TILT DETECTED" "$log_b")
  tilt_a=$(count_in "TILT PERSISTENT" "$log_a"); tilt_b=$(count_in "TILT PERSISTENT" "$log_b")
  dl_a=$(grep -c "PEER DEADLOCK\|FLEET RESERVATION HARD TIMEOUT" "$log_a" 2>/dev/null || true)
  dl_b=$(grep -c "PEER DEADLOCK\|FLEET RESERVATION HARD TIMEOUT" "$log_b" 2>/dev/null || true)
  ret_a=$(grep -c "retreat_unconfirmed\|RETREAT still unconfirmed" "$log_a" 2>/dev/null || true)
  ret_b=$(grep -c "retreat_unconfirmed\|RETREAT still unconfirmed" "$log_b" 2>/dev/null || true)
  min_dist=$(grep -ohE "PEER (DEADLOCK|PROXIMITY).*dist=[0-9.]+" "$log_a" "$log_b" 2>/dev/null \
             | grep -oE "[0-9.]+$" | sort -n | head -1)
  if [[ -f "$metrics" ]]; then
    read -r makespan planar_a planar_b < <(python3 - "$metrics" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1]))
r = d.get("robots", {})
print(d.get("fleet", {}).get("makespan_simulation_s", "NONE"),
      r.get("robot", {}).get("planar_distance_m", "NONE"),
      r.get("robot_b", {}).get("planar_distance_m", "NONE"))
EOF
)
  else
    makespan="NO_DUAL_METRICS_JSON"; planar_a="NONE"; planar_b="NONE"
  fi
  log "[$label] $(date +%H:%M:%S) wall=${waited}s A_status=${status_a:-NONE} B_status=${status_b:-NONE} A_picked=${picked_a:-NONE} B_picked=${picked_b:-NONE} A_tiltdet=${tiltdet_a:-0} B_tiltdet=${tiltdet_b:-0} A_tilt=${tilt_a:-0} B_tilt=${tilt_b:-0} A_deadlock=${dl_a:-0} B_deadlock=${dl_b:-0} A_retreat_unconf=${ret_a:-0} B_retreat_unconf=${ret_b:-0} min_peer_dist=${min_dist:-NONE} makespan_s=${makespan} planar_A_m=${planar_a} planar_B_m=${planar_b}"
  cp -f "$log_a" "$log_b" "$outdir/" 2>/dev/null
  hard_cleanup
  unset REPORT_SESSION_DIR
  log "=== $label: stopped $(date +%H:%M:%S) (avail=$(avail_mb)MB orphans=$(orphan_count)) ==="
}

log "TOPOLOGY MATRIX -- topologies: $TOPOLOGIES | seeds: $SEEDS | reps: $REPS -- $(date '+%F %T')"
log "frozen code (md5): $OUT_DIR/code_md5.txt"
for rep in $REPS; do
  log ""
  log "########## PASS $rep ########## $(date +%H:%M:%S)"
  for seed in $SEEDS; do
    for topo in $TOPOLOGIES; do
      run_one "dual_topology_${topo}_seed${seed}" "$rep"
    done
  done
done
log ""
log "ALL DONE $(date +%H:%M:%S) (avail=$(avail_mb)MB orphans=$(orphan_count))"
