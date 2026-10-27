#!/usr/bin/env bash
#
# Run ONE experiment profile end to end: tear down any previous Gazebo world,
# start a fresh one, wait for every service/controller, launch the robot
# node(s) with the profile's parameters, record metrics/rosbag/video, compute
# the report, and leave a fresh world running for the next profile.
#
#   experiments/run_experiment.sh <profile>        e.g. dual_topology_m2_seed37
#
# Environment:
#   HARBOT_WS                catkin workspace (default: ~/catkin_ws)
#   HARBOT_CALIBRATION_DIR   calibration + per-robot logs (default: ~/harbot_calibration)
#   HARBOT_GREENHOUSE_SDF    fallback greenhouse model if the launch file did not spawn it
#   REPORT_SESSION_DIR       if set, record a full report (trajectories, rosbag,
#                            screen video, metrics) into this directory
#
# The profile list below is the complete set used during development; the
# thesis experiments use sr_* (single robot), dual_equal..dual_speed_zone80
# (+ *_shortcut) for the work-split study, and dual_topology_{line,m1,m2,m3,
# full}_seed{37,41,45,54,162} for the 75-run topology matrix.

set -Eeuo pipefail

WORKSPACE="${HARBOT_WS:-$HOME/catkin_ws}"
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CALIB_DIR="${HARBOT_CALIBRATION_DIR:-$HOME/harbot_calibration}"
GREENHOUSE_SDF="${HARBOT_GREENHOUSE_SDF:-$HOME/.gazebo/models/harbot_greenhouse_visible/model.sdf}"
LOG_DIR="$WORKSPACE/test_logs"
WORLD_PID_FILE="$WORKSPACE/.fresh_world.pid"
PROFILE="${1:-}"
WORLD_LAUNCH_ARGS=()

if [[ -z "$PROFILE" ]]; then
  echo "Usage: $0 {smoke|dual_smoke|adaptive_pregrasp|full_global|dual_equal|dual_unequal_speed|dual_unequal_zone|dual_speed_zone70|dual_speed_zone80|dual_topology_shortcut|dual_scrambled|dual_head_on|dual_unequal_speed_shortcut|dual_unequal_zone_shortcut|dual_speed_zone70_shortcut|dual_speed_zone80_shortcut|dual_overlap_zone|dual_full_shared|dual_full_shared_speed85|dual_full_shared_speed50|dual_full_shared_speed25|dual_full_shared_seed7|dual_full_shared_seed99|dual_full_shared_basket5|dual_full_shared_radius_tight|dual_full_shared_radius_wide|dual_full_shared_finegrained|dual_full_shared_speed50_seed7|dual_full_shared_split3_7|dual_full_shared_split7_3|dual_full_shared_split2_8|dual_full_shared_split3_7_seed7|dual_full_shared_split7_3_speed50|dual_full_shared_split3_7_speed50|dual_full_shared_seed13|dual_full_shared_basket3|dual_full_shared_radius_tight_speed50|dual_full_shared_finegrained_seed7|dual_topology_full_seed37|dual_topology_full_seed41|dual_topology_full_seed45|dual_topology_full_seed54|dual_topology_full_seed162|dual_topology_path_seed37|dual_topology_path_seed41|dual_topology_path_seed45|dual_topology_path_seed54|dual_topology_path_seed162|dual_topology_line_seed37|dual_topology_line_seed41|dual_topology_line_seed45|dual_topology_line_seed54|dual_topology_line_seed162|dual_topology_m1_seed37|dual_topology_m1_seed41|dual_topology_m1_seed45|dual_topology_m1_seed54|dual_topology_m1_seed162|dual_topology_m2_seed37|dual_topology_m2_seed41|dual_topology_m2_seed45|dual_topology_m2_seed54|dual_topology_m2_seed162|dual_topology_m3_seed37|dual_topology_m3_seed41|dual_topology_m3_seed45|dual_topology_m3_seed54|dual_topology_m3_seed162|dual_topology_line_headon|dual_topology_line_headon_fast|dual_topology_line_headon_fast_p0|dual_topology_line_headon_fast_p1|dual_topology_line_headon_fast_p3|dual_topology_line_headon_fast_p4|p4}"
  exit 2
fi

# Do not allow two runs at the same time: a second run would kill the
# first run's world through WORLD_PID_FILE.
LOCK_FILE="$WORKSPACE/.run_experiment.lock"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "ERROR: another run_experiment.sh is already running (lock file: $LOCK_FILE)." >&2
  echo "Wait for it to finish (or stop it) before starting a new test -- running in parallel kills the other run's world." >&2
  exit 3
fi

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
mkdir -p "$LOG_DIR"

ACTIVE_WORLD_PID=""
REPORT_METRICS_PID=""
REPORT_METRICS_B_PID=""
REPORT_BAG_PID=""
REPORT_VIDEO_PID=""

stop_report_recorders() {
  local pid
  for pid in "$REPORT_METRICS_PID" "$REPORT_METRICS_B_PID" "$REPORT_BAG_PID" "$REPORT_VIDEO_PID"; do
    [[ -n "$pid" ]] || continue
    kill -INT "$pid" 2>/dev/null || true
  done
  for _ in {1..30}; do
    local alive=false
    for pid in "$REPORT_METRICS_PID" "$REPORT_METRICS_B_PID" "$REPORT_BAG_PID" "$REPORT_VIDEO_PID"; do
      [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null && alive=true
    done
    [[ "$alive" == false ]] && break
    sleep 0.2
  done
}

stop_process_group() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "$pid" 2>/dev/null || return 0
      sleep 0.2
    done
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  fi
}

stop_existing_world() {
  if [[ -f "$WORLD_PID_FILE" ]]; then
    local saved_pid
    saved_pid="$(tr -cd '0-9' < "$WORLD_PID_FILE")"
    stop_process_group "$saved_pid"
    rm -f "$WORLD_PID_FILE"
  fi

  # Clean stale processes from older manually started orchard launches.
  pkill -INT -f '[r]oslaunch.*harbot_full_greenhouse.launch' 2>/dev/null || true
  pkill -INT -f '[g]zserver' 2>/dev/null || true
  pkill -INT -f '[g]zclient' 2>/dev/null || true
  pkill -INT -f '[r]osmaster' 2>/dev/null || true
  pkill -INT -f '[r]oscore' 2>/dev/null || true

  # Gazebo and controller spawners may need several seconds to release shared
  # memory and the master port.  Never start the next world while gzserver or
  # rosmaster from the previous generation is still alive.
  for _ in {1..50}; do
    if ! pgrep -x gzserver >/dev/null 2>&1 \
       && ! pgrep -f '/opt/ros/noetic/bin/rosmaster' >/dev/null 2>&1; then
      break
    fi
    sleep 0.2
  done

  pkill -TERM -x gzserver 2>/dev/null || true
  pkill -TERM -x gzclient 2>/dev/null || true
  pkill -TERM -f '/opt/ros/noetic/bin/rosmaster' 2>/dev/null || true
  sleep 3
}

cleanup_on_exit() {
  local status=$?
  stop_report_recorders
  if [[ -n "$ACTIVE_WORLD_PID" ]]; then
    echo "Interrupted: stopping test world PID=$ACTIVE_WORLD_PID"
    stop_process_group "$ACTIVE_WORLD_PID"
  fi
  exit "$status"
}
trap cleanup_on_exit INT TERM ERR

wait_until() {
  local description="$1"
  local timeout="$2"
  shift 2
  local started=$SECONDS
  until "$@" >/dev/null 2>&1; do
    if (( SECONDS - started >= timeout )); then
      echo "ERROR: timed out waiting for $description (${timeout}s)"
      return 1
    fi
    if [[ -n "$ACTIVE_WORLD_PID" ]] && ! kill -0 "$ACTIVE_WORLD_PID" 2>/dev/null; then
      echo "ERROR: roslaunch exited while waiting for: $description"
      return 1
    fi
    sleep 2
  done
}

has_gazebo_service() {
  rosservice list | grep -q '^/gazebo/get_world_properties$'
}

has_spawn_service() {
  rosservice list | grep -q '^/gazebo/spawn_sdf_model$'
}

has_gazebo_model_services() {
  # The robot nodes wait for these two services at startup; make sure they
  # exist before launching them.
  rosservice list | grep -q '^/gazebo/get_model_state$' \
    && rosservice list | grep -q '^/gazebo/delete_model$'
}

has_move_group() {
  rosnode list | grep -q '^/robot/move_group$'
}

has_dual_move_group() {
  rosnode list | grep -q '^/robot/move_group$' \
    && rosnode list | grep -q '^/robot_b/move_group$'
}

has_dual_motion_controllers() {
  rostopic list | grep -q '^/robot/arm/pos_traj_controller/follow_joint_trajectory/status$' \
    && rostopic list | grep -q '^/robot/gripper/pos_traj_controller/follow_joint_trajectory/status$' \
    && rostopic list | grep -q '^/robot_b/arm/pos_traj_controller/follow_joint_trajectory/status$' \
    && rostopic list | grep -q '^/robot_b/gripper/pos_traj_controller/follow_joint_trajectory/status$'
}

has_odom() {
  rostopic list | grep -q '^/robot/robotnik_base_control/odom_gt$'
}

greenhouse_exists() {
  rosservice call /gazebo/get_world_properties '{}' 2>/dev/null \
    | grep -q 'harbot_greenhouse_visible'
}

# Shared-map defaults for the dual_full_shared_* profiles: whole map shared,
# all 28 shortcuts active, targets split randomly with a fixed seed.
# Profiles then adjust SPEED_A/SPEED_B/BASKET_CAPACITY.
apply_full_shared_topology_defaults() {
  local seed="${1:-42}"
  # split: number of the 10 targets given to robot A (rest to robot B), default 5.
  local split="${2:-5}"
  ZONE_A="[]"
  ZONE_B="[]"
  SHORTCUT_PAIRS="[$(python3 -c "print(','.join(\"'P%d-P%d:%d'\" % (p, p + 1, k) for p in range(4) for k in range(1, 8)))")]"
  read -r TARGETS_A TARGETS_B <<< "$(python3 -c "
import random
random.seed($seed)
targets = ['T%d' % i for i in range(1, 11)]
random.shuffle(targets)
split = $split
a = sorted(targets[:split], key=lambda t: int(t[1:]))
b = sorted(targets[split:], key=lambda t: int(t[1:]))
print(\"['\" + \"','\".join(a) + \"']\", \"['\" + \"','\".join(b) + \"']\")
")"
}

# apply_topology_comparison_defaults(seed, topology): the topology matrix.
# Both robots share the whole territory at the same speed, so the workspace
# graph (line, m1, m2, m3, full) is the only variable.
#
# 12 targets (T1-T12, config/orchard_targets_meta.csv), fixed 6/6 split,
# BASKET_CAPACITY=6 so there are no extra depot trips.
# The seed selects which 6 targets each robot gets. Seeds 37/41/45/54/162
# were chosen from ~300 candidates by the minimum distance between targets
# of the same robot (all reach 5.40 m).
apply_topology_comparison_defaults() {
  local seed="${1:?apply_topology_comparison_defaults: seed is required}"
  local topology="${2:?apply_topology_comparison_defaults: topology is required (full|path|line|m1|m2|m3)}"
  ZONE_A="[]"
  ZONE_B="[]"
  # "ladder": both endcaps connected. "line": minimal serpentine
  # (orchard_topology.line_edges()).
  TOPOLOGY_MODE="ladder"
  # Reset on every call; only "line" overrides it.
  PROST_HORIZON=20
  # Reset on every call; only m1/m2/m3 override it.
  TOPOLOGY_REDUNDANT_PAIRS="[]"
  case "$topology" in
    full)
      SHORTCUT_PAIRS="[$(python3 -c "print(','.join(\"'P%d-P%d:%d'\" % (p, p + 1, k) for p in range(4) for k in range(1, 8)))")]"
      ;;
    path)
      SHORTCUT_PAIRS="[]"
      ;;
    line)
      # No shortcuts and no redundant endcap links.
      SHORTCUT_PAIRS="[]"
      TOPOLOGY_MODE="line"
      # 40 stations in one line, longest path 39 hops; 45 leaves some margin.
      PROST_HORIZON=45
      ;;
    m1|m2|m3)
      # m1/m2/m3: between line and full, one passage pair at a time is upgraded
      # to full connectivity (both endcaps and all 7 shortcuts), starting from
      # the centre: m1={P1-P2}, m2={P1-P2,P2-P3}, m3={P0-P1,P1-P2,P2-P3}.
      # P3-P4 is upgraded only in "full".
      local pairs
      case "$topology" in
        m1) pairs="1" ;;
        m2) pairs="1,2" ;;
        m3) pairs="0,1,2" ;;
      esac
      TOPOLOGY_REDUNDANT_PAIRS="[$pairs]"
      SHORTCUT_PAIRS="[$(python3 -c "
pairs = [$pairs]
print(','.join(\"'P%d-P%d:%d'\" % (p, p + 1, k) for p in pairs for k in range(1, 8)))
")]"
      TOPOLOGY_MODE="line"
      # Still based on the serpentine, so the same horizon as "line".
      PROST_HORIZON=45
      ;;
    *)
      echo "apply_topology_comparison_defaults: unknown topology '$topology' (expected full|path|line|m1|m2|m3)" >&2
      exit 1
      ;;
  esac
  read -r TARGETS_A TARGETS_B <<< "$(python3 -c "
import random
random.seed($seed)
targets = ['T%d' % i for i in range(1, 13)]
random.shuffle(targets)
a = sorted(targets[:6], key=lambda t: int(t[1:]))
b = sorted(targets[6:], key=lambda t: int(t[1:]))
print(\"['\" + \"','\".join(a) + \"']\", \"['\" + \"','\".join(b) + \"']\")
")"
  BASKET_CAPACITY=6
}

start_world() {
  local label="$1"
  WORLD_LOG="$LOG_DIR/world_${label}_$(date +%Y%m%d_%H%M%S).txt"
  echo "Starting a new world. World log: $WORLD_LOG"
  # 9>&- : the world keeps running after this script exits, so it must not
  # inherit the lock fd, otherwise the lock is never released.
  nohup setsid roslaunch rbkairos_sim_bringup harbot_full_greenhouse.launch \
    "${WORLD_LAUNCH_ARGS[@]}" \
    </dev/null >"$WORLD_LOG" 2>&1 9>&- &
  ACTIVE_WORLD_PID=$!
  echo "$ACTIVE_WORLD_PID" > "$WORLD_PID_FILE"

  wait_until "the Gazebo service" 120 has_gazebo_service
  wait_until "the Gazebo spawn service" 120 has_spawn_service
  wait_until "odometry" 120 has_odom

  # The launch spawn process can still be inside the Gazebo service even
  # though the service itself is already visible.  Give it a short grace
  # period before attempting the fallback with the same model name.
  for _ in {1..15}; do
    greenhouse_exists && break
    sleep 1
  done

  if ! greenhouse_exists; then
    echo "Greenhouse was not spawned by the launch file; using the fallback spawn."
    rosrun gazebo_ros spawn_model \
      -sdf \
      -file "$GREENHOUSE_SDF" \
      -model harbot_greenhouse_visible \
      -x 0 -y 0 -z 0
  fi
  wait_until "the greenhouse model" 30 greenhouse_exists
  echo "Greenhouse confirmed."
}

common_manager_args=(
  _continuous_passage_motion:=true
  _continuous_forward_speed:=0.44
  _outer_lane_loaded_speed:=0.28
  _outer_lane_extra_clearance_right:=0.90
  _continuous_turn_in_place_threshold:=0.35
  _continuous_max_cross_track:=0.35
  _continuous_max_rejoin_cross_track:=0.65
  _holonomic_transfer:=true
  _holonomic_transfer_speed:=0.30
  _holonomic_transfer_turn:=0.30
  _smart_holonomic_entry:=true
  _smart_entry_max_vx:=0.18
  _smart_entry_max_vy:=0.18
  _smart_entry_max_w:=0.30
  _smart_final_max_vx:=0.18
  _smart_final_max_vy:=0.18
  _smart_final_max_w:=0.24
  _post_pick_holonomic_rejoin:=true
  _post_pick_rejoin_tolerance:=0.30
  _continuous_pick_departure:=true
  _moveit_planner_id:=RRTConnect
  _moveit_velocity_scaling:=0.30
  _moveit_acceleration_scaling:=0.15
  _moveit_planning_time:=3.0
  _moveit_planning_attempts:=3
  _adaptive_pregrasp_planning:=true
  _pregrasp_fast_planning_time:=0.60
  _pregrasp_fast_planning_attempts:=1
  _cartesian_goal_acceptance:=0.012
  _fruit_pose_max_xy_deviation:=0.40
  _fruit_pose_max_z_deviation:=0.30
  _smart_pick_max_base_approach_distance:=2.25
  _max_pick_attempts_per_target:=3
  _tilt_abort_angle_deg:=16.0
  _tilt_recovery_angle_deg:=8.0
  _tilt_recovery_stable_duration:=0.40
  # Tilt recovery: recover below 16.0 - 1.5 = 14.5 deg within 20 s.
  # Passed explicitly because these launch args override the code defaults.
  _tilt_recovery_timeout:=20.0
  _tilt_recovery_hysteresis_deg:=1.5
  _tilt_hard_abort_angle_deg:=18.0
  _basket_capacity:=8
)

DUAL_MODE=false
# Default graph for profiles that do not use the topology matrix.
TOPOLOGY_MODE="ladder"
# Only used with TOPOLOGY_MODE="line" (m1/m2/m3).
TOPOLOGY_REDUNDANT_PAIRS="[]"
# PROST horizon. "line" needs a much longer one (24+ hops for a single
# depot round trip); with 20, PROST found no reachable target and only
# returned noop().
PROST_HORIZON=20
# Start and depot locations (overridden only by the *_headon_fast profiles).
INITIAL_LOC_A="P0_I1"
DEPOT_LOC_A="P0_I1"
INITIAL_LOC_B="P4_I8"
DEPOT_LOC_B="P4_I8"
# Optional teleport after spawn via /gazebo/set_model_state. The spawn pose
# in the launch file stays unchanged; empty means no teleport.
TELEPORT_X_A=""
TELEPORT_Y_A=""
TELEPORT_QZ_A=""
TELEPORT_QW_A=""
TELEPORT_X_B=""
TELEPORT_Y_B=""
TELEPORT_QZ_B=""
TELEPORT_QW_B=""

case "$PROFILE" in
  smoke)
    LOG_PREFIX="auto_world_smoke"
    NEED_MOVEIT=false
    TEST_CMD=(
      rosservice call /gazebo/get_world_properties '{}'
    )
    ;;
  dual_smoke)
    LOG_PREFIX="auto_dual_smoke"
    NEED_MOVEIT=true
    DUAL_MODE=true
    WORLD_LAUNCH_ARGS=(
      launch_robot_b:=true
      gazebo_gui:="${DUAL_GAZEBO_GUI:-false}"
      launch_static_map:=true
      launch_map_rviz:="${DUAL_MAP_RVIZ:-false}"
    )
    TEST_CMD=(
      bash -lc
      "rostopic echo -n 1 /robot/robotnik_base_control/odom_gt >/dev/null && rostopic echo -n 1 /robot_b/robotnik_base_control/odom_gt >/dev/null && rosnode list | grep -q '^/robot/move_group$' && rosnode list | grep -q '^/robot_b/move_group$'"
    )
    ;;
  adaptive_pregrasp)
    LOG_PREFIX="auto_adaptive_pregrasp"
    NEED_MOVEIT=true
    TEST_CMD=(
      rosrun harbot_orchard_picker online_orchard_manager.py
      _max_cycles:=1
      _route_strategy:=time_aware
      _auto_drop:=false
      "${common_manager_args[@]}"
    )
    ;;
  full_global)
    LOG_PREFIX="auto_full_global"
    NEED_MOVEIT=true
    TEST_CMD=(
      rosrun harbot_orchard_picker online_orchard_manager.py
      _max_cycles:=30
      _route_strategy:=time_aware_global
      _route_passage_edge_cost:=10.2
      _route_transfer_first_cost:=20.4
      _route_transfer_continuation_cost:=13.2
      _auto_drop:=true
      _require_all_targets:=true
      _physical_drop_delete_fruits:=true
      "${common_manager_args[@]}"
    )
    ;;

  # ---------------------------------------------------------------------
  # Single-robot study: sr_* profiles.
  #
  # A target subset is selected with _allowed_targets. The baseline is
  # full_global (12 targets, basket 8, time_aware_global, 0.44 m/s).
  # Overrides replace one entry in the argument array (sr_args_with)
  # instead of appending a second "_param:=".
  # ---------------------------------------------------------------------
  sr_route_prost|sr_route_timeaware|sr_basket4|sr_basket12|sr_targets6|sr_nocontinuous|sr_speed30|sr_speed60|sr_arr1_prost|sr_arr2_global|sr_arr2_prost|sr_arr3_global|sr_arr3_prost|sr_basket24|sr_speed75|sr_targets3|sr_targets9)
    LOG_PREFIX="auto_${PROFILE}"
    NEED_MOVEIT=true
    sr_args=("${common_manager_args[@]}")
    sr_route="time_aware_global"
    sr_allowed=""
    sr_args_with() {
      local key="$1" val="$2" i
      for i in "${!sr_args[@]}"; do
        if [[ "${sr_args[$i]}" == "${key}:="* ]]; then
          sr_args[$i]="${key}:=${val}"
          return 0
        fi
      done
      echo "sr_args_with: parameter '$key' not found in common_manager_args" >&2
      return 1
    }
    case "$PROFILE" in
      sr_route_prost)     sr_route="prost" ;;
      sr_route_timeaware) sr_route="time_aware" ;;
      sr_basket4)         sr_args_with _basket_capacity 4 ;;
      sr_basket12)        sr_args_with _basket_capacity 12 ;;
      sr_targets6)        sr_allowed="['T1','T3','T5','T7','T9','T11']" ;;
      sr_nocontinuous)    sr_args_with _continuous_passage_motion false ;;
      sr_speed30)         sr_args_with _continuous_forward_speed 0.30 ;;
      sr_speed60)         sr_args_with _continuous_forward_speed 0.60 ;;

      # --- target arrangement (always 6 targets, only geometry changes) ---
      # R1 "spread-south" = same set as sr_targets6: all 5 passages, y from
      #                     -12.5 to -2.1 (southern half)
      # R2 "compact"      = only P1 and P3, full y range (-14.0 .. +12.3)
      # R3 "zig-zag"      = all 5 passages, alternating north/south, full y
      # R1 with time_aware_global is sr_targets6.
      sr_arr1_prost)
        sr_allowed="['T1','T3','T5','T7','T9','T11']"; sr_route="prost" ;;
      sr_arr2_global)
        sr_allowed="['T3','T4','T11','T7','T8','T12']" ;;
      sr_arr2_prost)
        sr_allowed="['T3','T4','T11','T7','T8','T12']"; sr_route="prost" ;;
      sr_arr3_global)
        sr_allowed="['T2','T11','T6','T12','T10','T9']" ;;
      sr_arr3_prost)
        sr_allowed="['T2','T11','T6','T12','T10','T9']"; sr_route="prost" ;;

      # --- scaling: nested sets T3 c T6 c T9 c T12 ---
      # 3 targets = 6 fruits = 1 depot trip; 9 targets = 18 fruits = 3 trips.
      sr_targets3)        sr_allowed="['T1','T5','T9']" ;;
      sr_targets9)        sr_allowed="['T1','T3','T5','T7','T9','T11','T2','T6','T10']" ;;

      # linearity check of the depot model at the low end: 24 fruits / 24 = 1 trip
      sr_basket24)        sr_args_with _basket_capacity 24 ;;
      # boundary probe above the saturation point (0.44) - does reliability drop?
      sr_speed75)         sr_args_with _continuous_forward_speed 0.75 ;;
    esac
    TEST_CMD=(
      rosrun harbot_orchard_picker online_orchard_manager.py
      _max_cycles:=30
      "_route_strategy:=$sr_route"
      _route_passage_edge_cost:=10.2
      _route_transfer_first_cost:=20.4
      _route_transfer_continuation_cost:=13.2
      _auto_drop:=true
      _require_all_targets:=true
      _physical_drop_delete_fruits:=true
      "${sr_args[@]}"
    )
    if [[ -n "$sr_allowed" ]]; then
      TEST_CMD+=("_allowed_targets:=$sr_allowed")
    fi
    ;;

  dual_equal|dual_unequal_speed|dual_unequal_zone|dual_speed_zone70|dual_speed_zone80|dual_topology_shortcut|dual_scrambled|dual_head_on|dual_unequal_speed_shortcut|dual_unequal_zone_shortcut|dual_speed_zone70_shortcut|dual_speed_zone80_shortcut|dual_overlap_zone|dual_full_shared|dual_full_shared_speed85|dual_full_shared_speed50|dual_full_shared_speed25|dual_full_shared_seed7|dual_full_shared_seed99|dual_full_shared_basket5|dual_full_shared_radius_tight|dual_full_shared_radius_wide|dual_full_shared_finegrained|dual_full_shared_speed50_seed7|dual_full_shared_split3_7|dual_full_shared_split7_3|dual_full_shared_split2_8|dual_full_shared_split3_7_seed7|dual_full_shared_split7_3_speed50|dual_full_shared_split3_7_speed50|dual_full_shared_seed13|dual_full_shared_basket3|dual_full_shared_radius_tight_speed50|dual_full_shared_finegrained_seed7|dual_topology_full_seed37|dual_topology_full_seed41|dual_topology_full_seed45|dual_topology_full_seed54|dual_topology_full_seed162|dual_topology_path_seed37|dual_topology_path_seed41|dual_topology_path_seed45|dual_topology_path_seed54|dual_topology_path_seed162|dual_topology_line_seed37|dual_topology_line_seed41|dual_topology_line_seed45|dual_topology_line_seed54|dual_topology_line_seed162|dual_topology_m1_seed37|dual_topology_m1_seed41|dual_topology_m1_seed45|dual_topology_m1_seed54|dual_topology_m1_seed162|dual_topology_m2_seed37|dual_topology_m2_seed41|dual_topology_m2_seed45|dual_topology_m2_seed54|dual_topology_m2_seed162|dual_topology_m3_seed37|dual_topology_m3_seed41|dual_topology_m3_seed45|dual_topology_m3_seed54|dual_topology_m3_seed162|dual_topology_line_headon|dual_topology_line_headon_fast|dual_topology_line_headon_fast_p0|dual_topology_line_headon_fast_p1|dual_topology_line_headon_fast_p3|dual_topology_line_headon_fast_p4)
    LOG_PREFIX="auto_${PROFILE}"
    NEED_MOVEIT=true
    DUAL_MODE=true
    WORLD_LAUNCH_ARGS=(
      launch_robot_b:=true
      gazebo_gui:="${DUAL_GAZEBO_GUI:-false}"
      launch_static_map:=true
      launch_map_rviz:="${DUAL_MAP_RVIZ:-false}"
    )
    # Zones follow passage boundaries (P0..P4 x I1..I8, 40 locations), which
    # gives exact 50/50, 70/30 and 80/20 splits of locations and targets.
    ZONE_A="['P0_I1','P0_I2','P0_I3','P0_I4','P0_I5','P0_I6','P0_I7','P0_I8','P1_I1','P1_I2','P1_I3','P1_I4','P1_I5','P1_I6','P1_I7','P1_I8','P2_I1','P2_I2','P2_I3','P2_I4']"
    ZONE_B="['P2_I5','P2_I6','P2_I7','P2_I8','P3_I1','P3_I2','P3_I3','P3_I4','P3_I5','P3_I6','P3_I7','P3_I8','P4_I1','P4_I2','P4_I3','P4_I4','P4_I5','P4_I6','P4_I7','P4_I8']"
    TARGETS_A="['T1','T2','T3','T4','T5']"
    TARGETS_B="['T6','T7','T8','T9','T10']"
    SPEED_A=0.44
    SPEED_B=0.44
    # Mid-row shortcuts, empty unless a profile sets them.
    SHORTCUT_PAIRS="[]"
    # Default basket capacity (same as in common_manager_args).
    BASKET_CAPACITY=8
    # Defaults, only dual_head_on changes these.
    CONTINUOUS_PASSAGE_MOTION=true
    PEER_SAFETY_RADIUS=1.75
    case "$PROFILE" in
      dual_scrambled)
        # Same speed for both robots, interleaved targets (odd to A, even to B) on
        # the full map, so both robots travel through the whole orchard and their
        # paths cross. A random split of locations was not used because it would
        # often disconnect the graph.
        ZONE_A="[]"
        ZONE_B="[]"
        TARGETS_A="['T1','T3','T5','T7','T9']"
        TARGETS_B="['T2','T4','T6','T8','T10']"
        ;;
      dual_head_on)
        # Forced head-on encounter in P2. Default spawn/depot (P0_I1 and P4_I8);
        # each robot gets one target in P2:
        #   robot   -> T6 (P2_I7): P0_I1 -> P1_I1 -> P2_I1, then along P2 to I7
        #   robot_b -> T5 (P2_I3): P4_I8 -> P3_I8 -> P2_I8, then along P2 to I3
        # Same speed, so both enter P2 at about the same time from opposite ends.
        #
        # Test-only settings:
        #   continuous_passage_motion=false: one clearance check per hop instead of
        #     one per whole segment;
        #   peer_safety_radius=3.0 (default 1.75): larger window for the check.
        ZONE_A="[]"
        ZONE_B="[]"
        TARGETS_A="['T6']"
        TARGETS_B="['T5']"
        # DUAL_HEAD_ON_CONTINUOUS=true runs the same test with continuous segments.
        CONTINUOUS_PASSAGE_MOTION="${DUAL_HEAD_ON_CONTINUOUS:-false}"
        PEER_SAFETY_RADIUS=3.0
        ;;
      dual_topology_shortcut)
        # Same zones and speeds as dual_equal, plus two shortcuts (one per zone),
        # so the shortcuts are the only difference.
        SHORTCUT_PAIRS="['P1-P2:2','P2-P3:6']"
        # Basket 10 so all 5 targets fit in one tour; with 8 the robot always
        # returned to the depot before the shortcut target and never used the
        # shortcut.
        BASKET_CAPACITY=10
        ;;
      dual_unequal_speed)
        # Robot A = vmax, Robot B = vmax/2, equal (50/50) zones.
        SPEED_B=0.22
        ;;
      dual_unequal_zone)
        # Equal (vmax) speeds, unequal (60/40) zones: isolates the zone-size effect.
        ZONE_A="['P0_I1','P0_I2','P0_I3','P0_I4','P0_I5','P0_I6','P0_I7','P0_I8','P1_I1','P1_I2','P1_I3','P1_I4','P1_I5','P1_I6','P1_I7','P1_I8','P2_I1','P2_I2','P2_I3','P2_I4','P2_I5','P2_I6','P2_I7','P2_I8']"
        ZONE_B="['P3_I1','P3_I2','P3_I3','P3_I4','P3_I5','P3_I6','P3_I7','P3_I8','P4_I1','P4_I2','P4_I3','P4_I4','P4_I5','P4_I6','P4_I7','P4_I8']"
        TARGETS_A="['T1','T2','T3','T4','T5','T6']"
        TARGETS_B="['T7','T8','T9','T10']"
        ;;
      dual_speed_zone70)
        # Robot A = vmax, Robot B = vmax/2, 70/30 zones (A gets the bigger share).
        SPEED_B=0.22
        ZONE_A="['P0_I1','P0_I2','P0_I3','P0_I4','P0_I5','P0_I6','P0_I7','P0_I8','P1_I1','P1_I2','P1_I3','P1_I4','P1_I5','P1_I6','P1_I7','P1_I8','P2_I1','P2_I2','P2_I3','P2_I4','P2_I5','P2_I6','P2_I7','P2_I8','P3_I1','P3_I2','P3_I3','P3_I4']"
        ZONE_B="['P3_I5','P3_I6','P3_I7','P3_I8','P4_I1','P4_I2','P4_I3','P4_I4','P4_I5','P4_I6','P4_I7','P4_I8']"
        TARGETS_A="['T1','T2','T3','T4','T5','T6','T7']"
        TARGETS_B="['T8','T9','T10']"
        ;;
      dual_speed_zone80)
        # Robot A = vmax, Robot B = vmax/2, 80/20 zones (A gets the bigger share).
        SPEED_B=0.22
        ZONE_A="['P0_I1','P0_I2','P0_I3','P0_I4','P0_I5','P0_I6','P0_I7','P0_I8','P1_I1','P1_I2','P1_I3','P1_I4','P1_I5','P1_I6','P1_I7','P1_I8','P2_I1','P2_I2','P2_I3','P2_I4','P2_I5','P2_I6','P2_I7','P2_I8','P3_I1','P3_I2','P3_I3','P3_I4','P3_I5','P3_I6','P3_I7','P3_I8']"
        ZONE_B="['P4_I1','P4_I2','P4_I3','P4_I4','P4_I5','P4_I6','P4_I7','P4_I8']"
        TARGETS_A="['T1','T2','T3','T4','T5','T6','T7','T8']"
        TARGETS_B="['T9','T10']"
        ;;
      # Speed/zone series repeated with two shortcuts (basket 10), to check
      # whether shortcuts change the work-split result.
      dual_unequal_speed_shortcut)
        SPEED_B=0.22
        SHORTCUT_PAIRS="['P1-P2:2','P2-P3:6']"
        BASKET_CAPACITY=10
        ;;
      dual_unequal_zone_shortcut)
        ZONE_A="['P0_I1','P0_I2','P0_I3','P0_I4','P0_I5','P0_I6','P0_I7','P0_I8','P1_I1','P1_I2','P1_I3','P1_I4','P1_I5','P1_I6','P1_I7','P1_I8','P2_I1','P2_I2','P2_I3','P2_I4','P2_I5','P2_I6','P2_I7','P2_I8']"
        ZONE_B="['P3_I1','P3_I2','P3_I3','P3_I4','P3_I5','P3_I6','P3_I7','P3_I8','P4_I1','P4_I2','P4_I3','P4_I4','P4_I5','P4_I6','P4_I7','P4_I8']"
        TARGETS_A="['T1','T2','T3','T4','T5','T6']"
        TARGETS_B="['T7','T8','T9','T10']"
        SHORTCUT_PAIRS="['P1-P2:2','P2-P3:6']"
        BASKET_CAPACITY=10
        ;;
      dual_speed_zone70_shortcut)
        SPEED_B=0.22
        ZONE_A="['P0_I1','P0_I2','P0_I3','P0_I4','P0_I5','P0_I6','P0_I7','P0_I8','P1_I1','P1_I2','P1_I3','P1_I4','P1_I5','P1_I6','P1_I7','P1_I8','P2_I1','P2_I2','P2_I3','P2_I4','P2_I5','P2_I6','P2_I7','P2_I8','P3_I1','P3_I2','P3_I3','P3_I4']"
        ZONE_B="['P3_I5','P3_I6','P3_I7','P3_I8','P4_I1','P4_I2','P4_I3','P4_I4','P4_I5','P4_I6','P4_I7','P4_I8']"
        TARGETS_A="['T1','T2','T3','T4','T5','T6','T7']"
        TARGETS_B="['T8','T9','T10']"
        SHORTCUT_PAIRS="['P1-P2:2','P2-P3:6']"
        BASKET_CAPACITY=10
        ;;
      dual_speed_zone80_shortcut)
        SPEED_B=0.22
        ZONE_A="['P0_I1','P0_I2','P0_I3','P0_I4','P0_I5','P0_I6','P0_I7','P0_I8','P1_I1','P1_I2','P1_I3','P1_I4','P1_I5','P1_I6','P1_I7','P1_I8','P2_I1','P2_I2','P2_I3','P2_I4','P2_I5','P2_I6','P2_I7','P2_I8','P3_I1','P3_I2','P3_I3','P3_I4','P3_I5','P3_I6','P3_I7','P3_I8']"
        ZONE_B="['P4_I1','P4_I2','P4_I3','P4_I4','P4_I5','P4_I6','P4_I7','P4_I8']"
        TARGETS_A="['T1','T2','T3','T4','T5','T6','T7','T8']"
        TARGETS_B="['T9','T10']"
        SHORTCUT_PAIRS="['P1-P2:2','P2-P3:6']"
        BASKET_CAPACITY=10
        ;;
      dual_overlap_zone)
        # Both robots share passage P2, each with one target inside it (robot -> T6,
        # robot_b -> T5) next to its own targets. Default coordination parameters.
        ZONE_A="['P0_I1','P0_I2','P0_I3','P0_I4','P0_I5','P0_I6','P0_I7','P0_I8','P1_I1','P1_I2','P1_I3','P1_I4','P1_I5','P1_I6','P1_I7','P1_I8','P2_I1','P2_I2','P2_I3','P2_I4','P2_I5','P2_I6','P2_I7','P2_I8']"
        ZONE_B="['P2_I1','P2_I2','P2_I3','P2_I4','P2_I5','P2_I6','P2_I7','P2_I8','P3_I1','P3_I2','P3_I3','P3_I4','P3_I5','P3_I6','P3_I7','P3_I8','P4_I1','P4_I2','P4_I3','P4_I4','P4_I5','P4_I6','P4_I7','P4_I8']"
        TARGETS_A="['T1','T2','T3','T4','T6']"
        TARGETS_B="['T5','T7','T8','T9','T10']"
        ;;
      dual_full_shared)
        # All 28 shortcuts active at once, whole map shared, random 5/5 target
        # split (seed 42), equal speed, basket 10.
        apply_full_shared_topology_defaults 42
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_speed85)
        # Mild speed imbalance (0.85x).
        apply_full_shared_topology_defaults 42
        SPEED_B=0.374
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_speed50)
        # Speed ratio 0.5x.
        apply_full_shared_topology_defaults 42
        SPEED_B=0.22
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_speed25)
        # Speed ratio 0.25x.
        apply_full_shared_topology_defaults 42
        SPEED_B=0.11
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_seed7)
        # Different random target arrangement, equal speed.
        apply_full_shared_topology_defaults 7
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_seed99)
        # Third arrangement sample.
        apply_full_shared_topology_defaults 99
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_basket5)
        # Smaller basket: two depot returns per robot.
        apply_full_shared_topology_defaults 42
        BASKET_CAPACITY=5
        ;;
      dual_full_shared_radius_tight)
        # peer_safety_radius 1.0 instead of 1.75.
        apply_full_shared_topology_defaults 42
        BASKET_CAPACITY=10
        PEER_SAFETY_RADIUS=1.0
        ;;
      dual_full_shared_radius_wide)
        # peer_safety_radius 3.0.
        apply_full_shared_topology_defaults 42
        BASKET_CAPACITY=10
        PEER_SAFETY_RADIUS=3.0
        ;;
      dual_full_shared_finegrained)
        # continuous_passage_motion:=false on the shared map.
        apply_full_shared_topology_defaults 42
        BASKET_CAPACITY=10
        CONTINUOUS_PASSAGE_MOTION=false
        ;;
      dual_full_shared_speed50_seed7)
        # Speed 0.5x combined with arrangement seed 7.
        apply_full_shared_topology_defaults 7
        SPEED_B=0.22
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_split3_7)
        # Unequal workload: robot A 3 targets, robot B 7, equal speed.
        apply_full_shared_topology_defaults 42 3
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_split7_3)
        # Mirror: robot A 7, robot B 3.
        apply_full_shared_topology_defaults 42 7
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_split2_8)
        # Extreme workload imbalance (2 vs 8).
        apply_full_shared_topology_defaults 42 2
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_split3_7_seed7)
        # 3/7 split with a different arrangement (seed 7).
        apply_full_shared_topology_defaults 7 3
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_split7_3_speed50)
        # Slower robot B (0.5x) gets fewer targets (3 of 10).
        apply_full_shared_topology_defaults 42 7
        SPEED_B=0.22
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_split3_7_speed50)
        # Slower robot B gets more targets (7 of 10).
        apply_full_shared_topology_defaults 42 3
        SPEED_B=0.22
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_seed13)
        # Another arrangement sample (seed 13).
        apply_full_shared_topology_defaults 13
        BASKET_CAPACITY=10
        ;;
      dual_full_shared_basket3)
        # Basket 3: three depot returns per robot.
        apply_full_shared_topology_defaults 42
        BASKET_CAPACITY=3
        ;;
      dual_full_shared_radius_tight_speed50)
        # peer_safety_radius 1.0 with speed ratio 0.5x.
        apply_full_shared_topology_defaults 42
        SPEED_B=0.22
        BASKET_CAPACITY=10
        PEER_SAFETY_RADIUS=1.0
        ;;
      dual_full_shared_finegrained_seed7)
        # continuous_passage_motion:=false with arrangement seed 7.
        apply_full_shared_topology_defaults 7
        BASKET_CAPACITY=10
        CONTINUOUS_PASSAGE_MOTION=false
        ;;
      dual_topology_full_seed37|dual_topology_full_seed41|dual_topology_full_seed45|dual_topology_full_seed54|dual_topology_full_seed162|dual_topology_path_seed37|dual_topology_path_seed41|dual_topology_path_seed45|dual_topology_path_seed54|dual_topology_path_seed162|dual_topology_line_seed37|dual_topology_line_seed41|dual_topology_line_seed45|dual_topology_line_seed54|dual_topology_line_seed162|dual_topology_m1_seed37|dual_topology_m1_seed41|dual_topology_m1_seed45|dual_topology_m1_seed54|dual_topology_m1_seed162|dual_topology_m2_seed37|dual_topology_m2_seed41|dual_topology_m2_seed45|dual_topology_m2_seed54|dual_topology_m2_seed162|dual_topology_m3_seed37|dual_topology_m3_seed41|dual_topology_m3_seed45|dual_topology_m3_seed54|dual_topology_m3_seed162)
        # Topology matrix: line / m1 / m2 / m3 / full (+ "path"), seeds
        # 37/41/45/54/162. See apply_topology_comparison_defaults().
        _seed="${PROFILE##*seed}"
        case "$PROFILE" in
          dual_topology_full_*) _topology=full ;;
          dual_topology_path_*) _topology=path ;;
          dual_topology_line_*) _topology=line ;;
          dual_topology_m1_*) _topology=m1 ;;
          dual_topology_m2_*) _topology=m2 ;;
          dual_topology_m3_*) _topology=m3 ;;
        esac
        apply_topology_comparison_defaults "$_seed" "$_topology"
        ;;
      dual_topology_line_headon)
        # Deterministic head-on on the line topology: each robot gets one target at
        # the far end of the line (robot -> T10 at P4_I6, robot_b -> T1 at P0_I3),
        # so they meet around P2.
        ZONE_A="[]"
        ZONE_B="[]"
        SHORTCUT_PAIRS="[]"
        TOPOLOGY_MODE="line"
        PROST_HORIZON=45
        TARGETS_A="['T10']"
        TARGETS_B="['T1']"
        BASKET_CAPACITY=2
        ;;
      dual_topology_line_headon_fast)
        # Same encounter as dual_topology_line_headon, but the robots are teleported
        # to the ends of P2 to save ~230 s of driving per test.
        ZONE_A="[]"
        ZONE_B="[]"
        SHORTCUT_PAIRS="[]"
        TOPOLOGY_MODE="line"
        PROST_HORIZON=45
        TARGETS_A="['T10']"
        TARGETS_B="['T1']"
        BASKET_CAPACITY=2
        INITIAL_LOC_A="P2_I1"
        DEPOT_LOC_A="P2_I1"
        INITIAL_LOC_B="P2_I8"
        DEPOT_LOC_B="P2_I8"
        # P2_I1/P2_I8 coordinates from the calibration file
        # (orchard_map_robot.json); yaw as in the launch file.
        TELEPORT_X_A="-0.5375"
        TELEPORT_Y_A="-12.7520"
        TELEPORT_QZ_A="0.7071068"
        TELEPORT_QW_A="0.7071068"
        TELEPORT_X_B="-0.5204"
        TELEPORT_Y_B="12.5371"
        TELEPORT_QZ_B="-0.7071068"
        TELEPORT_QW_B="0.7071068"
        ;;
      dual_topology_line_headon_fast_p1)
        # Same as dual_topology_line_headon_fast, staged in P1.
        # robot: P1_I1 -> P1_I8 -> T5 (P2_I3); robot_b: P1_I8 -> T11 (P1_I1).
        ZONE_A="[]"
        ZONE_B="[]"
        SHORTCUT_PAIRS="[]"
        TOPOLOGY_MODE="line"
        PROST_HORIZON=45
        TARGETS_A="['T5']"
        TARGETS_B="['T11']"
        BASKET_CAPACITY=2
        INITIAL_LOC_A="P1_I1"
        DEPOT_LOC_A="P1_I1"
        INITIAL_LOC_B="P1_I8"
        DEPOT_LOC_B="P1_I8"
        # P1_I1/P1_I8 coordinates from the calibration file.
        TELEPORT_X_A="-4.5563"
        TELEPORT_Y_A="-12.7480"
        TELEPORT_QZ_A="0.7071068"
        TELEPORT_QW_A="0.7071068"
        TELEPORT_X_B="-4.5392"
        TELEPORT_Y_B="12.5411"
        TELEPORT_QZ_B="-0.7071068"
        TELEPORT_QW_B="0.7071068"
        ;;
      dual_topology_line_headon_fast_p0)
        # Staged in P0 (outer row); both targets inside P0 (T2/T1) with
        # overlapping spans.
        ZONE_A="[]"
        ZONE_B="[]"
        SHORTCUT_PAIRS="[]"
        TOPOLOGY_MODE="line"
        PROST_HORIZON=45
        TARGETS_A="['T2']"
        TARGETS_B="['T1']"
        BASKET_CAPACITY=2
        INITIAL_LOC_A="P0_I1"
        DEPOT_LOC_A="P0_I1"
        INITIAL_LOC_B="P0_I8"
        DEPOT_LOC_B="P0_I8"
        TELEPORT_X_A="-8.0157"
        TELEPORT_Y_A="-12.7446"
        TELEPORT_QZ_A="0.7071068"
        TELEPORT_QW_A="0.7071068"
        TELEPORT_X_B="-7.9986"
        TELEPORT_Y_B="12.5446"
        TELEPORT_QZ_B="-0.7071068"
        TELEPORT_QW_B="0.7071068"
        ;;
      dual_topology_line_headon_fast_p3)
        # Staged in P3; T12/T8 are exactly at P3_I1/P3_I8.
        ZONE_A="[]"
        ZONE_B="[]"
        SHORTCUT_PAIRS="[]"
        TOPOLOGY_MODE="line"
        PROST_HORIZON=45
        TARGETS_A="['T8']"
        TARGETS_B="['T12']"
        BASKET_CAPACITY=2
        INITIAL_LOC_A="P3_I1"
        DEPOT_LOC_A="P3_I1"
        INITIAL_LOC_B="P3_I8"
        DEPOT_LOC_B="P3_I8"
        TELEPORT_X_A="3.4814"
        TELEPORT_Y_A="-12.7560"
        TELEPORT_QZ_A="0.7071068"
        TELEPORT_QW_A="0.7071068"
        TELEPORT_X_B="3.4984"
        TELEPORT_Y_B="12.5331"
        TELEPORT_QZ_B="-0.7071068"
        TELEPORT_QW_B="0.7071068"
        ;;
      dual_topology_line_headon_fast_p4)
        # Staged in P4 (outer row); both targets inside P4 (T10/T9).
        ZONE_A="[]"
        ZONE_B="[]"
        SHORTCUT_PAIRS="[]"
        TOPOLOGY_MODE="line"
        PROST_HORIZON=45
        TARGETS_A="['T10']"
        TARGETS_B="['T9']"
        BASKET_CAPACITY=2
        INITIAL_LOC_A="P4_I1"
        DEPOT_LOC_A="P4_I1"
        INITIAL_LOC_B="P4_I8"
        DEPOT_LOC_B="P4_I8"
        TELEPORT_X_A="7.7408"
        TELEPORT_Y_A="-12.7602"
        TELEPORT_QZ_A="0.7071068"
        TELEPORT_QW_A="0.7071068"
        TELEPORT_X_B="7.7578"
        TELEPORT_Y_B="12.5289"
        TELEPORT_QZ_B="-0.7071068"
        TELEPORT_QW_B="0.7071068"
        ;;
    esac
    ROBOT_A_CMD=(
      rosrun harbot_orchard_picker online_orchard_manager.py
      _robot_id:=robot _prost_port:=2323
      "_initial_loc:=$INITIAL_LOC_A" "_depot_loc:=$DEPOT_LOC_A"
      "_allowed_targets:=$TARGETS_A"
      "_allowed_locations:=$ZONE_A"
      "_shortcut_pairs:=$SHORTCUT_PAIRS"
      "_topology_mode:=$TOPOLOGY_MODE"
      "_topology_redundant_pairs:=$TOPOLOGY_REDUNDANT_PAIRS"
      "_prost_horizon:=$PROST_HORIZON"
      _max_cycles:=20 _route_strategy:=prost
      _route_passage_edge_cost:=10.2 _route_transfer_first_cost:=20.4
      _route_transfer_continuation_cost:=13.2
      _route_shortcut_feeder_cost:=10.2 _route_shortcut_crossing_cost:=13.2
      _auto_drop:=true _require_all_targets:=true
      _physical_drop_delete_fruits:=true
      "${common_manager_args[@]}"
      _continuous_forward_speed:="$SPEED_A"
      _basket_capacity:="$BASKET_CAPACITY"
      _continuous_passage_motion:="$CONTINUOUS_PASSAGE_MOTION"
      _peer_safety_radius:="$PEER_SAFETY_RADIUS"
    )
    ROBOT_B_CMD=(
      rosrun harbot_orchard_picker online_orchard_manager.py
      _robot_id:=robot_b _prost_port:=2324
      "_initial_loc:=$INITIAL_LOC_B" "_depot_loc:=$DEPOT_LOC_B"
      "_allowed_targets:=$TARGETS_B"
      "_allowed_locations:=$ZONE_B"
      "_shortcut_pairs:=$SHORTCUT_PAIRS"
      "_topology_mode:=$TOPOLOGY_MODE"
      "_topology_redundant_pairs:=$TOPOLOGY_REDUNDANT_PAIRS"
      "_prost_horizon:=$PROST_HORIZON"
      _max_cycles:=20 _route_strategy:=prost
      _route_passage_edge_cost:=10.2 _route_transfer_first_cost:=20.4
      _route_transfer_continuation_cost:=13.2
      _route_shortcut_feeder_cost:=10.2 _route_shortcut_crossing_cost:=13.2
      _auto_drop:=true _require_all_targets:=true
      _physical_drop_delete_fruits:=true
      "${common_manager_args[@]}"
      _continuous_forward_speed:="$SPEED_B"
      _basket_capacity:="$BASKET_CAPACITY"
      _continuous_passage_motion:="$CONTINUOUS_PASSAGE_MOTION"
      _peer_safety_radius:="$PEER_SAFETY_RADIUS"
    )
    TEST_CMD=(:)
    ;;
  p4)
    LOG_PREFIX="auto_p4"
    NEED_MOVEIT=false
    TEST_CMD=(
      rosrun harbot_orchard_picker passage_segment_check.py
      _position_from_passage:=0
      _passage:=4
      _start_index:=8
      _end_index:=6
      _holonomic_transfer:=true
      _holonomic_transfer_speed:=0.30
      _holonomic_transfer_turn:=0.30
      _continuous_forward_speed:=0.44
      _continuous_turn_in_place_threshold:=0.35
      _continuous_max_cross_track:=0.35
      _outer_lane_extra_clearance_right:=0.45
      _tilt_abort_angle_deg:=16.0
      _tilt_recovery_angle_deg:=8.0
      _tilt_recovery_stable_duration:=0.40
      # Tilt recovery, see above.
      _tilt_recovery_timeout:=20.0
      _tilt_recovery_hysteresis_deg:=1.5
      _tilt_hard_abort_angle_deg:=18.0
    )
    ;;
  *)
    echo "Unknown profile: $PROFILE"
    echo "Available: smoke, dual_smoke, adaptive_pregrasp, full_global, dual_equal, dual_unequal_speed, dual_unequal_zone, dual_speed_zone70, dual_speed_zone80, dual_topology_shortcut, dual_scrambled, dual_head_on, p4"
    exit 2
    ;;
esac

stop_existing_world
start_world "before_${PROFILE}"

if [[ "$NEED_MOVEIT" == true ]]; then
  if [[ "$DUAL_MODE" == true ]]; then
    wait_until "both MoveIt move_group processes" 180 has_dual_move_group
    wait_until "both robots' controllers" 120 has_dual_motion_controllers
    wait_until "gazebo get_model_state/delete_model services" 60 has_gazebo_model_services
    # Short pause before starting both nodes: right after the world is ready,
    # rospy.init_node sometimes failed with "Connection refused".
    sleep 3
  else
    wait_until "MoveIt move_group" 120 has_move_group
    wait_until "gazebo get_model_state/delete_model services" 60 has_gazebo_model_services
  fi
fi
echo "World and required services are ready."

if [[ -n "${REPORT_SESSION_DIR:-}" ]]; then
  mkdir -p "$REPORT_SESSION_DIR"
  mkdir -p "$REPORT_SESSION_DIR/ros_node_logs"
  rosparam dump "$REPORT_SESSION_DIR/ros_parameters.yaml"
  rostopic list > "$REPORT_SESSION_DIR/ros_topics.txt"
  {
    echo "date=$(date --iso-8601=seconds)"
    echo "hostname=$(hostname)"
    echo "ros_distro=${ROS_DISTRO:-unknown}"
    echo "profile=$PROFILE"
  } > "$REPORT_SESSION_DIR/run_metadata.txt"
  printf '%s\n' "${TEST_CMD[@]}" > "$REPORT_SESSION_DIR/effective_test_command.txt"
  if [[ "$DUAL_MODE" == true && "$PROFILE" != "dual_smoke" ]]; then
    # Save the exact robot commands with the report.
    printf '%s\n' "${ROBOT_A_CMD[@]}" > "$REPORT_SESSION_DIR/robot_command.txt"
    printf '%s\n' "${ROBOT_B_CMD[@]}" > "$REPORT_SESSION_DIR/robot_b_command.txt"
    ROUTE_STRATEGY_USED="$(grep -oP '(?<=_route_strategy:=)\S+' "$REPORT_SESSION_DIR/robot_command.txt")"
    {
      echo "profile=$PROFILE"
      echo "route_strategy=$ROUTE_STRATEGY_USED"
      echo "speed_a_mps=$SPEED_A"
      echo "speed_b_mps=$SPEED_B"
      # grep returns 1 when ZONE is "[]"; with pipefail that would fire the ERR
      # trap and kill the world, hence || true.
      echo "zone_a_location_count=$( (grep -o "'" <<<"$ZONE_A" || true) | wc -l | awk '{print $1/2}')"
      echo "zone_b_location_count=$( (grep -o "'" <<<"$ZONE_B" || true) | wc -l | awk '{print $1/2}')"
      echo "targets_a=$TARGETS_A"
      echo "targets_b=$TARGETS_B"
      echo "shortcut_pairs=$SHORTCUT_PAIRS"
      echo "topology_mode=$TOPOLOGY_MODE"
      echo "topology_redundant_pairs=$TOPOLOGY_REDUNDANT_PAIRS"
      echo "basket_capacity=$BASKET_CAPACITY"
    } > "$REPORT_SESSION_DIR/profile_parameters.txt"
  fi
  ROS_LOG_DIR="$REPORT_SESSION_DIR/ros_node_logs" \
    python3 "$PKG_DIR/scripts/report_metrics_recorder.py" \
    "$REPORT_SESSION_DIR/trajectory_robot.csv" \
    /robot/robotnik_base_control/odom_gt \
    >"$REPORT_SESSION_DIR/metrics_recorder.log" 2>&1 &
  REPORT_METRICS_PID=$!

  if [[ "$DUAL_MODE" == true ]]; then
    ROS_LOG_DIR="$REPORT_SESSION_DIR/ros_node_logs" \
      python3 "$PKG_DIR/scripts/report_metrics_recorder.py" \
      "$REPORT_SESSION_DIR/trajectory_robot_b.csv" \
      /robot_b/robotnik_base_control/odom_gt \
      >"$REPORT_SESSION_DIR/metrics_recorder_robot_b.log" 2>&1 &
    REPORT_METRICS_B_PID=$!
  fi

  rosbag record --lz4 -O "$REPORT_SESSION_DIR/full_global.bag" \
    /clock \
    /robot/robotnik_base_control/odom_gt \
    /robot_b/robotnik_base_control/odom_gt \
    /robot/joint_states \
    /robot/robotnik_base_control/cmd_vel \
    /tf /tf_static \
    >"$REPORT_SESSION_DIR/rosbag.log" 2>&1 &
  REPORT_BAG_PID=$!

  if [[ -n "${DISPLAY:-}" ]] && gst-inspect-1.0 ximagesrc >/dev/null 2>&1; then
    gst-launch-1.0 -e ximagesrc use-damage=false show-pointer=true \
      ! video/x-raw,framerate=20/1 \
      ! videoconvert \
      ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=5000 \
      ! mp4mux \
      ! filesink location="$REPORT_SESSION_DIR/full_simulation.mp4" \
      >"$REPORT_SESSION_DIR/video.log" 2>&1 &
    REPORT_VIDEO_PID=$!
  else
    echo "ERROR: X11/GStreamer screen recording is not available."
    exit 1
  fi

  sleep 1
  if ! kill -0 "$REPORT_METRICS_PID" 2>/dev/null; then
    echo "ERROR: the metrics recorder did not start. See metrics_recorder.log."
    exit 1
  fi
  if [[ "$DUAL_MODE" == true ]] && ! kill -0 "$REPORT_METRICS_B_PID" 2>/dev/null; then
    echo "ERROR: robot B's metrics recorder did not start."
    exit 1
  fi
  if ! kill -0 "$REPORT_BAG_PID" 2>/dev/null; then
    echo "ERROR: the rosbag recorder did not start. See rosbag.log."
    exit 1
  fi
  if [[ -n "$REPORT_VIDEO_PID" ]] && ! kill -0 "$REPORT_VIDEO_PID" 2>/dev/null; then
    echo "ERROR: the video recorder did not start. See video.log."
    exit 1
  fi
fi

TEST_LOG="$LOG_DIR/${LOG_PREFIX}_$(date +%Y%m%d_%H%M%S).txt"
echo "Running profile: $PROFILE"
echo "Test log: $TEST_LOG"

# Teleport (only *_headon_fast profiles) after spawn and before the
# manager nodes start.
if [[ -n "$TELEPORT_X_A" ]]; then
  echo "Teleporting robots to $INITIAL_LOC_A / $INITIAL_LOC_B (fast head-on test)..."
  for _t in \
    "robot:$TELEPORT_X_A:$TELEPORT_Y_A:$TELEPORT_QZ_A:$TELEPORT_QW_A" \
    "robot_b:$TELEPORT_X_B:$TELEPORT_Y_B:$TELEPORT_QZ_B:$TELEPORT_QW_B"
  do
    IFS=: read -r _model _x _y _qz _qw <<<"$_t"
    _resp=$(rosservice call /gazebo/set_model_state \
      "{model_state: {model_name: '$_model', pose: {position: {x: $_x, y: $_y, z: 0.1}, orientation: {x: 0.0, y: 0.0, z: $_qz, w: $_qw}}, reference_frame: 'world'}}" 2>&1)
    if [[ "$_resp" != *"success: True"* ]]; then
      echo "ERROR: teleporting model '$_model' failed: $_resp"
      exit 1
    fi
  done
  sleep 1
fi

if [[ "$PROFILE" == dual_* && "$PROFILE" != "dual_smoke" ]]; then
  ROBOT_A_LOG="${TEST_LOG%.txt}_robot.txt"
  ROBOT_B_LOG="${TEST_LOG%.txt}_robot_b.txt"
  set +e
  ( trap - ERR; set -o pipefail; "${ROBOT_A_CMD[@]}" 2>&1 | tee "$ROBOT_A_LOG" ) &
  ROBOT_A_PID=$!
  ( trap - ERR; set -o pipefail; "${ROBOT_B_CMD[@]}" 2>&1 | tee "$ROBOT_B_LOG" ) &
  ROBOT_B_PID=$!
  # `wait` returns the robot's exit code, which is 1 when a mission fails.
  # Disable the ERR trap here, otherwise the first failing robot would kill
  # the world while the other one is still running.
  trap - ERR
  wait "$ROBOT_A_PID"
  ROBOT_A_STATUS=$?
  wait "$ROBOT_B_PID"
  ROBOT_B_STATUS=$?
  trap cleanup_on_exit ERR
  set -e
  TEST_STATUS=0
  (( ROBOT_A_STATUS == 0 && ROBOT_B_STATUS == 0 )) || TEST_STATUS=1
  {
    echo "robot_status=$ROBOT_A_STATUS"
    echo "robot_b_status=$ROBOT_B_STATUS"
    echo "dual_status=$TEST_STATUS"
    echo "robot_log=$ROBOT_A_LOG"
    echo "robot_b_log=$ROBOT_B_LOG"
  } | tee "$TEST_LOG"
else
  set +e
  # Same for the single-robot test command.
  trap - ERR
  "${TEST_CMD[@]}" 2>&1 | tee "$TEST_LOG"
  TEST_STATUS=${PIPESTATUS[0]}
  trap cleanup_on_exit ERR
  set -e
fi

echo "LOG FILE: $TEST_LOG"
echo "Test exit status: $TEST_STATUS"

if [[ -n "${REPORT_SESSION_DIR:-}" ]]; then
  # Report generation errors are warnings, not fatal.
  set +e
  stop_report_recorders
  REPORT_METRICS_PID=""
  REPORT_METRICS_B_PID=""
  REPORT_BAG_PID=""
  REPORT_VIDEO_PID=""
  cp "$TEST_LOG" "$REPORT_SESSION_DIR/" \
    || echo "WARNING: copying $TEST_LOG failed"
  if [[ "$DUAL_MODE" == true && "$PROFILE" != "dual_smoke" ]]; then
    cp "$ROBOT_A_LOG" "$ROBOT_B_LOG" "$REPORT_SESSION_DIR/" \
      || echo "WARNING: copying the robot logs failed"
  fi
  cp "$WORLD_LOG" "$REPORT_SESSION_DIR/" \
    || echo "WARNING: copying $WORLD_LOG failed"
  if [[ "$DUAL_MODE" == true && "$PROFILE" != "dual_smoke" ]]; then
    # Copy the per-robot closed-loop logs.
    cp "$CALIB_DIR/online_closed_loop_log_robot.csv" \
      "$REPORT_SESSION_DIR/events_robot.csv" \
      || echo "WARNING: copying events_robot.csv failed"
    cp "$CALIB_DIR/online_closed_loop_log_robot_b.csv" \
      "$REPORT_SESSION_DIR/events_robot_b.csv" \
      || echo "WARNING: copying events_robot_b.csv failed"
    cp "$CALIB_DIR/field_moves_robot.csv" \
      "$REPORT_SESSION_DIR/field_moves_robot.csv" \
      || echo "WARNING: copying field_moves_robot.csv failed"
    cp "$CALIB_DIR/field_moves_robot_b.csv" \
      "$REPORT_SESSION_DIR/field_moves_robot_b.csv" \
      || echo "WARNING: copying field_moves_robot_b.csv failed"
  elif [[ -f "$CALIB_DIR/online_closed_loop_log.csv" ]]; then
    cp "$CALIB_DIR/online_closed_loop_log.csv" \
      "$REPORT_SESSION_DIR/pick_events.csv"
  fi
  if [[ "$DUAL_MODE" == true ]]; then
    python3 "$PKG_DIR/scripts/generate_dual_metrics.py" \
      "$REPORT_SESSION_DIR" | tee "$REPORT_SESSION_DIR/metrics_summary.txt"
    if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
      echo "WARNING: generate_dual_metrics.py failed (probably incomplete logs from a failed test)"
    fi
  else
    cp "$REPORT_SESSION_DIR/trajectory_robot.csv" "$REPORT_SESSION_DIR/trajectory.csv"
    python3 "$PKG_DIR/scripts/generate_report_metrics.py" \
      "$REPORT_SESSION_DIR" | tee "$REPORT_SESSION_DIR/metrics_summary.txt"
    if [[ ${PIPESTATUS[0]} -ne 0 ]]; then
      echo "WARNING: generate_report_metrics.py failed (probably incomplete logs from a failed test)"
    fi
  fi
  du -h "$REPORT_SESSION_DIR"/* | sort -h > "$REPORT_SESSION_DIR/file_sizes.txt"
  set -e
fi

echo "Stopping the finished test world."
ACTIVE_WORLD_PID=""
# Stop the owned launch and also its separately spawned ROS master/Gazebo
# children.  Starting the next launch while the old master is still accepting
# connections can leave the new world attached to a master that is shutting
# down a moment later.
stop_existing_world

echo "Starting the next clean world."
start_world "ready_after_${PROFILE}"
if [[ "$NEED_MOVEIT" == true ]]; then
  if [[ "$DUAL_MODE" == true ]]; then
    wait_until "both MoveIt processes in the new world" 180 has_dual_move_group
    wait_until "both robots' controllers in the new world" 120 has_dual_motion_controllers
  else
    wait_until "MoveIt move_group in the new world" 120 has_move_group
  fi
  wait_until "gazebo get_model_state/delete_model services in the new world" 60 has_gazebo_model_services
fi

echo "NEW CLEAN WORLD IS READY"
echo "WORLD LOG: $WORLD_LOG"
echo "$ACTIVE_WORLD_PID" > "$WORLD_PID_FILE"

# The final world is intentionally left running for the next test.
trap - INT TERM ERR
ACTIVE_WORLD_PID=""
exit "$TEST_STATUS"
