#!/usr/bin/env python3

import csv
import heapq
import itertools
import math
import os
import signal
import random
import re
import subprocess
import time
from pathlib import Path

import rospy
from gazebo_msgs.srv import DeleteModel
from nav_msgs.msg import Odometry

import sys
# Sibling ROS nodes live in this directory; the ROS-free core is in src/.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from harbot_orchard_picker import fleet_arbiter
from harbot_orchard_picker import orchard_topology

from orchard_map_picker import OrchardMapPicker
from target_pick_executor import (
    import_moveit,
    make_moveit_group,
    pick_location_for_prost_two_motions,
)


class PeerDeadlockDetected(Exception):
    """Raised by the non-priority robot when a mutual deadlock with the peer is confirmed.

    Caught in execute_move() / execute_passage_segment(), which then call
    resolve_peer_deadlock() instead of aborting the run.
    """

    def __init__(self, contested_loc, target_loc):
        super().__init__(
            "peer deadlock contested_loc=%s target_loc=%s"
            % (contested_loc, target_loc)
        )
        self.contested_loc = contested_loc
        self.target_loc = target_loc


class BypassArcAborted(Exception):
    """Raised when the peer gets too close during a bypass arc or retreat.

    Triggers below peer_deadlock_absolute_min_distance, or when the distance
    drops too far below the best value seen during the maneuver. Caught inside
    attempt_bypass_arc() / resolve_peer_deadlock(), which fall back to
    reroute or retreat.
    """

    def __init__(self, dist):
        super().__init__("bypass arc aborted, dist=%.3f" % dist)
        self.dist = dist


class OnlineOrchardManager:
    def __init__(self):
        rospy.init_node("online_orchard_manager", anonymous=True)

        self.robot_id = str(rospy.get_param("~robot_id", "robot")).strip("/")
        self.prost_port = int(rospy.get_param(
            "~prost_port", 2323 if self.robot_id == "robot" else 2324
        ))
        self.instance_name = str(rospy.get_param(
            "~prost_instance_name", "orchard_targets_%s" % self.robot_id
        ))
        configured_locations = rospy.get_param("~allowed_locations", [])
        self.allowed_locations = set(str(value) for value in configured_locations)

        self.grasp_success_prob = float(rospy.get_param("~grasp_success_prob", 0.8))
        self.random_seed = int(rospy.get_param("~random_seed", 1))
        self.max_cycles = int(rospy.get_param("~max_cycles", 30))
        self.require_all_targets = bool(rospy.get_param(
            "~require_all_targets", False
        ))
        self.max_pick_attempts_per_target = int(rospy.get_param(
            "~max_pick_attempts_per_target", 3
        ))
        self.basket_capacity = int(rospy.get_param("~basket_capacity", 8))
        self.depot_loc = rospy.get_param("~depot_loc", "P0_I1")
        self.auto_drop = bool(rospy.get_param("~auto_drop", False))
        self.optimize_plan_route = bool(rospy.get_param("~optimize_plan_route", True))
        self.route_strategy = str(rospy.get_param(
            "~route_strategy", "prost"
        )).strip().lower()
        # Diagnostic only: exhaustive search over the basket tour, very slow for
        # large baskets (~1100 s per mission at capacity 24). Off by default.
        self.time_aware_observer_enabled = bool(rospy.get_param(
            "~time_aware_observer", False
        ))
        if self.route_strategy not in ("prost", "time_aware", "time_aware_global"):
            rospy.logwarn(
                "Unknown route_strategy=%s; falling back to prost",
                self.route_strategy
            )
            self.route_strategy = "prost"
        if self.route_strategy == "time_aware_global" and not self.auto_drop:
            rospy.logwarn(
                "time_aware_global requires auto_drop=true; falling back to time_aware"
            )
            self.route_strategy = "time_aware"
        self.continuous_passage_motion = bool(rospy.get_param(
            "~continuous_passage_motion", True
        ))
        self.rejoin_passage_after_pick = bool(rospy.get_param(
            "~rejoin_passage_after_pick", True
        ))
        # Measured travel-time costs used only for route evaluation.
        self.route_passage_edge_cost = float(rospy.get_param(
            "~route_passage_edge_cost", 10.2
        ))
        self.route_transfer_first_cost = float(rospy.get_param(
            "~route_transfer_first_cost", 20.4
        ))
        self.route_transfer_continuation_cost = float(rospy.get_param(
            "~route_transfer_continuation_cost", 13.2
        ))
        # Mid-row shortcut costs, used only when shortcut_pairs is not empty.
        self.route_shortcut_feeder_cost = float(rospy.get_param(
            "~route_shortcut_feeder_cost", 10.2
        ))
        self.route_shortcut_crossing_cost = float(rospy.get_param(
            "~route_shortcut_crossing_cost", 13.2
        ))
        # Only the first PICK of each plan is executed before replanning,
        # so a long horizon is wasted work.
        self.prost_horizon = int(rospy.get_param("~prost_horizon", 20))

        # Optional mid-row shortcuts, e.g. ["P1-P2:2", "P2-P3:6"].
        self.shortcut_pairs = list(rospy.get_param("~shortcut_pairs", []))

        # "ladder": both endcaps connect adjacent passages. "line": single
        # serpentine path. Used by both write_instance() and build_graph_edges(),
        # which must agree.
        self.topology_mode = str(rospy.get_param("~topology_mode", "ladder")).strip().lower()
        if self.topology_mode not in ("ladder", "line"):
            rospy.logwarn(
                "Unknown ~topology_mode=%r, defaulting to 'ladder'",
                self.topology_mode
            )
            self.topology_mode = "ladder"
        # For topology "line": passage pairs that get both endcaps connected
        # (see orchard_topology.line_edges()).
        self.topology_redundant_pairs = list(rospy.get_param(
            "~topology_redundant_pairs", []
        ))

        random.seed(self.random_seed)

        self.prost_dir = Path.home() / "prost"
        self.testbed_dir = self.prost_dir / "testbed"
        self.benchmark_dir = self.testbed_dir / "benchmarks/orchard_targets"

        # prost.py copies its binaries into cwd under fixed names, so each robot
        # gets its own scratch directory to avoid a race between the two clients.
        self.prost_client_scratch_dir = Path.home() / (
            "harbot_rddl/prost_scratch_%s" % self.robot_id
        )
        self.prost_client_scratch_dir.mkdir(parents=True, exist_ok=True)

        self.instance_path = self.benchmark_dir / (
            "%s_mdp__1.rddl" % self.instance_name
        )
        self.server_log = Path.home() / (
            "harbot_rddl/online_orchard_targets_server_%s.log" % self.robot_id
        )
        self.meta_path = Path.home() / "harbot_rddl/orchard_targets_meta.csv"
        self.online_log = Path(rospy.get_param(
            "~online_log",
            str(Path.home() / (
                "harbot_calibration/online_closed_loop_log_%s.csv" % self.robot_id
            ))
        ))
        # One row per actual location change (not per control tick).
        self.field_moves_log = Path(rospy.get_param(
            "~field_moves_log",
            str(Path.home() / (
                "harbot_calibration/field_moves_%s.csv" % self.robot_id
            ))
        ))

        self.targets = {}
        self.load_targets()

        self.current_loc = str(rospy.get_param("~initial_loc", self.depot_loc))
        self.picked_targets = set()
        self.picked_fruit_parts = {}
        self.pick_attempts = {}
        self.basket_load = 0

        self.picker = OrchardMapPicker()
        self.picker.wait_for_odom()
        self.moveit_commander = None
        self.moveit_group = None
        self.pick_resources_ready = False

        # Peer awareness and proximity hold. If no peer odometry is published
        # (single-robot run), peer checks are a no-op.
        self.peer_robot_id = str(rospy.get_param(
            "~peer_robot_id",
            "robot_b" if self.robot_id == "robot" else "robot"
        )).strip("/")
        # Deterministic tie-break: "robot" < "robot_b", so "robot" has priority.
        self.peer_has_priority = self.robot_id < self.peer_robot_id
        # Broadcast files are not cleared automatically when a process is killed,
        # so clear this robot's own files on startup.
        fleet_arbiter.clear_waiting(self.robot_id)
        fleet_arbiter.clear_maneuvering(self.robot_id)
        fleet_arbiter.clear_finished(self.robot_id)
        self.peer_safety_radius = float(rospy.get_param(
            "~peer_safety_radius", 1.75
        ))
        self.peer_proximity_timeout = float(rospy.get_param(
            "~peer_proximity_timeout", 30.0
        ))
        # Used by wait_for_moving_peer_clearance().
        self.peer_proximity_hard_timeout = float(rospy.get_param(
            "~peer_proximity_hard_timeout", 180.0
        ))
        # Hold timeout for the priority robot when the peer is in near-contact
        # range. A shorter value (50 s) was tested and lowered the success rate,
        # so the default matches the global timeout.
        self.peer_proximity_priority_nearcontact_timeout = float(
            rospy.get_param(
                "~peer_proximity_priority_nearcontact_timeout", 180.0
            )
        )
        # How long an ended peer-block episode can be resumed (s).
        # 0 = every block starts a fresh timer.
        self.peer_block_episode_cooldown = float(rospy.get_param(
            "~peer_block_episode_cooldown", 0.0
        ))
        # Below this distance the robot holds regardless of priority or wait time.
        self.peer_proximity_hard_min_distance = float(rospy.get_param(
            "~peer_proximity_hard_min_distance", 0.85
        ))
        # Clearance used once the peer has finished and is parked.
        # Lower than peer_safety_radius because a parked peer will not move;
        # 0.75 m was chosen from the distance distribution in the experiment logs.
        self.finished_peer_clearance_distance = float(rospy.get_param(
            "~finished_peer_clearance_distance", 0.75
        ))
        # Hysteresis: resume only above hard_min + margin, to avoid stop/resume
        # chatter around a single threshold.
        self.peer_proximity_resume_margin = float(rospy.get_param(
            "~peer_proximity_resume_margin", 0.10
        ))
        # Abort threshold for bypass arc and retreat. Below this distance the
        # maneuver always aborts; above it, it aborts only if the distance drops
        # more than peer_deadlock_closing_epsilon below the best value seen so far.
        # 0.65 m leaves margin above the closest measured contact (~0.48 m).
        self.peer_deadlock_absolute_min_distance = float(rospy.get_param(
            "~peer_deadlock_absolute_min_distance", 0.65
        ))
        # Same meaning as ~lateral_closing_epsilon.
        self.peer_deadlock_closing_epsilon = float(rospy.get_param(
            "~peer_deadlock_closing_epsilon", 0.05
        ))
        # The closing check only applies below this distance; further away,
        # a decreasing distance is not treated as dangerous.
        self.peer_deadlock_closing_guard_distance = float(rospy.get_param(
            "~peer_deadlock_closing_guard_distance", 1.10
        ))
        self.peer_pose_received = False
        self.peer_x = None
        self.peer_y = None
        self.peer_yaw = None
        # Last observed peer position, used to check that the peer is really
        # standing still before the closing check is relaxed.
        self._peer_motion_ref = None
        self._peer_last_moved_at = None
        self.peer_motion_epsilon = float(rospy.get_param(
            "~peer_motion_epsilon", 0.05
        ))
        self.bypass_arc_peer_still_seconds = float(rospy.get_param(
            "~bypass_arc_peer_still_seconds", 2.0
        ))
        # Time of the last peer pose, used to detect a stale peer.
        self.peer_pose_updated_at = None
        self.peer_pose_max_age = float(rospy.get_param(
            "~peer_pose_max_age", 1.0
        ))
        self._peer_odom_topic = "/%s/robotnik_base_control/odom_gt" % self.peer_robot_id
        self._peer_odom_sub = rospy.Subscriber(
            self._peer_odom_topic, Odometry, self._peer_odom_callback,
            queue_size=1
        )
        rospy.loginfo(
            "PEER AWARENESS robot_id=%s peer_robot_id=%s has_priority=%s "
            "safety_radius=%.2f subscribed=%s",
            self.robot_id, self.peer_robot_id, self.peer_has_priority,
            self.peer_safety_radius, self._peer_odom_topic
        )

        # Node reservation through fleet_arbiter (same rules for both robots).
        self.fleet_reservation_enabled = bool(rospy.get_param(
            "~fleet_reservation_enabled", True
        ))
        self.fleet_reservation_timeout = float(rospy.get_param(
            "~fleet_reservation_timeout", 30.0
        ))
        # Hard timeout for the non-priority robot's reservation retries.
        # Only the priority robot may proceed after the short timeout.
        self.fleet_reservation_hard_timeout = float(rospy.get_param(
            "~fleet_reservation_hard_timeout", 180.0
        ))

        # Current peer-block episode, kept across calls so timers are measured
        # per episode and not per control tick.
        self._peer_block_episode = None
        # Most recently ended episode (see _enter_peer_block_episode()).
        self._peer_block_episode_recent = None
        self.peer_deadlock_confirm_threshold = float(rospy.get_param(
            "~peer_deadlock_confirm_threshold", 20.0
        ))
        # Try a bypass arc after this many seconds of blocking, without waiting
        # for a confirmed deadlock.
        self.peer_bypass_speculative_threshold = float(rospy.get_param(
            "~peer_bypass_speculative_threshold", 20.0
        ))
        self.peer_deadlock_block_expiry = float(rospy.get_param(
            "~peer_deadlock_block_expiry", 90.0
        ))
        self.peer_deadlock_retreat_grace = float(rospy.get_param(
            "~peer_deadlock_retreat_grace", 10.0
        ))
        # After an unconfirmed retreat, wait up to this long for real separation
        # before retrying.
        self.peer_deadlock_retreat_confirm_timeout = float(rospy.get_param(
            "~peer_deadlock_retreat_confirm_timeout", 15.0
        ))
        # Required distance gain over the distance at which the retreat stopped.
        self.peer_deadlock_retreat_progress_margin = float(rospy.get_param(
            "~peer_deadlock_retreat_progress_margin", 0.20
        ))
        # Keep-right rule for robots sharing a passage: each robot offsets its own
        # path to the right, so opposite-direction robots pass without coordination.
        # Offsets were found by placing the robot at candidate offsets in Gazebo and
        # checking that it stays level (tight side ~0.50 m, wide side ~1.29 m max),
        # then reduced for margin.
        self.lateral_bypass_offset_tight = float(rospy.get_param(
            "~lateral_bypass_offset_tight", 0.20
        ))
        self.lateral_bypass_offset_wide = float(rospy.get_param(
            "~lateral_bypass_offset_wide", 1.05
        ))
        # Lower speed on the tight side: the robot tilted there a few times while
        # driving at higher speed. 0.18 m/s still clears the stall detector.
        self.lateral_bypass_tight_speed = float(rospy.get_param(
            "~lateral_bypass_tight_speed", 0.18
        ))
        # Hold distance used instead of peer_safety_radius during a lateral bypass.
        self.lateral_bypass_hold_distance = float(rospy.get_param(
            "~lateral_bypass_hold_distance", 1.00
        ))
        # Trend tracking for _lateral_bypass_closing().
        self.lateral_closing_window = float(rospy.get_param(
            "~lateral_closing_window", 0.5
        ))
        self.lateral_closing_epsilon = float(rospy.get_param(
            "~lateral_closing_epsilon", 0.05
        ))
        self._lateral_closing_ref_dist = None
        self._lateral_closing_ref_time = None
        self._lateral_closing_last_result = False
        # Cross-track margin for the merge leg (overridden per call in
        # execute_passage_segment()).
        self.lateral_bypass_max_cross_track_margin = float(rospy.get_param(
            "~lateral_bypass_max_cross_track_margin", 0.35
        ))
        # Merge-leg length grows with the lateral offset so the initial heading
        # error stays small (< ~4 deg). With a fixed short merge leg, the wide
        # offset gave a steep bearing, heading_factor capped the speed and the
        # base stalled. 15.0 ~ 1 / tan(4 deg), rounded up.
        self.lateral_bypass_merge_slope = float(rospy.get_param(
            "~lateral_bypass_merge_slope", 15.0
        ))
        # Extra stall window for the lateral-bypass merge leg.
        self.lateral_bypass_stall_window = float(rospy.get_param(
            "~lateral_bypass_stall_window", 10.0
        ))
        # Max perpendicular distance of the peer from this passage line for it
        # to count as being in the same aisle.
        self.lateral_bypass_aisle_cross_track = float(rospy.get_param(
            "~lateral_bypass_aisle_cross_track", 1.2
        ))
        # Along-track margin (m) past both ends of the segment for the same check.
        self.lateral_bypass_aisle_margin = float(rospy.get_param(
            "~lateral_bypass_aisle_margin", 1.5
        ))
        # Both nodes start almost together, so wait once for the first peer
        # odometry message before the first aisle check.
        self._peer_pose_wait_done = False
        # Limit on deadlock resolutions per action (see run()).
        self.max_peer_deadlock_retries = int(rospy.get_param(
            "~max_peer_deadlock_retries", 8
        ))
        # {loc: expiry_monotonic_time}, see resolve_peer_deadlock().
        self.temporarily_blocked_locations = {}

        try:
            self.reserve_locations([self.current_loc], "startup")
        except PeerDeadlockDetected:
            # Before the run starts there is nowhere to reroute, so just proceed.
            rospy.logerr(
                "STARTUP RESERVATION deadlock at %s -- proceeding anyway",
                self.current_loc
            )
        self.pick_departure_pending = False
        self.basket_fruit_models = []

        self.online_log.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = open(self.online_log, "w", newline="")
        self.writer = csv.writer(self.log_file)
        self.writer.writerow([
            "cycle",
            "action",
            "target",
            "loc",
            "row",
            "side",
            "success",
            "upper_success",
            "lower_success",
            "picked_count",
            "picked_targets_count",
            "basket_load",
            "robot_x",
            "robot_y",
            "robot_yaw"
        ])
        self.log_file.flush()

        self.field_moves_log.parent.mkdir(parents=True, exist_ok=True)
        self.field_moves_file = open(self.field_moves_log, "w", newline="")
        self.field_moves_writer = csv.writer(self.field_moves_file)
        self.field_moves_writer.writerow([
            "wall_time_s", "sim_time_s", "from_loc", "to_loc", "method"
        ])
        self.field_moves_file.flush()

    def _log_field_move(self, from_loc, to_loc, method):
        """Write one row to the field-moves log when current_loc changes."""
        self.field_moves_writer.writerow([
            "%.6f" % time.time(), "%.6f" % rospy.get_time(),
            from_loc, to_loc, method
        ])
        self.field_moves_file.flush()

    def load_targets(self):
        allowed_targets = set(str(value) for value in rospy.get_param(
            "~allowed_targets", []
        ))
        with open(self.meta_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                if allowed_targets and row["target"] not in allowed_targets:
                    continue
                self.targets[row["target"]] = {
                    "loc": row["loc"],
                    "row": int(row["row"]),
                    "side": row["side"]
                }

    def all_done(self):
        return len(self.picked_targets) == len(self.targets)

    def _peer_odom_callback(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
        cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.peer_x = p.x
        self.peer_y = p.y
        self.peer_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.peer_pose_received = True
        now = time.monotonic()
        self.peer_pose_updated_at = now
        # Move the reference only after the peer travels more than
        # peer_motion_epsilon, so slow creep still counts as motion.
        if self._peer_motion_ref is None:
            self._peer_motion_ref = (p.x, p.y)
            self._peer_last_moved_at = now
        elif math.hypot(
            p.x - self._peer_motion_ref[0], p.y - self._peer_motion_ref[1]
        ) > self.peer_motion_epsilon:
            self._peer_motion_ref = (p.x, p.y)
            self._peer_last_moved_at = now

    def _peer_is_stationary(self, min_still_seconds):
        """True if the peer has not moved for at least `min_still_seconds`.

        A stale pose counts as not stationary.
        """
        if not self.peer_pose_received or self._peer_pose_stale():
            return False
        if self._peer_last_moved_at is None:
            return False
        return (
            time.monotonic() - self._peer_last_moved_at >= min_still_seconds
        )

    def _peer_pose_stale(self):
        """True if a peer pose was received before but is now older than ~peer_pose_max_age."""
        if not self.peer_pose_received or self.peer_pose_updated_at is None:
            return False
        return (
            time.monotonic() - self.peer_pose_updated_at
            > self.peer_pose_max_age
        )

    def wait_for_peer_clearance(self, context):
        """Wait while the peer is within ~peer_safety_radius.

        Returns True immediately if no peer pose was ever received (single-robot
        run) or if this robot has priority. Called before every MOVE/PICK step.
        """
        # A stale pose does not count as "no peer".
        if not self.peer_pose_received or (
            self.peer_has_priority and not self._peer_pose_stale()
        ):
            return True

        # A finished (parked) peer uses the smaller clearance.
        safety_radius = (
            self.finished_peer_clearance_distance
            if fleet_arbiter.is_finished(self.peer_robot_id)
            else self.peer_safety_radius
        )

        started_at = time.monotonic()
        holding = False
        while True:
            dist = math.hypot(
                self.picker.x - self.peer_x, self.picker.y - self.peer_y
            )
            # Never treat a stale reading as clear.
            if dist >= safety_radius and not self._peer_pose_stale():
                if holding:
                    rospy.loginfo(
                        "PEER PROXIMITY RELEASED context=%s dist=%.2f "
                        "limit=%.2f peer=%s",
                        context, dist, safety_radius,
                        self.peer_robot_id
                    )
                return True

            if not holding:
                rospy.logwarn(
                    "PEER PROXIMITY HOLD context=%s dist=%.2f limit=%.2f "
                    "peer=%s (waiting -- %s has priority)",
                    context, dist, safety_radius,
                    self.peer_robot_id, self.peer_robot_id
                )
                holding = True
            # Stop the base while holding.
            self.picker.stop()

            elapsed = time.monotonic() - started_at
            # Resume threshold (hysteresis). For a finished peer, use the same
            # relaxed clearance as for the hold decision.
            resume_floor = (
                self.finished_peer_clearance_distance
                if fleet_arbiter.is_finished(self.peer_robot_id)
                else self.peer_proximity_hard_min_distance
                + self.peer_proximity_resume_margin
            )
            if dist < resume_floor:
                # Do not proceed inside hard_min just because the soft timeout expired;
                # only the hard timeout ends this wait.
                if elapsed > self.peer_proximity_hard_timeout:
                    rospy.logerr(
                        "PEER PROXIMITY HARD TIMEOUT context=%s dist=%.2f "
                        "still below resume_floor=%.2f (hard_min=%.2f) "
                        "after %.1fs -- peer likely gone/stuck, "
                        "proceeding anyway",
                        context, dist, resume_floor,
                        self.peer_proximity_hard_min_distance, elapsed
                    )
                    return True
                rospy.sleep(0.2)
                continue

            if elapsed > self.peer_proximity_timeout:
                rospy.logerr(
                    "PEER PROXIMITY TIMEOUT context=%s dist=%.2f limit=%.2f "
                    "after %.1fs -- proceeding anyway",
                    context, dist, self.peer_safety_radius,
                    self.peer_proximity_timeout
                )
                return True

            rospy.sleep(0.2)

    def _enter_peer_block_episode(self, context, target_loc):
        """Start or continue a peer-block episode and return its start time."""
        if self._peer_block_episode is None:
            # A short release (peer stops closing for a moment) should not reset the
            # escalation timer, so a recently ended episode can be resumed.
            # Only the start time is reused; the waiting state is announced again below.
            started_at = time.monotonic()
            recent = self._peer_block_episode_recent
            if (
                # Cooldown <= 0 disables resuming.
                self.peer_block_episode_cooldown > 0.0
                and recent is not None
                and recent["context"] == context
                and recent["loc"] == target_loc
                and started_at - recent["ended_at"]
                <= self.peer_block_episode_cooldown
            ):
                started_at = recent["started_at"]
            self._peer_block_episode = {
                "context": context,
                "loc": target_loc,
                "started_at": started_at,
            }
            if target_loc:
                fleet_arbiter.set_waiting(self.robot_id, [target_loc])
        return self._peer_block_episode["started_at"]

    def _exit_peer_block_episode(self):
        """End the current peer-block episode (wait resolved or given up)."""
        if self._peer_block_episode is not None:
            # Keep the ended episode briefly so it can be resumed.
            episode = self._peer_block_episode
            self._peer_block_episode_recent = {
                "context": episode["context"],
                "loc": episode["loc"],
                "started_at": episode["started_at"],
                "ended_at": time.monotonic(),
            }
            self._peer_block_episode = None
            fleet_arbiter.clear_waiting(self.robot_id)

    def peer_shares_aisle(self, passage, index_a, index_b):
        """True if the peer is inside the given aisle, between index_a and index_b.

        Uses only this robot's corridor line and the peer odometry. Direction of
        travel is ignored, so both robots offset to their own side whenever they
        share an aisle.
        """
        # With a stale pose fall back to the normal peer_safety_radius handling.
        if (
            self.peer_x is None
            or self.peer_y is None
            or self._peer_pose_stale()
        ):
            return False
        corridor_x, corridor_y = self.picker.passage_pose(passage, 1)
        lu = self.picker.length_unit
        rel_x = self.peer_x - corridor_x
        rel_y = self.peer_y - corridor_y
        cross = -lu[1] * rel_x + lu[0] * rel_y
        if abs(cross) > self.lateral_bypass_aisle_cross_track:
            return False
        along = lu[0] * rel_x + lu[1] * rel_y
        lo_x, lo_y = self.picker.passage_pose(passage, min(index_a, index_b))
        hi_x, hi_y = self.picker.passage_pose(passage, max(index_a, index_b))
        along_lo = lu[0] * (lo_x - corridor_x) + lu[1] * (lo_y - corridor_y)
        along_hi = lu[0] * (hi_x - corridor_x) + lu[1] * (hi_y - corridor_y)
        margin = self.lateral_bypass_aisle_margin
        return (along_lo - margin) <= along <= (along_hi + margin)

    def _physical_index_estimate(self, passage):
        """Estimate the index along `passage` closest to the robot's current position.

        current_loc is only updated after a whole passage segment finishes, so
        deadlock handling uses this physical estimate instead.
        """
        corridor_x, corridor_y = self.picker.passage_pose(passage, 1)
        lu = self.picker.length_unit
        along = (
            lu[0] * (self.picker.x - corridor_x)
            + lu[1] * (self.picker.y - corridor_y)
        )
        index_estimate = 1 + along / self.picker.tree_spacing
        return max(1, min(self.picker.num_indices, int(round(index_estimate))))

    def check_symmetric_deadlock(self):
        """Return the contested location if a mutual deadlock is confirmed, else None.

        Both conditions must hold:
        (a) the current block episode is longer than ~peer_deadlock_confirm_threshold
        (20 s, below the 30 s priority bypass);
        (b) the peer is waiting on a location this robot holds (fleet_arbiter).

        Called only from the non-priority robot's wait paths.
        """
        episode = self._peer_block_episode
        if episode is None:
            return None
        elapsed = time.monotonic() - episode["started_at"]
        if elapsed < self.peer_deadlock_confirm_threshold:
            return None
        peer_waiting_for = fleet_arbiter.get_waiting(self.peer_robot_id)
        if not peer_waiting_for:
            return None
        contested = peer_waiting_for & fleet_arbiter.held()
        if not contested:
            return None
        return sorted(contested)[0]

    def attempt_bypass_arc(self, target_loc):
        """Pass the peer through the open corridor beyond the endcap.

        Only possible at an endcap station (I1 or I{num_indices}); trees exist
        only at indices 1..num_indices, so the area beyond the last station is
        free. Returns True if the arc was completed (caller retries the original
        action), False if not applicable or aborted (caller tries reroute/retreat).
        """
        try:
            passage, _stale_index = self.loc_to_passage_index(self.current_loc)
        except Exception:
            return False

        # Use the physical index, current_loc can lag during a segment.
        index = self._physical_index_estimate(passage)

        if index == 1:
            direction = -1
        elif index == self.picker.num_indices:
            direction = 1
        else:
            # Mid-row: no free corridor, leave it to reroute/retreat.
            return False

        # No arc without a valid peer pose.
        if not self.peer_pose_received or self._peer_pose_stale():
            return False

        radius = float(rospy.get_param("~bypass_arc_radius", 1.3))
        # How far the arc may go past the anchor. Free depth beyond the anchor is
        # ~0.74 m (max |y| seen in odometry 14.695, anchor at 13.96); a deeper arc
        # runs into the greenhouse wall.
        depth_radius = float(
            rospy.get_param("~bypass_arc_depth_radius", 0.55)
        )
        # Minimum planned clearance to the peer. Planned vs. actual closest approach
        # differed by up to ~0.24 m, so 0.65 + 0.24 ~= 0.90.
        min_peer_clearance = float(
            rospy.get_param("~bypass_arc_min_peer_clearance", 0.90)
        )
        # Max arc depth when deepening for clearance (keeps ~0.15 m wall margin).
        max_depth_radius = float(
            rospy.get_param("~bypass_arc_max_depth_radius", 0.58)
        )
        # Fixed live-distance limit during the arc. Not relative to the plan,
        # because path-following error on the arc can be ~0.5 m.
        arc_min_live_distance = float(
            rospy.get_param("~bypass_arc_min_live_distance", 0.75)
        )
        maneuver_budget = float(rospy.get_param("~bypass_arc_maneuver_budget", 60.0))

        peer_x, peer_y = self.peer_x, self.peer_y

        # Drive to the transfer-corridor anchor first (same point used by every
        # TRANSFER), then plan the arc from there. Waiting for the peer to clear
        # the row first did not work: in a real standoff it never does.

        # Best distance seen during this call, shared by both steps.
        best_dist_seen = [None]

        def hard_abort_check():
            dist = math.hypot(
                self.picker.x - self.peer_x, self.picker.y - self.peer_y
            )
            # Stop the base before raising, otherwise it keeps the last velocity command.
            if dist < self.peer_deadlock_absolute_min_distance:
                self.picker.stop()
                raise BypassArcAborted(dist)
            if best_dist_seen[0] is None or dist > best_dist_seen[0]:
                # New best distance, nothing to check.
                best_dist_seen[0] = dist
                return
            # Going around a body always reduces the distance to it, so the closing
            # check is skipped when the peer is known to be static: it has finished,
            # or it has not moved for ~bypass_arc_peer_still_seconds.
            # The absolute floor above still applies on every tick.
            peer_is_static = (
                fleet_arbiter.is_finished(self.peer_robot_id)
                or self._peer_is_stationary(
                    self.bypass_arc_peer_still_seconds
                )
            )
            if peer_is_static:
                # Fixed minimum distance during the arc (see ~bypass_arc_min_live_distance).
                if dist < arc_min_live_distance:
                    self.picker.stop()
                    raise BypassArcAborted(dist)
                return
            if (
                dist < self.peer_deadlock_closing_guard_distance
                and best_dist_seen[0] - dist
                > self.peer_deadlock_closing_epsilon
            ):
                self.picker.stop()
                raise BypassArcAborted(dist)

        rospy.logerr(
            "PEER DEADLOCK BYPASS ARC starting current=%s target=%s "
            "peer=(%.2f, %.2f) direction=%+d radius=%.2f -- step 1: "
            "driving to own safe transfer-corridor point first",
            self.current_loc, target_loc, peer_x, peer_y, direction,
            radius
        )
        # Ask the peer to hold for the whole maneuver.
        fleet_arbiter.set_maneuvering(
            self.robot_id, time.monotonic() + maneuver_budget
        )

        try:
            # Step 1: reach the safe anchor point.
            if not self.picker.go_to_transfer_corridor(
                passage, index, direction, peer_clearance_fn=hard_abort_check
            ):
                rospy.logerr(
                    "PEER DEADLOCK BYPASS ARC step 1 (reach safe anchor) "
                    "failed -- falling back to reroute/retreat"
                )
                return False

            # Step 2: depth is taken from our anchor (keeps the arc clear of trees),
            # lateral centre from the peer position. Plan the arc, check the
            # clearance, deepen if needed and allowed, otherwise give up.
            waypoints = None
            chosen_depth = None
            achieved = 0.0
            candidates = [depth_radius]
            step = 0.05
            d = depth_radius + step
            while d <= max_depth_radius + 1e-9:
                candidates.append(round(d, 3))
                d += step
            for cand in candidates:
                trial = self.picker.compute_bypass_arc(
                    self.picker.x, self.picker.y, direction, radius=radius,
                    depth_radius=cand,
                    lateral_center_x=peer_x, lateral_center_y=peer_y
                )
                clearance = self.picker.polyline_min_distance(
                    trial, peer_x, peer_y
                )
                if clearance > achieved:
                    achieved = clearance
                    waypoints, chosen_depth = trial, cand
                if clearance >= min_peer_clearance:
                    waypoints, chosen_depth = trial, cand
                    achieved = clearance
                    break
            if achieved < min_peer_clearance:
                rospy.logerr(
                    "PEER DEADLOCK BYPASS ARC REFUSED -- planned clearance "
                    "%.3f < required %.2f even at max depth_radius %.2f "
                    "(peer=(%.2f, %.2f) sits too close to our own lane "
                    "depth for any arc this corridor allows) -- falling "
                    "back to reroute/retreat instead of driving it",
                    achieved, min_peer_clearance, max_depth_radius,
                    peer_x, peer_y
                )
                return False
            rospy.loginfo(
                "PEER DEADLOCK BYPASS ARC verified clearance=%.3f "
                "(required %.2f) depth_radius=%.2f lateral=%.2f "
                "waypoints=%d",
                achieved, min_peer_clearance, chosen_depth, radius,
                len(waypoints)
            )
            # base_yaw always points in the +index direction. `direction` here is the
            # endcap side, which is opposite to the direction the robot continues in,
            # so flip the yaw for direction > 0.
            waypoint_yaw = self.picker.base_yaw
            if direction > 0:
                waypoint_yaw = self.picker.normalize_angle(waypoint_yaw + math.pi)

            rospy.logerr(
                "PEER DEADLOCK BYPASS ARC step 2: anchor=(%.2f, %.2f) "
                "reached -- looping, waypoints=%d",
                self.picker.x, self.picker.y, len(waypoints)
            )
            for wx, wy in waypoints:
                # Reset the best distance for each waypoint: the distance to a static peer
                # naturally rises and falls along the arc.
                best_dist_seen[0] = None
                ok = self.picker.go_to_point_holonomic(
                    wx, wy, waypoint_yaw,
                    peer_clearance_fn=hard_abort_check
                )
                if not ok:
                    rospy.logerr(
                        "PEER DEADLOCK BYPASS ARC waypoint failed at "
                        "(%.2f, %.2f) -- falling back", wx, wy
                    )
                    return False
            rospy.logerr(
                "PEER DEADLOCK BYPASS ARC complete current=%s -- "
                "resuming toward %s", self.current_loc, target_loc
            )
            return True
        except BypassArcAborted as exc:
            # Make sure the base is stopped before leaving.
            self.picker.stop()
            rospy.logerr(
                "PEER DEADLOCK BYPASS ARC ABORTED dist=%.3f "
                "(absolute_min=%.3f closing_epsilon=%.3f) -- falling back "
                "to reroute/retreat",
                exc.dist, self.peer_deadlock_absolute_min_distance,
                self.peer_deadlock_closing_epsilon
            )
            return False
        finally:
            fleet_arbiter.clear_maneuvering(self.robot_id)

    def resolve_peer_deadlock(self, contested_loc, target_loc):
        """Resolve a confirmed deadlock: bypass arc, then reroute, then retreat.

        Returns ("bypassed", None) or ("retreat", None) when the robot has moved
        and the caller should retry the action, or ("reroute", [loc, ...]) with
        new MOVE locations to insert in place of the failed action.
        """
        # Always stop first; the caller may not have zeroed the velocity.
        self.picker.stop()
        if self.attempt_bypass_arc(target_loc):
            return "bypassed", None

        # If the contested location is our own position or target, blocking it
        # would be undone below and the reroute would return the same route.
        # Go straight to retreat in that case.
        skip_reroute = contested_loc in (self.current_loc, target_loc)
        if skip_reroute:
            rospy.logerr(
                "PEER DEADLOCK REROUTE SKIPPED current=%s target=%s "
                "contested=%s -- blocking it has no effect (it IS our "
                "current position or target), going straight to retreat "
                "instead of repeating the identical stuck route",
                self.current_loc, target_loc, contested_loc
            )

        now = time.monotonic()
        self.temporarily_blocked_locations = {
            loc: expiry
            for loc, expiry in self.temporarily_blocked_locations.items()
            if expiry > now
        }
        self.temporarily_blocked_locations[contested_loc] = (
            now + self.peer_deadlock_block_expiry
        )
        # Never block our own position or the target.
        blocked = set(self.temporarily_blocked_locations) - {
            self.current_loc, target_loc
        }

        route = None
        if not skip_reroute:
            try:
                if self.route_strategy in ("time_aware", "time_aware_global"):
                    route, _ = self.shortest_time_route(
                        self.current_loc, target_loc, blocked_locations=blocked
                    )
                else:
                    route = self.shortest_route(
                        self.current_loc, target_loc, blocked_locations=blocked
                    )
            except RuntimeError:
                route = None

        if route:
            rospy.logerr(
                "PEER DEADLOCK REROUTE current=%s target=%s avoiding=%s "
                "route=%s",
                self.current_loc, target_loc, sorted(blocked), route
            )
            return "reroute", route

        rospy.logerr(
            "PEER DEADLOCK RETREAT current=%s target=%s avoiding=%s -- "
            "no alternate route exists, retreating to last safe station",
            self.current_loc, target_loc, sorted(blocked)
        )
        # No detour: back off to our own last station (we still hold its
        # reservation). No full peer wait here, since this motion is the escape
        # itself, but the same two-level distance check as the bypass arc is used
        # so both robots cannot drive into the same point.
        retreat_passage, _stale_retreat_index = self.loc_to_passage_index(
            self.current_loc
        )
        # Use the physical index, current_loc can lag during a segment.
        retreat_index = self._physical_index_estimate(retreat_passage)
        best_retreat_dist = [None]

        def retreat_hard_abort_check():
            if not self.peer_pose_received:
                return
            dist = math.hypot(
                self.picker.x - self.peer_x, self.picker.y - self.peer_y
            )
            # Abort on a stale peer pose.
            if self._peer_pose_stale():
                self.picker.stop()
                raise BypassArcAborted(dist)
            if dist < self.peer_deadlock_absolute_min_distance:
                self.picker.stop()
                raise BypassArcAborted(dist)
            if best_retreat_dist[0] is None or dist > best_retreat_dist[0]:
                best_retreat_dist[0] = dist
                return
            # Same distance gate as for the bypass arc.
            if (
                dist < self.peer_deadlock_closing_guard_distance
                and best_retreat_dist[0] - dist
                > self.peer_deadlock_closing_epsilon
            ):
                self.picker.stop()
                raise BypassArcAborted(dist)

        retreat_confirmed = False
        retreat_abort_dist = None
        try:
            retreated = self.picker.go_to_station(
                retreat_passage, retreat_index,
                peer_clearance_fn=retreat_hard_abort_check
            )
            if retreated:
                retreat_confirmed = True
            else:
                # go_to_station can also time out for reasons unrelated to the peer.
                rospy.logwarn(
                    "PEER DEADLOCK RETREAT did not confirm reaching %s_I%s "
                    "(go_to_station returned False, not peer-aborted)",
                    retreat_passage, retreat_index
                )
        except BypassArcAborted as exc:
            retreat_abort_dist = exc.dist
            rospy.logerr(
                "PEER DEADLOCK RETREAT ABORTED dist=%.3f "
                "(absolute_min=%.3f closing_epsilon=%.3f) -- peer was "
                "also converging on the same retreat point, stopped short "
                "of contact instead of driving blind",
                exc.dist, self.peer_deadlock_absolute_min_distance,
                self.peer_deadlock_closing_epsilon
            )

        # If the retreat was not confirmed, wait (bounded) for the distance to grow
        # by ~peer_deadlock_retreat_progress_margin over the distance at which the
        # retreat stopped. Otherwise an immediate retry puts the robot back in the
        # same spot. If it still does not improve, return "retreat_unconfirmed".
        result_mode = "retreat"
        if not retreat_confirmed:
            if retreat_abort_dist is not None:
                baseline_dist = retreat_abort_dist
            elif self.peer_pose_received:
                baseline_dist = math.hypot(
                    self.picker.x - self.peer_x, self.picker.y - self.peer_y
                )
            else:
                baseline_dist = None

            confirm_deadline = (
                time.monotonic() + self.peer_deadlock_retreat_confirm_timeout
            )
            recovered = baseline_dist is None
            dist = baseline_dist
            while not recovered and time.monotonic() < confirm_deadline:
                if not self.peer_pose_received:
                    recovered = True
                    break
                dist = math.hypot(
                    self.picker.x - self.peer_x, self.picker.y - self.peer_y
                )
                required = max(
                    baseline_dist + self.peer_deadlock_retreat_progress_margin,
                    self.peer_deadlock_absolute_min_distance
                )
                if dist >= required:
                    recovered = True
                    break
                rospy.sleep(0.2)

            if recovered:
                rospy.loginfo(
                    "PEER DEADLOCK RETREAT recovered separation dist=%s "
                    "(baseline=%s) after an unconfirmed retreat",
                    "%.3f" % dist if dist is not None else "n/a",
                    "%.3f" % baseline_dist if baseline_dist is not None else "n/a"
                )
            else:
                rospy.logerr(
                    "PEER DEADLOCK RETREAT still unconfirmed after %.1fs "
                    "wait (baseline=%.3f last_dist=%.3f required>=%.3f) -- "
                    "reporting retreat_unconfirmed instead of retreat; "
                    "caller's own deadlock-retry budget still governs how "
                    "many more cycles are allowed",
                    self.peer_deadlock_retreat_confirm_timeout,
                    baseline_dist, dist,
                    baseline_dist + self.peer_deadlock_retreat_progress_margin
                )
                result_mode = "retreat_unconfirmed"

        rospy.sleep(self.peer_deadlock_retreat_grace)
        return result_mode, None

    def _lateral_bypass_closing(self, dist):
        """True if the distance to the peer is decreasing over the last window.

        Sampled once per ~lateral_closing_window (0.5 s); the cached result is
        returned in between. The first sample returns False; the hard_min floor
        in _peer_needs_hold() covers that first window.
        """
        now = time.monotonic()
        if (
            self._lateral_closing_ref_time is None
            or now - self._lateral_closing_ref_time >= self.lateral_closing_window
        ):
            closing = (
                self._lateral_closing_ref_dist is not None
                and dist < self._lateral_closing_ref_dist - self.lateral_closing_epsilon
            )
            self._lateral_closing_ref_dist = dist
            self._lateral_closing_ref_time = now
            self._lateral_closing_last_result = closing
            return closing
        return self._lateral_closing_last_result

    def _peer_needs_hold(self, dist, safety_radius, lateral_bypass):
        """Decide whether wait_for_moving_peer_clearance() should hold now.

        Normal mode: distance below threshold. Lateral-bypass mode: below threshold
        and the peer is closing. Below ~peer_proximity_hard_min_distance it always holds.
        """
        # Stale pose: always hold.
        if self._peer_pose_stale():
            return True
        if not lateral_bypass:
            return dist < safety_radius
        if dist < self.peer_proximity_hard_min_distance:
            return True
        if dist >= safety_radius:
            return False
        return self._lateral_bypass_closing(dist)

    def wait_for_moving_peer_clearance(self, context, target_loc=None, lateral_bypass=False):
        """Like wait_for_peer_clearance(), but called on every control tick during a
        continuous passage motion.

        Both robots stop when the peer is within ~peer_safety_radius. After
        ~peer_proximity_timeout (30 s) the priority robot resumes; the other waits
        up to ~peer_proximity_hard_timeout (180 s). Timers are measured per block
        episode (_enter_peer_block_episode()), and the distance is checked on every
        tick even after a bypass.

        target_loc: location the wait is about, published to fleet_arbiter for
        check_symmetric_deadlock(). The non-priority robot raises
        PeerDeadlockDetected when a mutual deadlock is confirmed.

        lateral_bypass: the robots are passing each other with a lane offset
        (see execute_passage_segment()). The hold distance is then
        ~lateral_bypass_hold_distance (1.0 m) instead of ~peer_safety_radius,
        and the robot holds only if the peer is still closing.
        """
        if not self.peer_pose_received:
            return

        # A finished peer uses the relaxed clearance and the plain threshold check,
        # also in lateral-bypass mode (the trend check oscillates against a parked
        # robot and the wait never escalates).
        peer_finished = fleet_arbiter.is_finished(self.peer_robot_id)
        safety_radius = (
            self.finished_peer_clearance_distance if peer_finished
            else (
                self.lateral_bypass_hold_distance if lateral_bypass
                else self.peer_safety_radius
            )
        )
        effective_lateral_bypass = lateral_bypass and not peer_finished

        dist = math.hypot(
            self.picker.x - self.peer_x, self.picker.y - self.peer_y
        )
        if not self._peer_needs_hold(dist, safety_radius, effective_lateral_bypass):
            self._lateral_closing_ref_dist = None
            self._lateral_closing_ref_time = None
            self._exit_peer_block_episode()
            return

        # Fast path for the priority robot: if the current episode has already
        # reached the resume condition, return without calling stop() again.
        # Calling stop() on every tick caused stop/start jerking and cross-track drift.
        # Near-contact and a maneuvering peer still go through the full check below.
        existing_episode = self._peer_block_episode
        if (
            self.peer_has_priority
            and existing_episode is not None
            and (time.monotonic() - existing_episode["started_at"])
                > self.peer_proximity_timeout
            and dist >= (
                self.peer_proximity_hard_min_distance
                + self.peer_proximity_resume_margin
            )
            and not fleet_arbiter.is_maneuvering(self.peer_robot_id)
        ):
            return

        self.picker.stop()
        started_at = self._enter_peer_block_episode(context, target_loc)
        holding = False
        while True:
            dist = math.hypot(
                self.picker.x - self.peer_x, self.picker.y - self.peer_y
            )
            if not self._peer_needs_hold(dist, safety_radius, effective_lateral_bypass):
                self._lateral_closing_ref_dist = None
                self._lateral_closing_ref_time = None
                if holding:
                    rospy.loginfo(
                        "PEER PROXIMITY (moving) RELEASED context=%s "
                        "dist=%.2f limit=%.2f peer=%s",
                        context, dist, safety_radius,
                        self.peer_robot_id
                    )
                self._exit_peer_block_episode()
                return

            if not holding:
                rospy.logwarn(
                    "PEER PROXIMITY (moving) HOLD context=%s dist=%.2f "
                    "limit=%.2f peer=%s (stopped mid-segment)%s",
                    context, dist, safety_radius,
                    self.peer_robot_id,
                    " [lateral bypass]" if lateral_bypass else ""
                )
                holding = True

            if not self.peer_has_priority:
                contested = self.check_symmetric_deadlock()
                if contested is not None:
                    rospy.logerr(
                        "PEER DEADLOCK CONFIRMED context=%s "
                        "contested=%s dist=%.2f after %.1fs -- "
                        "rerouting instead of continuing to wait",
                        context, contested, dist,
                        time.monotonic() - started_at
                    )
                    self._exit_peer_block_episode()
                    raise PeerDeadlockDetected(contested, target_loc)

            elapsed = time.monotonic() - started_at

            if (
                not self.peer_has_priority
                # A finished peer also uses the fast escalation.
                and not effective_lateral_bypass
                and elapsed > self.peer_bypass_speculative_threshold
            ):
                # A pure proximity standoff never shows up as a reservation conflict, so try
                # a bypass arc early instead of waiting the full hard timeout.
                # Not in lateral-bypass mode: mid-row the arc is not possible and the only
                # result would be a retreat that undoes the pass; use the 180 s timeout there.
                rospy.logerr(
                    "PEER PROXIMITY (moving) SPECULATIVE BYPASS TRIGGER "
                    "context=%s dist=%.2f after %.1fs -- no confirmed "
                    "two-way deadlock, but trying bypass/reroute/retreat "
                    "now instead of waiting for the full hard timeout",
                    context, dist, elapsed
                )
                self._exit_peer_block_episode()
                raise PeerDeadlockDetected(target_loc, target_loc)

            if elapsed > self.peer_proximity_timeout:
                if self.peer_has_priority:
                    # Resume threshold with hysteresis (see ~peer_proximity_resume_margin).
                    resume_floor = (
                        self.finished_peer_clearance_distance
                        if fleet_arbiter.is_finished(self.peer_robot_id)
                        else self.peer_proximity_hard_min_distance
                        + self.peer_proximity_resume_margin
                    )
                    if dist < resume_floor:
                        # Near-contact floor for both robots, independent of priority.
                        # Also hold while the peer announces a maneuver (bypass arc), since the
                        # arc's planned clearance assumes this robot does not move.
                        if fleet_arbiter.is_maneuvering(self.peer_robot_id):
                            rospy.logwarn_throttle(
                                5.0,
                                "PEER PROXIMITY (moving) HOLD context=%s "
                                "dist=%.2f -- peer=%s is mid-maneuver, "
                                "holding regardless of priority "
                                "(near-contact)",
                                context, dist, self.peer_robot_id
                            )
                            rospy.sleep(0.2)
                            continue
                        # Near-contact timeout for the priority robot.
                        if elapsed > (
                            self.peer_proximity_priority_nearcontact_timeout
                        ):
                            # Peer has not moved away for a long time (probably stopped for good):
                            # escalate to reroute/retreat instead of holding forever.
                            rospy.logerr(
                                "PEER PROXIMITY (moving) HARD_MIN "
                                "TIMEOUT context=%s dist=%.2f still "
                                "below resume_floor=%.2f (hard_min=%.2f "
                                "+margin=%.2f) after %.1fs -- peer "
                                "likely gone/stuck, escalating to "
                                "reroute/retreat instead of holding "
                                "forever",
                                context, dist, resume_floor,
                                self.peer_proximity_hard_min_distance,
                                self.peer_proximity_resume_margin,
                                elapsed
                            )
                            self._exit_peer_block_episode()
                            raise PeerDeadlockDetected(target_loc, target_loc)
                        rospy.logwarn(
                            "PEER PROXIMITY (moving) TIMEOUT context=%s "
                            "dist=%.2f below resume_floor=%.2f "
                            "(hard_min=%.2f) -- holding despite priority "
                            "(near-contact range, not a yield)",
                            context, dist, resume_floor,
                            self.peer_proximity_hard_min_distance
                        )
                        rospy.sleep(0.2)
                        continue
                    if fleet_arbiter.is_maneuvering(self.peer_robot_id):
                        # Peer is in a bypass arc: keep holding until it ends or expires.
                        rospy.logwarn(
                            "PEER PROXIMITY (moving) TIMEOUT context=%s "
                            "dist=%.2f but peer=%s is mid-BYPASS ARC -- "
                            "holding despite priority",
                            context, dist, self.peer_robot_id
                        )
                        rospy.sleep(0.2)
                        continue
                    rospy.logerr(
                        "PEER PROXIMITY (moving) TIMEOUT context=%s "
                        "dist=%.2f limit=%.2f after %.1fs -- resuming "
                        "(priority; episode start persisted, so this "
                        "resolves immediately again on the next tick "
                        "the peer is still this close)",
                        context, dist, safety_radius,
                        self.peer_proximity_timeout
                    )
                    return
                if elapsed > self.peer_proximity_hard_timeout:
                    # 180 s of continuous blocking: reroute/retreat instead of proceeding.
                    rospy.logerr(
                        "PEER PROXIMITY (moving) HARD TIMEOUT context=%s "
                        "dist=%.2f limit=%.2f after %.1fs total -- no "
                        "confirmed two-way deadlock, but rerouting anyway "
                        "rather than resuming blind",
                        context, dist, safety_radius,
                        self.peer_proximity_hard_timeout
                    )
                    self._exit_peer_block_episode()
                    raise PeerDeadlockDetected(target_loc, target_loc)

            rospy.sleep(0.2)

    def reserve_locations(self, locs, context):
        """Block until all `locs` are reserved for this robot in fleet_arbiter.

        On ~fleet_reservation_timeout the priority robot proceeds; the other robot
        keeps retrying until ~fleet_reservation_hard_timeout. This resolves the
        swap case where two robots on adjacent nodes each want the other's node.
        """
        if not self.fleet_reservation_enabled:
            return True

        hard_deadline = time.monotonic() + self.fleet_reservation_hard_timeout
        started_at = self._enter_peer_block_episode(context, locs[0] if locs else None)
        fleet_arbiter.set_waiting(self.robot_id, locs)
        while True:
            ok = fleet_arbiter.reserve(
                self.robot_id, locs, timeout=self.fleet_reservation_timeout
            )
            if ok:
                self._exit_peer_block_episode()
                return True

            if self.peer_has_priority:
                rospy.logerr(
                    "FLEET RESERVATION TIMEOUT context=%s locs=%s after "
                    "%.1fs -- proceeding anyway (priority)",
                    context, locs, self.fleet_reservation_timeout
                )
                self._exit_peer_block_episode()
                return True

            # Escalate early on a confirmed mutual deadlock.
            contested = self.check_symmetric_deadlock()
            if contested is not None:
                rospy.logerr(
                    "PEER DEADLOCK CONFIRMED context=%s locs=%s "
                    "contested=%s after %.1fs -- rerouting instead of "
                    "continuing to wait",
                    context, locs, contested, time.monotonic() - started_at
                )
                self._exit_peer_block_episode()
                raise PeerDeadlockDetected(contested, locs[0] if locs else None)

            if time.monotonic() >= hard_deadline:
                # 180 s of contention: reroute/retreat.
                rospy.logerr(
                    "FLEET RESERVATION HARD TIMEOUT context=%s locs=%s "
                    "after %.1fs total -- no confirmed two-way deadlock, "
                    "but rerouting anyway rather than proceeding blind",
                    context, locs, self.fleet_reservation_hard_timeout
                )
                self._exit_peer_block_episode()
                raise PeerDeadlockDetected(
                    locs[0] if locs else None, locs[0] if locs else None
                )

            rospy.logwarn(
                "FLEET RESERVATION still contended context=%s locs=%s -- "
                "no priority, continuing to wait (peer likely mid-swap)",
                context, locs
            )

    def release_locations(self, locs):
        """Release locations this robot no longer occupies."""
        if self.fleet_reservation_enabled:
            fleet_arbiter.release(self.robot_id, locs)

    def loc_to_passage_index(self, loc):
        m = re.match(r"P(\d+)_I(\d+)", loc)
        if not m:
            raise RuntimeError("Bad loc: " + loc)
        return int(m.group(1)), int(m.group(2))

    def write_instance(self):
        locs = orchard_topology.base_locations(num_passages=5, num_indices=8)
        if self.allowed_locations:
            locs = [loc for loc in locs if loc in self.allowed_locations]

        # Shortcut midpoints are not filtered by ~allowed_locations.
        locs = locs + orchard_topology.shortcut_locations(self.shortcut_pairs)
        loc_set = set(locs)
        targets = list(self.targets.keys())

        if self.topology_mode == "line":
            edges = orchard_topology.line_edges(
                loc_set, num_passages=5, num_indices=8,
                redundant_pairs=self.topology_redundant_pairs
            )
        else:
            edges = orchard_topology.base_edges(loc_set, num_passages=5, num_indices=8)
        edges = edges + orchard_topology.shortcut_edges(self.shortcut_pairs, loc_set=loc_set)

        nf_lines = []
        for a, b in edges:
            nf_lines.append(f"        adjacent({a}, {b});")

        for tid, info in self.targets.items():
            nf_lines.append(f"        target_at({tid}, {info['loc']});")

        init_lines = [f"        at({self.current_loc});"]
        for tid in sorted(self.picked_targets, key=lambda x: int(x[1:])):
            init_lines.append(f"        picked({tid});")

        # The "line" topology uses a separate RDDL domain
        # (orchard_targets_line_mdp.rddl, see its header).
        domain_name = (
            "orchard_targets_line" if self.topology_mode == "line"
            else "orchard_targets"
        )
        instance = f"""
instance {self.instance_name} {{

    domain = {domain_name};

    objects {{
        loc : {{{", ".join(locs)}}};
        target : {{{", ".join(targets)}}};
    }};

    non-fluents {{
{chr(10).join(nf_lines)}
    }};

    init-state {{
{chr(10).join(init_lines)}
    }};

    horizon = {self.prost_horizon};
    discount = 1.0;
}}
""".strip()

        self.instance_path.write_text(instance + "\n")

    def cleanup_prost_processes(self):
        subprocess.run(
            ["fuser", "-k", "%d/tcp" % self.prost_port],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    def wait_for_prost_server(self, server_proc, timeout=8.0):
        """Wait for the RDDL server banner instead of a fixed startup sleep."""
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            if server_proc.poll() is not None:
                raise RuntimeError(
                    "PROST server exited during startup with code %s"
                    % server_proc.returncode
                )
            try:
                if "RDDL Server Initialized" in self.server_log.read_text():
                    return
            except (FileNotFoundError, OSError):
                pass
            time.sleep(0.10)
        raise RuntimeError("Timed out waiting for PROST RDDL server startup")

    def stop_server_process(self, server_proc):
        if server_proc is None:
            return

        try:
            pgid = os.getpgid(server_proc.pid)
        except Exception:
            pgid = None

        if server_proc.poll() is None:
            try:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGTERM)
                else:
                    server_proc.terminate()
                server_proc.wait(timeout=5)
            except Exception:
                try:
                    if pgid is not None:
                        os.killpg(pgid, signal.SIGKILL)
                    else:
                        server_proc.kill()
                except Exception:
                    pass

        subprocess.run(
            ["fuser", "-k", "%d/tcp" % self.prost_port],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    def run_prost_once(self):
        # No cross-robot lock: each robot runs the prost.py client from its own
        # scratch directory (prost.py copies binaries under fixed names into cwd).
        # The server can share testbed_dir/benchmarks: instance files and server
        # logs are already per robot / per port.
        self.cleanup_prost_processes()

        if self.server_log.exists():
            self.server_log.unlink()

        if hasattr(self, "prost_log") and self.prost_log.exists():
            self.prost_log.unlink()

        server_cmd = [
            "python3", "run-server.py",
            "-b", "benchmarks/orchard_targets",
            "--port", str(self.prost_port),
            "-r", "1",
            "-s", "1",
            "-t", "99999",
            "-l", "rddlsim_logs"
        ]

        server_proc = None

        try:
            with open(self.server_log, "w") as log:
                server_proc = subprocess.Popen(
                    server_cmd,
                    cwd=str(self.testbed_dir),
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid
                )

                self.wait_for_prost_server(server_proc)

                prost_cmd = [
                    str(self.prost_dir / "prost.py"),
                    self.instance_name,
                    "--port", str(self.prost_port),
                    "[PROST -s 1 -se [IPC2014]]"
                ]

                prost_proc = subprocess.run(
                    prost_cmd,
                    cwd=str(self.prost_client_scratch_dir),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=120
                )

                if hasattr(self, "prost_log"):
                    self.prost_log.write_text(prost_proc.stdout)

                return prost_proc.stdout

        finally:
            # Give the server time to flush its log before the safety timeout.
            if server_proc is not None and server_proc.poll() is None:
                try:
                    server_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass
            self.stop_server_process(server_proc)

    def parse_actions_until_pick(self):
        move_re = re.compile(r"move_to\((P\d+_I\d+)\)")
        pick_re = re.compile(r"pick\((T\d+)\)")
        drop_re = re.compile(r"\bdrop\b")

        actions = []

        with open(self.server_log) as f:
            for line in f:
                if "Actions received:" not in line:
                    continue

                moves = move_re.findall(line)
                picks = pick_re.findall(line)
                drops = drop_re.findall(line)

                for loc in moves:
                    actions.append(("MOVE", loc))
                    if len(actions) > 80:
                        return actions

                if drops:
                    actions.append(("DROP", ""))
                    continue

                for tid in picks:
                    actions.append(("PICK", tid))
                    return actions

        return actions

    def build_graph_edges(self, blocked_locations=None):
        """Build graph edges for routing.

        `blocked_locations`: optional set of locations to leave out for this call
        (used by resolve_peer_deadlock() for rerouting).
        """
        locs = orchard_topology.base_locations(num_passages=5, num_indices=8)
        if self.allowed_locations:
            locs = [loc for loc in locs if loc in self.allowed_locations]
        if blocked_locations:
            locs = [loc for loc in locs if loc not in blocked_locations]
        locs = locs + orchard_topology.shortcut_locations(self.shortcut_pairs)
        loc_set = set(locs)

        edges = {}
        for loc in locs:
            edges[loc] = []

        if self.topology_mode == "line":
            all_pairs = orchard_topology.line_edges(
                loc_set, num_passages=5, num_indices=8,
                redundant_pairs=self.topology_redundant_pairs
            )
        else:
            all_pairs = orchard_topology.base_edges(loc_set, num_passages=5, num_indices=8)
        all_pairs = all_pairs + orchard_topology.shortcut_edges(
            self.shortcut_pairs, loc_set=loc_set
        )
        for a, b in all_pairs:
            edges[a].append(b)

        return edges

    def shortest_route(self, start, goal, blocked_locations=None):
        if start == goal:
            return []

        edges = self.build_graph_edges(blocked_locations=blocked_locations)
        queue = [(start, [])]
        visited = set([start])

        while queue:
            cur, path = queue.pop(0)

            for nxt in edges.get(cur, []):
                if nxt in visited:
                    continue

                new_path = path + [nxt]

                if nxt == goal:
                    return new_path

                visited.add(nxt)
                queue.append((nxt, new_path))

        raise RuntimeError("No route found from %s to %s" % (start, goal))

    def estimate_route_travel_time(self, start, route):
        """Estimate physical travel time for an adjacent symbolic route.

        Passage edges use their measured continuous-motion cost. Consecutive
        transfers through the same endcap are cheaper after the first edge.
        Shortcut edges use their own flat costs. Diagnostic only, does not
        affect route selection.
        """
        previous = start
        previous_was_transfer = False
        previous_transfer_index = None
        passage_edges = 0
        first_transfers = 0
        continuation_transfers = 0
        feeder_edges = 0
        crossing_edges = 0
        total = 0.0

        for loc in route:
            classified = orchard_topology.classify_edge(
                previous, loc, num_indices=self.picker.num_indices
            )
            kind = classified[0]

            if kind == "passage":
                total += self.route_passage_edge_cost
                passage_edges += 1
                previous_was_transfer = False
                previous_transfer_index = None
            elif kind == "transfer":
                cur_index = classified[3]
                continuation = (
                    previous_was_transfer
                    and previous_transfer_index == cur_index
                )
                if continuation:
                    total += self.route_transfer_continuation_cost
                    continuation_transfers += 1
                else:
                    total += self.route_transfer_first_cost
                    first_transfers += 1
                previous_was_transfer = True
                previous_transfer_index = cur_index
            elif kind == "feeder":
                total += self.route_shortcut_feeder_cost
                feeder_edges += 1
                previous_was_transfer = False
                previous_transfer_index = None
            elif kind == "crossing":
                total += self.route_shortcut_crossing_cost
                crossing_edges += 1
                previous_was_transfer = False
                previous_transfer_index = None
            else:
                raise RuntimeError(
                    "Cannot estimate invalid route edge %s -> %s"
                    % (previous, loc)
                )

            previous = loc

        result = {
            "seconds": total,
            "passage_edges": passage_edges,
            "first_transfers": first_transfers,
            "continuation_transfers": continuation_transfers,
        }
        if feeder_edges or crossing_edges:
            result["feeder_edges"] = feeder_edges
            result["crossing_edges"] = crossing_edges
        return result

    def shortest_time_route(self, start, goal, blocked_locations=None):
        """Return the lowest measured-time route between two locations."""
        if start == goal:
            return [], 0.0

        graph = self.build_graph_edges(blocked_locations=blocked_locations)
        # State includes the active transfer endcap, so consecutive passage
        # changes use the continuation cost.
        tie_breaker = itertools.count()
        queue = [(0.0, next(tie_breaker), start, None, [])]
        best = {(start, None): 0.0}

        while queue:
            cost, _, current, transfer_index, path = heapq.heappop(queue)
            state = (current, transfer_index)
            if cost > best.get(state, float("inf")) + 1e-9:
                continue
            if current == goal:
                return path, cost

            for nxt in graph[current]:
                classified = orchard_topology.classify_edge(
                    current, nxt, num_indices=self.picker.num_indices
                )
                kind = classified[0]
                if kind == "passage":
                    edge_cost = self.route_passage_edge_cost
                    next_transfer_index = None
                elif kind == "transfer":
                    cur_index = classified[3]
                    continuation = transfer_index == cur_index
                    edge_cost = (
                        self.route_transfer_continuation_cost
                        if continuation
                        else self.route_transfer_first_cost
                    )
                    next_transfer_index = cur_index
                elif kind == "feeder":
                    edge_cost = self.route_shortcut_feeder_cost
                    next_transfer_index = None
                elif kind == "crossing":
                    edge_cost = self.route_shortcut_crossing_cost
                    next_transfer_index = None
                else:
                    raise RuntimeError(
                        "shortest_time_route: invalid edge %s -> %s"
                        % (current, nxt)
                    )

                next_cost = cost + edge_cost
                next_state = (nxt, next_transfer_index)
                if next_cost + 1e-9 < best.get(next_state, float("inf")):
                    best[next_state] = next_cost
                    heapq.heappush(
                        queue,
                        (
                            next_cost,
                            next(tie_breaker),
                            nxt,
                            next_transfer_index,
                            path + [nxt]
                        )
                    )

        raise RuntimeError("No time-aware route from %s to %s" % (start, goal))

    def plan_time_aware_current_basket(self, forced_first=None):
        """Exact optimization of the remaining targets that fit in this basket.

        Maximizes the number of fruits in the basket, then minimizes travel time
        including the return to the depot. Diagnostic only.
        """
        remaining_capacity = max(0, self.basket_capacity - self.basket_load)
        remaining_targets = [
            tid for tid in sorted(self.targets, key=lambda value: int(value[1:]))
            if tid not in self.picked_targets
        ]
        fruit_cost = {}
        for tid in remaining_targets:
            completed = self.picked_fruit_parts.get(tid, set())
            fruit_cost[tid] = max(0, 2 - len(completed))

        pair_cache = {}

        def leg(start, goal):
            key = (start, goal)
            if key not in pair_cache:
                pair_cache[key] = self.shortest_time_route(start, goal)
            return pair_cache[key]

        best_plan = None

        def consider(order, collected, travel_cost, current_loc):
            nonlocal best_plan
            _, depot_cost = leg(current_loc, self.depot_loc)
            total = travel_cost + depot_cost
            candidate = {
                "order": list(order),
                "fruits": collected,
                "seconds": total,
            }
            score = (-collected, total, tuple(order))
            if best_plan is None or score < best_plan[0]:
                best_plan = (score, candidate)

        def search(current_loc, available, capacity, order, collected, cost):
            feasible = [
                tid for tid in available
                if 0 < fruit_cost[tid] <= capacity
            ]
            if not feasible:
                consider(order, collected, cost, current_loc)
                return

            for tid in feasible:
                target_loc = self.targets[tid]["loc"]
                _, leg_cost = leg(current_loc, target_loc)
                search(
                    target_loc,
                    [value for value in available if value != tid],
                    capacity - fruit_cost[tid],
                    order + [tid],
                    collected + fruit_cost[tid],
                    cost + leg_cost,
                )

        if forced_first is not None:
            if (
                forced_first not in remaining_targets
                or fruit_cost[forced_first] <= 0
                or fruit_cost[forced_first] > remaining_capacity
            ):
                return None
            first_loc = self.targets[forced_first]["loc"]
            _, first_cost = leg(self.current_loc, first_loc)
            search(
                first_loc,
                [tid for tid in remaining_targets if tid != forced_first],
                remaining_capacity - fruit_cost[forced_first],
                [forced_first],
                fruit_cost[forced_first],
                first_cost,
            )
        else:
            search(
                self.current_loc,
                remaining_targets,
                remaining_capacity,
                [],
                0,
                0.0,
            )

        return best_plan[1] if best_plan is not None else None

    def plan_time_aware_global_mission(self):
        """Optimize all remaining targets across as many basket tours as needed."""
        target_ids = [
            tid for tid in sorted(self.targets, key=lambda value: int(value[1:]))
            if tid not in self.picked_targets
        ]
        fruit_cost = {}
        for tid in target_ids:
            completed = self.picked_fruit_parts.get(tid, set())
            fruit_cost[tid] = max(0, 2 - len(completed))

        pair_cost = {}

        def travel(start, goal):
            key = (start, goal)
            if key not in pair_cost:
                pair_cost[key] = self.shortest_time_route(start, goal)[1]
            return pair_cost[key]

        full_mask = (1 << len(target_ids)) - 1
        memo = {}

        def solve(current_loc, mask, basket_load):
            key = (current_loc, mask, basket_load)
            if key in memo:
                return memo[key]

            if mask == 0:
                if current_loc == self.depot_loc and basket_load == 0:
                    result = (0.0, 0, [])
                else:
                    result = (
                        travel(current_loc, self.depot_loc),
                        1 if basket_load > 0 else 0,
                        ["DEPOT"]
                    )
                memo[key] = result
                return result

            candidates = []

            # An early depot return is allowed whenever the current basket is
            # non-empty.  This enables globally optimal 2+4+4 partitions.
            if basket_load > 0:
                rest_cost, rest_drops, rest_plan = solve(
                    self.depot_loc, mask, 0
                )
                candidates.append((
                    travel(current_loc, self.depot_loc) + rest_cost,
                    1 + rest_drops,
                    ["DEPOT"] + rest_plan
                ))

            for index, tid in enumerate(target_ids):
                bit = 1 << index
                needed = fruit_cost[tid]
                if not (mask & bit):
                    continue
                if needed <= 0 or basket_load + needed > self.basket_capacity:
                    continue

                target_loc = self.targets[tid]["loc"]
                rest_cost, rest_drops, rest_plan = solve(
                    target_loc,
                    mask ^ bit,
                    basket_load + needed
                )
                candidates.append((
                    travel(current_loc, target_loc) + rest_cost,
                    rest_drops,
                    [tid] + rest_plan
                ))

            if not candidates:
                raise RuntimeError(
                    "Global route planner found no action at loc=%s load=%d mask=%d"
                    % (current_loc, basket_load, mask)
                )

            # Prefer lower travel time, then fewer depot visits, then a stable
            # numeric/lexical plan for deterministic tests.
            result = min(
                candidates,
                key=lambda item: (item[0], item[1], tuple(item[2]))
            )
            memo[key] = result
            return result

        seconds, drops, sequence = solve(
            self.current_loc, full_mask, self.basket_load
        )
        return {
            "sequence": sequence,
            "seconds": seconds,
            "drops": drops,
        }

    def log_time_aware_observer(self, actions):
        prost_first = next(
            (arg for action, arg in actions if action == "PICK"), None
        )
        if prost_first is None:
            return

        optimal = self.plan_time_aware_current_basket()
        forced = self.plan_time_aware_current_basket(
            forced_first=prost_first
        )
        if optimal is None or forced is None:
            rospy.logwarn(
                "TIME-AWARE OBSERVER could not score prost_first=%s",
                prost_first
            )
            return

        recommended = optimal["order"][0] if optimal["order"] else "none"
        savings = forced["seconds"] - optimal["seconds"]
        rospy.loginfo(
            "TIME-AWARE OBSERVER prost_first=%s recommended_first=%s "
            "optimal_order=%s optimal_seconds=%.1f "
            "prost_forced_order=%s prost_forced_seconds=%.1f savings=%.1f "
            "basket_remaining=%d planned_fruits=%d selection_active=%s",
            prost_first,
            recommended,
            optimal["order"],
            optimal["seconds"],
            forced["order"],
            forced["seconds"],
            savings,
            self.basket_capacity - self.basket_load,
            optimal["fruits"],
            self.route_strategy in ("time_aware", "time_aware_global")
        )

    def optimize_actions_to_first_pick(self, actions):
        if not self.optimize_plan_route:
            return actions

        pick_tid = None
        for action, arg in actions:
            if action == "PICK":
                pick_tid = arg
                break

        if pick_tid is None:
            return actions

        if pick_tid not in self.targets:
            return actions

        prost_pick_tid = pick_tid
        route_cost = None
        if self.route_strategy in ("time_aware", "time_aware_global"):
            if self.route_strategy == "time_aware_global":
                global_plan = self.plan_time_aware_global_mission()
                selectable = [
                    item for item in global_plan["sequence"] if item != "DEPOT"
                ]
                plan_order = selectable
                plan_seconds = global_plan["seconds"]
            else:
                plan = self.plan_time_aware_current_basket()
                plan_order = plan["order"] if plan is not None else []
                plan_seconds = plan["seconds"] if plan is not None else 0.0

            if plan_order:
                pick_tid = plan_order[0]
                rospy.loginfo(
                    "TIME-AWARE ACTIVE prost_first=%s selected_first=%s "
                    "basket_plan=%s estimated_tour_seconds=%.1f",
                    prost_pick_tid,
                    pick_tid,
                    plan_order,
                    plan_seconds
                )
            else:
                rospy.logwarn(
                    "TIME-AWARE ACTIVE planning failed; using prost_first=%s",
                    prost_pick_tid
                )

        target_loc = self.targets[pick_tid]["loc"]
        if self.route_strategy in ("time_aware", "time_aware_global"):
            route, route_cost = self.shortest_time_route(
                self.current_loc, target_loc
            )
        else:
            route = self.shortest_route(self.current_loc, target_loc)

        optimized = [("MOVE", loc) for loc in route]
        optimized.append(("PICK", pick_tid))

        rospy.loginfo(
            "ROUTE OPTIMIZATION prost_first=%s selected_first=%s "
            "target_loc=%s route_cost=%s original_actions=%s "
            "optimized_actions=%s",
            prost_pick_tid,
            pick_tid,
            target_loc,
            "%.1f" % route_cost if route_cost is not None else "unweighted",
            actions,
            optimized
        )

        return optimized

    def log_route_segments(self, actions):
        """Log the physical route segments without changing how they are executed.

        Consecutive moves in one passage are grouped into a continuous segment;
        passage changes happen only at endcaps I1 and I8.
        """
        move_locs = [arg for action, arg in actions if action == "MOVE"]

        if not move_locs:
            rospy.loginfo("ROUTE ANALYSIS no MOVE actions")
            return

        estimate = self.estimate_route_travel_time(
            self.current_loc, move_locs
        )
        rospy.loginfo(
            "ROUTE TIME ESTIMATE start=%s end=%s seconds=%.1f "
            "passage_edges=%d first_transfers=%d continuation_transfers=%d",
            self.current_loc,
            move_locs[-1],
            estimate["seconds"],
            estimate["passage_edges"],
            estimate["first_transfers"],
            estimate["continuation_transfers"]
        )

        route = [self.current_loc] + move_locs
        segment_start = None
        segment_end = None
        segment_passage = None
        segment_count = 0
        transfer_count = 0
        invalid_count = 0

        def flush_segment():
            nonlocal segment_start, segment_end, segment_passage, segment_count
            if segment_start is None:
                return
            start_p, start_i = self.loc_to_passage_index(segment_start)
            end_p, end_i = self.loc_to_passage_index(segment_end)
            direction = 1 if end_i > start_i else -1
            rospy.loginfo(
                "ROUTE SEGMENT passage=P%d start=I%d end=I%d direction=%+d edges=%d",
                segment_passage,
                start_i,
                end_i,
                direction,
                abs(end_i - start_i)
            )
            segment_count += 1
            segment_start = None
            segment_end = None
            segment_passage = None

        shortcut_count = 0

        for previous, current in zip(route, route[1:]):
            classified = orchard_topology.classify_edge(
                previous, current, num_indices=self.picker.num_indices
            )
            kind = classified[0]

            if kind == "passage":
                prev_passage = classified[1]
                if segment_start is None:
                    segment_start = previous
                    segment_passage = prev_passage
                segment_end = current
                continue

            flush_segment()

            if kind == "transfer":
                _, prev_passage, cur_passage, cur_index = classified
                rospy.loginfo(
                    "ROUTE TRANSFER from=P%d_I%d to=P%d_I%d endcap=I%d",
                    prev_passage,
                    cur_index,
                    cur_passage,
                    cur_index,
                    cur_index
                )
                transfer_count += 1
            elif kind in ("feeder", "crossing"):
                rospy.loginfo(
                    "ROUTE SHORTCUT kind=%s from=%s to=%s",
                    kind,
                    previous,
                    current
                )
                shortcut_count += 1
            else:
                rospy.logerr(
                    "ROUTE INVALID EDGE from=%s to=%s",
                    previous,
                    current
                )
                invalid_count += 1

        flush_segment()
        if shortcut_count:
            rospy.loginfo(
                "ROUTE ANALYSIS summary moves=%d segments=%d transfers=%d "
                "shortcuts=%d invalid=%d",
                len(move_locs),
                segment_count,
                transfer_count,
                shortcut_count,
                invalid_count
            )
        else:
            rospy.loginfo(
                "ROUTE ANALYSIS summary moves=%d segments=%d transfers=%d invalid=%d",
                len(move_locs),
                segment_count,
                transfer_count,
                invalid_count
            )


    def execute_move(self, loc):
        # Proximity check; covers ordinary, feeder and crossing moves.
        self.wait_for_peer_clearance("execute_move:%s" % loc)
        # Node reservation.
        self.reserve_locations([loc], "execute_move:%s" % loc)

        try:
            classified = orchard_topology.classify_edge(
                self.current_loc, loc, num_indices=self.picker.num_indices
            )
            edge_kind = classified[0]

            if edge_kind == "crossing":
                # Mid-row shortcut crossing. The robot is already at the near-side gap
                # (preceding feeder move).
                _, passage, next_passage, gap_index = classified
                rospy.loginfo(
                    "ONLINE MOVE %s -> %s (shortcut crossing)", self.current_loc, loc
                )
                ok = self.picker.cross_shortcut(
                    passage, next_passage, gap_index,
                    peer_clearance_fn=lambda: self.wait_for_moving_peer_clearance(
                        "execute_move:%s" % loc, target_loc=loc
                    )
                )
                if ok:
                    self.pick_departure_pending = False
            elif edge_kind == "feeder":
                # Station <-> shortcut midpoint within one passage. The midpoint has a
                # fractional index, so use the short holonomic hop instead of
                # follow_passage_segment().
                target_passage, target_index = orchard_topology.loc_fractional_index(loc)
                target_x, target_y = self.picker.passage_pose(target_passage, target_index)
                rospy.loginfo(
                    "ONLINE MOVE %s -> %s (shortcut feeder) target=(%.3f, %.3f)",
                    self.current_loc, loc, target_x, target_y
                )
                ok = self.picker.go_to_point_holonomic(
                    target_x, target_y, self.picker.base_yaw,
                    tolerance=self.picker.shortcut_align_tolerance,
                    peer_clearance_fn=lambda: self.wait_for_moving_peer_clearance(
                        "execute_move:%s" % loc, target_loc=loc
                    )
                )
                if ok:
                    self.pick_departure_pending = False
            else:
                passage, index = self.loc_to_passage_index(loc)
                cur_passage, cur_index = self.loc_to_passage_index(self.current_loc)

                rospy.loginfo("ONLINE MOVE %s -> %s", self.current_loc, loc)

                same_passage_edge = (
                    passage == cur_passage and abs(index - cur_index) == 1
                )
                if self.pick_departure_pending and same_passage_edge:
                    direction = 1 if index > cur_index else -1
                    ok = self.picker.merge_into_passage(
                        cur_passage, direction,
                        peer_clearance_fn=lambda: self.wait_for_moving_peer_clearance(
                            "execute_move:%s" % loc, target_loc=loc
                        )
                    )
                    if ok:
                        self.pick_departure_pending = False
                        # Continuous peer check during the move.
                        ok = self.picker.follow_passage_segment(
                            cur_passage, cur_index, index,
                            peer_clearance_fn=lambda: self.wait_for_moving_peer_clearance(
                                "execute_move:%s" % loc, target_loc=loc
                            )
                        )
                elif abs(passage - cur_passage) == 1 and index == cur_index and (index == 1 or index == self.picker.num_indices):
                    if self.pick_departure_pending:
                        rospy.logwarn(
                            "PICK DEPARTURE fallback before transfer from=%s to=%s",
                            self.current_loc, loc
                        )
                        ok = self.picker.go_to_station(
                            cur_passage, cur_index,
                            peer_clearance_fn=lambda: self.wait_for_moving_peer_clearance(
                                "execute_move:%s" % loc, target_loc=loc
                            )
                        )
                        self.pick_departure_pending = False
                        if not ok:
                            self.release_locations([loc])
                            return False
                    direction = 1 if index == self.picker.num_indices else -1
                    ok = self.picker.transfer_to_next_passage(
                        cur_passage, passage, direction,
                        peer_clearance_fn=lambda: self.wait_for_moving_peer_clearance(
                            "execute_move:%s" % loc, target_loc=loc
                        )
                    )
                else:
                    # Continuous peer check for single-index hops
                    # (continuous_passage_motion:=false).
                    ok = self.picker.go_to_station(
                        passage, index,
                        peer_clearance_fn=lambda: self.wait_for_moving_peer_clearance(
                            "execute_move:%s" % loc, target_loc=loc
                        )
                    )
                    if ok:
                        self.pick_departure_pending = False

        except PeerDeadlockDetected:
            # Release the target before propagating; the caller reroutes or retreats.
            self.release_locations([loc])
            raise
        # Release what we no longer occupy, keep holding the current location.
        old_loc = self.current_loc
        if ok:
            self.release_locations([old_loc])
            self.current_loc = loc
            self._log_field_move(old_loc, loc, edge_kind or "move")
        else:
            self.release_locations([loc])

        self.writer.writerow([
            "",
            "MOVE",
            "",
            loc,
            "",
            "",
            ok,
            "",
            "",
            "",
            len(self.picked_targets),
            self.basket_load,
            self.picker.x,
            self.picker.y,
            self.picker.yaw
        ])
        self.log_file.flush()

        return ok

    def execute_passage_segment(self, move_locs):
        """Execute validated, adjacent MOVE actions as one straight motion."""
        if not move_locs:
            return True

        # Validate the segment and decide on lateral bypass before the
        # peer-clearance pre-check below.
        start_loc = self.current_loc
        start_passage, start_index = self.loc_to_passage_index(start_loc)
        previous_index = start_index
        segment_direction = None

        for loc in move_locs:
            passage, index = self.loc_to_passage_index(loc)
            edge_direction = index - previous_index
            if segment_direction is None:
                segment_direction = edge_direction
            if (
                passage != start_passage
                or abs(edge_direction) != 1
                or edge_direction != segment_direction
            ):
                rospy.logerr(
                    "PASSAGE SEGMENT validation failed current=%s moves=%s",
                    self.current_loc,
                    move_locs
                )
                return False
            previous_index = index

        end_loc = move_locs[-1]
        end_passage, end_index = self.loc_to_passage_index(end_loc)

        # One-time wait (<= 2 s) for the first peer odometry message before the
        # first aisle check.
        if not self._peer_pose_wait_done:
            self._peer_pose_wait_done = True
            wait_deadline = time.monotonic() + 2.0
            while not self.peer_pose_received and time.monotonic() < wait_deadline:
                rospy.sleep(0.1)

        # If the peer is in the same aisle, use a lateral offset instead of
        # reserving the whole segment. Reserving even only end_loc does not help
        # head-on: each robot's destination is the other's start node, which stays
        # held during its whole transit.
        lateral_bypass = self.peer_shares_aisle(start_passage, start_index, end_index)

        # One-time clearance check before the segment, skipped in lateral-bypass
        # mode where the continuous check with the smaller hold distance takes over
        # (otherwise this check alone waits out the full hard timeout).
        if not lateral_bypass:
            self.wait_for_peer_clearance(
                "execute_passage_segment:%s" % (move_locs[-1] if move_locs else "?")
            )

        reserved_locs = [] if lateral_bypass else move_locs
        lateral_offset = 0.0
        if lateral_bypass:
            # Offset magnitude depends on which side of the passage the robot ends up
            # on: positive offset_x is the wide side, negative the tight side.
            offset_sign = 1 if (
                segment_direction * self.picker.length_unit[1]
            ) > 0 else -1
            offset_magnitude = (
                self.lateral_bypass_offset_wide if offset_sign > 0
                else self.lateral_bypass_offset_tight
            )
            lateral_offset = segment_direction * offset_magnitude
            rospy.logerr(
                "PASSAGE SEGMENT LATERAL BYPASS engaged passage=P%d "
                "%s -> %s peer=(%.2f, %.2f) shares this aisle -- "
                "skipping the fleet reservation entirely and offsetting "
                "%.2fm (%s side) instead of locking the whole segment",
                start_passage, start_loc, end_loc,
                self.peer_x, self.peer_y, offset_magnitude,
                "wide" if offset_sign > 0 else "tight"
            )

        # Reserve the whole segment up front (empty in lateral-bypass mode).
        # Every early return below releases it again.
        self.reserve_locations(reserved_locs, "execute_passage_segment:%s" % end_loc)

        corridor_x, corridor_y = self.picker.passage_pose(start_passage, 1)
        # In lateral-bypass mode cross-track is measured from the offset lane.
        if lateral_bypass:
            corridor_x += lateral_offset * self.picker.length_unit[1]
            corridor_y += -lateral_offset * self.picker.length_unit[0]

        def current_cross_track():
            rel_x = self.picker.x - corridor_x
            rel_y = self.picker.y - corridor_y
            return (
                -self.picker.length_unit[1] * rel_x
                + self.picker.length_unit[0] * rel_y
            )

        max_rejoin_cross_track = float(rospy.get_param(
            "~continuous_max_rejoin_cross_track", 0.65
        ))
        if lateral_bypass:
            # Moving onto the offset lane is intended, so widen the rejoin limit
            # by the offset.
            max_rejoin_cross_track += abs(lateral_offset)
        # Residual cross-track after a transfer (up to ~0.3 m) can clip the
        # neighbouring row near the passage entry, so rejoin at segment start
        # with a tighter threshold.
        start_rejoin_cross_track = float(rospy.get_param(
            "~continuous_start_rejoin_cross_track", 0.15
        ))
        # go_to_station converges on Euclidean distance (~station_tolerance),
        # so verify the rejoin with a looser bound.
        start_rejoin_verify_cross_track = float(rospy.get_param(
            "~continuous_start_rejoin_verify_cross_track", 0.30
        ))
        cross_track = current_cross_track()

        try:
            if self.pick_departure_pending:
                direction = 1 if end_index > start_index else -1
                # Pass the lateral offset to the merge as well, so the departure after a
                # pick uses the offset lane and the lane hold distance.
                if not self.picker.merge_into_passage(
                    start_passage, direction,
                    peer_clearance_fn=lambda: self.wait_for_moving_peer_clearance(
                        "execute_passage_segment:%s" % end_loc, target_loc=end_loc,
                        lateral_bypass=lateral_bypass
                    ),
                    lateral_offset=lateral_offset
                ):
                    rospy.logerr(
                        "PICK DEPARTURE merge failed passage=P%d start=I%d",
                        start_passage, start_index
                    )
                    self.release_locations(reserved_locs)
                    return False
                self.pick_departure_pending = False
                cross_track = current_cross_track()

            if abs(cross_track) > start_rejoin_cross_track:
                if abs(cross_track) > max_rejoin_cross_track:
                    # If the cross-track error is only slightly over the limit (within
                    # ~continuous_rejoin_recovery_margin), try one corrective rejoin hop and
                    # measure again instead of failing the segment.
                    recovery_margin = float(rospy.get_param(
                        "~continuous_rejoin_recovery_margin", 1.0
                    ))
                    recovered = False
                    if abs(cross_track) <= max_rejoin_cross_track + recovery_margin:
                        rospy.logerr(
                            "PASSAGE REJOIN over-limit passage=P%d start=I%d "
                            "cross_track=%.3f max_rejoin=%.3f (excess=%.3f "
                            "within recovery_margin=%.2f) -- attempting one "
                            "corrective rejoin hop instead of failing the "
                            "segment",
                            start_passage,
                            start_index,
                            cross_track,
                            max_rejoin_cross_track,
                            abs(cross_track) - max_rejoin_cross_track,
                            recovery_margin
                        )
                        recovery_yaw = None
                        if lateral_bypass:
                            recovery_yaw = self.picker.base_yaw
                            if segment_direction < 0:
                                recovery_yaw = self.picker.normalize_angle(
                                    recovery_yaw + math.pi
                                )
                        if self.picker.go_to_station(
                            start_passage, start_index,
                            peer_clearance_fn=lambda: (
                                self.wait_for_moving_peer_clearance(
                                    "execute_passage_segment:%s" % end_loc,
                                    target_loc=end_loc,
                                    lateral_bypass=lateral_bypass
                                )
                            ),
                            lateral_offset=lateral_offset,
                            target_yaw=recovery_yaw
                        ):
                            cross_track = current_cross_track()
                            recovered = (
                                abs(cross_track) <= max_rejoin_cross_track
                            )
                            rospy.loginfo(
                                "PASSAGE REJOIN recovery %s passage=P%d "
                                "start=I%d cross_track=%.3f max_rejoin=%.3f",
                                "succeeded" if recovered else "insufficient",
                                start_passage,
                                start_index,
                                cross_track,
                                max_rejoin_cross_track
                            )
                        else:
                            rospy.logerr(
                                "PASSAGE REJOIN recovery hop failed to reach "
                                "P%d_I%d", start_passage, start_index
                            )
                    if not recovered:
                        rospy.logerr(
                            "PASSAGE REJOIN refused passage=P%d start=I%d "
                            "cross_track=%.3f max_rejoin=%.3f",
                            start_passage,
                            start_index,
                            cross_track,
                            max_rejoin_cross_track
                        )
                        self.release_locations(reserved_locs)
                        return False

                rospy.loginfo(
                    "PASSAGE REJOIN passage=P%d station=I%d "
                    "cross_track=%.3f limit=%.3f",
                    start_passage,
                    start_index,
                    cross_track,
                    start_rejoin_cross_track
                )
                # Rejoin hop with a peer check. In lateral-bypass mode, target the offset
                # lane and pass a target_yaw so go_to_station drives holonomically
                # (rotating in place first can stall on a large offset).
                rejoin_target_yaw = None
                if lateral_bypass:
                    rejoin_target_yaw = self.picker.base_yaw
                    if segment_direction < 0:
                        rejoin_target_yaw = self.picker.normalize_angle(
                            rejoin_target_yaw + math.pi
                        )
                if not self.picker.go_to_station(
                    start_passage, start_index,
                    peer_clearance_fn=lambda: self.wait_for_moving_peer_clearance(
                        "execute_passage_segment:%s" % end_loc, target_loc=end_loc,
                        lateral_bypass=lateral_bypass
                    ),
                    lateral_offset=lateral_offset,
                    target_yaw=rejoin_target_yaw
                ):
                    rospy.logerr(
                        "PASSAGE REJOIN failed to reach P%d_I%d",
                        start_passage,
                        start_index
                    )
                    self.release_locations(reserved_locs)
                    return False

                cross_track = current_cross_track()
                if abs(cross_track) > start_rejoin_verify_cross_track:
                    rospy.logerr(
                        "PASSAGE REJOIN incomplete passage=P%d start=I%d "
                        "cross_track=%.3f limit=%.3f",
                        start_passage,
                        start_index,
                        cross_track,
                        start_rejoin_verify_cross_track
                    )
                    self.release_locations(reserved_locs)
                    return False

                rospy.loginfo(
                    "PASSAGE REJOIN complete passage=P%d station=I%d "
                    "cross_track=%.3f",
                    start_passage,
                    start_index,
                    cross_track
                )

            rospy.loginfo(
                "ONLINE MOVE SEGMENT %s -> %s logical_moves=%d",
                start_loc,
                end_loc,
                len(move_locs)
            )

            speed_override = None
            if start_passage == self.picker.num_rows and self.basket_load > 0:
                speed_override = float(rospy.get_param(
                    "~outer_lane_loaded_speed", 0.28
                ))
                rospy.loginfo(
                    "LOADED OUTER LANE passage=P%d basket_load=%d/%d speed=%.3f",
                    start_passage, self.basket_load, self.basket_capacity,
                    speed_override
                )
            # Slower speed on the tight side.
            if lateral_bypass and offset_sign < 0:
                tight_speed = self.lateral_bypass_tight_speed
                speed_override = (
                    tight_speed if speed_override is None
                    else min(speed_override, tight_speed)
                )
                rospy.loginfo(
                    "TIGHT LANE SPEED passage=P%d speed=%.3f",
                    start_passage, speed_override
                )
            # Peer check on every tick of the segment.
            peer_clearance = lambda: self.wait_for_moving_peer_clearance(
                "execute_passage_segment:%s" % end_loc, target_loc=end_loc,
                lateral_bypass=lateral_bypass
            )
            if lateral_bypass:
                # Reach the full lateral offset early with a short first leg, then drive
                # parallel to the peer's lane. Ramping the offset over the whole segment
                # left both robots only half offset where they meet.
                # First-leg length = |offset| * merge_slope, in stations, between 2 and
                # the segment length.
                remaining_span = abs(end_index - start_index)
                merge_forward_distance = (
                    abs(lateral_offset) * self.lateral_bypass_merge_slope
                )
                merge_span = min(
                    remaining_span,
                    max(
                        2,
                        int(math.ceil(
                            merge_forward_distance / self.picker.tree_spacing
                        ))
                    )
                )
                merge_index = start_index + segment_direction * merge_span
                merge_cross_track = (
                    abs(lateral_offset)
                    + self.lateral_bypass_max_cross_track_margin
                )
                ok = self.picker.follow_passage_segment(
                    start_passage, start_index, merge_index,
                    speed_override=speed_override,
                    peer_clearance_fn=peer_clearance,
                    lateral_offset=lateral_offset,
                    max_cross_track_override=merge_cross_track,
                    stall_window_override=self.lateral_bypass_stall_window
                )
                if ok and merge_index != end_index:
                    ok = self.picker.follow_passage_segment(
                        start_passage, merge_index, end_index,
                        speed_override=speed_override,
                        peer_clearance_fn=peer_clearance,
                        lateral_offset=lateral_offset,
                        max_cross_track_override=merge_cross_track,
                        stall_window_override=self.lateral_bypass_stall_window
                    )
            else:
                ok = self.picker.follow_passage_segment(
                    start_passage,
                    start_index,
                    end_index,
                    speed_override=speed_override,
                    peer_clearance_fn=peer_clearance,
                    lateral_offset=0.0,
                    max_cross_track_override=None
                )
        except PeerDeadlockDetected:
            # Release the reserved segment before propagating.
            self.release_locations(reserved_locs)
            raise
        # Same release logic as execute_move(), for the whole segment.
        if ok:
            self.release_locations(
                [start_loc] if lateral_bypass else [start_loc] + move_locs[:-1]
            )
            self.current_loc = end_loc
            self._log_field_move(start_loc, end_loc, "segment")
        else:
            self.release_locations(reserved_locs)

        self.writer.writerow([
            "",
            "MOVE_SEGMENT",
            "",
            end_loc,
            "",
            "",
            ok,
            "",
            "",
            "",
            len(self.picked_targets),
            self.basket_load,
            self.picker.x,
            self.picker.y,
            self.picker.yaw
        ])
        self.log_file.flush()
        return ok

    def execute_pick(self, tid, cycle):
        # The arm reaches out during a pick, so check clearance here too.
        self.wait_for_peer_clearance("execute_pick:%s" % tid)
        # current_loc is already held; kept explicit for safety.
        try:
            self.reserve_locations([self.current_loc], "execute_pick:%s" % tid)
        except PeerDeadlockDetected:
            # Rerouting does not apply to the location we are standing on.
            rospy.logerr(
                "PICK RESERVATION deadlock at %s target=%s -- "
                "proceeding anyway", self.current_loc, tid
            )

        info = self.targets[tid]
        target_loc = info["loc"]

        if self.current_loc != target_loc:
            rospy.logwarn("PICK %s requested at %s but robot is at %s", tid, target_loc, self.current_loc)
            return False

        row = info["row"]
        side = info["side"]

        attempt = self.pick_attempts.get(tid, 0) + 1
        self.pick_attempts[tid] = attempt

        if attempt > self.max_pick_attempts_per_target:
            rospy.logfatal(
                "PICK SAFETY STOP target=%s exceeded max attempts %d; refusing further base/arm motion",
                tid, self.max_pick_attempts_per_target
            )
            rospy.signal_shutdown("pick retry safety limit reached for %s" % tid)
            return False

        rospy.loginfo(
            "ONLINE PHYSICAL PICK %s at %s row=%d side=%s attempt=%d",
            tid,
            self.current_loc,
            row,
            side,
            attempt
        )

        completed_parts = self.picked_fruit_parts.setdefault(tid, set())
        pending_parts = [part for part in ("T", "B") if part not in completed_parts]
        picked_count = 0
        pick_result = {"T": "T" in completed_parts, "B": "B" in completed_parts}

        try:
            if self.moveit_group is None:
                self.moveit_commander = import_moveit()
                if self.moveit_commander is None:
                    raise RuntimeError("MoveIt initialization failed")
                self.moveit_commander.roscpp_initialize(sys.argv)
                self.moveit_group = make_moveit_group(self.moveit_commander)

            # Continuous peer check during the base approach to the pick pose
            # (the base can move up to ~2.25 m here).
            peer_clearance = lambda: self.wait_for_moving_peer_clearance(
                "execute_pick:%s" % tid, target_loc=self.current_loc
            )
            attempt_result = pick_location_for_prost_two_motions(
                target=tid,
                reset_arm=not self.pick_resources_ready,
                align_oranges=False,
                fruit_parts=pending_parts,
                return_details=True,
                picker=self.picker,
                moveit_group=self.moveit_group,
                peer_clearance_fn=peer_clearance,
            )
            newly_completed = {
                part for part in pending_parts if attempt_result.get(part, False)
            }
            completed_parts.update(newly_completed)
            _, target_index = self.loc_to_passage_index(target_loc)
            for part in sorted(newly_completed):
                model_name = "orange%d.%d.%s.%s" % (
                    row, target_index, side, part
                )
                if model_name not in self.basket_fruit_models:
                    self.basket_fruit_models.append(model_name)
            pick_result = {
                "T": "T" in completed_parts,
                "B": "B" in completed_parts,
            }
            picked_count = len(newly_completed)
            self.basket_load = min(
                self.basket_capacity, self.basket_load + picked_count
            )
            final_success = completed_parts == {"T", "B"}
        except PeerDeadlockDetected as exc:
            # A peer conflict during the approach is not a pick failure, so do not
            # count it toward max_pick_attempts_per_target. The next PROST cycle retries.
            self.pick_attempts[tid] = max(0, self.pick_attempts[tid] - 1)
            rospy.logerr(
                "ONLINE PHYSICAL PICK PEER CONFLICT %s: %s -- not counted "
                "against max_pick_attempts_per_target (attempt reverted, "
                "pick_attempts[%s]=%d)",
                tid, exc, tid, self.pick_attempts[tid]
            )
            final_success = False
        except Exception as e:
            rospy.logerr("ONLINE PHYSICAL PICK EXCEPTION %s: %s", tid, e)
            final_success = False

        if final_success:
            self.picked_targets.add(tid)
            self.pick_resources_ready = True

            rospy.loginfo(
                "ONLINE PICK FINAL SUCCESS %s two_motion_picked_count=%d basket_load=%d/%d",
                tid,
                picked_count,
                self.basket_load,
                self.basket_capacity
            )
        else:
            # A failed sequence can leave either controller away from its
            # known neutral/open state, so force reset on the next attempt.
            self.pick_resources_ready = False
            rospy.logwarn(
                "ONLINE PICK FINAL FAIL %s two_motion_picked_count=%d",
                tid,
                picked_count
            )

        # A physical pick deliberately moves the omnidirectional base closer
        # to the fruit.  Restore the symbolic station immediately, while its
        # passage/index are still unambiguous, instead of making the next
        # route segment discover and repair the lateral offset.
        route_ready = True
        if self.rejoin_passage_after_pick:
            passage, index = self.loc_to_passage_index(self.current_loc)
            station_x, station_y = self.picker.passage_pose(passage, index)
            station_error = ((station_x - self.picker.x) ** 2
                             + (station_y - self.picker.y) ** 2) ** 0.5
            rejoin_tolerance = float(rospy.get_param(
                "~post_pick_rejoin_tolerance", 0.30
            ))

            defer_departure = bool(rospy.get_param(
                "~continuous_pick_departure", True
            )) and final_success

            if defer_departure:
                # Arm departure even when XY is already inside the station
                # tolerance.  The next route supplies the required direction,
                # allowing yaw correction and forward progress to happen in
                # one motion instead of a later turn in place.
                self.pick_departure_pending = True
                rospy.loginfo(
                    "POST PICK DEPARTURE armed target=%s station=%s "
                    "error=%.3f tolerance=%.3f",
                    tid, self.current_loc, station_error, rejoin_tolerance
                )
            elif station_error > rejoin_tolerance:
                rospy.loginfo(
                    "POST PICK REJOIN target=%s station=%s error=%.3f tolerance=%.3f",
                    tid,
                    self.current_loc,
                    station_error,
                    rejoin_tolerance
                )
                use_holonomic_rejoin = bool(rospy.get_param(
                    "~post_pick_holonomic_rejoin", True
                ))
                try:
                    if use_holonomic_rejoin and hasattr(
                        self.picker, "go_to_point_holonomic"
                    ):
                        if index == self.picker.num_indices:
                            route_yaw = self.picker.normalize_angle(
                                self.picker.base_yaw + 3.141592653589793
                            )
                        else:
                            route_yaw = self.picker.base_yaw
                        route_ready = self.picker.go_to_point_holonomic(
                            station_x,
                            station_y,
                            route_yaw,
                            self.picker.station_tolerance,
                            peer_clearance_fn=lambda: self.wait_for_moving_peer_clearance(
                                "execute_pick_rejoin:%s" % tid, target_loc=self.current_loc
                            )
                        )
                    else:
                        route_ready = self.picker.go_to_station(
                            passage, index,
                            peer_clearance_fn=lambda: self.wait_for_moving_peer_clearance(
                                "execute_pick_rejoin:%s" % tid, target_loc=self.current_loc
                            )
                        )
                except PeerDeadlockDetected:
                    # Rejoin at the current station is not a move, so no reroute here.
                    route_ready = False
                if not route_ready:
                    rospy.logerr(
                        "POST PICK REJOIN failed target=%s station=%s",
                        tid,
                        self.current_loc
                    )
            else:
                rospy.loginfo(
                    "POST PICK REJOIN skipped target=%s station=%s error=%.3f",
                    tid,
                    self.current_loc,
                    station_error
                )

        self.writer.writerow([
            cycle,
            "PICK",
            tid,
            self.current_loc,
            row,
            side,
            final_success,
            pick_result.get("T", False),
            pick_result.get("B", False),
            picked_count,
            len(self.picked_targets),
            self.basket_load,
            self.picker.x,
            self.picker.y,
            self.picker.yaw
        ])
        self.log_file.flush()

        return final_success and route_ready


    def execute_drop(self, cycle):
        if self.current_loc != self.depot_loc:
            rospy.logwarn(
                "DROP requested but robot is at %s, depot is %s",
                self.current_loc,
                self.depot_loc
            )
            return False

        old_load = self.basket_load
        delete_physical = bool(rospy.get_param(
            "~physical_drop_delete_fruits", True
        ))
        if delete_physical and self.basket_fruit_models:
            try:
                rospy.wait_for_service("/gazebo/delete_model", timeout=10.0)
            except rospy.ROSException as exc:
                rospy.logerr(
                    "PHYSICAL DROP service unavailable; basket unchanged error=%s",
                    exc
                )
                return False
            delete_model = rospy.ServiceProxy(
                "/gazebo/delete_model", DeleteModel
            )
            failed_models = []
            for model_name in list(self.basket_fruit_models):
                try:
                    response = delete_model(model_name)
                    if response.success:
                        rospy.loginfo(
                            "PHYSICAL DROP removed fruit model=%s",
                            model_name
                        )
                    else:
                        failed_models.append(model_name)
                        rospy.logerr(
                            "PHYSICAL DROP failed model=%s message=%s",
                            model_name, response.status_message
                        )
                except Exception as exc:
                    failed_models.append(model_name)
                    rospy.logerr(
                        "PHYSICAL DROP exception model=%s error=%s",
                        model_name, exc
                    )

            if failed_models:
                self.basket_fruit_models = failed_models
                rospy.logerr(
                    "PHYSICAL DROP incomplete remaining_models=%s",
                    failed_models
                )
                return False

            self.basket_fruit_models = []

        self.basket_load = 0

        rospy.loginfo(
            "ONLINE DROP at %s basket_load %d -> %d physical_models=%d",
            self.current_loc,
            old_load,
            self.basket_load,
            len(self.basket_fruit_models)
        )

        self.writer.writerow([
            cycle,
            "DROP",
            "",
            self.current_loc,
            "",
            "",
            True,
            "",
            "",
            "",
            len(self.picked_targets),
            self.basket_load,
            self.picker.x,
            self.picker.y,
            self.picker.yaw
        ])
        self.log_file.flush()

        return True


    def return_to_depot_and_drop(self, cycle):
        if self.basket_load <= 0:
            return True

        rospy.loginfo(
            "BASKET RETURN/NEEDS DROP physical_basket_load=%d/%d current=%s depot=%s",
            self.basket_load,
            self.basket_capacity,
            self.current_loc,
            self.depot_loc
        )

        current_passage, current_index = self.loc_to_passage_index(self.current_loc)
        depot_passage, depot_index = self.loc_to_passage_index(self.depot_loc)

        route = []

        # Go along current passage to depot index, normally I1.
        if current_index != depot_index:
            step = -1 if current_index > depot_index else 1
            for i in range(current_index + step, depot_index + step, step):
                route.append(f"P{current_passage}_I{i}")

        # Transfer passage-by-passage at depot index.
        if current_passage != depot_passage:
            step = -1 if current_passage > depot_passage else 1
            for p in range(current_passage + step, depot_passage + step, step):
                route.append(f"P{p}_I{depot_index}")

        rospy.loginfo("RETURN TO DEPOT ROUTE: %s", route)
        if route:
            estimate = self.estimate_route_travel_time(
                self.current_loc, route
            )
            rospy.loginfo(
                "RETURN TO DEPOT TIME ESTIMATE seconds=%.1f passage_edges=%d "
                "first_transfers=%d continuation_transfers=%d",
                estimate["seconds"],
                estimate["passage_edges"],
                estimate["first_transfers"],
                estimate["continuation_transfers"]
            )

        route_index = 0
        deadlock_retry_count = 0
        while route_index < len(route):
            segment_moves = []

            if self.continuous_passage_motion:
                previous_loc = self.current_loc
                scan_index = route_index
                segment_direction = None

                while scan_index < len(route):
                    scan_loc = route[scan_index]
                    # current_loc could be a shortcut midpoint; classify_edge handles both.
                    classified = orchard_topology.classify_edge(
                        previous_loc, scan_loc,
                        num_indices=self.picker.num_indices
                    )
                    if classified[0] != "passage":
                        break
                    _, cur_passage, prev_index, cur_index = classified
                    edge_direction = cur_index - prev_index

                    if segment_direction is None:
                        segment_direction = edge_direction
                    if (
                        abs(edge_direction) != 1
                        or edge_direction != segment_direction
                    ):
                        break

                    segment_moves.append(scan_loc)
                    previous_loc = scan_loc
                    scan_index += 1

            if len(segment_moves) >= 1:
                rospy.loginfo(
                    "RETURN TO DEPOT continuous segment: %s",
                    segment_moves
                )
                try:
                    segment_ok = self.execute_passage_segment(segment_moves)
                except PeerDeadlockDetected as exc:
                    segment_ok = None
                    # "exc" is deleted at the end of the except block, so copy what we need.
                    deadlock_contested_loc = exc.contested_loc
                if segment_ok is None:
                    deadlock_retry_count += 1
                    if deadlock_retry_count > self.max_peer_deadlock_retries:
                        rospy.logerr(
                            "Return-to-depot segment failed: %s "
                            "(deadlock retries exhausted, %d attempts)",
                            segment_moves, deadlock_retry_count
                        )
                        return False
                    mode, new_route = self.resolve_peer_deadlock(
                        deadlock_contested_loc, segment_moves[-1]
                    )
                    if mode == "reroute":
                        route[route_index:route_index + len(segment_moves)] = new_route
                    continue
                if not segment_ok:
                    rospy.logerr(
                        "Return-to-depot segment failed: %s",
                        segment_moves
                    )
                    return False
                deadlock_retry_count = 0
                route_index += len(segment_moves)
                continue

            loc = route[route_index]
            try:
                move_ok = self.execute_move(loc)
            except PeerDeadlockDetected as exc:
                move_ok = None
                deadlock_contested_loc = exc.contested_loc
            if move_ok is None:
                deadlock_retry_count += 1
                if deadlock_retry_count > self.max_peer_deadlock_retries:
                    rospy.logerr(
                        "Return-to-depot MOVE failed: %s (deadlock "
                        "retries exhausted, %d attempts)",
                        loc, deadlock_retry_count
                    )
                    return False
                mode, new_route = self.resolve_peer_deadlock(deadlock_contested_loc, loc)
                if mode == "reroute":
                    route[route_index:route_index + 1] = new_route
                continue
            if not move_ok:
                rospy.logerr("Return-to-depot MOVE failed: %s", loc)
                return False
            deadlock_retry_count = 0
            route_index += 1

        if self.current_loc != self.depot_loc:
            rospy.logerr(
                "Return-to-depot ended at %s, expected %s",
                self.current_loc,
                self.depot_loc
            )
            return False

        if not self.auto_drop:
            rospy.loginfo(
                "BASKET FULL AT DEPOT basket_load=%d/%d; auto_drop=false, keeping load",
                self.basket_load,
                self.basket_capacity
            )
            return True

        return self.execute_drop(cycle)


    def run(self):
        rospy.loginfo("ONLINE CLOSED LOOP START")
        rospy.loginfo(
            "ROBOT CONFIG id=%s prost_port=%d instance=%s targets=%s",
            self.robot_id, self.prost_port, self.instance_name,
            sorted(self.targets)
        )
        if self.allowed_locations:
            rospy.loginfo(
                "ROBOT ZONE id=%s locations=%s",
                self.robot_id, sorted(self.allowed_locations)
            )
        rospy.loginfo("grasp_success_prob=%.2f random_seed=%d", self.grasp_success_prob, self.random_seed)
        rospy.loginfo(
            "basket_capacity=%d physical_fruits depot_loc=%s auto_drop=%s",
            self.basket_capacity,
            self.depot_loc,
            self.auto_drop
        )
        rospy.loginfo(
            "ROUTE COST MODEL passage_edge=%.1fs transfer_first=%.1fs "
            "transfer_continuation=%.1fs strategy=%s selection_active=%s",
            self.route_passage_edge_cost,
            self.route_transfer_first_cost,
            self.route_transfer_continuation_cost,
            self.route_strategy,
            self.route_strategy in ("time_aware", "time_aware_global")
        )

        for cycle in range(1, self.max_cycles + 1):
            if rospy.is_shutdown():
                return self.finish_run(False)

            if self.basket_load >= self.basket_capacity:
                if not self.return_to_depot_and_drop(cycle):
                    return self.finish_run(False)
                if not self.auto_drop:
                    rospy.loginfo(
                        "ONLINE STOP basket full at depot and automatic DROP is disabled"
                    )
                    break

            if self.all_done():
                if self.basket_load > 0:
                    if not self.return_to_depot_and_drop(cycle):
                        return self.finish_run(False)
                break

            if self.route_strategy == "time_aware_global":
                global_plan = self.plan_time_aware_global_mission()
                rospy.loginfo(
                    "GLOBAL ROUTE PLAN sequence=%s seconds=%.1f drops=%d "
                    "current=%s basket_load=%d/%d",
                    global_plan["sequence"],
                    global_plan["seconds"],
                    global_plan["drops"],
                    self.current_loc,
                    self.basket_load,
                    self.basket_capacity
                )
                if (
                    global_plan["sequence"]
                    and global_plan["sequence"][0] == "DEPOT"
                ):
                    rospy.loginfo(
                        "GLOBAL ROUTE EARLY DEPOT current=%s basket_load=%d/%d",
                        self.current_loc,
                        self.basket_load,
                        self.basket_capacity
                    )
                    if not self.return_to_depot_and_drop(cycle):
                        rospy.logerr("GLOBAL ROUTE early depot/drop failed")
                        return self.finish_run(False)
                    continue

            rospy.loginfo("ONLINE CYCLE %d current=%s picked=%s basket_load=%d/%d", cycle, self.current_loc, sorted(self.picked_targets), self.basket_load, self.basket_capacity)

            self.write_instance()
            self.run_prost_once()
            actions = self.parse_actions_until_pick()

            if not actions:
                rospy.logerr("No actions parsed from PROST/server log.")
                return self.finish_run(False)

            rospy.loginfo("ONLINE actions until pick: %s", actions)
            if self.time_aware_observer_enabled:
                self.log_time_aware_observer(actions)
            actions = self.optimize_actions_to_first_pick(actions)
            rospy.loginfo("ONLINE optimized actions until pick: %s", actions)
            self.log_route_segments(actions)

            got_pick = False

            action_index = 0
            # Deadlock retries; reset on real progress. Past
            # ~max_peer_deadlock_retries the run fails.
            deadlock_retry_count = 0
            while action_index < len(actions):
                action, arg = actions[action_index]

                if action == "MOVE":
                    segment_moves = []

                    if self.continuous_passage_motion:
                        previous_loc = self.current_loc
                        scan_index = action_index
                        segment_direction = None

                        while scan_index < len(actions):
                            scan_action, scan_loc = actions[scan_index]
                            if scan_action != "MOVE":
                                break

                            # Only plain same-passage moves are batched into one segment; shortcut
                            # edges go through execute_move().
                            classified = orchard_topology.classify_edge(
                                previous_loc, scan_loc,
                                num_indices=self.picker.num_indices
                            )
                            if classified[0] != "passage":
                                break
                            _, prev_index, cur_index = (
                                classified[1], classified[2], classified[3]
                            )
                            edge_direction = cur_index - prev_index
                            if segment_direction is None:
                                segment_direction = edge_direction
                            if (
                                abs(edge_direction) != 1
                                or edge_direction != segment_direction
                            ):
                                break

                            segment_moves.append(scan_loc)
                            previous_loc = scan_loc
                            scan_index += 1

                    if len(segment_moves) >= 1:
                        try:
                            segment_ok = self.execute_passage_segment(segment_moves)
                        except PeerDeadlockDetected as exc:
                            segment_ok = None
                            deadlock_contested_loc = exc.contested_loc
                        if segment_ok is None:
                            deadlock_retry_count += 1
                            if deadlock_retry_count > self.max_peer_deadlock_retries:
                                rospy.logerr(
                                    "MOVE SEGMENT failed: %s (deadlock "
                                    "retries exhausted, %d attempts)",
                                    segment_moves, deadlock_retry_count
                                )
                                return self.finish_run(False)
                            mode, route = self.resolve_peer_deadlock(
                                deadlock_contested_loc, segment_moves[-1]
                            )
                            if mode == "reroute":
                                actions[action_index:action_index + len(segment_moves)] = [
                                    ("MOVE", loc) for loc in route
                                ]
                            continue
                        if not segment_ok:
                            rospy.logerr("MOVE SEGMENT failed: %s", segment_moves)
                            return self.finish_run(False)
                        deadlock_retry_count = 0
                        action_index += len(segment_moves)
                        continue

                    try:
                        move_ok = self.execute_move(arg)
                    except PeerDeadlockDetected as exc:
                        move_ok = None
                        deadlock_contested_loc = exc.contested_loc
                    if move_ok is None:
                        deadlock_retry_count += 1
                        if deadlock_retry_count > self.max_peer_deadlock_retries:
                            rospy.logerr(
                                "MOVE failed: %s (deadlock retries "
                                "exhausted, %d attempts)",
                                arg, deadlock_retry_count
                            )
                            return self.finish_run(False)
                        mode, route = self.resolve_peer_deadlock(
                            deadlock_contested_loc, arg
                        )
                        if mode == "reroute":
                            actions[action_index:action_index + 1] = [
                                ("MOVE", loc) for loc in route
                            ]
                        continue
                    if not move_ok:
                        rospy.logerr("MOVE failed: %s", arg)
                        return self.finish_run(False)
                    deadlock_retry_count = 0

                elif action == "DROP":
                    if not self.execute_drop(cycle):
                        rospy.logerr("DROP failed")
                        return self.finish_run(False)

                elif action == "PICK":
                    self.execute_pick(arg, cycle)
                    got_pick = True
                    break

                action_index += 1

            if self.basket_load >= self.basket_capacity:
                if not self.return_to_depot_and_drop(cycle):
                    return self.finish_run(False)
                if not self.auto_drop:
                    rospy.loginfo(
                        "ONLINE STOP basket full at depot and automatic DROP is disabled"
                    )
                    break

            if not got_pick:
                rospy.logwarn("No PICK reached in this cycle. Stopping to avoid endless loop.")
                return self.finish_run(False)

        success = not self.require_all_targets or self.all_done()
        if self.require_all_targets and not success:
            rospy.logerr(
                "MISSION INCOMPLETE picked=%d/%d after max_cycles=%d",
                len(self.picked_targets), len(self.targets), self.max_cycles
            )
        return self.finish_run(success)

    def finish_run(self, success):
        self.picker.stop()
        # Release reservations explicitly; the process does not always exit
        # right away (non-daemon ROS threads), and a parked robot would keep them.
        fleet_arbiter.release(self.robot_id, list(fleet_arbiter.held()))
        # Also clear the waiting/maneuvering broadcasts, so the peer does not see
        # a stale "waiting for X".
        fleet_arbiter.clear_waiting(self.robot_id)
        fleet_arbiter.clear_maneuvering(self.robot_id)
        # Announce that this robot is finished, so the peer can use the relaxed
        # clearance. Only if the robot is level: checked against
        # ~tilt_recovery_angle_deg (8 deg), not the 16 deg abort threshold.
        recovery_deg = float(rospy.get_param("~tilt_recovery_angle_deg", 8.0))
        if self.picker.robot_is_tilted(max_tilt_deg=recovery_deg):
            rospy.logerr(
                "FINISH RUN robot is NOT level (roll=%.2fdeg pitch=%.2fdeg, "
                "recovery_threshold=%.1fdeg) -- NOT broadcasting "
                "set_finished(); peer keeps full peer_safety_radius "
                "clearance near this robot",
                math.degrees(self.picker.roll), math.degrees(self.picker.pitch),
                recovery_deg
            )
        else:
            fleet_arbiter.set_finished(self.robot_id)
        rospy.loginfo("ONLINE CLOSED LOOP FINISHED")
        rospy.loginfo("ONLINE RUN STATUS success=%s", bool(success))
        rospy.loginfo("Picked targets: %s", sorted(self.picked_targets))
        rospy.loginfo("Picked target count: %d / %d", len(self.picked_targets), len(self.targets))
        physical_fruits_picked = sum(
            len(parts) for parts in self.picked_fruit_parts.values()
        )
        rospy.loginfo("Physical fruits picked: %d", physical_fruits_picked)
        rospy.loginfo("Online log saved to: %s", self.online_log)
        rospy.loginfo("Field moves log saved to: %s", self.field_moves_log)

        if not self.log_file.closed:
            self.log_file.close()
        if not self.field_moves_file.closed:
            self.field_moves_file.close()
        return bool(success)


if __name__ == "__main__":
    manager = OnlineOrchardManager()
    sys.exit(0 if manager.run() else 1)
