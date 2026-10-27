#!/usr/bin/env python3

import json
import math
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, DeleteModel

from harbot_orchard_picker import orchard_topology


class OrchardMapPicker:
    def __init__(self):
        if not rospy.core.is_initialized():
            rospy.init_node("orchard_map_picker")

        self.robot_id = str(rospy.get_param("~robot_id", "robot")).strip("/")
        robot_ns = "/" + self.robot_id
        self.cmd_vel_topic = rospy.get_param(
            "~cmd_vel_topic", robot_ns + "/robotnik_base_control/cmd_vel"
        )
        self.odom_topic = rospy.get_param(
            "~odom_topic", robot_ns + "/robotnik_base_control/odom_gt"
        )

        self.tree_points_file = Path(rospy.get_param(
            "~tree_points_file",
            str(Path.home() / "harbot_calibration/tree_points.jsonl")
        ))

        self.map_out_file = Path(rospy.get_param(
            "~map_out_file",
            str(Path.home() / (
                "harbot_calibration/orchard_map_%s.json" % self.robot_id
            ))
        ))

        self.delete_after_pick = rospy.get_param("~delete_after_pick", False)

        self.num_rows = int(rospy.get_param("~num_rows", 4))
        self.num_indices = int(rospy.get_param("~num_indices", 8))
        # Optional mid-row shortcuts (empty by default).
        self.shortcut_pairs = list(rospy.get_param("~shortcut_pairs", []))
        # Shortcut crossing: slower than a normal transfer (trees on both sides)
        # and a tighter alignment tolerance before crossing.
        self.shortcut_cross_speed = float(rospy.get_param(
            "~shortcut_cross_speed", 0.15
        ))
        self.shortcut_align_tolerance = float(rospy.get_param(
            "~shortcut_align_tolerance", 0.12
        ))
        self.shortcut_cross_tolerance = float(rospy.get_param(
            "~shortcut_cross_tolerance", 0.28
        ))

        self.forward_speed = rospy.get_param("~forward_speed", 0.36)
        self.turn_limit = rospy.get_param("~turn_limit", 0.24)
        self.turn_gain = rospy.get_param("~turn_gain", 0.80)

        self.go_tolerance = rospy.get_param("~go_tolerance", 0.28)
        self.station_tolerance = rospy.get_param("~station_tolerance", 0.24)

        self.outer_lane_offset = rospy.get_param("~outer_lane_offset", 1.35)
        self.transfer_margin = rospy.get_param("~transfer_margin", 1.20)
        self.keep_transfer_endcap = rospy.get_param("~keep_transfer_endcap", True)

        self.face_before_pick = rospy.get_param("~face_before_pick", False)
        self.pause_at_pick = rospy.get_param("~pause_at_pick", 0.20)
        self.enable_pick_approach = rospy.get_param("~enable_pick_approach", True)
        self.pick_approach_distance = rospy.get_param("~pick_approach_distance", 0.22)
        self.pick_approach_tolerance = rospy.get_param("~pick_approach_tolerance", 0.08)
        self.pick_min_fruit_clearance = rospy.get_param("~pick_min_fruit_clearance", 0.55)
        self.return_after_pick = rospy.get_param("~return_after_pick", True)
        self.fruit_side_offset = rospy.get_param("~fruit_side_offset", 0.45)
        self.pick_pose_clearance = rospy.get_param("~pick_pose_clearance", 0.62)
        self.skip_return_if_close = rospy.get_param("~skip_return_if_close", True)
        self.align_before_station_pick = rospy.get_param("~align_before_station_pick", False)
        self.face_yaw_threshold = rospy.get_param("~face_yaw_threshold", 0.45)

        self.max_angular_drive = rospy.get_param("~max_angular_drive", 0.30)
        self.yaw_drive_gain = rospy.get_param("~yaw_drive_gain", 0.70)
        self.drive_yaw_threshold = rospy.get_param("~drive_yaw_threshold", 0.18)
        self.turn_in_place_threshold = rospy.get_param("~turn_in_place_threshold", 0.22)

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.has_odom = False

        self.picked = set()
        self.deleted = set()

        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/delete_model")

        self.get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        self.rate = rospy.Rate(10)

        self.load_tree_calibration()
        self.build_map()
        self.save_map()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)

        if abs(sinp) >= 1.0:
            self.pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            self.pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.has_odom = True

    def load_tree_calibration(self):
        points = {}

        with open(self.tree_points_file) as f:
            for line in f:
                if line.strip():
                    p = json.loads(line)
                    points[p["label"]] = p

        required = ["T11_row1_tree1", "T18_row1_tree8", "T21_row2_tree1"]

        for label in required:
            if label not in points:
                raise RuntimeError("Missing tree point: " + label)

        T11 = points["T11_row1_tree1"]
        T18 = points["T18_row1_tree8"]
        T21 = points["T21_row2_tree1"]

        self.t11 = (T11["x"], T11["y"])
        self.t18 = (T18["x"], T18["y"])
        self.t21 = (T21["x"], T21["y"])

        self.length_vec = (
            self.t18[0] - self.t11[0],
            self.t18[1] - self.t11[1]
        )

        self.row_vec = (
            self.t21[0] - self.t11[0],
            self.t21[1] - self.t11[1]
        )

        self.row_length = math.sqrt(self.length_vec[0] ** 2 + self.length_vec[1] ** 2)
        self.row_spacing = math.sqrt(self.row_vec[0] ** 2 + self.row_vec[1] ** 2)

        if self.row_length < 1.0:
            raise RuntimeError("Bad calibration: row length too small.")
        if self.row_spacing < 1.0:
            raise RuntimeError("Bad calibration: row spacing too small.")

        self.length_unit = (
            self.length_vec[0] / self.row_length,
            self.length_vec[1] / self.row_length
        )

        self.width_unit = (
            self.row_vec[0] / self.row_spacing,
            self.row_vec[1] / self.row_spacing
        )

        self.tree_spacing = self.row_length / float(self.num_indices - 1)
        self.base_yaw = math.atan2(self.length_unit[1], self.length_unit[0])

        rospy.loginfo("Tree calibration loaded.")
        rospy.loginfo("T11: %.3f %.3f", self.t11[0], self.t11[1])
        rospy.loginfo("T18: %.3f %.3f", self.t18[0], self.t18[1])
        rospy.loginfo("T21: %.3f %.3f", self.t21[0], self.t21[1])
        rospy.loginfo("row_length: %.3f", self.row_length)
        rospy.loginfo("tree_spacing: %.3f", self.tree_spacing)
        rospy.loginfo("row_spacing: %.3f", self.row_spacing)

    def build_map(self):
        self.map = {
            "passages": {},
            "trees": {},
            "adjacent": [],
            "accessible": {}
        }

        for row in range(1, self.num_rows + 1):
            for index in range(1, self.num_indices + 1):
                node = self.tree_node(row, index)
                x, y = self.tree_pose(row, index)

                self.map["trees"][node] = {
                    "row": row,
                    "index": index,
                    "x": x,
                    "y": y,
                    "fruits": [
                        f"orange{row}.{index}.L.B",
                        f"orange{row}.{index}.L.T",
                        f"orange{row}.{index}.R.B",
                        f"orange{row}.{index}.R.T"
                    ]
                }

        for passage in range(0, self.num_rows + 1):
            passage_name = self.passage_name(passage)
            self.map["passages"][passage_name] = {}

            for index in range(1, self.num_indices + 1):
                node = self.passage_node(passage, index)
                x, y = self.passage_pose(passage, index)

                groups = self.accessible_groups(passage, index)

                self.map["passages"][passage_name][str(index)] = {
                    "node": node,
                    "passage": passage_name,
                    "index": index,
                    "x": x,
                    "y": y,
                    "accessible_groups": [
                        {"row": row, "side": side} for row, side in groups
                    ]
                }

                self.map["accessible"][node] = [
                    {"row": row, "index": index, "side": side}
                    for row, side in groups
                ]

        loc_set = set(orchard_topology.base_locations(
            num_passages=self.num_rows + 1, num_indices=self.num_indices
        ))
        base_pairs = orchard_topology.base_edges(
            loc_set, num_passages=self.num_rows + 1, num_indices=self.num_indices
        )
        for a, b in base_pairs:
            self.map["adjacent"].append([a, b])

        shortcut_loc_set = loc_set | set(
            orchard_topology.shortcut_locations(self.shortcut_pairs)
        )
        for a, b in orchard_topology.shortcut_edges(
            self.shortcut_pairs, loc_set=shortcut_loc_set
        ):
            self.map["adjacent"].append([a, b])

    def save_map(self):
        self.map_out_file.parent.mkdir(parents=True, exist_ok=True)

        with open(self.map_out_file, "w") as f:
            json.dump(self.map, f, indent=2)

        rospy.loginfo("Saved orchard map to: %s", self.map_out_file)

    def tree_node(self, row, index):
        letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        return f"{letters[row - 1]}{index}"

    def passage_name(self, passage):
        return f"P{passage}"

    def passage_node(self, passage, index):
        return f"P{passage}_{index}"

    def tree_pose(self, row, index):
        x = (
            self.t11[0]
            + (row - 1) * self.row_vec[0]
            + (index - 1) * self.tree_spacing * self.length_unit[0]
        )

        y = (
            self.t11[1]
            + (row - 1) * self.row_vec[1]
            + (index - 1) * self.tree_spacing * self.length_unit[1]
        )

        return x, y

    def passage_pose(self, passage, index):
        base_x, base_y = self.tree_pose(1, index)

        # Extra safety clearance for the two outer passages.
        # The symbolic locations P0_I* and P4_I* are valid, but in Gazebo
        # the physical path can be too close to the outer tree rows.
        # This pushes only the outer lanes farther away from trees.
        if passage == 0:
            # P0 must be moved inward to clear greenhouse supports near the
            # negative-x wall.
            outer_extra_left = float(rospy.get_param(
                "~outer_lane_extra_clearance_left", 0.10
            ))
            safe_offset = self.outer_lane_offset + outer_extra_left
            x = base_x - safe_offset * self.width_unit[0]
            y = base_y - safe_offset * self.width_unit[1]
        elif passage == self.num_rows:
            # P4 needs the original larger offset: its physical right-side
            # fruit/tree geometry reaches much farther into the outer lane.
            outer_extra_right = float(rospy.get_param(
                "~outer_lane_extra_clearance_right", 0.45
            ))
            safe_offset = self.outer_lane_offset + outer_extra_right
            row_x, row_y = self.tree_pose(self.num_rows, index)
            x = row_x + safe_offset * self.width_unit[0]
            y = row_y + safe_offset * self.width_unit[1]
        else:
            x = base_x + (passage - 0.5) * self.row_vec[0]
            y = base_y + (passage - 0.5) * self.row_vec[1]

        return x, y

    def compute_bypass_arc(self, center_x, center_y, direction, radius=1.3,
                           num_waypoints=10, depth_radius=None,
                           lateral_center_x=None, lateral_center_y=None):
        """Waypoints of a half-ellipse arc for passing a peer beyond an endcap.

        `direction`: +1 at the I{num_indices} end, -1 at the I1 end (the side away
        from the row). Only the half with depth >= 0 is used, so the whole arc stays
        beyond the last station, where there are no trees.

        Depth comes from `center_x/center_y` (the caller's anchor) and is limited by
        `depth_radius`: the free corridor beyond the anchor is only ~0.74 m before
        the greenhouse wall. The lateral centre comes from
        `lateral_center_x/lateral_center_y` (the peer), with `radius` as the lateral
        half-width. Without a lateral centre the arc is centred on the anchor.

        The caller is responsible for the live distance check while driving it.
        """
        direction = 1 if direction >= 0 else -1
        lu = self.length_unit
        wu = self.width_unit
        if depth_radius is None:
            depth_radius = radius
        # Lateral offset of the obstacle from the anchor along width_unit.
        lateral_shift = 0.0
        if lateral_center_x is not None and lateral_center_y is not None:
            lateral_shift = (
                (lateral_center_x - center_x) * wu[0]
                + (lateral_center_y - center_y) * wu[1]
            )
        waypoints = []
        for i in range(num_waypoints + 1):
            theta = -math.pi / 2 + (math.pi * i / num_waypoints)
            u = depth_radius * math.cos(theta)
            v = lateral_shift + radius * math.sin(theta)
            x = center_x + direction * u * lu[0] + v * wu[0]
            y = center_y + direction * u * lu[1] + v * wu[1]
            waypoints.append((x, y))
        return waypoints

    @staticmethod
    def polyline_min_distance(points, px, py):
        """Smallest distance from (px, py) to the polyline through `points`.

        Segments are included, not only the vertices, since the closest approach
        can fall between two waypoints. Returns inf for an empty list.
        """
        if not points:
            return float("inf")
        best = float("inf")
        for i, (ax, ay) in enumerate(points):
            d = math.hypot(px - ax, py - ay)
            if d < best:
                best = d
            if i == 0:
                continue
            bx, by = points[i - 1]
            sx, sy = ax - bx, ay - by
            seg_sq = sx * sx + sy * sy
            if seg_sq <= 1e-12:
                continue
            # Projection of the point onto the segment, clamped to it.
            t = ((px - bx) * sx + (py - by) * sy) / seg_sq
            t = max(0.0, min(1.0, t))
            d = math.hypot(px - (bx + t * sx), py - (by + t * sy))
            if d < best:
                best = d
        return best

    def accessible_groups(self, passage, index):
        if passage == 0:
            return [(1, "L")]

        if passage == self.num_rows:
            return [(self.num_rows, "R")]

        return [
            (passage, "R"),
            (passage + 1, "L")
        ]

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop(self):
        self.cmd_pub.publish(Twist())

    def robot_is_tilted(self, max_tilt_deg=None):
        """True if roll or pitch exceeds the tilt threshold.

        `max_tilt_deg` overrides ~tilt_abort_angle_deg for this call. finish_run()
        passes the stricter recovery angle (8 deg) to decide if the robot is level.
        """
        max_tilt = math.radians(
            max_tilt_deg if max_tilt_deg is not None
            else float(rospy.get_param("~tilt_abort_angle_deg", 16.0))
        )
        return abs(self.roll) > max_tilt or abs(self.pitch) > max_tilt

    def ensure_robot_level(self, context):
        """Stop on a tilt spike and continue only after stable recovery."""
        if not self.robot_is_tilted():
            return True

        abort_deg = float(rospy.get_param("~tilt_abort_angle_deg", 16.0))
        recovery_deg = float(rospy.get_param(
            "~tilt_recovery_angle_deg", 8.0
        ))
        # Recover below (abort angle - hysteresis) within the recovery window.
        # Tilt decays slowly once the robot stops (~0.4 deg/s), so the old rule
        # (8 deg within 2 s) never recovered in the logged tilt events; 14.5 deg
        # within 20 s recovered all of them. ~tilt_recovery_angle_deg (8 deg)
        # is still used by finish_run().
        recovery_timeout = float(rospy.get_param(
            "~tilt_recovery_timeout", 20.0
        ))
        stable_duration = float(rospy.get_param(
            "~tilt_recovery_stable_duration", 0.40
        ))
        recovery_hysteresis = float(rospy.get_param(
            "~tilt_recovery_hysteresis_deg", 1.5
        ))
        # Immediate abort for a real tip-over. 18 deg is above anything recorded
        # in the experiments (max 16.74 deg).
        hard_abort_deg = float(rospy.get_param(
            "~tilt_hard_abort_angle_deg", 18.0
        ))
        recovery_limit = math.radians(
            min(max(recovery_deg, abort_deg - recovery_hysteresis), abort_deg)
        )

        self.stop()
        rospy.logwarn(
            "TILT DETECTED context=%s roll=%.2fdeg pitch=%.2fdeg "
            "position=(%.3f, %.3f) abort=%.1fdeg; waiting for recovery",
            context,
            math.degrees(self.roll),
            math.degrees(self.pitch),
            self.x,
            self.y,
            abort_deg
        )

        started_at = time.monotonic()
        stable_since = None
        while not rospy.is_shutdown():
            self.stop()
            now = time.monotonic()
            level_for_recovery = (
                abs(self.roll) <= recovery_limit
                and abs(self.pitch) <= recovery_limit
            )
            if level_for_recovery:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= stable_duration:
                    rospy.loginfo(
                        "TILT RECOVERED context=%s roll=%.2fdeg pitch=%.2fdeg "
                        "position=(%.3f, %.3f) stable=%.2fs",
                        context,
                        math.degrees(self.roll),
                        math.degrees(self.pitch),
                        self.x,
                        self.y,
                        now - stable_since
                    )
                    return True
            else:
                stable_since = None

            # Abort at once above the hard limit.
            if (
                abs(math.degrees(self.roll)) >= hard_abort_deg
                or abs(math.degrees(self.pitch)) >= hard_abort_deg
            ):
                rospy.logerr(
                    "TILT HARD ABORT context=%s roll=%.2fdeg pitch=%.2fdeg "
                    "position=(%.3f, %.3f) -- beyond %.1fdeg, which no "
                    "attitude in the measured history ever reached",
                    context, math.degrees(self.roll),
                    math.degrees(self.pitch), self.x, self.y, hard_abort_deg
                )
                return False

            if now - started_at >= recovery_timeout:
                rospy.logerr(
                    "TILT PERSISTENT context=%s roll=%.2fdeg pitch=%.2fdeg "
                    "position=(%.3f, %.3f) timeout=%.2fs",
                    context,
                    math.degrees(self.roll),
                    math.degrees(self.pitch),
                    self.x,
                    self.y,
                    recovery_timeout
                )
                return False
            self.rate.sleep()

        return False

    def turn_to_yaw(self, target_yaw, peer_clearance_fn=None):
        # Timeout so this loop cannot hang; same peer check as the other primitives.
        timeout = float(rospy.get_param("~turn_to_yaw_timeout", 30.0))
        absolute_timeout = float(rospy.get_param(
            "~turn_to_yaw_absolute_timeout", 300.0
        ))
        started_at = time.monotonic()
        absolute_started_at = started_at

        while not rospy.is_shutdown():
            if not self.ensure_robot_level("turn_to_yaw"):
                return False

            if peer_clearance_fn is not None:
                wait_started = time.monotonic()
                peer_clearance_fn()
                started_at += time.monotonic() - wait_started

            yaw_error = self.normalize_angle(target_yaw - self.yaw)

            if abs(yaw_error) < math.radians(4.0):
                self.stop()
                rospy.sleep(0.15)
                return True

            now = time.monotonic()
            if now - absolute_started_at > absolute_timeout:
                self.stop()
                rospy.logerr(
                    "TURN TO YAW absolute timeout target=%.3f yaw_error=%.3f "
                    "absolute_timeout=%.1fs (peer-wait time included)",
                    target_yaw, yaw_error, absolute_timeout
                )
                return False

            if now - started_at > timeout:
                self.stop()
                rospy.logerr(
                    "TURN TO YAW timeout target=%.3f yaw_error=%.3f timeout=%.1fs",
                    target_yaw, yaw_error, timeout
                )
                return False

            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = max(min(self.turn_gain * yaw_error, self.turn_limit), -self.turn_limit)

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

        return False

    def go_to_point(self, target_x, target_y, tolerance=None, peer_clearance_fn=None):
        """Drive to (x, y) by turning first and then moving forward.

        `peer_clearance_fn`, if given, is called on every control tick
        (same contract as in follow_passage_segment()).
        """
        if tolerance is None:
            tolerance = self.go_tolerance

        # Timeout, so a wedged robot fails the MOVE instead of spinning forever.
        timeout = float(rospy.get_param("~go_to_point_timeout", 60.0))
        # Absolute ceiling, never extended by peer waits. Excluding all peer-wait
        # time from `timeout` alone could keep a mutual standoff going forever.
        absolute_timeout = float(rospy.get_param(
            "~go_to_point_absolute_timeout", 300.0
        ))
        started_at = time.monotonic()
        absolute_started_at = started_at

        rospy.loginfo("MOVE target x=%.2f y=%.2f", target_x, target_y)

        while not rospy.is_shutdown():
            if not self.ensure_robot_level("go_to_point"):
                return False

            if peer_clearance_fn is not None:
                # Time spent waiting for the peer does not count against `timeout`.
                wait_started = time.monotonic()
                peer_clearance_fn()
                started_at += time.monotonic() - wait_started

            dx = target_x - self.x
            dy = target_y - self.y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist <= tolerance:
                self.stop()
                rospy.sleep(0.15)
                return True

            now = time.monotonic()
            if now - absolute_started_at > absolute_timeout:
                self.stop()
                rospy.logerr(
                    "MOVE absolute timeout target=(%.2f, %.2f) "
                    "position=(%.3f, %.3f) dist=%.3f "
                    "absolute_timeout=%.1fs (peer-wait time included)",
                    target_x, target_y, self.x, self.y, dist,
                    absolute_timeout
                )
                return False

            if now - started_at > timeout:
                self.stop()
                rospy.logerr(
                    "MOVE timeout target=(%.2f, %.2f) position=(%.3f, %.3f) "
                    "dist=%.3f timeout=%.1fs",
                    target_x, target_y, self.x, self.y, dist, timeout
                )
                return False

            target_yaw = math.atan2(dy, dx)
            yaw_error = self.normalize_angle(target_yaw - self.yaw)

            cmd = Twist()

            if abs(yaw_error) > self.turn_in_place_threshold:
                cmd.linear.x = 0.0
                cmd.angular.z = max(
                    min(self.turn_gain * yaw_error, self.turn_limit),
                    -self.turn_limit
                )
            elif abs(yaw_error) > self.drive_yaw_threshold:
                cmd.linear.x = 0.0
                cmd.angular.z = max(
                    min(self.yaw_drive_gain * yaw_error, self.max_angular_drive),
                    -self.max_angular_drive
                )
            else:
                heading_factor = max(
                    0.35,
                    1.0 - abs(yaw_error) / max(self.drive_yaw_threshold, 0.01)
                )

                cmd.linear.x = min(
                    self.forward_speed * heading_factor,
                    0.35 * dist
                )

                cmd.angular.z = max(
                    min(self.yaw_drive_gain * yaw_error, self.max_angular_drive),
                    -self.max_angular_drive
                )

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

        return False

    def go_to_station(self, passage, index, peer_clearance_fn=None,
                       lateral_offset=0.0, target_yaw=None):
        x, y = self.passage_pose(passage, index)
        # Same lateral-offset convention as follow_passage_segment().
        x += lateral_offset * self.length_unit[1]
        y += -lateral_offset * self.length_unit[0]
        if target_yaw is not None:
            # With target_yaw use the holonomic primitive: a large sideways move with
            # rotate-first go_to_point sometimes never finished the turn and stalled
            # for the whole timeout.
            return self.go_to_point_holonomic(
                x, y, target_yaw, tolerance=self.station_tolerance,
                peer_clearance_fn=peer_clearance_fn
            )
        return self.go_to_point(
            x, y, self.station_tolerance, peer_clearance_fn=peer_clearance_fn
        )

    def merge_into_passage(self, passage, direction, peer_clearance_fn=None, lateral_offset=0.0):
        """Holonomically leave a pick pose and merge into passage motion.

        The base translates away from the trees, rotates toward the requested
        passage direction and makes forward progress at the same time. On a
        successful merge it leaves the last velocity command active so
        ``follow_passage_segment`` can take over without a stop.

        `lateral_offset`: same convention as in follow_passage_segment().
        """
        passage = int(passage)
        direction = 1 if int(direction) > 0 else -1
        corridor_x, corridor_y = self.passage_pose(passage, 1)
        corridor_x += lateral_offset * self.length_unit[1]
        corridor_y += -lateral_offset * self.length_unit[0]
        normal_x = -self.length_unit[1]
        normal_y = self.length_unit[0]
        target_yaw = self.base_yaw
        if direction < 0:
            target_yaw = self.normalize_angle(target_yaw + math.pi)

        max_initial_cross_track = float(rospy.get_param(
            "~pick_departure_max_cross_track", 1.25
        ))
        merge_cross_track = float(rospy.get_param(
            "~pick_departure_merge_cross_track", 0.30
        ))
        yaw_tolerance = float(rospy.get_param(
            "~pick_departure_yaw_tolerance", 0.18
        ))
        forward_speed = float(rospy.get_param(
            "~pick_departure_forward_speed", 0.22
        ))
        lateral_speed = float(rospy.get_param(
            "~pick_departure_lateral_speed", 0.22
        ))
        angular_speed = float(rospy.get_param(
            "~pick_departure_turn", 0.30
        ))
        timeout = float(rospy.get_param(
            "~pick_departure_timeout", 25.0
        ))

        rel_x = self.x - corridor_x
        rel_y = self.y - corridor_y
        initial_cross_track = normal_x * rel_x + normal_y * rel_y
        if abs(initial_cross_track) > max_initial_cross_track:
            rospy.logerr(
                "PICK DEPARTURE refused passage=P%d cross_track=%.3f limit=%.3f",
                passage, initial_cross_track, max_initial_cross_track
            )
            return False

        rospy.loginfo(
            "PICK DEPARTURE merge passage=P%d direction=%+d "
            "cross_track=%.3f target_yaw=%.3f",
            passage, direction, initial_cross_track, target_yaw
        )
        # Same timeout and absolute ceiling as go_to_point().
        absolute_timeout = float(rospy.get_param(
            "~pick_departure_absolute_timeout", 300.0
        ))
        started_at = time.monotonic()
        absolute_started_at = started_at

        while not rospy.is_shutdown():
            if not self.ensure_robot_level("pick_departure"):
                return False

            if peer_clearance_fn is not None:
                wait_started = time.monotonic()
                peer_clearance_fn()
                started_at += time.monotonic() - wait_started

            rel_x = self.x - corridor_x
            rel_y = self.y - corridor_y
            cross_track = normal_x * rel_x + normal_y * rel_y
            yaw_error = self.normalize_angle(target_yaw - self.yaw)

            if (
                abs(cross_track) <= merge_cross_track
                and abs(yaw_error) <= yaw_tolerance
            ):
                rospy.loginfo(
                    "PICK DEPARTURE merged passage=P%d cross_track=%.3f "
                    "yaw_error=%.3f",
                    passage, cross_track, yaw_error
                )
                return True

            now = time.monotonic()
            if now - absolute_started_at > absolute_timeout:
                self.stop()
                rospy.logerr(
                    "PICK DEPARTURE absolute timeout passage=P%d "
                    "cross_track=%.3f yaw_error=%.3f absolute_timeout=%.1fs "
                    "(peer-wait time included)",
                    passage, cross_track, yaw_error, absolute_timeout
                )
                return False

            if now - started_at > timeout:
                self.stop()
                rospy.logerr(
                    "PICK DEPARTURE timeout passage=P%d cross_track=%.3f "
                    "yaw_error=%.3f",
                    passage, cross_track, yaw_error
                )
                return False

            correction = max(
                min(-0.75 * cross_track, lateral_speed), -lateral_speed
            )
            world_vx = (
                direction * forward_speed * self.length_unit[0]
                + correction * normal_x
            )
            world_vy = (
                direction * forward_speed * self.length_unit[1]
                + correction * normal_y
            )

            cy = math.cos(self.yaw)
            sy = math.sin(self.yaw)
            cmd = Twist()
            cmd.linear.x = cy * world_vx + sy * world_vy
            cmd.linear.y = -sy * world_vx + cy * world_vy
            cmd.angular.z = max(
                min(self.turn_gain * yaw_error, angular_speed), -angular_speed
            )
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

        self.stop()
        return False

    def follow_passage_segment(
        self, passage, start_index, end_index, speed_override=None,
        peer_clearance_fn=None, lateral_offset=0.0,
        max_cross_track_override=None, stall_window_override=None
    ):
        """Drive directly to the end of a straight orchard-passage segment.

        Intermediate stations are graph waypoints, not physical stops. All passage
        stations come from the same calibrated length vector, so a same-passage
        segment is a straight corridor.

        `peer_clearance_fn`: called on every control tick, may block.
        `lateral_offset`: signed offset (m) perpendicular to the centerline, applied
        to both the target and the cross-track reference. Positive is 90 deg
        clockwise from the index-increasing direction.
        `max_cross_track_override`, `stall_window_override`: per-call overrides of
        ~continuous_max_cross_track and ~continuous_stall_window (used by the
        lateral-bypass merge leg).
        """
        passage = int(passage)
        start_index = int(start_index)
        end_index = int(end_index)

        if passage < 0 or passage > self.num_rows:
            rospy.logerr("PASSAGE SEGMENT invalid passage=P%d", passage)
            return False

        if not (1 <= start_index <= self.num_indices):
            rospy.logerr("PASSAGE SEGMENT invalid start index=I%d", start_index)
            return False

        if not (1 <= end_index <= self.num_indices):
            rospy.logerr("PASSAGE SEGMENT invalid end index=I%d", end_index)
            return False

        if start_index == end_index:
            rospy.loginfo(
                "PASSAGE SEGMENT already at P%d_I%d",
                passage,
                end_index
            )
            return True

        direction = 1 if end_index > start_index else -1
        target_x, target_y = self.passage_pose(passage, end_index)

        # Shift target and reference line by the same vector, so cross-track is
        # measured from the offset line.
        offset_x = lateral_offset * self.length_unit[1]
        offset_y = -lateral_offset * self.length_unit[0]
        target_x += offset_x
        target_y += offset_y

        rospy.loginfo(
            "PASSAGE SEGMENT execute passage=P%d start=I%d end=I%d "
            "direction=%+d skipped_stops=%d target=(%.3f, %.3f) "
            "lateral_offset=%.2f",
            passage,
            start_index,
            end_index,
            direction,
            max(0, abs(end_index - start_index) - 1),
            target_x,
            target_y,
            lateral_offset
        )

        corridor_x, corridor_y = self.passage_pose(passage, 1)
        corridor_x += offset_x
        corridor_y += offset_y
        max_cross_track = float(
            max_cross_track_override
            if max_cross_track_override is not None
            else rospy.get_param("~continuous_max_cross_track", 0.35)
        )
        stall_window = float(
            stall_window_override
            if stall_window_override is not None
            else rospy.get_param("~continuous_stall_window", 4.0)
        )
        min_progress = float(rospy.get_param(
            "~continuous_min_progress", 0.10
        ))
        timeout = float(rospy.get_param(
            "~continuous_segment_timeout", 120.0
        ))
        # Absolute ceiling, not extended by peer waits (see go_to_point()).
        absolute_timeout = float(rospy.get_param(
            "~continuous_segment_absolute_timeout", 300.0
        ))
        if speed_override is None:
            segment_speed = float(rospy.get_param(
                "~continuous_forward_speed", 0.44
            ))
        else:
            segment_speed = float(speed_override)
            rospy.loginfo(
                "PASSAGE SEGMENT speed override passage=P%d speed=%.3f",
                passage, segment_speed
            )
        segment_turn_in_place = float(rospy.get_param(
            "~continuous_turn_in_place_threshold", 0.35
        ))

        started_at = time.monotonic()
        absolute_started_at = started_at
        progress_started_at = started_at
        progress_start_dist = math.hypot(target_x - self.x, target_y - self.y)

        # Scale the time budget with segment length and commanded speed. At the
        # tight-lane speed (0.18 m/s) a long segment (~18 m) cannot finish in the
        # flat 120 s. Normal-speed segments keep the flat value.
        timeout_slack = float(rospy.get_param(
            "~continuous_segment_timeout_slack", 1.4
        ))
        timeout_margin = float(rospy.get_param(
            "~continuous_segment_timeout_margin", 15.0
        ))
        drive_budget = (
            progress_start_dist / max(segment_speed, 0.01)
        ) * timeout_slack + timeout_margin
        if drive_budget > timeout:
            # Raise the absolute ceiling by the same amount so the scaled budget is
            # reachable.
            peer_wait_allowance = float(rospy.get_param(
                "~continuous_segment_peer_wait_allowance", 180.0
            ))
            scaled_absolute = max(
                absolute_timeout, drive_budget + peer_wait_allowance
            )
            rospy.loginfo(
                "PASSAGE SEGMENT timeout scaled passage=P%d dist=%.2f "
                "speed=%.3f timeout=%.1f->%.1f absolute=%.1f->%.1f",
                passage, progress_start_dist, segment_speed,
                timeout, drive_budget, absolute_timeout, scaled_absolute
            )
            timeout = drive_budget
            absolute_timeout = scaled_absolute

        while not rospy.is_shutdown():
            if not self.ensure_robot_level(
                "passage_P%d_I%d_to_I%d" % (
                    passage, start_index, end_index
                )
            ):
                return False

            if peer_clearance_fn is not None:
                # Peer-wait time counts against neither the timeout nor the stall window.
                wait_started = time.monotonic()
                peer_clearance_fn()
                wait_elapsed = time.monotonic() - wait_started
                started_at += wait_elapsed
                progress_started_at += wait_elapsed

            dx = target_x - self.x
            dy = target_y - self.y
            dist = math.hypot(dx, dy)

            if dist <= self.station_tolerance:
                self.stop()
                rospy.sleep(0.15)
                rospy.loginfo(
                    "PASSAGE SEGMENT reached passage=P%d end=I%d dist=%.3f",
                    passage,
                    end_index,
                    dist
                )
                return True

            # Signed distance from the calibrated straight passage centerline.
            rel_x = self.x - corridor_x
            rel_y = self.y - corridor_y
            cross_track = -self.length_unit[1] * rel_x + self.length_unit[0] * rel_y
            if abs(cross_track) > max_cross_track:
                self.stop()
                rospy.logerr(
                    "PASSAGE SEGMENT aborted: cross_track=%.3f limit=%.3f "
                    "current=(%.3f, %.3f)",
                    cross_track,
                    max_cross_track,
                    self.x,
                    self.y
                )
                return False

            now = time.monotonic()
            if now - absolute_started_at > absolute_timeout:
                self.stop()
                rospy.logerr(
                    "PASSAGE SEGMENT aborted: absolute_timeout=%.1f dist=%.3f "
                    "(peer-wait time included)",
                    absolute_timeout,
                    dist
                )
                return False

            if now - started_at > timeout:
                self.stop()
                rospy.logerr(
                    "PASSAGE SEGMENT aborted: timeout=%.1f dist=%.3f",
                    timeout,
                    dist
                )
                return False

            target_yaw = math.atan2(dy, dx)
            yaw_error = self.normalize_angle(target_yaw - self.yaw)
            cmd = Twist()

            if abs(yaw_error) > segment_turn_in_place:
                cmd.angular.z = max(
                    min(self.turn_gain * yaw_error, self.turn_limit),
                    -self.turn_limit
                )
                # Rotation is intentional and must not trigger translation
                # stall detection.
                progress_started_at = now
                progress_start_dist = dist
            elif abs(yaw_error) > self.drive_yaw_threshold:
                cmd.linear.x = min(0.18, 0.25 * dist)
                cmd.angular.z = max(
                    min(self.yaw_drive_gain * yaw_error, self.max_angular_drive),
                    -self.max_angular_drive
                )
            else:
                heading_factor = max(
                    0.35,
                    1.0 - abs(yaw_error) / max(self.drive_yaw_threshold, 0.01)
                )
                cmd.linear.x = min(
                    segment_speed * heading_factor,
                    0.35 * dist
                )
                cmd.angular.z = max(
                    min(self.yaw_drive_gain * yaw_error, self.max_angular_drive),
                    -self.max_angular_drive
                )

                if now - progress_started_at >= stall_window:
                    actual_progress = progress_start_dist - dist
                    required_progress = min_progress if dist > 0.8 else min(0.03, min_progress)
                    if actual_progress < required_progress:
                        self.stop()
                        rospy.logerr(
                            "PASSAGE SEGMENT aborted: stalled progress=%.3f "
                            "required=%.3f window=%.1f dist=%.3f",
                            actual_progress,
                            required_progress,
                            stall_window,
                            dist
                        )
                        return False
                    progress_started_at = now
                    progress_start_dist = dist

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

        self.stop()
        return False

    def go_to_transfer_corridor(self, passage, index, direction, peer_clearance_fn=None):
        x, y = self.passage_pose(passage, index)

        x = x + direction * self.transfer_margin * self.length_unit[0]
        y = y + direction * self.transfer_margin * self.length_unit[1]

        return self.go_to_point(
            x, y, self.go_tolerance, peer_clearance_fn=peer_clearance_fn
        )

    def go_to_point_holonomic(
        self, target_x, target_y, target_yaw, tolerance=None,
        max_linear_override=None, peer_clearance_fn=None
    ):
        """Translate in the base frame while converging to a requested yaw.

        max_linear_override: optional per-call speed limit (used by cross_shortcut()).
        peer_clearance_fn: same contract as in go_to_point().
        """
        if tolerance is None:
            tolerance = self.go_tolerance

        timeout = float(rospy.get_param("~holonomic_transfer_timeout", 60.0))
        absolute_timeout = float(rospy.get_param(
            "~holonomic_transfer_absolute_timeout", 300.0
        ))
        if max_linear_override is None:
            max_linear = float(rospy.get_param("~holonomic_transfer_speed", 0.30))
        else:
            max_linear = float(max_linear_override)
        max_angular = float(rospy.get_param("~holonomic_transfer_turn", 0.30))
        yaw_tolerance = float(rospy.get_param(
            "~holonomic_transfer_yaw_tolerance", 0.10
        ))
        stall_window = float(rospy.get_param(
            "~holonomic_transfer_stall_window", 4.0
        ))
        min_progress = float(rospy.get_param(
            "~holonomic_transfer_min_progress", 0.10
        ))
        started_at = time.monotonic()
        absolute_started_at = started_at
        progress_started_at = started_at
        progress_start_dist = math.hypot(target_x - self.x, target_y - self.y)

        while not rospy.is_shutdown():
            if not self.ensure_robot_level("holonomic_transfer"):
                return False

            if peer_clearance_fn is not None:
                wait_started = time.monotonic()
                peer_clearance_fn()
                wait_elapsed = time.monotonic() - wait_started
                started_at += wait_elapsed
                progress_started_at += wait_elapsed

            dx = target_x - self.x
            dy = target_y - self.y
            dist = math.hypot(dx, dy)
            yaw_error = self.normalize_angle(target_yaw - self.yaw)

            if dist <= tolerance and abs(yaw_error) <= yaw_tolerance:
                self.stop()
                rospy.sleep(0.15)
                return True

            now = time.monotonic()
            if now - absolute_started_at > absolute_timeout:
                self.stop()
                rospy.logerr(
                    "Holonomic transfer absolute timeout dist=%.3f "
                    "yaw_error=%.3f absolute_timeout=%.1fs "
                    "(peer-wait time included)",
                    dist, yaw_error, absolute_timeout
                )
                return False

            if now - started_at > timeout:
                self.stop()
                rospy.logerr(
                    "Holonomic transfer timeout dist=%.3f yaw_error=%.3f",
                    dist,
                    yaw_error
                )
                return False

            if dist <= tolerance:
                # Positionally arrived; any remaining time is spent
                # converging yaw in place, which makes no further positional
                # progress by design. Do not stall-check this phase.
                progress_started_at = now
                progress_start_dist = dist
            elif now - progress_started_at >= stall_window:
                actual_progress = progress_start_dist - dist
                if actual_progress < min_progress:
                    self.stop()
                    rospy.logerr(
                        "Holonomic transfer stalled progress=%.3f "
                        "required=%.3f window=%.1f dist=%.3f yaw_error=%.3f",
                        actual_progress,
                        min_progress,
                        stall_window,
                        dist,
                        yaw_error
                    )
                    return False
                progress_started_at = now
                progress_start_dist = dist

            cy = math.cos(self.yaw)
            sy = math.sin(self.yaw)
            error_x_body = cy * dx + sy * dy
            error_y_body = -sy * dx + cy * dy

            cmd = Twist()
            cmd.linear.x = max(min(0.65 * error_x_body, max_linear), -max_linear)
            cmd.linear.y = max(min(0.65 * error_y_body, max_linear), -max_linear)
            cmd.angular.z = max(
                min(self.turn_gain * yaw_error, max_angular),
                -max_angular
            )
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

        self.stop()
        return False

    def cross_shortcut(self, passage, next_passage, gap_index, peer_clearance_fn=None):
        """Cross to the adjacent passage through a mid-row gap between two trees.

        Two holonomic moves: align at the near-side gap point
        (passage_pose(passage, gap_index + 0.5)) with a tight tolerance, then cross
        to the far-side gap point at ~shortcut_cross_speed. Unlike an endcap
        transfer there is no open space to pull out into, so no transfer_margin.
        The caller must already be near (passage, gap_index).
        """
        near_x, near_y = self.passage_pose(passage, gap_index + 0.5)
        far_x, far_y = self.passage_pose(next_passage, gap_index + 0.5)

        rospy.loginfo(
            "SHORTCUT CROSS passage=P%d next_passage=P%d gap_index=%d "
            "near=(%.3f, %.3f) far=(%.3f, %.3f) cross_speed=%.3f",
            passage, next_passage, gap_index,
            near_x, near_y, far_x, far_y, self.shortcut_cross_speed
        )

        if not self.go_to_point_holonomic(
            near_x, near_y, self.base_yaw,
            tolerance=self.shortcut_align_tolerance,
            peer_clearance_fn=peer_clearance_fn
        ):
            rospy.logerr(
                "SHORTCUT CROSS FAILED could not align at near-side gap "
                "point passage=P%d gap_index=%d", passage, gap_index
            )
            return False

        if not self.go_to_point_holonomic(
            far_x, far_y, self.base_yaw,
            tolerance=self.shortcut_cross_tolerance,
            max_linear_override=self.shortcut_cross_speed,
            peer_clearance_fn=peer_clearance_fn
        ):
            rospy.logerr(
                "SHORTCUT CROSS FAILED could not cross passage=P%d -> "
                "P%d gap_index=%d", passage, next_passage, gap_index
            )
            return False

        rospy.loginfo(
            "SHORTCUT CROSS SUCCESS passage=P%d -> P%d gap_index=%d "
            "current=(%.3f, %.3f, %.3f)",
            passage, next_passage, gap_index, self.x, self.y, self.yaw
        )
        return True

    def face_tree_group(self, row, index):
        tx, ty = self.tree_pose(row, index)
        yaw = math.atan2(ty - self.y, tx - self.x)
        return self.turn_to_yaw(yaw)


    def fruit_pose(self, row, index, side):
        tree_x, tree_y = self.tree_pose(row, index)

        side_sign = -1.0 if side == "L" else 1.0

        fruit_x = tree_x + side_sign * self.fruit_side_offset * self.width_unit[0]
        fruit_y = tree_y + side_sign * self.fruit_side_offset * self.width_unit[1]

        return fruit_x, fruit_y

    def path_yaw_for_pick(self, row=None, index=None, side=None):
        # Parallel path mode:
        # The robot stays aligned with the orchard row/path.
        # There are two valid parallel orientations, yaw_pos and yaw_neg.
        # In side-consistent mode we choose the one that puts the fruit on
        # the expected physical side of the robot:
        #   symbolic side R -> fruit should be on robot right -> left_robot < 0
        #   symbolic side L -> fruit should be on robot left  -> left_robot > 0

        yaw_pos = math.atan2(self.length_unit[1], self.length_unit[0])
        yaw_neg = self.normalize_angle(yaw_pos + math.pi)

        yaw_mode = str(rospy.get_param("~parallel_path_yaw", "auto")).lower()
        side_policy = str(rospy.get_param("~parallel_path_side_policy", "symbolic_side")).lower()

        candidates = [yaw_pos, yaw_neg]

        def yaw_error(yaw):
            return abs(self.normalize_angle(yaw - self.yaw))

        def fruit_left_for_yaw(yaw):
            if row is None or index is None or side is None:
                return None

            fruit_x, fruit_y = self.fruit_pose(row, index, side)
            dx = fruit_x - self.x
            dy = fruit_y - self.y

            c = math.cos(yaw)
            ss = math.sin(yaw)

            left = -ss * dx + c * dy
            return left

        if yaw_mode in ["positive", "pos", "+", "forward"]:
            chosen = yaw_pos
            reason = "forced_positive"
        elif yaw_mode in ["negative", "neg", "-", "backward"]:
            chosen = yaw_neg
            reason = "forced_negative"
        elif side_policy in ["symbolic_side", "side", "match_side"] and side in ["L", "R"]:
            desired_left_sign = 1.0 if side == "L" else -1.0

            valid = []
            for cand in candidates:
                left = fruit_left_for_yaw(cand)
                if left is None:
                    continue

                if desired_left_sign * left > 0.0:
                    valid.append((cand, left, yaw_error(cand)))

            if valid:
                valid.sort(key=lambda item: item[2])
                chosen = valid[0][0]
                reason = "match_symbolic_side"
            else:
                chosen = min(candidates, key=yaw_error)
                reason = "fallback_min_turn_no_side_match"
        else:
            chosen = min(candidates, key=yaw_error)
            reason = "auto_min_turn"

        final_left = fruit_left_for_yaw(chosen)

        rospy.loginfo(
            "PARALLEL_PATH_YAW mode=%s side_policy=%s symbolic_side=%s "
            "yaw_pos=%.3f yaw_neg=%.3f current=%.3f chosen=%.3f fruit_left=%.3f reason=%s",
            yaw_mode,
            side_policy,
            str(side),
            yaw_pos,
            yaw_neg,
            self.yaw,
            chosen,
            final_left if final_left is not None else 999.0,
            reason
        )

        return chosen


    def face_fruit(self, row, index, side):
        fruit_x, fruit_y = self.fruit_pose(row, index, side)

        yaw = math.atan2(fruit_y - self.y, fruit_x - self.x)

        rospy.loginfo(
            "FACE fruit row=%d index=%d side=%s target=(%.2f, %.2f)",
            row,
            index,
            side,
            fruit_x,
            fruit_y
        )

        return self.turn_to_yaw(yaw)

    def approach_for_pick(self, passage, index, row, side):
        station_x, station_y = self.passage_pose(passage, index)
        fruit_x, fruit_y = self.fruit_pose(row, index, side)

        dx = fruit_x - station_x
        dy = fruit_y - station_y
        dist = math.sqrt(dx * dx + dy * dy)

        rospy.loginfo(
            "APPROACH plan station=P%d_%d row=%d side=%s station=(%.2f, %.2f) fruit=(%.2f, %.2f) dist=%.2f",
            passage,
            index,
            row,
            side,
            station_x,
            station_y,
            fruit_x,
            fruit_y,
            dist
        )

        if dist < 0.05:
            return False

        fruit_yaw = math.atan2(fruit_y - self.y, fruit_x - self.x)
        yaw_error = self.normalize_angle(fruit_yaw - self.yaw)

        if abs(yaw_error) > self.face_yaw_threshold:
            rospy.loginfo(
                "FACE needed row=%d index=%d side=%s yaw_error=%.1f deg",
                row,
                index,
                side,
                math.degrees(yaw_error)
            )
            if not self.turn_to_yaw(fruit_yaw):
                return False
        else:
            rospy.loginfo(
                "FACE skipped row=%d index=%d side=%s yaw_error=%.1f deg",
                row,
                index,
                side,
                math.degrees(yaw_error)
            )

        if not self.enable_pick_approach:
            return False

        max_safe = max(0.0, dist - self.pick_min_fruit_clearance)
        approach = min(self.pick_approach_distance, max_safe)

        if approach <= 0.02:
            rospy.loginfo("APPROACH skipped, already close enough.")
            return False

        ux = dx / dist
        uy = dy / dist

        target_x = station_x + approach * ux
        target_y = station_y + approach * uy

        rospy.loginfo(
            "APPROACH execute row=%d index=%d side=%s approach=%.2f target=(%.2f, %.2f)",
            row,
            index,
            side,
            approach,
            target_x,
            target_y
        )

        return self.go_to_point(target_x, target_y, self.pick_approach_tolerance)

    def return_to_station_after_pick(self, passage, index):
        if not self.return_after_pick:
            return True

        station_x, station_y = self.passage_pose(passage, index)

        rospy.loginfo(
            "RETURN station=P%d_%d target=(%.2f, %.2f)",
            passage,
            index,
            station_x,
            station_y
        )

        return self.go_to_point(station_x, station_y, self.pick_approach_tolerance)



    def pick_pose(self, passage, index, row, side):
        station_x, station_y = self.passage_pose(passage, index)
        fruit_x, fruit_y = self.fruit_pose(row, index, side)

        dx = fruit_x - station_x
        dy = fruit_y - station_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.05:
            return station_x, station_y

        ux = dx / dist
        uy = dy / dist

        # Desired distance from robot base to fruit.
        # This controls how far in front of the robot the fruit should remain.
        desired_forward = float(rospy.get_param("~desired_fruit_forward", 1.10))
        desired_forward = max(0.65, min(desired_forward, 1.50))

        # Safety limit: the base must not leave the passage too much.
        # The arm should do the final approach, not the mobile base.
        max_move_from_station = float(rospy.get_param("~max_pick_pose_move_from_station", 0.35))
        max_move_from_station = max(0.0, min(max_move_from_station, 0.80))

        ideal_move_distance = max(0.0, dist - desired_forward)
        move_distance = min(ideal_move_distance, max_move_from_station)

        pick_x = station_x + move_distance * ux
        pick_y = station_y + move_distance * uy

        rospy.loginfo(
            "PICK_POSE_SAFE dist_station_to_fruit=%.3f desired_forward=%.3f "
            "ideal_move=%.3f max_move_from_station=%.3f actual_move=%.3f",
            dist,
            desired_forward,
            ideal_move_distance,
            max_move_from_station,
            move_distance
        )

        return pick_x, pick_y


    def move_to_pick_pose(self, passage, index, row, side):
        pick_x, pick_y = self.pick_pose(passage, index, row, side)
        station_x, station_y = self.passage_pose(passage, index)
        fruit_x, fruit_y = self.fruit_pose(row, index, side)

        orientation_mode = str(rospy.get_param("~pick_orientation_mode", "face_fruit")).lower()

        rospy.loginfo(
            "PICK_POSE station=P%d_%d row=%d side=%s mode=%s station=(%.2f,%.2f) pick=(%.2f,%.2f) fruit=(%.2f,%.2f)",
            passage,
            index,
            row,
            side,
            orientation_mode,
            station_x,
            station_y,
            pick_x,
            pick_y,
            fruit_x,
            fruit_y
        )

        moved = self.go_to_point(pick_x, pick_y, self.pick_approach_tolerance)

        if not moved:
            return False

        if orientation_mode in ["parallel_path", "side_pick", "side_picking", "parallel"]:
            rospy.loginfo("SIDE PICKING: keeping robot parallel to path instead of facing fruit.")
            return self.turn_to_yaw(self.path_yaw_for_pick(row=row, index=index, side=side))

        return self.face_fruit(row, index, side)


    def model_exists(self, name):
        try:
            state = self.get_state(name, "world")
            return state.success
        except Exception:
            return False

    def arm_side_from_fruit_geometry(self, row, index, symbolic_side):
        fruit_x, fruit_y = self.fruit_pose(row, index, symbolic_side)

        dx = fruit_x - self.x
        dy = fruit_y - self.y

        c = math.cos(self.yaw)
        ss = math.sin(self.yaw)

        forward = c * dx + ss * dy
        left = -ss * dx + c * dy

        arm_side = "L" if left >= 0.0 else "R"

        rospy.loginfo(
            "ARM SIDE GEOMETRY symbolic_side=%s fruit=(%.3f,%.3f) robot=(%.3f,%.3f,yaw=%.3f) "
            "forward=%.3f left=%.3f -> arm_side=%s",
            symbolic_side,
            fruit_x,
            fruit_y,
            self.x,
            self.y,
            self.yaw,
            forward,
            left,
            arm_side
        )

        return arm_side

    def pick_group(self, passage, index, row, side):
        newly_picked = 0

        names = [
            f"orange{row}.{index}.{side}.B",
            f"orange{row}.{index}.{side}.T"
        ]

        # Align the base for picking before using the arm.
        if rospy.get_param("~ensure_pick_pose_before_pick", True):
            rospy.loginfo(
                "ENSURE PICK POSE before PICK passage=P%d index=%d row=%d side=%s",
                passage,
                index,
                row,
                side
            )

            if not self.move_to_pick_pose(passage, index, row, side):
                rospy.logwarn("PICK aborted: robot could not reach/face pick pose.")
                return 0

            rospy.sleep(float(rospy.get_param("~pick_pose_settle_time", 0.25)))
        else:
            if self.enable_pick_approach:
                if not self.move_to_pick_pose(passage, index, row, side):
                    rospy.logwarn("PICK aborted: move_to_pick_pose failed.")
                    return 0
            elif self.face_before_pick:
                if not self.face_fruit(row, index, side):
                    rospy.logwarn("PICK aborted: face_fruit failed.")
                    return 0

        context_results = []

        if rospy.get_param("~use_fruit_context", False):
            try:
                if not hasattr(self, "fruit_context") or self.fruit_context is None:
                    from fruit_pick_context import FruitPickContextLogger
                    self.fruit_context = FruitPickContextLogger()

                context_results = self.fruit_context.log_group(
                    row=row,
                    index=index,
                    side=side,
                    robot_x=self.x,
                    robot_y=self.y,
                    robot_yaw=self.yaw
                )

                require_reachable = rospy.get_param("~require_fruit_reachable", True)

                if require_reachable:
                    bad = [
                        (name, reason)
                        for name, reachable, reason in context_results
                        if not reachable
                    ]

                    if bad:
                        rospy.logwarn("PICK aborted: fruit not reachable: %s", bad)
                        return 0

            except Exception as e:
                rospy.logerr("FRUIT CONTEXT failed during PICK: %s", str(e))

                if rospy.get_param("~fruit_context_required", True):
                    rospy.logerr("fruit_context_required=true, so PICK is aborted.")
                    return 0

        if rospy.get_param("~use_arm_demo", False):
            try:
                if not hasattr(self, "arm_demo") or self.arm_demo is None:
                    from arm_pick_demo import ArmPickDemo
                    self.arm_demo = ArmPickDemo()

                arm_side = side
                orientation_mode = str(rospy.get_param("~pick_orientation_mode", "face_fruit")).lower()

                if orientation_mode in ["parallel_path", "side_pick", "side_picking", "parallel"]:
                    arm_side = self.arm_side_from_fruit_geometry(row, index, side)

                rospy.loginfo(
                    "ARM PICK SIDE selected symbolic_side=%s actual_arm_side=%s mode=%s",
                    side,
                    arm_side,
                    orientation_mode
                )

                self.arm_demo.pick(side=arm_side, row=row, index=index)

            except Exception as e:
                rospy.logerr("ARM DEMO failed during PICK: %s", str(e))

                if rospy.get_param("~arm_demo_required", False):
                    rospy.logerr("arm_demo_required=true, so PICK is aborted.")
                    return 0

                rospy.logwarn("Continuing abstract PICK because arm_demo_required=false.")

        rospy.loginfo(
            "PICK passage=P%d index=%d row=%d side=%s names=%s",
            passage,
            index,
            row,
            side,
            names
        )

        rospy.sleep(self.pause_at_pick)

        for name in names:
            if name in self.picked:
                continue

            if not self.model_exists(name):
                rospy.logwarn("Fruit model not found: %s", name)
                continue

            self.picked.add(name)
            newly_picked += 1

            if self.delete_after_pick:
                try:
                    result = self.delete_model(name)
                    self.deleted.add(name)
                    rospy.loginfo("Picked and deleted: %s | %s", name, result.status_message)
                except Exception as exc:
                    rospy.logwarn("Cannot delete %s: %s", name, exc)
            else:
                rospy.loginfo("Picked without deleting: %s", name)

        rospy.loginfo("Picked new=%d total=%d", newly_picked, len(self.picked))

        if self.return_after_pick:
            self.return_to_station_after_pick(passage, index)

        return newly_picked


    def pick_at_station(self, passage, index):
        node = self.passage_node(passage, index)
        groups = self.map["accessible"][node]

        rospy.loginfo("STATION %s accessible=%s", node, groups)

        total = 0

        for group in groups:
            row = group["row"]
            side = group["side"]
            total += self.pick_group(passage, index, row, side)

        return total

    def traverse_passage(self, passage, direction):
        if direction > 0:
            indices = range(1, self.num_indices + 1)
            desired_yaw = self.base_yaw
        else:
            indices = range(self.num_indices, 0, -1)
            desired_yaw = self.normalize_angle(self.base_yaw + math.pi)

        rospy.loginfo(
            "TRAVERSE passage=P%d direction=%d",
            passage,
            direction
        )

        self.turn_to_yaw(desired_yaw)

        for index in indices:
            if not self.go_to_station(passage, index):
                return False

            if self.align_before_station_pick:
                self.turn_to_yaw(desired_yaw)

            self.pick_at_station(passage, index)

        return True

    def transfer_to_next_passage(self, passage, next_passage, direction, peer_clearance_fn=None):
        if direction > 0:
            index = self.num_indices
            desired_yaw = math.atan2(-self.length_unit[1], -self.length_unit[0])
        else:
            index = 1
            desired_yaw = math.atan2(self.length_unit[1], self.length_unit[0])

        rospy.loginfo(
            "TRANSFER P%d -> P%d at index=%d",
            passage,
            next_passage,
            index
        )

        if not self.go_to_transfer_corridor(
            passage, index, direction, peer_clearance_fn=peer_clearance_fn
        ):
            return False

        holonomic_transfer = bool(rospy.get_param("~holonomic_transfer", True))
        if holonomic_transfer:
            # Converge heading to desired_yaw here, in the open transfer
            # corridor, before the lateral cross-passage translation. If the
            # approach left the robot facing far from desired_yaw (e.g.
            # arriving via a within-passage move in the opposite direction),
            # resolving that large yaw error concurrently with translation
            # sweeps an arc that can clip the neighboring row.
            if not self.turn_to_yaw(desired_yaw, peer_clearance_fn=peer_clearance_fn):
                return False

            target_x, target_y = self.passage_pose(next_passage, index)
            target_x += direction * self.transfer_margin * self.length_unit[0]
            target_y += direction * self.transfer_margin * self.length_unit[1]
            if not self.go_to_point_holonomic(
                target_x,
                target_y,
                desired_yaw,
                self.go_tolerance,
                peer_clearance_fn=peer_clearance_fn
            ):
                return False
        elif not self.go_to_transfer_corridor(
            next_passage, index, direction, peer_clearance_fn=peer_clearance_fn
        ):
            return False

        if self.keep_transfer_endcap:
            rospy.loginfo("Keeping robot in safe transfer endcap after row change.")
            if not holonomic_transfer:
                self.turn_to_yaw(desired_yaw, peer_clearance_fn=peer_clearance_fn)
            return True

        if not self.go_to_station(next_passage, index, peer_clearance_fn=peer_clearance_fn):
            return False

        return True

    def wait_for_odom(self):
        rospy.loginfo("Waiting for odometry: %s", self.odom_topic)

        while not rospy.is_shutdown() and not self.has_odom:
            self.rate.sleep()

    def run(self):
        self.wait_for_odom()

        direction = 1

        for passage in range(0, self.num_rows + 1):
            if not self.traverse_passage(passage, direction):
                rospy.logwarn("Traversal failed at passage P%d", passage)
                break

            if passage < self.num_rows:
                if not self.transfer_to_next_passage(passage, passage + 1, direction):
                    rospy.logwarn("Transfer failed from P%d to P%d", passage, passage + 1)
                    break

            direction *= -1

        self.stop()
        rospy.loginfo("MAP PICKING FINISHED.")
        rospy.loginfo("UKUPNO OBRANO PLODOVA: %d", len(self.picked))


if __name__ == "__main__":
    picker = OrchardMapPicker()
    picker.run()
