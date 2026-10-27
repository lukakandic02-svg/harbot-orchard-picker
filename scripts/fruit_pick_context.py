#!/usr/bin/env python3

import csv
import json
import math
from pathlib import Path

import rospy


class FruitPickContextLogger:
    def __init__(self):
        self.calib_file = Path(
            rospy.get_param(
                "~fruit_calibration_file",
                str(Path.home() / "harbot_calibration/fruit_calibration.json")
            )
        )

        self.log_file = Path(
            rospy.get_param(
                "~fruit_pick_detail_log_file",
                str(Path.home() / "harbot_calibration/fruit_pick_detail_log.csv")
            )
        )

        self.max_reach = float(rospy.get_param("~arm_max_reach", 2.20))
        self.min_reach = float(rospy.get_param("~arm_min_reach", 0.20))
        self.max_abs_lateral = float(rospy.get_param("~arm_max_abs_lateral", 2.00))
        self.min_forward = float(rospy.get_param("~arm_min_forward", 0.10))
        self.max_forward = float(rospy.get_param("~arm_max_forward", 1.05))
        self.min_z = float(rospy.get_param("~arm_min_z", 0.30))
        self.max_z = float(rospy.get_param("~arm_max_z", 1.40))

        self.fruits = self.load_fruits()

        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        new_file = not self.log_file.exists()

        self.csv = open(self.log_file, "a", newline="")
        self.writer = csv.writer(self.csv)

        if new_file:
            self.writer.writerow([
                "time",
                "row",
                "index",
                "side",
                "height",
                "fruit_name",
                "fruit_x",
                "fruit_y",
                "fruit_z",
                "robot_x",
                "robot_y",
                "robot_yaw",
                "dx_world",
                "dy_world",
                "distance_xy",
                "forward_robot",
                "left_robot",
                "z",
                "reachable",
                "reason"
            ])
            self.csv.flush()

        rospy.loginfo("FruitPickContextLogger loaded %d fruits from %s", len(self.fruits), self.calib_file)

    def load_fruits(self):
        if not self.calib_file.exists():
            rospy.logwarn("Fruit calibration file does not exist: %s", self.calib_file)
            return {}

        with open(self.calib_file) as f:
            return json.load(f)

    def fruit_names_for_group(self, row, index, side):
        return [
            f"orange{row}.{index}.{side}.B",
            f"orange{row}.{index}.{side}.T",
        ]

    def world_to_robot(self, robot_x, robot_y, robot_yaw, fruit_x, fruit_y):
        dx = fruit_x - robot_x
        dy = fruit_y - robot_y

        c = math.cos(robot_yaw)
        s = math.sin(robot_yaw)

        forward = c * dx + s * dy
        left = -s * dx + c * dy

        return dx, dy, forward, left

    def reachability(self, distance_xy, forward, left, z):
        reasons = []

        orientation_mode = str(rospy.get_param("~pick_orientation_mode", "face_fruit")).lower()

        if distance_xy > self.max_reach:
            reasons.append("too_far")

        if distance_xy < self.min_reach:
            reasons.append("too_close")

        if z < self.min_z:
            reasons.append("too_low")

        if z > self.max_z:
            reasons.append("too_high")

        if orientation_mode in ["parallel_path", "side_pick", "side_picking", "parallel"]:
            # Side-picking model:
            # forward = error along the path direction
            # left    = lateral distance from robot to fruit
            max_abs_forward = float(rospy.get_param("~side_pick_max_abs_forward", 0.55))
            min_lateral = float(rospy.get_param("~side_pick_min_lateral", 0.55))
            max_lateral = float(rospy.get_param("~side_pick_max_lateral", 1.65))

            lateral = abs(left)

            if abs(forward) > max_abs_forward:
                reasons.append("too_far_along_path")

            if lateral < min_lateral:
                reasons.append("too_close_lateral")

            if lateral > max_lateral:
                reasons.append("too_far_lateral")

        else:
            # Face-fruit model:
            # fruit should be in front of the robot.
            if abs(left) > self.max_abs_lateral:
                reasons.append("too_lateral")

            if forward < self.min_forward:
                reasons.append("behind_or_not_in_front")

            if forward > self.max_forward:
                reasons.append("too_far_forward")

        if reasons:
            return False, "+".join(reasons)

        return True, "ok"

    def log_group(self, row, index, side, robot_x, robot_y, robot_yaw):
        names = self.fruit_names_for_group(row, index, side)

        result = []

        for name in names:
            if name not in self.fruits:
                rospy.logwarn("Fruit not found in calibration: %s", name)
                result.append((name, False, "missing_calibration"))
                continue

            f = self.fruits[name]
            fx = float(f["x"])
            fy = float(f["y"])
            fz = float(f["z"])

            dx, dy, forward, left = self.world_to_robot(
                robot_x,
                robot_y,
                robot_yaw,
                fx,
                fy
            )

            distance_xy = math.sqrt(dx * dx + dy * dy)
            reachable, reason = self.reachability(distance_xy, forward, left, fz)

            height = name.split(".")[-1]

            rospy.loginfo(
                "FRUIT CONTEXT %s: fruit=(%.3f, %.3f, %.3f) robot=(%.3f, %.3f, yaw=%.3f) "
                "dist=%.3f forward=%.3f left=%.3f reachable=%s reason=%s",
                name,
                fx,
                fy,
                fz,
                robot_x,
                robot_y,
                robot_yaw,
                distance_xy,
                forward,
                left,
                reachable,
                reason
            )

            self.writer.writerow([
                rospy.Time.now().to_sec(),
                row,
                index,
                side,
                height,
                name,
                fx,
                fy,
                fz,
                robot_x,
                robot_y,
                robot_yaw,
                dx,
                dy,
                distance_xy,
                forward,
                left,
                fz,
                reachable,
                reason
            ])
            self.csv.flush()

            result.append((name, reachable, reason))

        return result
