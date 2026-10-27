#!/usr/bin/env python3

import csv
import math
import sys
import time
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from harbot_orchard_picker.orchard_geometry import OrchardGeometry


class MetricsRecorder:
    def __init__(self, output_path, odom_topic, geometry=None):
        self.output = open(output_path, "w", newline="")
        self.writer = csv.writer(self.output)
        self.writer.writerow([
            "wall_time_s", "sim_time_s", "x_m", "y_m", "z_m",
            "roll_deg", "pitch_deg", "yaw_deg", "linear_x_mps",
            "linear_y_mps", "linear_speed_mps", "angular_z_radps"
        ])

        # Second output with the same samples in map terms (passage, fractional
        # index), e.g. trajectory_robot.csv -> trajectory_robot_map.csv.
        self.geometry = geometry
        self.map_output = None
        self.map_writer = None
        if self.geometry is not None:
            map_path = str(output_path)
            if map_path.endswith(".csv"):
                map_path = map_path[:-len(".csv")] + "_map.csv"
            else:
                map_path = map_path + "_map.csv"
            self.map_output = open(map_path, "w", newline="")
            self.map_writer = csv.writer(self.map_output)
            self.map_writer.writerow([
                "wall_time_s", "sim_time_s", "passage", "index",
                "linear_speed_mps"
            ])

        self.last_write = 0.0
        self.subscription = rospy.Subscriber(
            odom_topic,
            Odometry,
            self.on_odom,
            queue_size=100,
        )

    def on_odom(self, msg):
        now = msg.header.stamp.to_sec() or rospy.get_time()
        if now - self.last_write < 0.05:
            return
        self.last_write = now
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        speed = math.hypot(linear.x, linear.y)
        wall_time = time.time()
        self.writer.writerow([
            "%.6f" % wall_time, "%.6f" % now,
            "%.6f" % p.x, "%.6f" % p.y, "%.6f" % p.z,
            "%.6f" % math.degrees(roll),
            "%.6f" % math.degrees(pitch),
            "%.6f" % math.degrees(yaw),
            "%.6f" % linear.x, "%.6f" % linear.y,
            "%.6f" % speed,
            "%.6f" % angular.z,
        ])
        self.output.flush()

        if self.map_writer is not None:
            passage, frac_index = self.geometry.xy_to_passage_index(p.x, p.y)
            self.map_writer.writerow([
                "%.6f" % wall_time, "%.6f" % now,
                passage, "%.3f" % frac_index, "%.6f" % speed,
            ])
            self.map_output.flush()

    def close(self):
        self.output.close()
        if self.map_output is not None:
            self.map_output.close()


if __name__ == "__main__":
    if len(sys.argv) not in (2, 3):
        raise SystemExit("usage: report_metrics_recorder.py OUTPUT.csv [ODOM_TOPIC]")
    topic = sys.argv[2] if len(sys.argv) == 3 else "/robot/robotnik_base_control/odom_gt"
    rospy.init_node("harbot_report_metrics_recorder", anonymous=True)

    geometry = None
    try:
        tree_points_file = Path(rospy.get_param(
            "~tree_points_file",
            str(Path.home() / "harbot_calibration/tree_points.jsonl")
        ))
        num_rows = int(rospy.get_param("~num_rows", 4))
        num_indices = int(rospy.get_param("~num_indices", 8))
        geometry = OrchardGeometry(tree_points_file, num_rows, num_indices)
    except Exception as exc:
        # The map output is optional; keep recording XY even without calibration.
        rospy.logwarn(
            "report_metrics_recorder: map-path output disabled (%s)", exc
        )
        geometry = None

    recorder = MetricsRecorder(sys.argv[1], topic, geometry=geometry)
    rospy.on_shutdown(recorder.close)
    rospy.spin()
