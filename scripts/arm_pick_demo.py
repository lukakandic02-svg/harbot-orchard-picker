#!/usr/bin/env python3

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class ArmPickDemo:
    def __init__(self):
        self.arm_action = rospy.get_param(
            "~arm_action",
            "/robot/arm/pos_traj_controller/follow_joint_trajectory"
        )

        self.gripper_action = rospy.get_param(
            "~gripper_action",
            "/robot/gripper/pos_traj_controller/follow_joint_trajectory"
        )

        self.arm_joints = rospy.get_param(
            "~arm_joints",
            [
                "robot_joint1",
                "robot_joint2",
                "robot_joint3",
                "robot_joint4",
                "robot_joint5",
                "robot_joint6",
                "robot_joint7",
            ]
        )

        self.gripper_joints = rospy.get_param(
            "~gripper_joints",
            ["robot_finger_joint1"]
        )

        self.open_gripper_pos = float(rospy.get_param("~open_gripper_pos", 0.04))
        self.close_gripper_pos = float(rospy.get_param("~close_gripper_pos", 0.0))

        self.arm_duration = float(rospy.get_param("~arm_duration", 2.4))
        self.short_arm_duration = float(rospy.get_param("~short_arm_duration", 1.4))
        self.gripper_duration = float(rospy.get_param("~gripper_duration", 0.9))

        self.neutral_pose = rospy.get_param(
            "~arm_neutral_pose",
            [0.0, 0.0, 0.0, -0.50, 0.0, 0.85, 0.0]
        )

        self.retreat_pose = rospy.get_param(
            "~arm_retreat_pose",
            [0.0, -0.20, 0.0, -0.70, 0.0, 0.95, 0.0]
        )

        # Basket/drop pose on top of the mobile robot.
        # This is a scripted visual deposit motion, not physics attach yet.
        self.use_basket_motion = bool(rospy.get_param("~arm_use_basket_motion", True))

        self.basket_pose = rospy.get_param(
            "~arm_basket_pose",
            [0.0, -0.10, 0.0, -0.45, 0.0, 0.95, 0.0]
        )

        self.basket_duration = float(rospy.get_param("~arm_basket_duration", 1.8))

        # Joint 1 sign is used as a simple left/right side selector.
        # These are still scripted poses, not IK yet.
        self.side_yaw = float(rospy.get_param("~arm_side_yaw", 0.38))

        # Lower fruit sequence.
        self.lower_pre_base = rospy.get_param(
            "~arm_lower_pre_base",
            [0.0, -0.25, 0.0, -0.80, 0.0, 1.00, 0.0]
        )

        self.lower_grasp_base = rospy.get_param(
            "~arm_lower_grasp_base",
            [0.0, -0.42, 0.0, -1.02, 0.0, 1.18, 0.0]
        )

        # Upper fruit sequence.
        self.upper_pre_base = rospy.get_param(
            "~arm_upper_pre_base",
            [0.0, -0.15, 0.0, -0.62, 0.0, 0.92, 0.0]
        )

        self.upper_grasp_base = rospy.get_param(
            "~arm_upper_grasp_base",
            [0.0, -0.30, 0.0, -0.82, 0.0, 1.08, 0.0]
        )

        self.arm_client = actionlib.SimpleActionClient(
            self.arm_action,
            FollowJointTrajectoryAction
        )

        self.gripper_client = actionlib.SimpleActionClient(
            self.gripper_action,
            FollowJointTrajectoryAction
        )

        rospy.loginfo("Waiting for arm action server: %s", self.arm_action)
        if not self.arm_client.wait_for_server(rospy.Duration(10.0)):
            raise RuntimeError("Arm action server not available: " + self.arm_action)

        rospy.loginfo("Waiting for gripper action server: %s", self.gripper_action)
        if not self.gripper_client.wait_for_server(rospy.Duration(10.0)):
            raise RuntimeError("Gripper action server not available: " + self.gripper_action)

        rospy.loginfo("ArmPickDemo improved sequence initialized.")

    def side_pose(self, base_pose, side):
        pose = list(base_pose)

        if side == "L":
            pose[0] = self.side_yaw
        elif side == "R":
            pose[0] = -self.side_yaw
        else:
            pose[0] = 0.0

        return pose

    def send_trajectory(self, client, joint_names, positions, duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = rospy.Duration(duration)

        goal.trajectory.points.append(point)

        client.send_goal(goal)
        finished = client.wait_for_result(rospy.Duration(duration + 5.0))

        if not finished:
            client.cancel_goal()
            raise RuntimeError("Trajectory timeout")

    def move_arm(self, pose, label, duration=None):
        if duration is None:
            duration = self.arm_duration

        rospy.loginfo("ARM SEQ: move arm to %s pose: %s", label, pose)

        self.send_trajectory(
            self.arm_client,
            self.arm_joints,
            pose,
            duration
        )

    def open_gripper(self):
        rospy.loginfo("ARM SEQ: open gripper")
        self.send_trajectory(
            self.gripper_client,
            self.gripper_joints,
            [self.open_gripper_pos],
            self.gripper_duration
        )

    def close_gripper(self):
        rospy.loginfo("ARM SEQ: close gripper")
        self.send_trajectory(
            self.gripper_client,
            self.gripper_joints,
            [self.close_gripper_pos],
            self.gripper_duration
        )

    def pick_one_height(self, side, height):
        if height == "B":
            pre_pose = self.side_pose(self.lower_pre_base, side)
            grasp_pose = self.side_pose(self.lower_grasp_base, side)
            label = "LOWER/B"
        else:
            pre_pose = self.side_pose(self.upper_pre_base, side)
            grasp_pose = self.side_pose(self.upper_grasp_base, side)
            label = "UPPER/T"

        rospy.loginfo("ARM SEQ: start %s fruit pick, side=%s", label, side)

        # 1) Prepare gripper.
        self.open_gripper()
        rospy.sleep(0.15)

        # 2) Approach fruit.
        self.move_arm(pre_pose, label + " pre-grasp", duration=self.arm_duration)
        rospy.sleep(0.15)

        # 3) Grasp fruit.
        self.move_arm(grasp_pose, label + " grasp", duration=self.short_arm_duration)
        rospy.sleep(0.15)

        self.close_gripper()
        rospy.sleep(0.15)

        # 4) Pull fruit away from tree.
        self.move_arm(pre_pose, label + " retreat-with-fruit", duration=self.short_arm_duration)
        rospy.sleep(0.15)

        # 5) Deposit into basket on robot.
        if self.use_basket_motion:
            rospy.loginfo("ARM SEQ: move %s fruit to basket pose: %s", label, self.basket_pose)
            self.move_arm(self.basket_pose, label + " basket/drop", duration=self.basket_duration)
            rospy.sleep(0.15)

            rospy.loginfo("ARM SEQ: release %s fruit into basket", label)
            self.open_gripper()
            rospy.sleep(0.20)

            # Return to a safe pre pose before next fruit.
            self.move_arm(pre_pose, label + " leave basket/retreat", duration=self.short_arm_duration)
            rospy.sleep(0.15)

        rospy.loginfo("ARM SEQ: finished %s fruit pick-and-basket", label)


    def pick(self, side="L", row=None, index=None):
        rospy.loginfo("ARM SEQ PICK start side=%s row=%s index=%s", side, row, index)

        # Start from a safe pose.
        self.move_arm(self.neutral_pose, "neutral/start", duration=self.short_arm_duration)
        rospy.sleep(0.2)

        # Pick bottom fruit first, then top fruit.
        self.pick_one_height(side, "B")
        rospy.sleep(0.3)

        self.pick_one_height(side, "T")
        rospy.sleep(0.3)

        # Finish in neutral pose.
        self.move_arm(self.neutral_pose, "neutral/end", duration=self.arm_duration)
        rospy.sleep(0.2)

        rospy.loginfo("ARM SEQ PICK finished")
