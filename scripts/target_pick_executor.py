#!/usr/bin/env python3

import math
import sys
import time
import copy
from pathlib import Path

import rospy
import actionlib

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import (
    GetLinkProperties,
    GetModelState,
    SetLinkProperties,
    SetModelState,
)
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from geometry_msgs.msg import PoseStamped, Twist
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import JointTrajectoryControllerState
from actionlib_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint

import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler

sys.path.insert(0, str(Path(__file__).resolve().parent))

from orchard_map_picker_pick_eff import OrchardMapPicker
from pick_eff_common import load_targets
import re


_arm_action_client = None
_gripper_action_client = None
_arm_joints = None
_gripper_joints = None
_nominal_fruit_poses = {}
_tf_buffer = None
_tf_listener = None


def robot_id():
    return str(rospy.get_param("~robot_id", "robot")).strip("/")


def robot_namespace():
    return "/" + robot_id()


def controller_resource(controller, suffix):
    return "%s/%s/pos_traj_controller/%s" % (
        robot_namespace(), controller, suffix
    )


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def get_model_xyz(name):
    rospy.wait_for_service("/gazebo/get_model_state")
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    res = get_state(name, "world")
    if not res.success:
        raise RuntimeError("Could not get model state: %s" % name)
    p = res.pose.position
    return p.x, p.y, p.z


def set_model_xyz(name, x, y, z):
    rospy.wait_for_service("/gazebo/set_model_state")
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    msg = ModelState()
    msg.model_name = name
    msg.reference_frame = "world"
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = float(z)
    msg.pose.orientation.w = 1.0

    res = set_state(msg)
    return bool(res.success)


def restore_fruit_to_nominal(model_name):
    """Put a failed/released fruit back at its first validated Gazebo pose."""
    nominal = _nominal_fruit_poses.get(model_name)
    if nominal is None:
        rospy.logerr("FRUIT RESTORE unavailable model=%s", model_name)
        return False
    ok = set_model_xyz(model_name, *nominal)
    rospy.logwarn(
        "FRUIT RESTORE model=%s xyz=(%.3f, %.3f, %.3f) success=%s",
        model_name, nominal[0], nominal[1], nominal[2], ok
    )
    return ok


def set_fruit_attachment(model_name, attach=True):
    service_name = "/link_attacher_node/attach" if attach else "/link_attacher_node/detach"
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        service = rospy.ServiceProxy(service_name, Attach)

        req = AttachRequest()
        req.model_name_1 = str(rospy.get_param(
            "~grasp_robot_model", robot_id()
        ))
        req.link_name_1 = str(rospy.get_param(
            "~grasp_robot_link", "%s_link7" % robot_id()
        ))
        req.model_name_2 = model_name
        req.link_name_2 = str(rospy.get_param("~fruit_link", "body"))

        response = service(req)
        action = "ATTACH" if attach else "DETACH"
        if response.ok:
            rospy.loginfo(
                "%s fruit %s::%s %s robot %s::%s",
                action, req.model_name_2, req.link_name_2,
                "to" if attach else "from",
                req.model_name_1, req.link_name_1
            )
            return True

        rospy.logerr("%s failed for fruit model=%s", action, model_name)
        return False
    except Exception as exc:
        rospy.logerr("Fruit attachment service failed model=%s attach=%s: %s", model_name, attach, exc)
        return False


def enable_fruit_gravity(model_name):
    link_name = "%s::%s" % (
        model_name,
        str(rospy.get_param("~fruit_link", "body"))
    )
    try:
        rospy.wait_for_service("/gazebo/get_link_properties", timeout=5.0)
        rospy.wait_for_service("/gazebo/set_link_properties", timeout=5.0)
        get_properties = rospy.ServiceProxy(
            "/gazebo/get_link_properties", GetLinkProperties
        )
        set_properties = rospy.ServiceProxy(
            "/gazebo/set_link_properties", SetLinkProperties
        )

        current = get_properties(link_name)
        if not current.success:
            rospy.logerr(
                "Cannot read fruit link properties %s: %s",
                link_name, current.status_message
            )
            return False

        result = set_properties(
            link_name=link_name,
            com=current.com,
            gravity_mode=True,
            mass=current.mass,
            ixx=current.ixx,
            ixy=current.ixy,
            ixz=current.ixz,
            iyy=current.iyy,
            iyz=current.iyz,
            izz=current.izz,
        )
        if not result.success:
            rospy.logerr(
                "Cannot enable fruit gravity %s: %s",
                link_name, result.status_message
            )
            return False

        rospy.loginfo("ENABLED GRAVITY fruit=%s", link_name)
        return True
    except Exception as exc:
        rospy.logerr("Fruit gravity service failed link=%s: %s", link_name, exc)
        return False


def gazebo_physical_side(side):
    """
    Map logical PROST side to physical Gazebo fruit side.
    Default: no flip. The target CSV side is treated as the physical Gazebo side.
    """
    flip = bool(rospy.get_param("~flip_gazebo_fruit_side", False))
    if not flip:
        return side

    if side == "L":
        return "R"
    if side == "R":
        return "L"
    return side


def pose_from_fruit_frame(fruit_x, fruit_y, yaw, target_left, target_forward):
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)

    left_x = -math.sin(yaw)
    left_y = math.cos(yaw)

    base_x = fruit_x - target_forward * forward_x - target_left * left_x
    base_y = fruit_y - target_forward * forward_y - target_left * left_y
    return base_x, base_y


def publish_stop(pub):
    msg = Twist()
    for _ in range(5):
        pub.publish(msg)
        rospy.sleep(0.05)


def move_to_point_keep_yaw(
    picker,
    pub,
    target_x,
    target_y,
    desired_yaw,
    xy_tolerance=0.055,
    yaw_tolerance=0.18,
    timeout=22.0,
    k_xy=0.55,
    k_yaw=1.20,
    max_vx=0.12,
    max_vy=0.12,
    max_w=0.20,
    peer_clearance_fn=None,
):
    # Peer check on every tick (same contract as go_to_point); the base can move
    # up to ~2.25 m here. Time spent holding for the peer is excluded from the timeout.
    rate = rospy.Rate(20)
    start = time.time()

    while not rospy.is_shutdown():
        if peer_clearance_fn is not None:
            wait_started = time.time()
            peer_clearance_fn()
            start += time.time() - wait_started

        dx = target_x - picker.x
        dy = target_y - picker.y
        dist = math.sqrt(dx * dx + dy * dy)

        yaw_err = norm_angle(desired_yaw - picker.yaw)

        if dist <= xy_tolerance and abs(yaw_err) <= yaw_tolerance:
            rospy.loginfo(
                "KEEP_YAW reached dist=%.3f yaw_err=%.3f current=(%.3f, %.3f, %.3f)",
                dist, yaw_err, picker.x, picker.y, picker.yaw
            )
            publish_stop(pub)
            return True

        if time.time() - start > timeout:
            rospy.logerr(
                "KEEP_YAW timeout dist=%.3f yaw_err=%.3f current=(%.3f, %.3f, %.3f)",
                dist, yaw_err, picker.x, picker.y, picker.yaw
            )
            publish_stop(pub)
            return False

        cy = math.cos(picker.yaw)
        sy = math.sin(picker.yaw)

        ex_body = cy * dx + sy * dy
        ey_body = -sy * dx + cy * dy

        msg = Twist()
        msg.linear.x = clamp(k_xy * ex_body, -max_vx, max_vx)
        msg.linear.y = clamp(k_xy * ey_body, -max_vy, max_vy)

        if abs(yaw_err) > yaw_tolerance:
            msg.angular.z = clamp(k_yaw * yaw_err, -max_w, max_w)
        else:
            msg.angular.z = clamp(0.35 * yaw_err, -0.06, 0.06)

        pub.publish(msg)
        rate.sleep()


def move_base_to_pick_pose(picker, target, fruit_part="B", peer_clearance_fn=None):
    """
    Smart base positioning for physical pick.

    Does not use picker.target_to_row_index_side().
    It gets the Gazebo model name from expected_fruit_pose_for_target(),
    then parses model names like orange1.2.R.B.
    """
    if hasattr(picker, "ensure_robot_level") and not picker.ensure_robot_level(
        "before_pick_approach_%s" % target
    ):
        rospy.logerr(
            "SMART_PICK SAFETY STOP target=%s: robot is not level",
            target
        )
        return False

    model_name, fruit_x, fruit_y, fruit_z = expected_fruit_pose_for_target(
        target, fruit_part, picker=picker
    )

    m = re.match(r"orange(\d+)\.(\d+)\.([LR])\.([BT])", model_name)
    if not m:
        rospy.logwarn("SMART_PICK could not parse model_name=%s, using side=R fallback", model_name)
        row = 0
        index = 0
        side = "R"
    else:
        row = int(m.group(1))
        index = int(m.group(2))
        side = m.group(3)

    target_left = float(rospy.get_param("~target_left", 0.55))
    target_forward_final = float(rospy.get_param("~target_forward_final", 0.05))
    entry_back_distance = float(rospy.get_param("~entry_back_distance", 0.50))

    prefer_side_weight = float(rospy.get_param("~smart_prefer_side_weight", 0.35))
    yaw_weight = float(rospy.get_param("~smart_yaw_weight", 0.45))
    skip_entry_distance = float(rospy.get_param("~skip_entry_distance", 0.35))

    if side == "L":
        preferred_yaw = -math.pi / 2.0
    elif side == "R":
        preferred_yaw = math.pi / 2.0
    else:
        preferred_yaw = picker.path_yaw_for_pick(row=row, index=index, side=side)

    opposite_yaw = norm_angle(preferred_yaw + math.pi)

    # Crossing to the opposite side may produce a slightly shorter numerical
    # path or avoid a 180-degree turn, but in an orchard it sends the base
    # through the tree row.  Keep that candidate disabled unless explicitly
    # requested for a special test environment.
    candidate_specs = [("preferred", preferred_yaw, 0.0)]
    if bool(rospy.get_param("~allow_opposite_pick_side", False)):
        candidate_specs.append(("opposite", opposite_yaw, prefer_side_weight))

    candidates = []

    for label, yaw, side_penalty in candidate_specs:
        final_x, final_y = pose_from_fruit_frame(
            fruit_x, fruit_y, yaw, target_left, target_forward_final
        )

        entry_x, entry_y = pose_from_fruit_frame(
            fruit_x, fruit_y, yaw, target_left, target_forward_final - entry_back_distance
        )

        dist_final = math.hypot(final_x - picker.x, final_y - picker.y)
        yaw_err = abs(norm_angle(yaw - picker.yaw))
        cost = dist_final + yaw_weight * yaw_err + side_penalty

        candidates.append({
            "label": label,
            "yaw": yaw,
            "final_x": final_x,
            "final_y": final_y,
            "entry_x": entry_x,
            "entry_y": entry_y,
            "dist_final": dist_final,
            "yaw_err": yaw_err,
            "cost": cost,
        })

    best = min(candidates, key=lambda c: c["cost"])

    max_base_approach = float(rospy.get_param(
        "~smart_pick_max_base_approach_distance", 2.25
    ))
    if best["dist_final"] > max_base_approach:
        rospy.logerr(
            "SMART_PICK SAFETY STOP target=%s dist=%.3f limit=%.3f; refusing cross-orchard approach",
            target, best["dist_final"], max_base_approach
        )
        return False

    rospy.loginfo(
        "SMART_PICK_POSE target=%s model=%s row=%s index=%s side=%s robot=(%.3f, %.3f, %.3f) fruit=(%.3f, %.3f, %.3f)",
        target, model_name, row, index, side,
        picker.x, picker.y, picker.yaw,
        fruit_x, fruit_y, fruit_z
    )

    for c in candidates:
        rospy.loginfo(
            "SMART_PICK_CANDIDATE %s yaw=%.3f entry=(%.3f, %.3f) final=(%.3f, %.3f) dist=%.3f yaw_err=%.3f cost=%.3f",
            c["label"], c["yaw"],
            c["entry_x"], c["entry_y"],
            c["final_x"], c["final_y"],
            c["dist_final"], c["yaw_err"], c["cost"]
        )

    rospy.loginfo(
        "SMART_PICK_SELECTED target=%s label=%s entry=(%.3f, %.3f) final=(%.3f, %.3f) yaw=%.3f dist=%.3f yaw_err=%.3f",
        target, best["label"],
        best["entry_x"], best["entry_y"],
        best["final_x"], best["final_y"],
        best["yaw"], best["dist_final"], best["yaw_err"]
    )

    use_holonomic_entry = bool(rospy.get_param("~smart_holonomic_entry", True))
    cmd_pub = getattr(picker, "cmd_pub", None)
    if cmd_pub is None:
        cmd_pub = rospy.Publisher(
            robot_namespace() + "/robotnik_base_control/cmd_vel",
            Twist,
            queue_size=1
        )

    if best["dist_final"] > skip_entry_distance:
        if use_holonomic_entry:
            if not move_to_point_keep_yaw(
                picker,
                cmd_pub,
                target_x=best["entry_x"],
                target_y=best["entry_y"],
                desired_yaw=best["yaw"],
                xy_tolerance=picker.pick_approach_tolerance,
                yaw_tolerance=float(rospy.get_param(
                    "~smart_entry_yaw_tolerance", 0.15
                )),
                timeout=float(rospy.get_param("~smart_entry_timeout", 25.0)),
                max_vx=float(rospy.get_param("~smart_entry_max_vx", 0.18)),
                max_vy=float(rospy.get_param("~smart_entry_max_vy", 0.18)),
                max_w=float(rospy.get_param("~smart_entry_max_w", 0.30)),
                peer_clearance_fn=peer_clearance_fn,
            ):
                rospy.logerr("Failed holonomic smart entry for target=%s", target)
                return False
        elif not picker.go_to_point(
            best["entry_x"], best["entry_y"], picker.pick_approach_tolerance,
            peer_clearance_fn=peer_clearance_fn,
        ):
            rospy.logerr("Failed to reach smart entry pose for target=%s", target)
            return False
    else:
        rospy.loginfo(
            "SMART_PICK_SKIP_ENTRY target=%s dist_to_final=%.3f threshold=%.3f",
            target, best["dist_final"], skip_entry_distance
        )

    yaw_now_err = abs(norm_angle(best["yaw"] - picker.yaw))
    yaw_skip_tol = float(rospy.get_param("~smart_yaw_skip_tolerance", 0.22))

    if not use_holonomic_entry and yaw_now_err > yaw_skip_tol:
        if not picker.turn_to_yaw(best["yaw"], peer_clearance_fn=peer_clearance_fn):
            rospy.logerr("Failed to reach smart pick yaw for target=%s", target)
            return False
    else:
        rospy.loginfo(
            "SMART_PICK_SKIP_TURN target=%s yaw_err=%.3f threshold=%.3f",
            target, yaw_now_err, yaw_skip_tol
        )

    return move_to_point_keep_yaw(
        picker,
        cmd_pub,
        target_x=best["final_x"],
        target_y=best["final_y"],
        desired_yaw=best["yaw"],
        xy_tolerance=float(rospy.get_param("~final_xy_tolerance", 0.055)),
        yaw_tolerance=float(rospy.get_param("~final_yaw_tolerance", 0.18)),
        max_vx=float(rospy.get_param("~smart_final_max_vx", 0.18)),
        max_vy=float(rospy.get_param("~smart_final_max_vy", 0.18)),
        max_w=float(rospy.get_param("~smart_final_max_w", 0.24)),
        peer_clearance_fn=peer_clearance_fn,
    )

def command_gripper(position, duration=1.0, allow_contact=False):
    global _gripper_action_client, _gripper_joints
    if _gripper_joints is None:
        msg = rospy.wait_for_message(
            controller_resource("gripper", "state"),
            JointTrajectoryControllerState,
            timeout=8.0
        )
        _gripper_joints = list(msg.joint_names)
    joints = _gripper_joints

    rospy.loginfo("GRIPPER position=%.3f duration=%.2f joints=%s", position, duration, joints)

    if _gripper_action_client is None:
        _gripper_action_client = actionlib.SimpleActionClient(
            controller_resource("gripper", "follow_joint_trajectory"),
            FollowJointTrajectoryAction
        )
        if not _gripper_action_client.wait_for_server(rospy.Duration(8.0)):
            rospy.logerr("Gripper action server not available")
            _gripper_action_client = None
            return False
    client = _gripper_action_client

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joints
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)

    pt = JointTrajectoryPoint()
    pt.positions = [float(position)] * len(joints)
    pt.velocities = [0.0] * len(joints)
    pt.time_from_start = rospy.Duration(float(duration))

    goal.trajectory.points = [pt]

    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(float(duration) + 5.0)):
        client.cancel_goal()
        rospy.logerr("Gripper trajectory timed out")
        return False
    state = client.get_state()
    if state != GoalStatus.SUCCEEDED:
        if allow_contact:
            try:
                feedback = rospy.wait_for_message(
                    controller_resource("gripper", "state"),
                    JointTrajectoryControllerState,
                    timeout=2.0
                )
                actual = list(feedback.actual.positions)
                desired = list(feedback.desired.positions)
                error = list(feedback.error.positions)
                contact_margin = float(rospy.get_param("~gripper_contact_margin", 0.002))
                max_open = float(rospy.get_param("~gripper_max_open", 0.040))
                goal_tolerance = float(rospy.get_param(
                    "~gripper_reached_goal_tolerance", 0.0015
                ))
                contact = bool(actual) and all(
                    float(position) + contact_margin < p < max_open
                    for p in actual
                )
                reached_goal = bool(actual) and all(
                    abs(p - float(position)) <= goal_tolerance
                    for p in actual
                )
                accepted = contact or reached_goal
                rospy.loginfo(
                    "GRIPPER CONTACT CHECK target=%.4f actual=%s desired=%s "
                    "error=%s contact=%s reached_goal=%s accepted=%s "
                    "contact_range=(%.4f, %.4f) goal_tolerance=%.4f state=%d",
                    position,
                    actual,
                    desired,
                    error,
                    contact,
                    reached_goal,
                    accepted,
                    float(position) + contact_margin,
                    max_open,
                    goal_tolerance,
                    state
                )
                if accepted:
                    rospy.loginfo(
                        "Gripper close accepted after controller abort "
                        "target=%.3f actual=%s contact=%s reached_goal=%s state=%d",
                        position, actual, contact, reached_goal, state
                    )
                    return True
            except Exception as exc:
                rospy.logwarn("Could not verify gripper contact: %s", exc)
        rospy.logerr(
            "Gripper trajectory failed state=%d status=%s",
            state,
            client.get_goal_status_text()
        )
        return False
    return True


def get_arm_joints_from_controller():
    global _arm_joints
    if _arm_joints is None:
        msg = rospy.wait_for_message(
            controller_resource("arm", "state"),
            JointTrajectoryControllerState,
            timeout=8.0
        )
        _arm_joints = list(msg.joint_names)
    return _arm_joints


def send_arm_joint_pose(positions, duration=3.0, label="arm_pose"):
    global _arm_action_client
    joints = get_arm_joints_from_controller()

    if len(joints) != len(positions):
        rospy.logerr("%s joint length mismatch joints=%d positions=%d", label, len(joints), len(positions))
        return False

    rospy.loginfo("ARM JOINT %s duration=%.2f positions=%s", label, duration, positions)

    if _arm_action_client is None:
        _arm_action_client = actionlib.SimpleActionClient(
            controller_resource("arm", "follow_joint_trajectory"),
            FollowJointTrajectoryAction
        )
        if not _arm_action_client.wait_for_server(rospy.Duration(8.0)):
            rospy.logerr("Arm action server not available")
            _arm_action_client = None
            return False
    client = _arm_action_client

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = joints
    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)

    pt = JointTrajectoryPoint()
    pt.positions = [float(x) for x in positions]
    pt.velocities = [0.0] * len(positions)
    pt.time_from_start = rospy.Duration(float(duration))

    goal.trajectory.points = [pt]

    client.send_goal(goal)
    if not client.wait_for_result(rospy.Duration(float(duration) + 6.0)):
        client.cancel_goal()
        rospy.logerr("Arm trajectory timed out: %s", label)
        return False
    state = client.get_state()
    if state != GoalStatus.SUCCEEDED:
        rospy.logerr(
            "Arm trajectory failed label=%s state=%d status=%s",
            label,
            state,
            client.get_goal_status_text()
        )
        return False
    return True


def import_moveit():
    try:
        import moveit_commander
        return moveit_commander
    except Exception as e:
        rospy.logerr("moveit_commander import failed: %s", e)
        return None


def transform_world_to_frame(x, y, z, target_frame):
    if target_frame in ["world", "/world"]:
        return x, y, z

    # One TF buffer/listener for the whole process instead of a new one
    # (plus a 1 s sleep) on every call.
    global _tf_buffer, _tf_listener
    first_init = _tf_buffer is None
    if first_init:
        _tf_buffer = tf2_ros.Buffer()
        _tf_listener = tf2_ros.TransformListener(_tf_buffer)
        rospy.sleep(1.0)

    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = rospy.Time(0)
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0

    trans = _tf_buffer.lookup_transform(
        target_frame,
        "world",
        rospy.Time(0),
        rospy.Duration(3.0)
    )

    out = tf2_geometry_msgs.do_transform_pose(pose, trans)
    return out.pose.position.x, out.pose.position.y, out.pose.position.z


def make_moveit_group(moveit_commander):
    ns = str(rospy.get_param("~moveit_namespace", robot_namespace()))
    robot_description = str(rospy.get_param(
        "~robot_description", "%s/robot_description" % ns.rstrip("/")
    ))

    robot = moveit_commander.RobotCommander(
        robot_description=robot_description,
        ns=ns
    )

    groups = robot.get_group_names()
    rospy.loginfo("MoveIt groups: %s", groups)

    group_name = "arm" if "arm" in groups else groups[0]

    group = moveit_commander.MoveGroupCommander(
        group_name,
        robot_description=robot_description,
        ns=ns
    )

    velocity_scaling = float(rospy.get_param("~moveit_velocity_scaling", 0.30))
    acceleration_scaling = float(rospy.get_param("~moveit_acceleration_scaling", 0.15))
    group.set_max_velocity_scaling_factor(velocity_scaling)
    group.set_max_acceleration_scaling_factor(acceleration_scaling)
    planner_id = str(rospy.get_param("~moveit_planner_id", "")).strip()
    if planner_id:
        group.set_planner_id(planner_id)
    planning_time = float(rospy.get_param("~moveit_planning_time", 6.0))
    planning_attempts = int(rospy.get_param("~moveit_planning_attempts", 6))
    group.set_planning_time(planning_time)
    group.set_num_planning_attempts(planning_attempts)
    rospy.loginfo(
        "MoveIt settings planner=%s velocity=%.2f acceleration=%.2f "
        "planning_time=%.2f attempts=%d",
        planner_id or "default", velocity_scaling, acceleration_scaling,
        planning_time, planning_attempts
    )

    return group


def move_to_vertical_pregrasp(group, x, y, z):
    ee = group.get_end_effector_link()
    frame = group.get_planning_frame()

    tx, ty, tz = transform_world_to_frame(x, y, z, frame)

    roll = float(rospy.get_param("~vertical_roll", 3.14159))
    pitch = float(rospy.get_param("~vertical_pitch", 0.0))
    yaw = float(rospy.get_param("~vertical_yaw", 0.0))

    q = quaternion_from_euler(roll, pitch, yaw)

    target = PoseStamped()
    target.header.frame_id = frame
    target.header.stamp = rospy.Time.now()
    target.pose.position.x = tx
    target.pose.position.y = ty
    target.pose.position.z = tz
    target.pose.orientation.x = q[0]
    target.pose.orientation.y = q[1]
    target.pose.orientation.z = q[2]
    target.pose.orientation.w = q[3]

    rospy.loginfo("MOVEIT pregrasp frame=%s target=(%.3f, %.3f, %.3f)", frame, tx, ty, tz)

    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    group.set_pose_target(target, ee)

    standard_planning_time = float(rospy.get_param(
        "~moveit_planning_time", 6.0
    ))
    standard_attempts = int(rospy.get_param(
        "~moveit_planning_attempts", 6
    ))
    fast_enabled = bool(rospy.get_param(
        "~adaptive_pregrasp_planning", True
    ))
    fast_planning_time = float(rospy.get_param(
        "~pregrasp_fast_planning_time", 0.60
    ))
    fast_attempts = int(rospy.get_param(
        "~pregrasp_fast_planning_attempts", 1
    ))

    if fast_enabled:
        group.set_planning_time(fast_planning_time)
        group.set_num_planning_attempts(fast_attempts)
        rospy.loginfo(
            "PREGRASP FAST attempt planning_time=%.2fs attempts=%d",
            fast_planning_time, fast_attempts
        )

    planning_started = time.monotonic()
    planned = group.plan()
    planning_wall = time.monotonic() - planning_started

    # MoveIt Commander in ROS Noetic returns
    # (success, RobotTrajectory, planning_time, MoveItErrorCodes).  Keep
    # compatibility with older versions that returned RobotTrajectory only.
    reported_planning = None
    if isinstance(planned, tuple):
        plan_ok = bool(planned[0])
        plan = planned[1]
        if len(planned) > 2:
            try:
                reported_planning = float(planned[2])
            except (TypeError, ValueError):
                reported_planning = None
    else:
        plan = planned
        trajectory = getattr(plan, "joint_trajectory", None)
        plan_ok = bool(trajectory and trajectory.points)

    point_count = len(getattr(
        getattr(plan, "joint_trajectory", None), "points", []
    ))

    if fast_enabled and not plan_ok:
        rospy.logwarn(
            "PREGRASP FAST failed wall=%.3fs; retrying standard "
            "planning_time=%.2fs attempts=%d",
            planning_wall, standard_planning_time, standard_attempts
        )
        group.set_planning_time(standard_planning_time)
        group.set_num_planning_attempts(standard_attempts)
        group.set_start_state_to_current_state()
        group.set_pose_target(target, ee)

        planning_started = time.monotonic()
        planned = group.plan()
        planning_wall = time.monotonic() - planning_started
        reported_planning = None
        if isinstance(planned, tuple):
            plan_ok = bool(planned[0])
            plan = planned[1]
            if len(planned) > 2:
                try:
                    reported_planning = float(planned[2])
                except (TypeError, ValueError):
                    reported_planning = None
        else:
            plan = planned
            trajectory = getattr(plan, "joint_trajectory", None)
            plan_ok = bool(trajectory and trajectory.points)
        point_count = len(getattr(
            getattr(plan, "joint_trajectory", None), "points", []
        ))
        rospy.loginfo(
            "PREGRASP STANDARD fallback success=%s wall=%.3fs "
            "reported=%s points=%d",
            plan_ok,
            planning_wall,
            "%.3fs" % reported_planning
            if reported_planning is not None else "unknown",
            point_count
        )

    # Never leak the temporary fast budget into another MoveIt operation.
    group.set_planning_time(standard_planning_time)
    group.set_num_planning_attempts(standard_attempts)
    rospy.loginfo(
        "PREGRASP PLAN success=%s wall=%.3fs reported=%s points=%d",
        plan_ok,
        planning_wall,
        "%.3fs" % reported_planning
        if reported_planning is not None else "unknown",
        point_count
    )

    ok = False
    if plan_ok:
        execution_started = time.monotonic()
        ok = group.execute(plan, wait=True)
        execution_wall = time.monotonic() - execution_started
        rospy.loginfo(
            "PREGRASP EXECUTE success=%s wall=%.3fs points=%d",
            bool(ok), execution_wall, point_count
        )
    group.stop()
    group.clear_pose_targets()
    return bool(ok)


def cartesian_vertical_descent(group, final_z_planning, step=0.015):
    ee = group.get_end_effector_link()
    start_pose = group.get_current_pose(ee).pose

    waypoints = []
    z = start_pose.position.z

    while z - step > final_z_planning:
        z -= step
        p = copy.deepcopy(start_pose)
        p.position.z = z
        waypoints.append(p)

    p = copy.deepcopy(start_pose)
    p.position.z = final_z_planning
    waypoints.append(p)

    rospy.loginfo("DESCENT current_z=%.3f final_z=%.3f waypoints=%d",
                  start_pose.position.z, final_z_planning, len(waypoints))

    try:
        plan, fraction = group.compute_cartesian_path(waypoints, 0.01, True)
    except Exception:
        plan, fraction = group.compute_cartesian_path(waypoints, 0.01, False)

    rospy.loginfo("DESCENT fraction=%.3f", fraction)

    if fraction < 0.75:
        return False

    ok = group.execute(plan, wait=True)
    group.stop()
    if not ok:
        actual_z = group.get_current_pose(ee).pose.position.z
        tolerance = float(rospy.get_param("~cartesian_goal_acceptance", 0.012))
        if abs(actual_z - final_z_planning) <= tolerance:
            rospy.logwarn(
                "DESCENT controller aborted inside tolerance actual_z=%.4f target_z=%.4f tolerance=%.4f; accepting",
                actual_z, final_z_planning, tolerance
            )
            ok = True
    return bool(ok)


def lift_current_pose_cartesian(group, lift_height=0.12):
    ee = group.get_end_effector_link()
    start_pose = group.get_current_pose(ee).pose

    p = copy.deepcopy(start_pose)
    p.position.z += float(lift_height)

    try:
        plan, fraction = group.compute_cartesian_path([p], 0.01, True)
    except Exception:
        plan, fraction = group.compute_cartesian_path([p], 0.01, False)

    rospy.loginfo("LIFT fraction=%.3f", fraction)

    if fraction < 0.75:
        return False

    target_z = p.position.z
    ok = group.execute(plan, wait=True)
    group.stop()
    if not ok:
        actual_z = group.get_current_pose(ee).pose.position.z
        tolerance = float(rospy.get_param("~cartesian_goal_acceptance", 0.012))
        if abs(actual_z - target_z) <= tolerance:
            rospy.logwarn(
                "LIFT controller aborted inside tolerance actual_z=%.4f target_z=%.4f tolerance=%.4f; accepting",
                actual_z, target_z, tolerance
            )
            ok = True
    return bool(ok)


def reset_arm_and_gripper():
    gripper_ok = command_gripper(
        float(rospy.get_param("~reset_gripper_open", 0.039)),
        duration=float(rospy.get_param("~reset_gripper_duration", 1.2))
    )
    neutral_pose = rospy.get_param("~neutral_pose", [0.0, 0.0, 0.0, -0.5, 0.0, 0.85, 0.0])
    arm_ok = send_arm_joint_pose(
        neutral_pose,
        duration=float(rospy.get_param("~reset_arm_duration", 2.5)),
        label="neutral_reset"
    )
    return gripper_ok and arm_ok


def pick_arm_and_store(target, fruit_part="T"):
    targets = load_targets()
    meta = targets[target]

    row = int(meta["row"])
    index = int(meta["index"])
    side = str(meta["side"])

    model_name = "orange%d.%d.%s.%s" % (row, index, side, fruit_part)
    fruit_x, fruit_y, fruit_z = get_model_xyz(model_name)

    pre_above = float(rospy.get_param("~pre_above", 0.160))
    # Aim at the fruit centre.  A positive offset makes the fingers contact
    # the upper hemisphere and squeeze the spherical fruit downwards.
    grip_z_offset = float(rospy.get_param("~grip_z_offset", 0.000))

    grip_forward_offset = float(rospy.get_param("~grip_forward_offset", 0.000))
    grip_left_offset = float(rospy.get_param("~grip_left_offset", 0.000))

    picker = OrchardMapPicker()
    picker.wait_for_odom()

    yaw = picker.yaw
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)
    left_x = -math.sin(yaw)
    left_y = math.cos(yaw)

    target_x = fruit_x + grip_forward_offset * forward_x + grip_left_offset * left_x
    target_y = fruit_y + grip_forward_offset * forward_y + grip_left_offset * left_y

    pre_z = fruit_z + pre_above
    grip_z = fruit_z + grip_z_offset

    rospy.loginfo("ARM PICK target=%s model=%s fruit=(%.3f,%.3f,%.3f) target_xy=(%.3f,%.3f)",
                  target, model_name, fruit_x, fruit_y, fruit_z, target_x, target_y)

    moveit_commander = import_moveit()
    if moveit_commander is None:
        return False

    moveit_commander.roscpp_initialize(sys.argv)
    group = make_moveit_group(moveit_commander)

    open_pos = float(rospy.get_param("~open_gripper_pos", 0.039))
    close_pos = float(rospy.get_param("~close_gripper_pos", 0.000))

    if not command_gripper(open_pos, duration=float(rospy.get_param("~open_gripper_duration", 1.5))):
        return False

    if not move_to_vertical_pregrasp(group, target_x, target_y, pre_z):
        return False

    frame = group.get_planning_frame()
    _, _, grip_z_planning = transform_world_to_frame(target_x, target_y, grip_z, frame)

    if not cartesian_vertical_descent(group, grip_z_planning, step=0.015):
        return False

    if not command_gripper(
        close_pos,
        duration=float(rospy.get_param("~close_gripper_duration", 3.0)),
        allow_contact=True
    ):
        return False

    if not lift_current_pose_cartesian(group, lift_height=float(rospy.get_param("~lift_after_grip", 0.12))):
        return False

    basket_pose = rospy.get_param("~basket_pose", [0.0, -0.10, 0.0, -0.45, 0.0, 0.95, 0.0])
    neutral_pose = rospy.get_param("~neutral_pose", [0.0, 0.0, 0.0, -0.5, 0.0, 0.85, 0.0])

    send_arm_joint_pose(basket_pose, duration=float(rospy.get_param("~basket_duration", 2.5)), label="basket_pose")
    if not command_gripper(open_pos, duration=float(rospy.get_param("~open_gripper_duration", 1.5))):
        return False
    send_arm_joint_pose(neutral_pose, duration=float(rospy.get_param("~neutral_duration", 2.5)), label="neutral_pose")

    return True


def pick_and_store_target(target="T1", reset_arm=True, align_oranges=False):
    picker = OrchardMapPicker()
    picker.wait_for_odom()

    if reset_arm:
        reset_arm_and_gripper()

    if align_oranges:


        rospy.loginfo("SKIP align_target_oranges: oranges are fixed in greenhouse.world")

    if not move_base_to_pick_pose(picker, target, fruit_part="B"):
        rospy.logerr("Base approach failed for %s", target)
        return False

    if not pick_arm_and_store(target, fruit_part="T"):
        rospy.logerr("Arm pick/store failed for %s", target)
        return False

    rospy.loginfo("SUCCESS pick_and_store_target(%s)", target)
    return True


def pick_and_store_target_both_fruits(target="T1", reset_arm=True, align_oranges=False):
    """
    Pick both fruits at one target location:
    - upper fruit: .T
    - lower fruit: .B

    Base approach is done once using lower fruit position.
    Arm then picks both fruits and drops each into basket.

    This is the function we want to connect to PROST pick(target).
    """
    picker = OrchardMapPicker()
    picker.wait_for_odom()

    rospy.loginfo("============================================================")
    rospy.loginfo("PICK BOTH FRUITS target=%s", target)
    rospy.loginfo("============================================================")

    if reset_arm:
        reset_arm_and_gripper()

    if align_oranges:


        rospy.loginfo("SKIP align_target_oranges: oranges are fixed in greenhouse.world")

    if not move_base_to_pick_pose(picker, target, fruit_part="B"):
        rospy.logerr("Base approach failed for %s", target)
        return False

    # Pick upper first so the arm has more clearance.
    order = rospy.get_param("~fruit_pick_order", ["T", "B"])

    ok_all = True

    for fruit_part in order:
        rospy.loginfo("------------------------------------------------------------")
        rospy.loginfo("PICK target=%s fruit_part=%s", target, fruit_part)
        rospy.loginfo("------------------------------------------------------------")

        ok = pick_arm_and_store(target, fruit_part=fruit_part)

        if not ok:
            rospy.logerr("Failed pick/store target=%s fruit_part=%s", target, fruit_part)
            ok_all = False

            if bool(rospy.get_param("~stop_on_first_failed_fruit", False)):
                return False

    if ok_all:
        rospy.loginfo("SUCCESS pick_and_store_target_both_fruits(%s)", target)
    else:
        rospy.logwarn("PARTIAL SUCCESS pick_and_store_target_both_fruits(%s)", target)

    return ok_all


def pick_arm_and_store_expected_pose(
    target, fruit_part="T", picker=None, moveit_group=None,
    open_before_pick=True, return_to_neutral=True
):
    """
    Execute pick motion using expected calibrated pose instead of current Gazebo pose.
    Useful for the second fruit, because it may have moved/fallen after the first pick.
    """
    model_name, fruit_x, fruit_y, fruit_z = expected_fruit_pose_for_target(
        target, fruit_part, picker=picker
    )

    pre_above = float(rospy.get_param("~pre_above", 0.160))
    # Same centre-grasp offset as pick_arm_and_store().
    grip_z_offset = float(rospy.get_param("~grip_z_offset", 0.000))

    grip_forward_offset = float(rospy.get_param("~grip_forward_offset", 0.000))
    grip_left_offset = float(rospy.get_param("~grip_left_offset", 0.000))

    if picker is None:
        picker = OrchardMapPicker()
        picker.wait_for_odom()

    yaw = picker.yaw
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)
    left_x = -math.sin(yaw)
    left_y = math.cos(yaw)

    target_x = fruit_x + grip_forward_offset * forward_x + grip_left_offset * left_x
    target_y = fruit_y + grip_forward_offset * forward_y + grip_left_offset * left_y

    pre_z = fruit_z + pre_above
    grip_z = fruit_z + grip_z_offset

    rospy.loginfo(
        "ARM PICK EXPECTED target=%s fruit_part=%s model=%s expected_fruit=(%.3f,%.3f,%.3f) "
        "grasp_target=(%.3f,%.3f,%.3f) grip_z_offset=%.3f",
        target, fruit_part, model_name, fruit_x, fruit_y, fruit_z,
        target_x, target_y, grip_z, grip_z_offset
    )

    group = moveit_group
    if group is None:
        moveit_commander = import_moveit()
        if moveit_commander is None:
            return False

        moveit_commander.roscpp_initialize(sys.argv)
        group = make_moveit_group(moveit_commander)

    open_pos = float(rospy.get_param("~open_gripper_pos", 0.039))
    close_pos = float(rospy.get_param("~close_gripper_pos", 0.000))

    if open_before_pick:
        if not command_gripper(
            open_pos,
            duration=float(rospy.get_param("~open_gripper_duration", 1.2))
        ):
            rospy.logerr("Expected-pose gripper open failed target=%s fruit_part=%s", target, fruit_part)
            return False
    else:
        rospy.loginfo(
            "INTERFRUIT SKIP gripper already open target=%s fruit_part=%s",
            target, fruit_part
        )

    if not move_to_vertical_pregrasp(group, target_x, target_y, pre_z):
        rospy.logerr("Expected-pose pregrasp failed target=%s fruit_part=%s", target, fruit_part)
        return False

    frame = group.get_planning_frame()
    _, _, grip_z_planning = transform_world_to_frame(target_x, target_y, grip_z, frame)

    if not cartesian_vertical_descent(group, grip_z_planning, step=0.015):
        rospy.logerr("Expected-pose descent failed target=%s fruit_part=%s", target, fruit_part)
        return False

    use_attacher = bool(rospy.get_param("~use_link_attacher", True))
    attached = False
    if use_attacher:
        attached = set_fruit_attachment(model_name, attach=True)
        if not attached:
            return False

    if not command_gripper(
        close_pos,
        duration=float(rospy.get_param("~close_gripper_duration", 2.5)),
        allow_contact=True
    ):
        if attached:
            set_fruit_attachment(model_name, attach=False)
            restore_fruit_to_nominal(model_name)
        return False

    if not lift_current_pose_cartesian(group, lift_height=float(rospy.get_param("~lift_after_grip", 0.12))):
        if attached:
            set_fruit_attachment(model_name, attach=False)
            restore_fruit_to_nominal(model_name)
        return False

    basket_pose = rospy.get_param("~basket_pose", [0.0, -0.10, 0.0, -0.45, 0.0, 0.95, 0.0])
    neutral_pose = rospy.get_param("~neutral_pose", [0.0, 0.0, 0.0, -0.5, 0.0, 0.85, 0.0])

    if not send_arm_joint_pose(
        basket_pose,
        duration=float(rospy.get_param("~basket_duration", 2.0)),
        label="basket_pose_%s" % fruit_part
    ):
        if attached:
            set_fruit_attachment(model_name, attach=False)
            restore_fruit_to_nominal(model_name)
        return False

    if not command_gripper(open_pos, duration=float(rospy.get_param("~open_gripper_duration", 1.2))):
        if attached:
            set_fruit_attachment(model_name, attach=False)
            restore_fruit_to_nominal(model_name)
        return False

    if attached and not set_fruit_attachment(model_name, attach=False):
        return False

    if attached and not enable_fruit_gravity(model_name):
        return False

    if return_to_neutral:
        if not send_arm_joint_pose(
            neutral_pose,
            duration=float(rospy.get_param("~neutral_duration", 2.0)),
            label="neutral_pose_%s" % fruit_part
        ):
            return False
    else:
        rospy.loginfo(
            "INTERFRUIT SKIP neutral pose target=%s fruit_part=%s",
            target, fruit_part
        )

    rospy.loginfo("DONE expected-pose pick/store target=%s fruit_part=%s", target, fruit_part)
    return True


def pick_location_for_prost_two_motions(
    target="T1", reset_arm=True, align_oranges=False,
    fruit_parts=None, return_details=False, picker=None, moveit_group=None,
    peer_clearance_fn=None,
):
    """
    PROST-facing pick location:
    - approach correct side once
    - perform upper fruit pick motion using expected pose
    - perform lower fruit pick motion using expected pose, even if fruit moved/fell
    - report the outcome of each fruit motion separately
    - return success only when every requested fruit motion completed
    """
    if picker is None:
        picker = OrchardMapPicker()
        picker.wait_for_odom()

    rospy.loginfo("============================================================")
    rospy.loginfo("PICK LOCATION FOR PROST TWO MOTIONS target=%s", target)
    rospy.loginfo("============================================================")

    result = {"base_success": False, "T": False, "B": False}

    if reset_arm and not reset_arm_and_gripper():
        rospy.logerr("Arm/gripper reset failed for %s", target)
        return result if return_details else False

    if align_oranges:


        rospy.loginfo("SKIP align_target_oranges: oranges are fixed in greenhouse.world")

    if not move_base_to_pick_pose(
        picker, target, fruit_part="B", peer_clearance_fn=peer_clearance_fn
    ):
        rospy.logerr("Base approach failed for %s", target)
        return result if return_details else False

    result["base_success"] = True

    order = list(fruit_parts) if fruit_parts is not None else rospy.get_param("~fruit_pick_order", ["T", "B"])

    if moveit_group is None:
        moveit_commander = import_moveit()
        if moveit_commander is None:
            return result if return_details else False
        moveit_commander.roscpp_initialize(sys.argv)
        moveit_group = make_moveit_group(moveit_commander)

    ok_motion_all = True

    optimize_interfruit = bool(rospy.get_param("~optimize_interfruit_motion", True))
    previous_motion_ok = False

    for part_index, fruit_part in enumerate(order):
        rospy.loginfo("------------------------------------------------------------")
        rospy.loginfo("PROST LOCATION MOTION target=%s fruit_part=%s", target, fruit_part)
        rospy.loginfo("------------------------------------------------------------")

        ok = pick_arm_and_store_expected_pose(
            target,
            fruit_part=fruit_part,
            picker=picker,
            moveit_group=moveit_group,
            open_before_pick=not (optimize_interfruit and previous_motion_ok),
            return_to_neutral=not (
                optimize_interfruit and part_index + 1 < len(order)
            )
        )
        result[fruit_part] = bool(ok)
        previous_motion_ok = bool(ok)

        if not ok:
            ok_motion_all = False
            rospy.logerr("Motion failed target=%s fruit_part=%s", target, fruit_part)

            if bool(rospy.get_param("~stop_on_first_failed_fruit", False)):
                break

    rospy.loginfo("SKIP remove_target_oranges: keeping oranges visible for PROST test")

    if ok_motion_all:
        rospy.loginfo("SUCCESS pick_location_for_prost_two_motions(%s)", target)
    else:
        rospy.logwarn("PICK MOTION FAILED target=%s result=%s", target, result)

    final_success = ok_motion_all and all(result.get(part, False) for part in order)
    return result if return_details else final_success


def remove_target_oranges(target):
    rospy.loginfo("SKIP remove_target_oranges(%s): oranges are kept in Gazebo", target)
    return True


# ============================================================
# OVERRIDE: corrected physical fruit positions for PROST/Gazebo
# ============================================================

def corrected_gazebo_fruit_xy(picker, row, index, side):
    """
    Use the logical L/R side, but shrink the lateral offset toward the tree center.
    This avoids placing oranges too far into the corridor / wrong row.

    scale:
      1.00 = original picker.fruit_pose side position
      0.50 = half-way from tree center to side
      0.00 = exactly on row/tree center
    """
    physical_side = gazebo_physical_side(side)

    lx, ly = picker.fruit_pose(row, index, "L")
    rx, ry = picker.fruit_pose(row, index, "R")

    cx = 0.5 * (lx + rx)
    cy = 0.5 * (ly + ry)

    if physical_side == "L":
        sx, sy = lx, ly
    elif physical_side == "R":
        sx, sy = rx, ry
    else:
        sx, sy = picker.fruit_pose(row, index, physical_side)

    scale = float(rospy.get_param("~gazebo_fruit_side_scale", 0.45))

    x = cx + scale * (sx - cx)
    y = cy + scale * (sy - cy)

    rospy.loginfo(
        "CORRECTED_FRUIT_XY row=%s index=%s logical_side=%s physical_side=%s "
        "center=(%.3f, %.3f) original_side=(%.3f, %.3f) scale=%.3f corrected=(%.3f, %.3f)",
        row, index, side, physical_side, cx, cy, sx, sy, scale, x, y
    )

    return x, y


def expected_fruit_pose_for_target(target, fruit_part, picker=None):
    """
    Override old expected_fruit_pose_for_target:
    use corrected Gazebo fruit position, not the full side offset.
    """
    if picker is None:
        picker = OrchardMapPicker()
    targets = load_targets()
    meta = targets[target]

    row = int(meta["row"])
    index = int(meta["index"])
    side = str(meta["side"])

    fx, fy = corrected_gazebo_fruit_xy(picker, row, index, side)

    if fruit_part == "T":
        fz = 1.25
    elif fruit_part == "B":
        fz = 0.85
    else:
        raise ValueError("fruit_part must be T or B, got: %s" % fruit_part)

    model_name = "orange%d.%d.%s.%s" % (row, index, side, fruit_part)

    rospy.loginfo(
        "EXPECTED CORRECTED target=%s fruit_part=%s model=%s xyz=(%.3f, %.3f, %.3f)",
        target, fruit_part, model_name, fx, fy, fz
    )

    return model_name, fx, fy, fz

# ============================================================
# FINAL OVERRIDE FOR PROST PICK:
# Use actual Gazebo model position, not corrected/orchard-map XY.
# This must stay at the end of the file so it overrides previous definitions.
# ============================================================

_old_expected_fruit_pose_for_target = expected_fruit_pose_for_target

def expected_fruit_pose_for_target(target, fruit_part, picker=None):
    """
    Final PROST pick pose source:
    1. Use old function only to get the correct Gazebo model name.
    2. Read the actual model pose from /gazebo/get_model_state.
    3. Return real Gazebo xyz so MoveIt targets the visible orange.
    """
    if picker is None:
        model_name, old_x, old_y, old_z = _old_expected_fruit_pose_for_target(
            target, fruit_part
        )
    else:
        targets = load_targets()
        meta = targets[target]
        row = int(meta["row"])
        index = int(meta["index"])
        side = str(meta["side"])
        model_name = "orange%d.%d.%s.%s" % (row, index, side, fruit_part)
        old_x, old_y = corrected_gazebo_fruit_xy(picker, row, index, side)
        old_z = 1.25 if fruit_part == "T" else 0.85

    try:
        gx, gy, gz = get_model_xyz(model_name)
        actual = (gx, gy, gz)
        nominal = _nominal_fruit_poses.get(model_name)
        if nominal is None:
            min_z = float(rospy.get_param("~fruit_pose_initial_min_z", 0.60))
            max_z = float(rospy.get_param("~fruit_pose_initial_max_z", 1.45))
            if not (min_z <= gz <= max_z):
                raise RuntimeError(
                    "initial Gazebo fruit height out of bounds z=%.3f" % gz
                )
            _nominal_fruit_poses[model_name] = actual
            nominal = actual
            rospy.loginfo(
                "FRUIT NOMINAL REGISTER model=%s xyz=(%.3f, %.3f, %.3f)",
                model_name, gx, gy, gz
            )
        xy_error = math.hypot(gx - nominal[0], gy - nominal[1])
        z_error = abs(gz - nominal[2])
        max_xy = float(rospy.get_param("~fruit_pose_max_xy_deviation", 0.40))
        max_z = float(rospy.get_param("~fruit_pose_max_z_deviation", 0.30))
        if xy_error > max_xy or z_error > max_z:
            rospy.logerr(
                "FRUIT POSE INVALID model=%s actual=(%.3f, %.3f, %.3f) nominal=(%.3f, %.3f, %.3f) xy_error=%.3f z_error=%.3f; refusing chase and restoring",
                model_name, gx, gy, gz, nominal[0], nominal[1], nominal[2],
                xy_error, z_error
            )
            restore_fruit_to_nominal(model_name)
            return model_name, nominal[0], nominal[1], nominal[2]
        rospy.loginfo(
            "EXPECTED GAZEBO MODEL STATE target=%s fruit_part=%s model=%s xyz=(%.3f, %.3f, %.3f) old_expected=(%.3f, %.3f, %.3f)",
            target, fruit_part, model_name, gx, gy, gz, old_x, old_y, old_z
        )
        return model_name, gx, gy, gz
    except Exception as e:
        rospy.logwarn(
            "GAZEBO MODEL STATE FAILED target=%s fruit_part=%s model=%s err=%s ; using old expected xyz=(%.3f, %.3f, %.3f)",
            target, fruit_part, model_name, str(e), old_x, old_y, old_z
        )
        return model_name, old_x, old_y, old_z
