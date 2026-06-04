#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Reset the Gazebo demo scene to the initial robot and object poses."""
import math

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_srvs.srv import Empty


DEFAULT_POSES = {
    "turtlebot3_with_open_manipulator": (-1.35, -1.75, 0.01, 3.14159),
    "small_item_red": (0.46, 0.36, 0.03, 0.2),
    "small_item_blue": (0.66, -0.34, 0.0275, -0.35),
    "small_item_green": (0.88, 0.18, 0.03, 0.75),
    "small_item_yellow": (1.02, -0.12, 0.0275, 0.0),
}


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


def make_state(name, pose_tuple):
    x, y, z, yaw = pose_tuple
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    state = ModelState()
    state.model_name = name
    state.reference_frame = "world"
    state.pose.position.x = x
    state.pose.position.y = y
    state.pose.position.z = z
    state.pose.orientation.x = qx
    state.pose.orientation.y = qy
    state.pose.orientation.z = qz
    state.pose.orientation.w = qw
    state.twist = Twist()
    return state


def main():
    rospy.init_node("reset_gazebo_scene", anonymous=True)
    use_reset_world = rospy.get_param("~use_reset_world", True)
    settle_time = float(rospy.get_param("~settle_time", 0.4))

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(0.2)
    for _ in range(5):
        cmd_pub.publish(Twist())
        rospy.sleep(0.03)

    if use_reset_world:
        rospy.loginfo("[reset_gazebo_scene] waiting for /gazebo/reset_world")
        rospy.wait_for_service("/gazebo/reset_world", timeout=10.0)
        reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        reset_world()
        rospy.sleep(settle_time)

    rospy.loginfo("[reset_gazebo_scene] waiting for /gazebo/set_model_state")
    rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    for name, pose in DEFAULT_POSES.items():
        try:
            resp = set_model_state(make_state(name, pose))
        except rospy.ServiceException as exc:
            rospy.logwarn("[reset_gazebo_scene] failed to reset %s: %s", name, exc)
            continue
        if not resp.success:
            rospy.logwarn("[reset_gazebo_scene] failed to reset %s: %s", name, resp.status_message)
        else:
            rospy.loginfo("[reset_gazebo_scene] reset %s", name)

    for _ in range(5):
        cmd_pub.publish(Twist())
        rospy.sleep(0.03)
    rospy.loginfo("[reset_gazebo_scene] done")


if __name__ == "__main__":
    main()
