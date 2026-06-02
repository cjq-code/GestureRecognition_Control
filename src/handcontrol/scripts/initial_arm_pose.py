#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish the startup arm pose once Gazebo arm_controller is ready."""

import time
import ast

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _float_list_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        value = ast.literal_eval(value)
    return [float(v) for v in value]


class InitialArmPose:
    def __init__(self):
        rospy.init_node("initial_arm_pose", anonymous=True)
        self.arm_topic = rospy.get_param("~arm_command_topic", "/arm_controller/command")
        self.arm_joint_names = list(
            rospy.get_param("~arm_joint_names", ["joint1", "joint2", "joint3", "joint4"])
        )
        self.ready_pose = _float_list_param("~ready_pose", [0.0, 0.75, -0.25, -0.35])
        self.arm_traj_time = float(rospy.get_param("~arm_traj_time", 1.0))
        self.wait_timeout = float(rospy.get_param("~wait_timeout", 30.0))
        self.repeat_count = int(rospy.get_param("~repeat_count", 8))
        self.repeat_rate = float(rospy.get_param("~repeat_rate", 4.0))

        if len(self.ready_pose) != len(self.arm_joint_names):
            raise ValueError("~ready_pose length must match ~arm_joint_names")
        if self.repeat_count <= 0:
            raise ValueError("~repeat_count must be positive")
        if self.repeat_rate <= 0.0:
            raise ValueError("~repeat_rate must be positive")

        self.arm_pub = rospy.Publisher(self.arm_topic, JointTrajectory, queue_size=2)

    def _make_msg(self):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time(0)
        msg.joint_names = list(self.arm_joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(self.ready_pose)
        point.velocities = [0.0] * len(self.arm_joint_names)
        point.accelerations = [0.0] * len(self.arm_joint_names)
        point.time_from_start = rospy.Duration(self.arm_traj_time)
        msg.points.append(point)
        return msg

    def run(self):
        deadline = time.monotonic() + self.wait_timeout
        while (
            self.arm_pub.get_num_connections() == 0
            and time.monotonic() < deadline
            and not rospy.is_shutdown()
        ):
            rospy.sleep(0.1)

        if self.arm_pub.get_num_connections() == 0:
            rospy.logwarn("[initial_arm_pose] no subscriber on %s", self.arm_topic)
            return

        rate = rospy.Rate(self.repeat_rate)
        msg = self._make_msg()
        rospy.loginfo("[initial_arm_pose] publishing ready pose: %s", self.ready_pose)
        for _ in range(self.repeat_count):
            if rospy.is_shutdown():
                return
            self.arm_pub.publish(msg)
            rate.sleep()


def main():
    try:
        InitialArmPose().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except ValueError as exc:
        rospy.logerr("[initial_arm_pose] invalid configuration: %s", exc)


if __name__ == "__main__":
    main()
