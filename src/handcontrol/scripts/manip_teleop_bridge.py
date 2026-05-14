#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 BodyPose 中的肘弯与右手张开度映射为 OpenMANIPULATOR 关节轨迹（仿真 arm / gripper 控制器）。
"""
import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from handcontrol.msg import BodyPose


class ManipTeleopBridge:
    def __init__(self):
        rospy.init_node("manip_teleop_bridge", anonymous=True)
        self.use_course_mapping = rospy.get_param("~use_course_mapping", True)
        self.arm_topic = rospy.get_param("~arm_command_topic", "/arm_controller/command")
        self.grip_topic = rospy.get_param("~gripper_command_topic", "/gripper_controller/command")
        self.rate_hz = rospy.get_param("~publish_rate", 8.0)
        self.min_people = rospy.get_param("~min_people", 1)

        self.arm_pub = rospy.Publisher(self.arm_topic, JointTrajectory, queue_size=2)
        self.grip_pub = rospy.Publisher(self.grip_topic, JointTrajectory, queue_size=2)
        rospy.Subscriber("/body_pose", BodyPose, self._cb)
        self._pose = None

    def _cb(self, msg):
        self._pose = msg

    def _send_arm(self, j1, j2, j3, j4):
        jt = JointTrajectory()
        jt.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(j1), float(j2), float(j3), float(j4)]
        pt.time_from_start = rospy.Duration(0.35)
        jt.points.append(pt)
        self.arm_pub.publish(jt)

    def _send_grip(self, g):
        jt = JointTrajectory()
        jt.joint_names = ["gripper"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(g)]
        pt.time_from_start = rospy.Duration(0.25)
        jt.points.append(pt)
        self.grip_pub.publish(jt)

    def run(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            p = self._pose
            if p is None or not self.use_course_mapping:
                r.sleep()
                continue
            if p.num_people < self.min_people:
                r.sleep()
                continue

            bend = float(np.clip(p.avg_elbow_bend_deg, 0.0, 120.0))
            # 肘越弯，机械臂「折叠」略增大（启发式，单位 rad）
            j2 = float(np.clip(-bend / 120.0 * 1.15, -1.5, 0.3))
            j3 = float(np.clip(bend / 120.0 * 1.05, -0.5, 1.35))
            j4 = float(np.clip(-bend / 120.0 * 0.35, -0.9, 0.9))
            self._send_arm(0.0, j2, j3, j4)

            g_open = float(np.clip(p.right_palm_open_score, 0.0, 1.0))
            # URDF prismatic: lower=-0.01 upper=0.019
            g_pos = -0.01 + g_open * 0.029
            self._send_grip(g_pos)
            r.sleep()


def main():
    try:
        ManipTeleopBridge().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
