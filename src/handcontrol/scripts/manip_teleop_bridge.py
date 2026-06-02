#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 BodyPose 中的 README 体感动作映射为 OpenMANIPULATOR 关节轨迹。

只有左手握拳举在肩旁进入 ARM 模式时才发布机械臂/夹爪命令；
待机、底盘模式和安全锁定时保持当前控制器目标。
机械臂姿态复用键鼠控制的两个稳定姿态，避免连续映射导致穿模。
"""
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from handcontrol.msg import BodyPose


MODE_STANDBY = 0
MODE_BASE = 1
MODE_ARM = 2
MODE_SAFETY = 3


class ManipTeleopBridge:
    def __init__(self):
        rospy.init_node("manip_teleop_bridge", anonymous=True)
        self.use_course_mapping = rospy.get_param("~use_course_mapping", True)
        self.arm_topic = rospy.get_param("~arm_command_topic", "/arm_controller/command")
        self.grip_topic = rospy.get_param("~gripper_command_topic", "/gripper_controller/command")
        self.rate_hz = rospy.get_param("~publish_rate", 8.0)
        self.min_people = rospy.get_param("~min_people", 1)
        self.signal_timeout = rospy.get_param("~signal_timeout", 1.0)
        self.arm_joint_names = list(
            rospy.get_param("~arm_joint_names", ["joint1", "joint2", "joint3", "joint4"])
        )
        self.gripper_joint_names = list(
            rospy.get_param("~gripper_joint_names", ["gripper", "gripper_sub"])
        )
        self.ready_pose = list(rospy.get_param("~ready_pose", [0.0, 0.75, -0.25, -0.35]))
        self.grasp_pose = list(rospy.get_param("~grasp_pose", [0.0, 1.55, -0.94, -0.24]))
        self.gripper_open_pos = rospy.get_param("~gripper_open_pos", 0.032)
        self.gripper_close_pos = rospy.get_param("~gripper_close_pos", -0.018)
        self.arm_traj_time = rospy.get_param("~arm_traj_time", 1.0)
        self.gripper_traj_time = rospy.get_param("~gripper_traj_time", 0.25)
        self._last_arm_pose_name = None
        self._last_grip_name = None

        if len(self.arm_joint_names) != 4:
            raise ValueError("~arm_joint_names must contain exactly 4 joints")
        if len(self.ready_pose) != len(self.arm_joint_names):
            raise ValueError("~ready_pose length must match ~arm_joint_names")
        if len(self.grasp_pose) != len(self.arm_joint_names):
            raise ValueError("~grasp_pose length must match ~arm_joint_names")
        if not self.gripper_joint_names:
            raise ValueError("~gripper_joint_names must not be empty")

        self.arm_pub = rospy.Publisher(self.arm_topic, JointTrajectory, queue_size=2)
        self.grip_pub = rospy.Publisher(self.grip_topic, JointTrajectory, queue_size=2)
        rospy.Subscriber("/body_pose", BodyPose, self._cb)
        self._pose = None
        self._last_pose_time = rospy.Time.now()

    def _cb(self, msg):
        self._pose = msg
        self._last_pose_time = rospy.Time.now()

    def _send_arm_pose(self, positions, pose_name):
        jt = JointTrajectory()
        jt.header.stamp = rospy.Time(0)
        jt.joint_names = list(self.arm_joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.velocities = [0.0] * len(self.arm_joint_names)
        pt.accelerations = [0.0] * len(self.arm_joint_names)
        pt.time_from_start = rospy.Duration(self.arm_traj_time)
        jt.points.append(pt)
        self.arm_pub.publish(jt)
        self._last_arm_pose_name = pose_name
        rospy.loginfo("[manip_teleop_bridge] arm %s: %s", pose_name, pt.positions)

    def _send_grip(self, g, grip_name):
        jt = JointTrajectory()
        jt.header.stamp = rospy.Time(0)
        jt.joint_names = list(self.gripper_joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(g)] * len(self.gripper_joint_names)
        pt.velocities = [0.0] * len(self.gripper_joint_names)
        pt.accelerations = [0.0] * len(self.gripper_joint_names)
        pt.time_from_start = rospy.Duration(self.gripper_traj_time)
        jt.points.append(pt)
        self.grip_pub.publish(jt)
        self._last_grip_name = grip_name
        rospy.loginfo("[manip_teleop_bridge] gripper %s: %s", grip_name, pt.positions)

    def _pose_mode(self, pose):
        if hasattr(pose, "control_mode"):
            return int(pose.control_mode)
        if pose.state == 2:
            return MODE_SAFETY
        return MODE_ARM if pose.course_base_motion_enabled == 0 and pose.num_people >= self.min_people else MODE_STANDBY

    def _desired_arm_pose_name(self, pose):
        action = str(getattr(pose, "action_label", "")).upper()
        depth = float(getattr(pose, "right_hand_depth_norm", 0.0))
        y_axis = float(getattr(pose, "right_hand_y_norm", 0.0))
        if y_axis < -0.18:
            return "grasp"
        if y_axis > 0.18:
            return "ready"
        if "GRASP" in action or "LOWER" in action or "EXTEND" in action or depth > 0.55:
            return "grasp"
        if "READY" in action or "LIFT" in action:
            return "ready"
        return None

    def run(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            p = self._pose
            if p is None or not self.use_course_mapping:
                r.sleep()
                continue
            if (rospy.Time.now() - self._last_pose_time).to_sec() > self.signal_timeout:
                r.sleep()
                continue
            if p.num_people < self.min_people:
                r.sleep()
                continue
            if self._pose_mode(p) != MODE_ARM:
                r.sleep()
                continue

            pose_name = self._desired_arm_pose_name(p)
            if pose_name == "ready" and self._last_arm_pose_name != "ready":
                self._send_arm_pose(self.ready_pose, "ready")
            elif pose_name == "grasp" and self._last_arm_pose_name != "grasp":
                self._send_arm_pose(self.grasp_pose, "grasp")

            if getattr(p, "right_fist", 0) and self._last_grip_name != "close":
                self._send_grip(self.gripper_close_pos, "close")
            elif getattr(p, "right_palm_open", 0) and self._last_grip_name != "open":
                self._send_grip(self.gripper_open_pos, "open")
            r.sleep()


def main():
    try:
        ManipTeleopBridge().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
