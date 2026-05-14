#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop_mapper.py — 订阅 /body_pose，发布速度命令。
- use_course_mapping=false：原双臂角度差速映射 + 急停。
- use_course_mapping=true：左手「2」解锁后，腿前向/侧向映射线速度/角速度；双手并拢后停止；
  急停仍为双手举过头顶 (state==2)。
"""
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from handcontrol.msg import BodyPose


class TeleopMapper:
    def __init__(self):
        rospy.init_node("teleop_mapper", anonymous=True)

        self.use_course_mapping = rospy.get_param("~use_course_mapping", True)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.22)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.0)
        self.wheel_base = rospy.get_param("~wheel_base", 0.287)

        self.linear_smooth_factor = rospy.get_param("~linear_smooth_factor", 0.3)
        self.angular_smooth_factor = rospy.get_param("~angular_smooth_factor", 0.3)
        self.linear_ramp_rate = rospy.get_param("~linear_ramp_rate", 0.05)
        self.angular_ramp_rate = rospy.get_param("~angular_ramp_rate", 0.2)
        self.angle_deadzone = rospy.get_param("~angle_deadzone", 10.0)
        self.control_sensitivity = rospy.get_param("~control_sensitivity", 1.0)
        self.signal_timeout = rospy.get_param("~signal_timeout", 1.0)
        self.publish_rate = rospy.get_param("~publish_rate", 30)
        self.cmd_vel_out_topic = rospy.get_param("~cmd_vel_out_topic", "/cmd_vel_body")

        self.current_body_pose = None
        self.last_pose_time = rospy.Time.now()
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.state = 0
        self.debug_counter = 0
        self._last_armed = None

        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_out_topic, Twist, queue_size=10)
        self.armed_pub = rospy.Publisher("/course_base_armed", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/body_pose", BodyPose, self._body_pose_callback)

        rospy.loginfo("[teleop_mapper] 初始化完成")
        rospy.loginfo(f"  - use_course_mapping: {self.use_course_mapping}")
        rospy.loginfo(f"  - cmd_vel_out_topic: {self.cmd_vel_out_topic}")

        self.armed_pub.publish(Bool(data=False))

    def _body_pose_callback(self, msg):
        self.current_body_pose = msg
        self.last_pose_time = rospy.Time.now()

    def _angle_to_speed(self, angle):
        if abs(angle) < self.angle_deadzone:
            return 0.0
        normalized = np.clip(angle / 90.0, -1.0, 1.0)
        return float(
            np.clip(
                np.sign(normalized) * (abs(normalized) ** (1.0 / self.control_sensitivity)),
                -1.0,
                1.0,
            )
        )

    def _apply_smoothing(self, current, target, smooth_factor, ramp_rate):
        smoothed = current + smooth_factor * (target - current)
        delta = np.clip(smoothed - current, -ramp_rate, ramp_rate)
        return current + delta

    def _compute_velocity_legacy(self, pose):
        v_left = self._angle_to_speed(pose.left_arm_angle)
        v_right = self._angle_to_speed(pose.right_arm_angle)
        linear = (v_left + v_right) / 2.0 * self.max_linear_speed
        angular = (v_right - v_left) / self.wheel_base * self.max_angular_speed / 2.0
        linear = float(np.clip(linear, -self.max_linear_speed, self.max_linear_speed))
        angular = float(np.clip(angular, -self.max_angular_speed, self.max_angular_speed))
        return linear, angular, v_left, v_right

    def _compute_velocity_course(self, pose):
        if pose.course_base_motion_enabled == 0:
            return 0.0, 0.0
        lin = float(pose.leg_forward_norm) * self.max_linear_speed
        ang = -float(pose.leg_lateral_norm) * self.max_angular_speed
        lin = float(np.clip(lin, -self.max_linear_speed, self.max_linear_speed))
        ang = float(np.clip(ang, -self.max_angular_speed, self.max_angular_speed))
        return lin, ang

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        rospy.loginfo("[teleop_mapper] 运行中…")
        while not rospy.is_shutdown():
            twist = Twist()
            pose = self.current_body_pose
            time_since_last = (rospy.Time.now() - self.last_pose_time).to_sec()
            if time_since_last > self.signal_timeout:
                pose = None

            if pose is None:
                self.target_linear = 0.0
                self.target_angular = 0.0
                self.current_linear = self._apply_smoothing(
                    self.current_linear, 0.0, self.linear_smooth_factor, self.linear_ramp_rate
                )
                self.current_angular = self._apply_smoothing(
                    self.current_angular, 0.0, self.angular_smooth_factor, self.angular_ramp_rate
                )
            elif self.use_course_mapping:
                armed = pose.course_base_motion_enabled != 0
                if self._last_armed != armed:
                    self.armed_pub.publish(Bool(data=armed))
                    self._last_armed = armed
                if pose.state == 2:
                    self.target_linear = 0.0
                    self.target_angular = 0.0
                    self.current_linear = 0.0
                    self.current_angular = 0.0
                else:
                    self.target_linear, self.target_angular = self._compute_velocity_course(pose)
                    self.current_linear = self._apply_smoothing(
                        self.current_linear,
                        self.target_linear,
                        self.linear_smooth_factor,
                        self.linear_ramp_rate,
                    )
                    self.current_angular = self._apply_smoothing(
                        self.current_angular,
                        self.target_angular,
                        self.angular_smooth_factor,
                        self.angular_ramp_rate,
                    )
                self.debug_counter += 1
                if self.debug_counter % 30 == 0:
                    rospy.loginfo(
                        f"[course] armed={armed} L={self.current_linear:+.3f} A={self.current_angular:+.3f}"
                    )
            else:
                self.state = pose.state
                if self.state == 2:
                    self.target_linear = 0.0
                    self.target_angular = 0.0
                    self.current_linear = 0.0
                    self.current_angular = 0.0
                elif self.state == 1:
                    self.target_linear, self.target_angular, vl, vr = self._compute_velocity_legacy(pose)
                    self.current_linear = self._apply_smoothing(
                        self.current_linear,
                        self.target_linear,
                        self.linear_smooth_factor,
                        self.linear_ramp_rate,
                    )
                    self.current_angular = self._apply_smoothing(
                        self.current_angular,
                        self.target_angular,
                        self.angular_smooth_factor,
                        self.angular_ramp_rate,
                    )
                else:
                    self.target_linear = 0.0
                    self.target_angular = 0.0
                    self.current_linear = self._apply_smoothing(
                        self.current_linear, 0.0, self.linear_smooth_factor, self.linear_ramp_rate
                    )
                    self.current_angular = self._apply_smoothing(
                        self.current_angular, 0.0, self.angular_smooth_factor, self.angular_ramp_rate
                    )

            twist.linear.x = self.current_linear
            twist.angular.z = self.current_angular
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self._stop_robot()

    def _stop_robot(self):
        t = Twist()
        self.cmd_vel_pub.publish(t)


def main():
    try:
        TeleopMapper().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
