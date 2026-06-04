#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop_mapper.py — 订阅 /body_pose，发布底盘速度命令。
- use_course_mapping=false：原双臂角度差速映射 + 急停。
- use_course_mapping=true：按 README 的左手模式选择启停，底盘模式下读取右手空中摇杆。
"""
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from handcontrol.msg import BodyPose


MODE_STANDBY = 0
MODE_BASE = 1
MODE_ARM = 2
MODE_SAFETY = 3


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
        self.turn_step_deadzone = rospy.get_param("~turn_step_deadzone", 0.12)
        self.angle_deadzone = rospy.get_param("~angle_deadzone", 10.0)
        self.control_sensitivity = rospy.get_param("~control_sensitivity", 1.0)
        self.signal_timeout = rospy.get_param("~signal_timeout", 1.0)
        self.publish_rate = rospy.get_param("~publish_rate", 30)
        self.cmd_vel_out_topic = rospy.get_param("~cmd_vel_out_topic", "/cmd_vel_body")
        self.semi_auto_active_topic = rospy.get_param("~semi_auto_active_topic", "/vision_grasp/active")
        self.gesture_auto_grasp_enabled = rospy.get_param("~gesture_auto_grasp_enabled", True)
        self.gesture_auto_grasp_start_topic = rospy.get_param(
            "~gesture_auto_grasp_start_topic", "/vision_grasp/start"
        )
        self.gesture_auto_grasp_hold_time = float(rospy.get_param("~gesture_auto_grasp_hold_time", 0.6))
        self.gesture_auto_grasp_debounce = float(rospy.get_param("~gesture_auto_grasp_debounce", 2.0))
        self.gesture_auto_grasp_start_actions = self._string_list_param(
            "~gesture_auto_grasp_start_actions", ["BASE_RIGHT"]
        )

        self.current_body_pose = None
        self.last_pose_time = rospy.Time.now()
        self.semi_auto_active = False
        self._last_semi_auto_active = False
        self._gesture_start_since = None
        self._last_gesture_start_pub = rospy.Time(0)
        self._last_gesture_cancel_pub = rospy.Time(0)
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.state = 0
        self.debug_counter = 0
        self._last_armed = None

        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_out_topic, Twist, queue_size=10)
        self.armed_pub = rospy.Publisher("/course_base_armed", Bool, queue_size=1, latch=True)
        self.auto_grasp_start_pub = rospy.Publisher(
            self.gesture_auto_grasp_start_topic, Bool, queue_size=1
        )
        rospy.Subscriber("/body_pose", BodyPose, self._body_pose_callback)
        rospy.Subscriber(self.semi_auto_active_topic, Bool, self._semi_auto_active_callback, queue_size=1)

        rospy.loginfo("[teleop_mapper] 初始化完成")
        rospy.loginfo(f"  - use_course_mapping: {self.use_course_mapping}")
        rospy.loginfo(f"  - cmd_vel_out_topic: {self.cmd_vel_out_topic}")
        rospy.loginfo(f"  - gesture_auto_grasp_enabled: {self.gesture_auto_grasp_enabled}")
        rospy.loginfo(f"  - gesture_auto_grasp_start_topic: {self.gesture_auto_grasp_start_topic}")
        rospy.loginfo(f"  - gesture_auto_grasp_start_actions: {self.gesture_auto_grasp_start_actions}")

        self.armed_pub.publish(Bool(data=False))

    def _body_pose_callback(self, msg):
        self.current_body_pose = msg
        self.last_pose_time = rospy.Time.now()

    def _semi_auto_active_callback(self, msg):
        self.semi_auto_active = bool(msg.data)

    def _string_list_param(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            return [part.strip().upper() for part in value.split(",") if part.strip()]
        return [str(part).strip().upper() for part in value if str(part).strip()]

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
        mode = getattr(pose, "control_mode", None)
        if mode is not None:
            if mode != MODE_BASE:
                return 0.0, 0.0
            lin_src = float(getattr(pose, "right_hand_y_norm", 0.0))
            ang_src = float(getattr(pose, "right_hand_x_norm", 0.0))
        else:
            if pose.course_base_motion_enabled == 0:
                return 0.0, 0.0
            lin_src = float(pose.leg_forward_norm)
            ang_src = float(pose.leg_lateral_norm)

        if abs(lin_src) < 0.02:
            lin_src = 0.0
        if abs(ang_src) < 0.02:
            ang_src = 0.0

        lin = lin_src * self.max_linear_speed
        if abs(ang_src) < self.turn_step_deadzone:
            ang = 0.0
        else:
            # right hand moves right => right turn => negative angular.z
            ang = -np.sign(ang_src) * self.max_angular_speed
        lin = float(np.clip(lin, -self.max_linear_speed, self.max_linear_speed))
        ang = float(np.clip(ang, -self.max_angular_speed, self.max_angular_speed))
        return lin, ang

    def _pose_mode(self, pose):
        if hasattr(pose, "control_mode"):
            return int(pose.control_mode)
        if pose.state == 2:
            return MODE_SAFETY
        if pose.course_base_motion_enabled != 0:
            return MODE_BASE
        return MODE_STANDBY

    def _reset_targets(self, immediate=False):
        self.target_linear = 0.0
        self.target_angular = 0.0
        if immediate:
            self.current_linear = 0.0
            self.current_angular = 0.0

    def _publish_zero_now(self):
        self._reset_targets(immediate=True)
        self.cmd_vel_pub.publish(Twist())

    def _publish_auto_grasp_start(self, enabled, reason):
        now = rospy.Time.now()
        last_pub = self._last_gesture_start_pub if enabled else self._last_gesture_cancel_pub
        if (now - last_pub).to_sec() < self.gesture_auto_grasp_debounce:
            return
        self.auto_grasp_start_pub.publish(Bool(data=bool(enabled)))
        if enabled:
            self._last_gesture_start_pub = now
            rospy.loginfo("[teleop_mapper] gesture auto grasp start: %s", reason)
        else:
            self._last_gesture_cancel_pub = now
            self._gesture_start_since = None
            rospy.loginfo("[teleop_mapper] gesture auto grasp cancel: %s", reason)

    def _handle_auto_grasp_gesture(self, pose):
        if not self.gesture_auto_grasp_enabled or pose is None or not self.use_course_mapping:
            self._gesture_start_since = None
            return

        mode = self._pose_mode(pose)
        action = str(getattr(pose, "action_label", "")).upper()
        now = rospy.Time.now()

        if mode == MODE_SAFETY:
            if self.semi_auto_active:
                self._publish_auto_grasp_start(False, "SAFETY")
            else:
                self._gesture_start_since = None
            return

        start_gesture = mode == MODE_ARM and any(
            token in action for token in self.gesture_auto_grasp_start_actions
        )
        if not start_gesture or self.semi_auto_active:
            self._gesture_start_since = None
            return

        if self._gesture_start_since is None:
            self._gesture_start_since = now
            return

        if (now - self._gesture_start_since).to_sec() >= self.gesture_auto_grasp_hold_time:
            self._publish_auto_grasp_start(True, "ARM+" + action)
            self._gesture_start_since = None

    def _smooth_to_targets(self):
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

    def _smooth_to_stop(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self._smooth_to_targets()

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        rospy.loginfo("[teleop_mapper] 运行中…")
        while not rospy.is_shutdown():
            twist = Twist()
            pose = self.current_body_pose
            time_since_last = (rospy.Time.now() - self.last_pose_time).to_sec()
            if time_since_last > self.signal_timeout:
                pose = None
            self._handle_auto_grasp_gesture(pose)

            if self.semi_auto_active:
                if not self._last_semi_auto_active:
                    self._publish_zero_now()
                    self._last_semi_auto_active = True
                if self._last_armed is not False:
                    self.armed_pub.publish(Bool(data=False))
                    self._last_armed = False
                rate.sleep()
                continue
            if self._last_semi_auto_active:
                self._reset_targets(immediate=True)
                self._last_semi_auto_active = False

            if pose is None:
                self._publish_zero_now()
            elif self.use_course_mapping:
                mode = self._pose_mode(pose)
                armed = mode == MODE_BASE
                if self._last_armed != armed:
                    self.armed_pub.publish(Bool(data=armed))
                    self._last_armed = armed
                if mode in (MODE_STANDBY, MODE_SAFETY, MODE_ARM) or pose.state == 2:
                    self._publish_zero_now()
                else:
                    self.target_linear, self.target_angular = self._compute_velocity_course(pose)
                    self._smooth_to_targets()
                self.debug_counter += 1
                if self.debug_counter % 30 == 0:
                    rospy.loginfo(
                        "[course] mode=%s action=%s L=%+.3f A=%+.3f"
                        % (
                            getattr(pose, "mode_label", str(mode)),
                            getattr(pose, "action_label", ""),
                            self.current_linear,
                            self.current_angular,
                        )
                    )
            else:
                self.state = pose.state
                if self.state == 2:
                    self._publish_zero_now()
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
