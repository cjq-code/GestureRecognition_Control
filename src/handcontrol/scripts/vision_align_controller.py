#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Rotate the robot to center a visual target, optionally grasp after centered."""
import json
import ast

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class VisionAlignController:
    def __init__(self):
        rospy.init_node("vision_align_controller", anonymous=True)

        self.visible_topic = rospy.get_param("~visible_topic", "/vision_grasp/target_visible")
        self.x_topic = rospy.get_param("~x_topic", "/vision_grasp/target_x_norm")
        self.y_topic = rospy.get_param("~y_topic", "/vision_grasp/target_y_norm")
        self.front_y_topic = rospy.get_param("~front_y_topic", "/vision_grasp/target_front_y_norm")
        self.area_topic = rospy.get_param("~area_topic", "/vision_grasp/target_area_norm")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.status_topic = rospy.get_param("~status_topic", "/vision_grasp/align_status")
        self.center_deadzone = float(rospy.get_param("~center_deadzone", 0.06))
        self.angular_speed = float(rospy.get_param("~angular_speed", 0.12))
        self.min_angular_speed = float(rospy.get_param("~min_angular_speed", 0.02))
        self.angular_kp = float(rospy.get_param("~angular_kp", 0.36))
        self.angular_smooth = float(rospy.get_param("~angular_smooth", 0.65))
        self.target_timeout = float(rospy.get_param("~target_timeout", 0.5))
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self.send_observe_pose = self._bool_param("~send_observe_pose", True)
        self.auto_grasp = self._bool_param("~auto_grasp", False)
        self.auto_approach = self._bool_param("~auto_approach", False)
        self.centered_hold_time = float(rospy.get_param("~centered_hold_time", 0.8))
        self.min_grasp_area_norm = float(rospy.get_param("~min_grasp_area_norm", 0.0005))
        self.target_grasp_area_norm = float(rospy.get_param("~target_grasp_area_norm", 0.020))
        self.area_deadzone = float(rospy.get_param("~area_deadzone", 0.008))
        self.linear_speed = float(rospy.get_param("~linear_speed", 0.025))
        self.min_linear_speed = float(rospy.get_param("~min_linear_speed", 0.0))
        self.linear_kp = float(rospy.get_param("~linear_kp", 1.0))
        self.linear_smooth = float(rospy.get_param("~linear_smooth", 0.35))
        self.allow_reverse = self._bool_param("~allow_reverse", False)
        self.gripper_target_y_norm = float(rospy.get_param("~gripper_target_y_norm", -0.35))
        self.gripper_front_y_norm = float(rospy.get_param("~gripper_front_y_norm", self.gripper_target_y_norm))
        self.gripper_y_deadzone = float(rospy.get_param("~gripper_y_deadzone", 0.08))
        self.gripper_front_close_margin = float(rospy.get_param("~gripper_front_close_margin", 0.02))
        self.gripper_y_kp = float(rospy.get_param("~gripper_y_kp", 0.16))
        self.post_grasp_approach_timeout = float(rospy.get_param("~post_grasp_approach_timeout", 5.0))
        self.post_grasp_hold_time = float(rospy.get_param("~post_grasp_hold_time", 0.5))
        self.stop_settle_time = float(rospy.get_param("~stop_settle_time", 0.4))
        self.post_arm_settle_time = float(rospy.get_param("~post_arm_settle_time", 0.8))
        self.post_arm_recenter_timeout = float(rospy.get_param("~post_arm_recenter_timeout", 4.0))
        self.post_arm_recenter_hold_time = float(rospy.get_param("~post_arm_recenter_hold_time", 0.4))
        self.continue_on_recenter_timeout = self._bool_param("~continue_on_recenter_timeout", True)
        self.close_on_approach_timeout = self._bool_param("~close_on_approach_timeout", True)
        self.close_on_target_lost_after_grasp = self._bool_param("~close_on_target_lost_after_grasp", True)
        self.target_lost_grasp_hold_time = float(rospy.get_param("~target_lost_grasp_hold_time", 0.4))
        self.observe_pose = self._float_list_param("~observe_pose", [1.05, 0.45, -0.95, 0.45])
        self.ready_pose = self._float_list_param("~ready_pose", [0.0, 0.75, -0.25, -0.35])
        self.grasp_pose = self._float_list_param("~grasp_pose", [0.0, 1.55, -0.94, -0.24])
        self.observe_wait = float(rospy.get_param("~observe_wait", 1.2))
        self.arm_topic = rospy.get_param("~arm_command_topic", "/arm_controller/command")
        self.gripper_topic = rospy.get_param("~gripper_command_topic", "/gripper_controller/command")
        self.arm_joint_names = list(
            rospy.get_param("~arm_joint_names", ["joint1", "joint2", "joint3", "joint4"])
        )
        self.gripper_joint_names = list(
            rospy.get_param("~gripper_joint_names", ["gripper", "gripper_sub"])
        )
        self.gripper_open_pos = float(rospy.get_param("~gripper_open_pos", 0.032))
        self.gripper_close_pos = float(rospy.get_param("~gripper_close_pos", -0.018))
        self.arm_traj_time = float(rospy.get_param("~arm_traj_time", 1.0))
        self.gripper_traj_time = float(rospy.get_param("~gripper_traj_time", 0.25))
        self.grasp_wait = float(rospy.get_param("~grasp_wait", 1.2))
        self.gripper_wait = float(rospy.get_param("~gripper_wait", 0.5))
        self.lift_wait = float(rospy.get_param("~lift_wait", 1.0))

        self.target_visible = False
        self.target_x_norm = 0.0
        self.target_y_norm = 0.0
        self.target_front_y_norm = 0.0
        self.target_area_norm = 0.0
        self.last_target_time = rospy.Time(0)
        self.current_angular = 0.0
        self.current_linear = 0.0
        self.centered_since = None
        self.grasp_started = False
        self.grasp_done = False
        self.grasp_state = "IDLE"

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.arm_pub = rospy.Publisher(self.arm_topic, JointTrajectory, queue_size=2)
        self.gripper_pub = rospy.Publisher(self.gripper_topic, JointTrajectory, queue_size=2)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        rospy.Subscriber(self.visible_topic, Bool, self._visible_callback, queue_size=10)
        rospy.Subscriber(self.x_topic, Float32, self._x_callback, queue_size=10)
        rospy.Subscriber(self.y_topic, Float32, self._y_callback, queue_size=10)
        rospy.Subscriber(self.front_y_topic, Float32, self._front_y_callback, queue_size=10)
        rospy.Subscriber(self.area_topic, Float32, self._area_callback, queue_size=10)

        rospy.on_shutdown(self._publish_zero)
        rospy.loginfo("[vision_align_controller] ready")
        rospy.loginfo("  - cmd_vel_topic: %s", self.cmd_vel_topic)
        rospy.loginfo("  - center_deadzone: %.3f", self.center_deadzone)
        rospy.loginfo("  - angular_speed: %.3f", self.angular_speed)
        rospy.loginfo("  - min_angular_speed: %.3f", self.min_angular_speed)
        rospy.loginfo("  - angular_kp: %.3f", self.angular_kp)
        rospy.loginfo("  - angular_smooth: %.3f", self.angular_smooth)
        rospy.loginfo("  - send_observe_pose: %s", self.send_observe_pose)
        rospy.loginfo("  - auto_grasp: %s", self.auto_grasp)
        rospy.loginfo("  - auto_approach: %s", self.auto_approach)
        rospy.loginfo("  - target_grasp_area_norm: %.4f", self.target_grasp_area_norm)
        rospy.loginfo("  - gripper_target_y_norm: %.3f", self.gripper_target_y_norm)
        rospy.loginfo("  - gripper_front_y_norm: %.3f", self.gripper_front_y_norm)
        rospy.loginfo("  - gripper_y_deadzone: %.3f", self.gripper_y_deadzone)
        rospy.loginfo("  - observe_pose: %s", self.observe_pose)

    def _bool_param(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            return value.strip().lower() not in ("0", "false", "off", "no", "")
        return bool(value)

    def _float_list_param(self, name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            value = ast.literal_eval(value)
        return [float(v) for v in value]

    def _visible_callback(self, msg):
        self.target_visible = bool(msg.data)
        if self.target_visible:
            self.last_target_time = rospy.Time.now()

    def _x_callback(self, msg):
        self.target_x_norm = float(msg.data)
        self.last_target_time = rospy.Time.now()

    def _y_callback(self, msg):
        self.target_y_norm = float(msg.data)
        self.last_target_time = rospy.Time.now()

    def _front_y_callback(self, msg):
        self.target_front_y_norm = float(msg.data)
        self.last_target_time = rospy.Time.now()

    def _area_callback(self, msg):
        self.target_area_norm = float(msg.data)

    def _target_is_fresh(self):
        if self.last_target_time == rospy.Time(0):
            return False
        return (rospy.Time.now() - self.last_target_time).to_sec() <= self.target_timeout

    def _compute_cmd(self):
        twist = Twist()
        if not self.target_visible or not self._target_is_fresh():
            self.current_angular = 0.0
            self.current_linear = 0.0
            self.centered_since = None
            return twist, "LOST"
        x = self.target_x_norm
        if abs(x) <= self.center_deadzone:
            self.current_angular = 0.0
            if self.auto_approach and not self.auto_grasp and not self.grasp_started and not self.grasp_done:
                linear_x, distance_ready, approach_state = self._compute_approach_linear()
                if not distance_ready:
                    twist.linear.x = linear_x
                    self.centered_since = None
                    return twist, approach_state
            self.current_linear = 0.0
            if self.centered_since is None:
                self.centered_since = rospy.Time.now()
            return twist, "CENTERED"
        self.current_linear = 0.0
        self.centered_since = None
        # x_norm < 0 means target is left in the camera image. Positive
        # angular.z turns the robot left to bring the target toward center.
        target_angular = -self.angular_kp * x
        if abs(target_angular) < self.min_angular_speed:
            target_angular = self.min_angular_speed if target_angular >= 0.0 else -self.min_angular_speed
        if abs(target_angular) > self.angular_speed:
            target_angular = self.angular_speed if target_angular >= 0.0 else -self.angular_speed
        self.current_angular += self.angular_smooth * (target_angular - self.current_angular)
        twist.angular.z = self.current_angular
        return twist, "TURN_LEFT" if x < 0.0 else "TURN_RIGHT"

    def _area_is_ready(self):
        if self.target_area_norm < self.min_grasp_area_norm:
            return False
        if not self.auto_approach:
            return True
        return abs(self.target_area_norm - self.target_grasp_area_norm) <= self.area_deadzone

    def _compute_approach_linear(self):
        if self._area_is_ready():
            self.current_linear = 0.0
            return 0.0, True, "DISTANCE_READY"

        area_error = self.target_grasp_area_norm - self.target_area_norm
        target_linear = self.linear_kp * area_error
        if target_linear < 0.0 and not self.allow_reverse:
            self.current_linear = 0.0
            return 0.0, True, "DISTANCE_READY"
        if self.min_linear_speed > 0.0 and abs(target_linear) < self.min_linear_speed:
            target_linear = self.min_linear_speed if target_linear >= 0.0 else -self.min_linear_speed
        if abs(target_linear) > self.linear_speed:
            target_linear = self.linear_speed if target_linear >= 0.0 else -self.linear_speed
        self.current_linear += self.linear_smooth * (target_linear - self.current_linear)
        state = "APPROACH_FORWARD" if self.current_linear > 0.0 else "APPROACH_BACKWARD"
        return self.current_linear, False, state

    def _compute_gripper_relative_linear(self):
        y_error = self.target_front_y_norm - self.gripper_front_y_norm
        if y_error >= -self.gripper_front_close_margin:
            self.current_linear = 0.0
            return 0.0, True, "GRIPPER_FRONT_AHEAD_CLOSE"
        if abs(y_error) <= self.gripper_y_deadzone:
            self.current_linear = 0.0
            return 0.0, True, "GRIPPER_FRONT_ALIGNED"

        target_linear = -self.gripper_y_kp * y_error
        if target_linear < 0.0 and not self.allow_reverse:
            self.current_linear = 0.0
            return 0.0, True, "GRIPPER_ALIGNED_NO_REVERSE"
        if self.min_linear_speed > 0.0 and abs(target_linear) < self.min_linear_speed:
            target_linear = self.min_linear_speed if target_linear >= 0.0 else -self.min_linear_speed
        if abs(target_linear) > self.linear_speed:
            target_linear = self.linear_speed if target_linear >= 0.0 else -self.linear_speed
        self.current_linear += self.linear_smooth * (target_linear - self.current_linear)
        state = "GRIPPER_APPROACH_FORWARD" if self.current_linear > 0.0 else "GRIPPER_APPROACH_BACKWARD"
        return self.current_linear, False, state

    def _ready_to_grasp(self):
        if not self.auto_grasp or self.grasp_started or self.grasp_done:
            return False
        if self.centered_since is None:
            return False
        if not self.target_visible or not self._target_is_fresh():
            return False
        if self.target_area_norm < self.min_grasp_area_norm:
            return False
        return (rospy.Time.now() - self.centered_since).to_sec() >= self.centered_hold_time

    def _publish_zero(self):
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.cmd_pub.publish(Twist())

    def _hold_zero(self, duration, state="STOP_SETTLE"):
        deadline = rospy.Time.now() + rospy.Duration(max(0.0, duration))
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_zero()
            self.status_pub.publish(
                String(data=json.dumps(self._status_dict(state, 0.0, 0.0), ensure_ascii=False))
            )
            rate.sleep()

    def _make_arm_msg(self, positions):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time(0)
        msg.joint_names = list(self.arm_joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]
        pt.velocities = [0.0] * len(self.arm_joint_names)
        pt.accelerations = [0.0] * len(self.arm_joint_names)
        pt.time_from_start = rospy.Duration(self.arm_traj_time)
        msg.points.append(pt)
        return msg

    def _make_gripper_msg(self, position):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time(0)
        msg.joint_names = list(self.gripper_joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)] * len(self.gripper_joint_names)
        pt.velocities = [0.0] * len(self.gripper_joint_names)
        pt.accelerations = [0.0] * len(self.gripper_joint_names)
        pt.time_from_start = rospy.Duration(self.gripper_traj_time)
        msg.points.append(pt)
        return msg

    def _wait_for_pub(self, pub, topic, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while pub.get_num_connections() == 0 and rospy.Time.now() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.05)
        if pub.get_num_connections() == 0:
            rospy.logwarn("no subscriber on %s", topic)
            return False
        return True

    def _publish_arm_pose(self, positions, label):
        if not self._wait_for_pub(self.arm_pub, self.arm_topic):
            return False
        msg = self._make_arm_msg(positions)
        rospy.loginfo("[vision_align_controller] arm %s: %s", label, positions)
        for _ in range(3):
            if rospy.is_shutdown():
                return False
            self.arm_pub.publish(msg)
            rospy.sleep(0.08)
        return True

    def _publish_gripper(self, position, label):
        if not self._wait_for_pub(self.gripper_pub, self.gripper_topic):
            return False
        msg = self._make_gripper_msg(position)
        rospy.loginfo("[vision_align_controller] gripper %s: %.3f", label, position)
        for _ in range(3):
            if rospy.is_shutdown():
                return False
            self.gripper_pub.publish(msg)
            rospy.sleep(0.08)
        return True

    def _send_observe_pose(self):
        if not self.send_observe_pose:
            return
        if len(self.observe_pose) != len(self.arm_joint_names):
            rospy.logwarn(
                "observe_pose length %s does not match arm_joint_names length %s; skip observe pose",
                len(self.observe_pose),
                len(self.arm_joint_names),
            )
            return
        if not self._wait_for_pub(self.arm_pub, self.arm_topic):
            rospy.logwarn("skip observe pose")
            return
        msg = self._make_arm_msg(self.observe_pose)
        rospy.loginfo("[vision_align_controller] arm observe pose: %s", self.observe_pose)
        for _ in range(5):
            if rospy.is_shutdown():
                return
            self.arm_pub.publish(msg)
            rospy.sleep(0.1)
        if self.observe_wait > 0.0:
            rospy.sleep(self.observe_wait)

    def _run_grasp_sequence(self):
        self.grasp_started = True
        self.grasp_state = "OPEN_GRIPPER"
        self._hold_zero(self.stop_settle_time, "PRE_GRASP_STOP")
        if not self._publish_gripper(self.gripper_open_pos, "open"):
            self.grasp_state = "FAILED"
            return
        rospy.sleep(self.gripper_wait)

        self.grasp_state = "GRASP_POSE"
        self._hold_zero(self.stop_settle_time, "PRE_ARM_STOP")
        if not self._publish_arm_pose(self.grasp_pose, "grasp"):
            self.grasp_state = "FAILED"
            return
        rospy.sleep(self.grasp_wait)
        self._hold_zero(self.post_arm_settle_time, "POST_ARM_SETTLE")

        self.grasp_state = "GRIPPER_RELATIVE_APPROACH"
        if not self._approach_after_grasp_pose():
            self.grasp_state = "FAILED"
            return

        self.grasp_state = "CLOSE_GRIPPER"
        if not self._publish_gripper(self.gripper_close_pos, "close"):
            self.grasp_state = "FAILED"
            return
        rospy.sleep(self.gripper_wait)

        self.grasp_state = "LIFT"
        if not self._publish_arm_pose(self.ready_pose, "lift"):
            self.grasp_state = "FAILED"
            return
        rospy.sleep(self.lift_wait)

        self.grasp_done = True
        self.grasp_state = "DONE"

    def _approach_after_grasp_pose(self):
        if not self._recenter_after_grasp_pose():
            return False

        deadline = rospy.Time.now() + rospy.Duration(self.post_grasp_approach_timeout)
        aligned_since = None
        lost_since = None
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not self.target_visible or not self._target_is_fresh():
                self._publish_zero()
                aligned_since = None
                if lost_since is None:
                    lost_since = rospy.Time.now()
                self.status_pub.publish(
                    String(data=json.dumps(self._status_dict("POST_GRASP_TARGET_LOST", 0.0, 0.0), ensure_ascii=False))
                )
                if self.close_on_target_lost_after_grasp:
                    if (rospy.Time.now() - lost_since).to_sec() >= self.target_lost_grasp_hold_time:
                        rospy.logwarn("[vision_align_controller] target lost after grasp pose; closing gripper")
                        return True
                rate.sleep()
                continue

            lost_since = None
            linear_x, ready, state = self._compute_gripper_relative_linear()
            twist = Twist()
            twist.linear.x = linear_x
            self.cmd_pub.publish(twist)
            self.status_pub.publish(
                String(data=json.dumps(self._status_dict(state, 0.0, linear_x), ensure_ascii=False))
            )

            if ready:
                if aligned_since is None:
                    aligned_since = rospy.Time.now()
                if (rospy.Time.now() - aligned_since).to_sec() >= self.post_grasp_hold_time:
                    self._publish_zero()
                    return True
            else:
                aligned_since = None
            rate.sleep()

        self._publish_zero()
        rospy.logwarn("[vision_align_controller] post-grasp approach timed out")
        return bool(self.close_on_approach_timeout)

    def _recenter_after_grasp_pose(self):
        deadline = rospy.Time.now() + rospy.Duration(self.post_arm_recenter_timeout)
        centered_since = None
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not self.target_visible or not self._target_is_fresh():
                self._publish_zero()
                centered_since = None
                self.status_pub.publish(
                    String(data=json.dumps(self._status_dict("POST_ARM_TARGET_LOST", 0.0, 0.0), ensure_ascii=False))
                )
                rate.sleep()
                continue

            if abs(self.target_x_norm) <= self.center_deadzone:
                self._publish_zero()
                if centered_since is None:
                    centered_since = rospy.Time.now()
                self.status_pub.publish(
                    String(data=json.dumps(self._status_dict("POST_ARM_RECENTERED", 0.0, 0.0), ensure_ascii=False))
                )
                if (rospy.Time.now() - centered_since).to_sec() >= self.post_arm_recenter_hold_time:
                    return True
                rate.sleep()
                continue

            centered_since = None
            twist, state = self._compute_cmd()
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            self.status_pub.publish(
                String(data=json.dumps(self._status_dict("POST_ARM_" + state, twist.angular.z, 0.0), ensure_ascii=False))
            )
            rate.sleep()

        self._publish_zero()
        rospy.logwarn("[vision_align_controller] post-arm recenter timed out")
        return bool(self.continue_on_recenter_timeout and self.target_visible)

    def run(self):
        self._send_observe_pose()
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            twist, state = self._compute_cmd()
            if self._ready_to_grasp():
                state = "GRASPING"
                self.status_pub.publish(
                    String(data=json.dumps(self._status_dict(state, 0.0, 0.0), ensure_ascii=False))
                )
                self._run_grasp_sequence()
                twist = Twist()
                state = self.grasp_state
            self.cmd_pub.publish(twist)
            self.status_pub.publish(
                String(data=json.dumps(self._status_dict(state, twist.angular.z, twist.linear.x), ensure_ascii=False))
            )
            rate.sleep()

    def _status_dict(self, state, angular_z, linear_x):
        centered_duration = 0.0
        if self.centered_since is not None:
            centered_duration = (rospy.Time.now() - self.centered_since).to_sec()
        return {
            "state": state,
            "target_visible": bool(self.target_visible),
            "target_x_norm": round(float(self.target_x_norm), 4),
            "target_y_norm": round(float(self.target_y_norm), 4),
            "target_front_y_norm": round(float(self.target_front_y_norm), 4),
            "target_area_norm": round(float(self.target_area_norm), 6),
            "angular_z": round(float(angular_z), 4),
            "linear_x": round(float(linear_x), 4),
            "center_deadzone": self.center_deadzone,
            "centered_duration": round(float(centered_duration), 3),
            "centered_hold_time": self.centered_hold_time,
            "min_grasp_area_norm": self.min_grasp_area_norm,
            "target_grasp_area_norm": self.target_grasp_area_norm,
            "area_deadzone": self.area_deadzone,
            "gripper_target_y_norm": self.gripper_target_y_norm,
            "gripper_front_y_norm": self.gripper_front_y_norm,
            "gripper_y_deadzone": self.gripper_y_deadzone,
            "gripper_front_close_margin": self.gripper_front_close_margin,
            "post_arm_recenter_timeout": self.post_arm_recenter_timeout,
            "post_grasp_approach_timeout": self.post_grasp_approach_timeout,
            "close_on_approach_timeout": bool(self.close_on_approach_timeout),
            "auto_grasp": bool(self.auto_grasp),
            "auto_approach": bool(self.auto_approach),
            "grasp_state": self.grasp_state,
            "grasp_done": bool(self.grasp_done),
            "angular_kp": self.angular_kp,
            "angular_speed_limit": self.angular_speed,
        }


if __name__ == "__main__":
    try:
        VisionAlignController().run()
    except rospy.ROSInterruptException:
        pass
