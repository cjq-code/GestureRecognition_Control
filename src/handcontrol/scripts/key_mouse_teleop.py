#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard and mouse GUI teleop for TurtleBot3 + OpenMANIPULATOR-X simulation.

Keyboard:
  W/A/S/D or arrows  move base while held
  X or Space         stop base
  R / U              emergency stop / clear emergency stop
  1 / 2              arm ready pose / forward grasp pose
  O / C              gripper open / close

Mouse:
  Drag inside the velocity pad. Up/down maps linear speed, left/right maps turn
  speed. Releasing the mouse stops the base.
"""

import math
import tkinter as tk

import rospy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from handcontrol.msg import BodyPose


class KeyMouseTeleop:
    def __init__(self):
        rospy.init_node("key_mouse_teleop", anonymous=True)

        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.max_linear_speed = float(rospy.get_param("~linear_speed", 0.3))
        self.max_angular_speed = float(rospy.get_param("~angular_speed", 0.5))
        self.mouse_deadzone = float(rospy.get_param("~mouse_deadzone", 0.05))
        self.retract_on_shutdown = self._bool_param("~retract_on_shutdown", True)
        self.publish_body_pose = self._bool_param("~publish_body_pose", True)

        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.body_pose_topic = rospy.get_param("~body_pose_topic", "/body_pose")
        self.arm_topic = rospy.get_param("~arm_command_topic", "/arm_controller/command")
        self.gripper_topic = rospy.get_param(
            "~gripper_command_topic", "/gripper_controller/command"
        )

        self.arm_joint_names = list(
            rospy.get_param("~arm_joint_names", ["joint1", "joint2", "joint3", "joint4"])
        )
        self.ready_pose = [
            float(v) for v in rospy.get_param("~ready_pose", [0.0, 0.75, -0.25, -0.35])
        ]
        self.grasp_pose = [
            float(v) for v in rospy.get_param("~grasp_pose", [0.0, 1.55, -0.94, -0.24])
        ]
        self.arm_traj_time = float(rospy.get_param("~arm_traj_time", 1.0))
        self.initial_arm_pose = str(rospy.get_param("~initial_arm_pose", "ready")).strip().lower()

        self.gripper_joint_names = list(
            rospy.get_param("~gripper_joint_names", ["gripper", "gripper_sub"])
        )
        self.gripper_open_pos = float(rospy.get_param("~gripper_open_pos", 0.032))
        self.gripper_close_pos = float(rospy.get_param("~gripper_close_pos", -0.018))
        self.gripper_traj_time = float(rospy.get_param("~gripper_traj_time", 0.25))

        self._validate_config()

        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.body_pose_pub = rospy.Publisher(self.body_pose_topic, BodyPose, queue_size=10)
        self.arm_pub = rospy.Publisher(self.arm_topic, JointTrajectory, queue_size=2)
        self.gripper_pub = rospy.Publisher(self.gripper_topic, JointTrajectory, queue_size=2)

        self.pressed_keys = set()
        self.mouse_active = False
        self.mouse_linear = 0.0
        self.mouse_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.right_palm_open_score = 0.0
        self.estop = False
        self.running = True
        self.shutdown_done = False
        self.initial_arm_retry_count = 0

        self.root = tk.Tk()
        self.root.title("TB3 Key/Mouse Teleop")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._request_close)

        self._build_ui()
        self._bind_events()
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo("[key_mouse_teleop] started")
        rospy.loginfo("[key_mouse_teleop] cmd_vel=%s", self.cmd_vel_topic)

    @staticmethod
    def _bool_param(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _validate_config(self):
        if self.publish_rate <= 0.0:
            raise ValueError("~publish_rate must be > 0")
        if self.max_linear_speed < 0.0 or self.max_angular_speed < 0.0:
            raise ValueError("speed parameters must be non-negative")
        if not 0.0 <= self.mouse_deadzone < 1.0:
            raise ValueError("~mouse_deadzone must be in [0.0, 1.0)")
        if len(self.ready_pose) != len(self.arm_joint_names):
            raise ValueError("~ready_pose length must match ~arm_joint_names")
        if len(self.grasp_pose) != len(self.arm_joint_names):
            raise ValueError("~grasp_pose length must match ~arm_joint_names")
        if self.initial_arm_pose == "retract":
            self.initial_arm_pose = "ready"
        if self.initial_arm_pose not in ("", "none", "off", "false", "ready", "grasp"):
            raise ValueError("~initial_arm_pose must be ready, grasp, none, off or false")
        if not self.gripper_joint_names:
            raise ValueError("~gripper_joint_names must not be empty")

    def _build_ui(self):
        self.root.configure(bg="#1f2933")
        self.root.geometry("620x520")

        title = tk.Label(
            self.root,
            text="TurtleBot3 + OpenMANIPULATOR-X Key/Mouse Control",
            fg="#f5f7fa",
            bg="#1f2933",
            font=("DejaVu Sans", 15, "bold"),
        )
        title.pack(pady=(14, 8))

        self.status_var = tk.StringVar(value="Ready")
        self.vel_var = tk.StringVar(value="linear +0.00 m/s   angular +0.00 rad/s")
        self.estop_var = tk.StringVar(value="E-STOP: OFF")

        status_frame = tk.Frame(self.root, bg="#1f2933")
        status_frame.pack(fill="x", padx=18)

        tk.Label(
            status_frame,
            textvariable=self.status_var,
            anchor="w",
            fg="#d9e2ec",
            bg="#1f2933",
            font=("DejaVu Sans", 11),
        ).pack(fill="x")
        tk.Label(
            status_frame,
            textvariable=self.vel_var,
            anchor="w",
            fg="#9fb3c8",
            bg="#1f2933",
            font=("DejaVu Sans Mono", 10),
        ).pack(fill="x", pady=(4, 0))
        tk.Label(
            status_frame,
            textvariable=self.estop_var,
            anchor="w",
            fg="#f97066",
            bg="#1f2933",
            font=("DejaVu Sans Mono", 10, "bold"),
        ).pack(fill="x", pady=(4, 0))

        self.canvas_size = 260
        self.pad_radius = 112
        self.pad_center = self.canvas_size // 2
        self.canvas = tk.Canvas(
            self.root,
            width=self.canvas_size,
            height=self.canvas_size,
            bg="#102a43",
            highlightthickness=2,
            highlightbackground="#486581",
        )
        self.canvas.pack(pady=16)
        c = self.pad_center
        r = self.pad_radius
        self.canvas.create_oval(c - r, c - r, c + r, c + r, outline="#bcccdc", width=2)
        self.canvas.create_line(c, c - r, c, c + r, fill="#486581")
        self.canvas.create_line(c - r, c, c + r, c, fill="#486581")
        self.knob = self.canvas.create_oval(
            c - 10, c - 10, c + 10, c + 10, fill="#38bec9", outline="#e0fcff", width=2
        )

        help_text = (
            "Keyboard: W/A/S/D or arrows move while held, X/Space stop, R/U E-stop/clear\n"
            "Arm: 1 ready, 2 forward grasp    Gripper: O open, C close\n"
            "Mouse: drag in the pad for continuous base velocity, release to stop"
        )
        tk.Label(
            self.root,
            text=help_text,
            fg="#d9e2ec",
            bg="#1f2933",
            justify="left",
            font=("DejaVu Sans", 10),
        ).pack(padx=18, pady=(0, 12), anchor="w")

        button_frame = tk.Frame(self.root, bg="#1f2933")
        button_frame.pack(fill="x", padx=18)
        tk.Button(button_frame, text="Stop", command=self._stop_base, width=12).pack(
            side="left", padx=(0, 8)
        )
        tk.Button(button_frame, text="Arm Ready", command=self._send_ready, width=12).pack(
            side="left", padx=(0, 8)
        )
        tk.Button(button_frame, text="Arm Grasp", command=self._send_grasp, width=12).pack(
            side="left", padx=(0, 8)
        )
        tk.Button(button_frame, text="Gripper Open", command=self._open_gripper, width=13).pack(
            side="left", padx=(0, 8)
        )
        tk.Button(button_frame, text="Gripper Close", command=self._close_gripper, width=13).pack(
            side="left"
        )

    def _bind_events(self):
        self.root.bind_all("<KeyPress>", self._on_key_press)
        self.root.bind_all("<KeyRelease>", self._on_key_release)
        self.canvas.bind("<ButtonPress-1>", self._on_mouse_drag)
        self.canvas.bind("<B1-Motion>", self._on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_mouse_release)
        self.root.focus_set()

    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key in ("w", "a", "s", "d", "up", "down", "left", "right"):
            self.pressed_keys.add(key)
            return
        if key in ("x", "space"):
            self._stop_base()
        elif key == "r":
            self.estop = True
            self._stop_base()
            self.status_var.set("Emergency stop enabled")
        elif key == "u":
            self.estop = False
            self.status_var.set("Emergency stop cleared")
        elif key == "1":
            self._send_ready()
        elif key == "2":
            self._send_grasp()
        elif key == "o":
            self._open_gripper()
        elif key == "c":
            self._close_gripper()

    def _on_key_release(self, event):
        self.pressed_keys.discard(event.keysym.lower())

    def _on_mouse_drag(self, event):
        if self.estop:
            return
        dx = float(event.x - self.pad_center)
        dy = float(event.y - self.pad_center)
        distance = math.hypot(dx, dy)
        if distance > self.pad_radius:
            scale = self.pad_radius / distance
            dx *= scale
            dy *= scale
        normalized_x = dx / self.pad_radius
        normalized_y = dy / self.pad_radius
        if abs(normalized_x) < self.mouse_deadzone:
            normalized_x = 0.0
        if abs(normalized_y) < self.mouse_deadzone:
            normalized_y = 0.0
        self.mouse_active = True
        self.mouse_linear = -normalized_y * self.max_linear_speed
        self.mouse_angular = -normalized_x * self.max_angular_speed
        self._move_knob(dx, dy)

    def _on_mouse_release(self, _event):
        self.mouse_active = False
        self.mouse_linear = 0.0
        self.mouse_angular = 0.0
        self._move_knob(0.0, 0.0)

    def _move_knob(self, dx, dy):
        c = self.pad_center
        self.canvas.coords(self.knob, c + dx - 10, c + dy - 10, c + dx + 10, c + dy + 10)

    def _keyboard_velocity(self):
        linear = 0.0
        angular = 0.0
        if "w" in self.pressed_keys or "up" in self.pressed_keys:
            linear += self.max_linear_speed
        if "s" in self.pressed_keys or "down" in self.pressed_keys:
            linear -= self.max_linear_speed
        if "a" in self.pressed_keys or "left" in self.pressed_keys:
            angular += self.max_angular_speed
        if "d" in self.pressed_keys or "right" in self.pressed_keys:
            angular -= self.max_angular_speed
        return linear, angular

    def _target_velocity(self):
        if self.estop:
            return 0.0, 0.0
        if self.mouse_active:
            return self.mouse_linear, self.mouse_angular
        return self._keyboard_velocity()

    def _publish_tick(self):
        if not self.running or rospy.is_shutdown():
            self._request_close()
            return

        linear, angular = self._target_velocity()
        self.current_linear = linear
        self.current_angular = angular

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        self._publish_body_pose(linear, angular)
        self._update_status()

        delay_ms = max(1, int(1000.0 / self.publish_rate))
        self.root.after(delay_ms, self._publish_tick)

    def _publish_body_pose(self, linear, angular):
        if not self.publish_body_pose:
            return
        moving = abs(linear) > 1e-6 or abs(angular) > 1e-6
        msg = BodyPose()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "key_mouse"
        msg.state = 2 if self.estop else (1 if moving else 0)
        msg.left_arm_angle = 0.0
        msg.right_arm_angle = 0.0
        msg.left_arm_up = 0
        msg.right_arm_up = 0
        msg.body_center_x = 0.5
        msg.body_center_y = 0.5
        msg.num_people = 1
        msg.use_course_fields = 1
        msg.course_base_motion_enabled = 1 if moving else 0
        msg.left_hand_two_detected = 1 if moving else 0
        msg.palms_together = 0 if moving else 1
        msg.right_palm_open_score = self.right_palm_open_score
        msg.avg_elbow_bend_deg = 0.0
        msg.leg_forward_norm = (
            linear / self.max_linear_speed if self.max_linear_speed > 0.0 else 0.0
        )
        msg.leg_lateral_norm = (
            -angular / self.max_angular_speed if self.max_angular_speed > 0.0 else 0.0
        )
        self.body_pose_pub.publish(msg)

    def _update_status(self):
        self.vel_var.set(
            "linear {:+.2f} m/s   angular {:+.2f} rad/s".format(
                self.current_linear, self.current_angular
            )
        )
        self.estop_var.set("E-STOP: {}".format("ON" if self.estop else "OFF"))
        if self.estop:
            self.status_var.set("Emergency stop is active. Press U to clear.")
        elif self.mouse_active:
            self.status_var.set("Mouse velocity pad active")
        elif self.pressed_keys:
            self.status_var.set("Keyboard control active")
        else:
            self.status_var.set("Ready")

    def _stop_base(self):
        self._publish_stop_only()
        self._move_knob(0.0, 0.0)

    def _publish_stop_only(self):
        self.pressed_keys.clear()
        self.mouse_active = False
        self.mouse_linear = 0.0
        self.mouse_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.cmd_vel_pub.publish(Twist())

    def _set_status(self, text):
        try:
            self.status_var.set(text)
        except tk.TclError:
            pass

    def _send_arm_pose(self, positions, label, update_status=True):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time(0)
        msg.joint_names = list(self.arm_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]
        point.velocities = [0.0] * len(self.arm_joint_names)
        point.accelerations = [0.0] * len(self.arm_joint_names)
        point.time_from_start = rospy.Duration(self.arm_traj_time)
        msg.points.append(point)
        self.arm_pub.publish(msg)
        if update_status:
            self._set_status("Arm {}: {}".format(label, [round(v, 2) for v in positions]))
        rospy.loginfo("[key_mouse_teleop] arm %s: %s", label, point.positions)

    def _send_gripper(self, position, label, update_status=True):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time(0)
        msg.joint_names = list(self.gripper_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(position)] * len(self.gripper_joint_names)
        point.velocities = [0.0] * len(self.gripper_joint_names)
        point.accelerations = [0.0] * len(self.gripper_joint_names)
        point.time_from_start = rospy.Duration(self.gripper_traj_time)
        msg.points.append(point)
        self.gripper_pub.publish(msg)
        if update_status:
            self._set_status("Gripper {}".format(label))
        rospy.loginfo("[key_mouse_teleop] gripper %s: %.3f", label, position)

    def _send_ready(self):
        self._send_arm_pose(self.ready_pose, "ready")

    def _send_grasp(self):
        if not self.estop:
            self._send_arm_pose(self.grasp_pose, "grasp")

    def _send_initial_arm_pose(self):
        if self.initial_arm_pose in ("", "none", "off", "false") or not self.running:
            return
        if self.arm_pub.get_num_connections() == 0 and self.initial_arm_retry_count < 20:
            self.initial_arm_retry_count += 1
            self.root.after(500, self._send_initial_arm_pose)
            return

        if self.initial_arm_pose == "grasp":
            self._send_arm_pose(self.grasp_pose, "initial grasp", update_status=False)
        else:
            self._send_arm_pose(self.ready_pose, "initial ready", update_status=False)

    def _open_gripper(self):
        if not self.estop:
            self.right_palm_open_score = 1.0
            self._send_gripper(self.gripper_open_pos, "open")

    def _close_gripper(self):
        if not self.estop:
            self.right_palm_open_score = 0.0
            self._send_gripper(self.gripper_close_pos, "close")

    def _request_close(self):
        self.running = False
        self._shutdown()
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def _shutdown(self):
        if self.shutdown_done:
            return
        self.shutdown_done = True
        try:
            self._publish_stop_only()
            if self.retract_on_shutdown:
                self._send_arm_pose(self.ready_pose, "shutdown ready", update_status=False)
        except Exception:
            pass

    def run(self):
        self.root.after(500, self._send_initial_arm_pose)
        self.root.after(max(1, int(1000.0 / self.publish_rate)), self._publish_tick)
        self.root.mainloop()


def main():
    try:
        KeyMouseTeleop().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except tk.TclError as exc:
        rospy.logerr("[key_mouse_teleop] cannot start GUI: %s", exc)
    except ValueError as exc:
        rospy.logerr("[key_mouse_teleop] invalid configuration: %s", exc)


if __name__ == "__main__":
    main()
