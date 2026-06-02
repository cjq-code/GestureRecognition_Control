#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
body_pose_estimation.py - MediaPipe Holistic body-action recognition.

Implements the control scheme documented in the workspace README:
  - left hand down: standby
  - left hand open near shoulder: chassis/base mode
  - left fist near shoulder: manipulator mode
  - hands together or crossed at chest: safety lock
  - right hand: air joystick / manipulator intent
"""
import os
from datetime import datetime

import cv2
import mediapipe as mp
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from handcontrol.msg import BodyPose


MODE_STANDBY = 0
MODE_BASE = 1
MODE_ARM = 2
MODE_SAFETY = 3

MODE_LABELS = {
    MODE_STANDBY: "STANDBY",
    MODE_BASE: "BASE",
    MODE_ARM: "ARM",
    MODE_SAFETY: "SAFETY",
}

MODE_CN = {
    MODE_STANDBY: "待机",
    MODE_BASE: "底盘模式",
    MODE_ARM: "机械臂模式",
    MODE_SAFETY: "安全锁定",
}


def _probe_v4l2_camera_index(max_index=15):
    """Return the first available V4L2 camera index, or None."""
    for i in range(max_index + 1):
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        try:
            if cap.isOpened():
                return i
        finally:
            cap.release()

        cap = cv2.VideoCapture(i)
        try:
            if cap.isOpened():
                return i
        finally:
            cap.release()
    return None


def _clip(value, low=-1.0, high=1.0):
    return float(np.clip(float(value), low, high))


def _dist2(a, b):
    return float(np.hypot(a.x - b.x, a.y - b.y))


def _angle_deg_at_bend(a, b, c):
    """Angle at point b between ba and bc. About 180 deg means straight."""
    ba = np.array([a.x - b.x, a.y - b.y], dtype=np.float64)
    bc = np.array([c.x - b.x, c.y - b.y], dtype=np.float64)
    n1 = np.linalg.norm(ba)
    n2 = np.linalg.norm(bc)
    if n1 < 1e-9 or n2 < 1e-9:
        return 180.0
    cosv = float(np.clip(np.dot(ba, bc) / (n1 * n2), -1.0, 1.0))
    return float(np.degrees(np.arccos(cosv)))


def _finger_extended(lm, tip_i, pip_i):
    """Heuristic for a non-thumb finger extended upward in the image."""
    return lm[tip_i].y < lm[pip_i].y


def _palm_open_score(lm):
    """0..1: ratio of extended index/middle/ring/pinky fingers."""
    fingers = [
        _finger_extended(lm, 8, 6),
        _finger_extended(lm, 12, 10),
        _finger_extended(lm, 16, 14),
        _finger_extended(lm, 20, 18),
    ]
    return float(sum(1 for f in fingers if f)) / 4.0


def _axis_with_deadzone(value, deadzone, full_scale):
    value = float(value)
    sign = 1.0 if value >= 0.0 else -1.0
    mag = abs(value)
    if mag <= deadzone:
        return 0.0
    if full_scale <= deadzone:
        return sign
    return _clip(sign * ((mag - deadzone) / (full_scale - deadzone)))


def _landmark_visible(lm, threshold=0.45):
    return not hasattr(lm, "visibility") or lm.visibility >= threshold


def _resolve_input_path(raw_path):
    raw_path = str(raw_path).strip()
    if not raw_path:
        return ""
    expanded = os.path.expanduser(raw_path)
    if os.path.isabs(expanded):
        return os.path.abspath(expanded)

    cwd_path = os.path.abspath(expanded)
    if os.path.exists(cwd_path):
        return cwd_path

    ws_path = os.path.abspath(os.path.join(os.path.expanduser("~/catkin_ws"), expanded))
    if os.path.exists(ws_path):
        return ws_path

    return ws_path


class BodyPoseEstimation:
    def __init__(self):
        rospy.init_node("body_pose_estimation", anonymous=True)

        raw_camera = int(rospy.get_param("~camera_index", 0))
        self.video_path = _resolve_input_path(rospy.get_param("~video_path", ""))
        self.loop_video = rospy.get_param("~loop_video", True)
        self.mirror_image = rospy.get_param("~mirror_image", False)
        self.write_report = rospy.get_param("~write_report", False)
        raw_report_path = str(rospy.get_param("~report_path", "")).strip()
        self.report_path = self._resolve_report_path(raw_report_path)
        self.report_sample_interval = float(rospy.get_param("~report_sample_interval", 0.0))

        self.use_ros_topic = rospy.get_param("~use_ros_topic", False)
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.image_width = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 480)
        self.publish_rate = rospy.get_param("~publish_rate", 30)
        self.show_debug = rospy.get_param("~show_debug", True)
        self.use_course_mapping = rospy.get_param("~use_course_mapping", True)
        self.min_detection_confidence = rospy.get_param("~min_detection_confidence", 0.6)
        self.min_tracking_confidence = rospy.get_param("~min_tracking_confidence", 0.5)

        self.open_threshold = rospy.get_param("~open_threshold", 0.75)
        self.fist_threshold = rospy.get_param("~fist_threshold", 0.25)
        self.mode_stable_frames = int(rospy.get_param("~mode_stable_frames", 3))
        self.axis_deadzone = rospy.get_param("~right_hand_deadzone", 0.18)
        self.axis_full_scale = rospy.get_param("~right_hand_full_scale", 0.95)
        self.palms_together_max = rospy.get_param("~palms_together_max_dist", 0.55)

        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.body_pose_pub = rospy.Publisher("/body_pose", BodyPose, queue_size=10)
        self.debug_image_pub = rospy.Publisher("/body_pose/debug_image", Image, queue_size=1)
        self.bridge = CvBridge()

        self.current_frame = None
        self.cap = None
        self.source_label = ""
        self._video_fps = 0.0
        self._frame_index = 0
        self._last_frame_time = 0.0
        self._start_time = rospy.Time.now()
        self._report_samples = []
        self._last_report_sample_time = None
        self._report_written = False
        if self.use_ros_topic:
            rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
            self.source_label = "ROS topic {}".format(self.image_topic)
        elif self.video_path:
            self._init_video()
        else:
            if raw_camera < 0:
                found = _probe_v4l2_camera_index()
                if found is None:
                    raise RuntimeError(
                        "camera_index<0 means auto-detect, but no usable V4L2 camera was found. "
                        "Run: rosrun handcontrol list_usb_cameras.py"
                    )
                self.camera_index = found
                rospy.logwarn(
                    "[body_pose_estimation] auto-selected camera index %s", self.camera_index
                )
            else:
                self.camera_index = int(raw_camera)
            self._init_camera()

        self.LEFT_SHOULDER, self.LEFT_ELBOW, self.LEFT_WRIST = 11, 13, 15
        self.RIGHT_SHOULDER, self.RIGHT_ELBOW, self.RIGHT_WRIST = 12, 14, 16
        self.LEFT_HIP, self.RIGHT_HIP = 23, 24

        self._stable_mode = MODE_STANDBY
        self._candidate_mode = MODE_STANDBY
        self._candidate_count = 0
        self._active_control_mode = None
        self._mode_switch_unlocked = True

        rospy.loginfo("[body_pose_estimation] Holistic action recognizer ready")
        rospy.loginfo("  - image source: %s", self.source_label)
        rospy.loginfo("  - show_debug: %s", self.show_debug)
        rospy.loginfo("  - use_course_mapping: %s", self.use_course_mapping)
        if self.report_path:
            rospy.loginfo("  - report_path: %s", self.report_path)

    def _resolve_report_path(self, raw_report_path):
        if raw_report_path:
            lowered = raw_report_path.lower()
            if lowered in ("auto", "true", "1"):
                return self._default_report_path()
            return os.path.abspath(os.path.expanduser(raw_report_path))
        if self.write_report:
            return self._default_report_path()
        return ""

    def _default_report_path(self):
        if self.video_path:
            base, _ext = os.path.splitext(self.video_path)
            return base + "_recognition.md"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.abspath(os.path.expanduser("~/catkin_ws/pose_test_videos/body_pose_report_{}.md".format(stamp)))

    def _init_camera(self):
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera index {}".format(self.camera_index))
        self.source_label = "V4L2 index {}".format(self.camera_index)
        rospy.loginfo("[body_pose_estimation] camera opened: %s", self.source_label)

    def _init_video(self):
        if not os.path.exists(self.video_path):
            raise RuntimeError("Video file does not exist: {}".format(self.video_path))
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open video file: {}".format(self.video_path))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self._video_fps = float(fps) if fps and fps > 0.1 else 0.0
        if fps and fps > 1.0 and not rospy.has_param("~publish_rate"):
            self.publish_rate = min(60.0, float(fps))
        self.source_label = "video {}".format(self.video_path)
        rospy.loginfo("[body_pose_estimation] video opened: %s", self.video_path)

    def _image_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logerr("image conversion failed: %s", exc)

    def _calculate_arm_angle(self, shoulder, elbow, wrist):
        upper = np.array([elbow.x - shoulder.x, elbow.y - shoulder.y])
        if np.linalg.norm(upper) < 1e-6:
            return 0.0, False
        angle_se = float(np.degrees(np.arctan2(-(elbow.y - shoulder.y), elbow.x - shoulder.x)))
        is_arm_up = wrist.y < elbow.y
        return angle_se, is_arm_up

    def _new_msg(self):
        msg = BodyPose()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera"
        msg.state = 0
        msg.left_arm_angle = 0.0
        msg.right_arm_angle = 0.0
        msg.left_arm_up = 0
        msg.right_arm_up = 0
        msg.body_center_x = 0.5
        msg.body_center_y = 0.5
        msg.num_people = 0
        msg.use_course_fields = 1 if self.use_course_mapping else 0
        msg.course_base_motion_enabled = 0
        msg.left_hand_two_detected = 0
        msg.palms_together = 0
        msg.right_palm_open_score = 0.0
        msg.avg_elbow_bend_deg = 0.0
        msg.leg_forward_norm = 0.0
        msg.leg_lateral_norm = 0.0
        msg.control_mode = MODE_STANDBY
        msg.mode_label = MODE_LABELS[MODE_STANDBY]
        msg.action_label = "STOP"
        msg.left_palm_open = 0
        msg.left_fist = 0
        msg.right_palm_open = 0
        msg.right_fist = 0
        msg.right_hand_x_norm = 0.0
        msg.right_hand_y_norm = 0.0
        msg.right_hand_depth_norm = 0.0
        return msg

    def _stabilize_mode(self, raw_mode):
        if raw_mode in (MODE_STANDBY, MODE_SAFETY):
            self._stable_mode = raw_mode
            self._candidate_mode = raw_mode
            self._candidate_count = 0
            return raw_mode

        if raw_mode != self._candidate_mode:
            self._candidate_mode = raw_mode
            self._candidate_count = 1
        else:
            self._candidate_count += 1

        if self._candidate_count >= self.mode_stable_frames:
            self._stable_mode = raw_mode
        return self._stable_mode

    def _apply_mode_switch_gate(self, raw_mode):
        if raw_mode == MODE_SAFETY:
            self._mode_switch_unlocked = True
            return MODE_SAFETY
        if raw_mode not in (MODE_BASE, MODE_ARM):
            return raw_mode
        if self._active_control_mode is None:
            return raw_mode
        if raw_mode == self._active_control_mode:
            return raw_mode
        if self._mode_switch_unlocked:
            return raw_mode
        return MODE_STANDBY

    def _detect_safety(self, plm, torso_scale, shoulder_mid_y, hip_mid_y):
        lw = plm[self.LEFT_WRIST]
        rw = plm[self.RIGHT_WRIST]
        ls = plm[self.LEFT_SHOULDER]
        rs = plm[self.RIGHT_SHOULDER]

        upper = shoulder_mid_y - 0.20 * torso_scale
        lower = hip_mid_y + 0.20 * torso_scale
        hands_near_chest = upper <= lw.y <= lower and upper <= rw.y <= lower
        if not hands_near_chest:
            return False, False, False

        wrist_dist = _dist2(lw, rw) / torso_scale
        palms_together = wrist_dist <= self.palms_together_max

        left_to_right_shoulder = _dist2(lw, rs) / torso_scale
        right_to_left_shoulder = _dist2(rw, ls) / torso_scale
        wrists_crossed = left_to_right_shoulder < 0.95 and right_to_left_shoulder < 0.95

        return palms_together or wrists_crossed, palms_together, wrists_crossed

    def _right_hand_axes(self, plm, torso_scale, shoulder_mid, hip_mid):
        rw = plm[self.RIGHT_WRIST]
        rs = plm[self.RIGHT_SHOULDER]
        re = plm[self.RIGHT_ELBOW]

        zero_x = rs.x
        zero_y = rs.y
        raw_x = (rw.x - zero_x) / torso_scale
        raw_y = (zero_y - rw.y) / torso_scale
        x_norm = _axis_with_deadzone(raw_x, self.axis_deadzone, self.axis_full_scale)
        y_norm = _axis_with_deadzone(raw_y, self.axis_deadzone, self.axis_full_scale)

        elbow_angle = _angle_deg_at_bend(rs, re, rw)
        elbow_straight = _clip((elbow_angle - 75.0) / 85.0, 0.0, 1.0)
        shoulder_wrist = _clip((_dist2(rs, rw) / torso_scale - 0.45) / 0.85, 0.0, 1.0)
        extension = _clip(0.55 * elbow_straight + 0.45 * shoulder_wrist, 0.0, 1.0)
        depth_norm = _clip(extension * 2.0 - 1.0)
        return x_norm, y_norm, depth_norm, elbow_angle

    def _base_action_label(self, x_norm, y_norm):
        parts = []
        if y_norm > 0.12:
            parts.append("FORWARD")
        elif y_norm < -0.12:
            parts.append("BACK")
        if x_norm < -0.12:
            parts.append("TURN_LEFT")
        elif x_norm > 0.12:
            parts.append("TURN_RIGHT")
        return "+".join(parts) if parts else "STOP"

    def _arm_action_label(self, msg):
        parts = []
        if msg.right_hand_y_norm > 0.18:
            parts.append("READY")
        elif msg.right_hand_y_norm < -0.18:
            parts.append("GRASP")
        if msg.right_hand_x_norm < -0.18:
            parts.append("BASE_LEFT")
        elif msg.right_hand_x_norm > 0.18:
            parts.append("BASE_RIGHT")
        if msg.right_hand_depth_norm > 0.28:
            parts.append("EXTEND")
        elif msg.right_hand_depth_norm < -0.28:
            parts.append("RETRACT")
        if msg.right_fist:
            parts.append("GRIP_CLOSE")
        elif msg.right_palm_open:
            parts.append("GRIP_OPEN")
        return "+".join(parts) if parts else "HOLD"

    def _fill_pose_fields(self, msg, res):
        plm = res.pose_landmarks.landmark
        msg.num_people = 1

        ls = plm[self.LEFT_SHOULDER]
        le = plm[self.LEFT_ELBOW]
        lw = plm[self.LEFT_WRIST]
        rs = plm[self.RIGHT_SHOULDER]
        re = plm[self.RIGHT_ELBOW]
        rw = plm[self.RIGHT_WRIST]
        lh = plm[self.LEFT_HIP]
        rh = plm[self.RIGHT_HIP]

        left_angle, left_up = self._calculate_arm_angle(ls, le, lw)
        right_angle, right_up = self._calculate_arm_angle(rs, re, rw)
        msg.left_arm_angle = float(left_angle)
        msg.right_arm_angle = float(right_angle)
        msg.left_arm_up = 1 if left_up else 0
        msg.right_arm_up = 1 if right_up else 0

        shoulder_mid = ((ls.x + rs.x) * 0.5, (ls.y + rs.y) * 0.5)
        hip_mid = ((lh.x + rh.x) * 0.5, (lh.y + rh.y) * 0.5)
        shoulder_width = max(_dist2(ls, rs), 0.18)
        torso_height = max(abs(hip_mid[1] - shoulder_mid[1]), 0.20)
        torso_scale = max(shoulder_width, 0.75 * torso_height, 0.18)
        msg.body_center_x = float(shoulder_mid[0])
        msg.body_center_y = float(shoulder_mid[1] + 0.35 * torso_height)

        left_score = _palm_open_score(res.left_hand_landmarks.landmark) if res.left_hand_landmarks else -1.0
        right_score = _palm_open_score(res.right_hand_landmarks.landmark) if res.right_hand_landmarks else -1.0
        msg.left_palm_open = 1 if left_score >= self.open_threshold else 0
        msg.left_fist = 1 if 0.0 <= left_score <= self.fist_threshold else 0
        msg.right_palm_open = 1 if right_score >= self.open_threshold else 0
        msg.right_fist = 1 if 0.0 <= right_score <= self.fist_threshold else 0
        msg.right_palm_open_score = float(np.clip(right_score if right_score >= 0.0 else 0.0, 0.0, 1.0))

        right_x, right_y, right_depth, right_elbow_angle = self._right_hand_axes(
            plm, torso_scale, shoulder_mid, hip_mid
        )
        msg.right_hand_x_norm = right_x
        msg.right_hand_y_norm = right_y
        msg.right_hand_depth_norm = right_depth
        msg.avg_elbow_bend_deg = float(max(0.0, 180.0 - right_elbow_angle))

        safety, palms_together, crossed = self._detect_safety(
            plm, torso_scale, shoulder_mid[1], hip_mid[1]
        )
        msg.palms_together = 1 if palms_together or crossed else 0

        left_down = lw.y > hip_mid[1] - 0.05 * torso_height
        left_raised_near_shoulder = (
            _landmark_visible(lw)
            and lw.y <= ls.y + 0.45 * torso_height
            and _dist2(lw, ls) <= 1.45 * torso_scale
        )

        if safety:
            raw_mode = MODE_SAFETY
        elif left_down:
            raw_mode = MODE_STANDBY
        elif left_raised_near_shoulder and msg.left_palm_open:
            raw_mode = MODE_BASE
        elif left_raised_near_shoulder and msg.left_fist:
            raw_mode = MODE_ARM
        else:
            raw_mode = MODE_STANDBY

        mode = self._stabilize_mode(self._apply_mode_switch_gate(raw_mode))
        if mode in (MODE_BASE, MODE_ARM):
            self._active_control_mode = mode
            self._mode_switch_unlocked = False
        msg.control_mode = mode
        msg.mode_label = MODE_LABELS[mode]
        msg.state = 2 if mode == MODE_SAFETY else (1 if mode in (MODE_BASE, MODE_ARM) else 0)
        msg.course_base_motion_enabled = 1 if mode == MODE_BASE else 0

        # Legacy course fields now mirror the right-hand joystick so older
        # tooling still shows useful values.
        msg.leg_forward_norm = msg.right_hand_y_norm if mode == MODE_BASE else 0.0
        msg.leg_lateral_norm = msg.right_hand_x_norm if mode == MODE_BASE else 0.0

        if mode == MODE_SAFETY:
            msg.action_label = "STOP_LOCK"
        elif mode == MODE_BASE:
            msg.action_label = self._base_action_label(msg.right_hand_x_norm, msg.right_hand_y_norm)
        elif mode == MODE_ARM:
            msg.action_label = self._arm_action_label(msg)
        else:
            msg.action_label = "STOP"

        return palms_together, crossed, left_score, right_score

    def _draw_debug_overlay(self, frame, msg, palms_together=False, crossed=False,
                            left_score=-1.0, right_score=-1.0):
        color = (0, 255, 255)
        if msg.control_mode == MODE_SAFETY:
            color = (0, 0, 255)
        elif msg.control_mode == MODE_BASE:
            color = (0, 220, 0)
        elif msg.control_mode == MODE_ARM:
            color = (255, 180, 0)

        lines = [
            "MODE: {} | ACTION: {}".format(msg.mode_label, msg.action_label),
            "R x/y/depth: {:+.2f} {:+.2f} {:+.2f}".format(
                msg.right_hand_x_norm, msg.right_hand_y_norm, msg.right_hand_depth_norm
            ),
            "L open/fist: {}/{} ({:.2f}) | R open/fist: {}/{} ({:.2f})".format(
                msg.left_palm_open,
                msg.left_fist,
                max(left_score, 0.0),
                msg.right_palm_open,
                msg.right_fist,
                max(right_score, 0.0),
            ),
            "safety palms/cross: {}/{} | source: {}".format(
                1 if palms_together else 0,
                1 if crossed else 0,
                self.source_label,
            ),
        ]

        pad = 8
        line_h = 24
        box_h = pad * 2 + line_h * len(lines)
        cv2.rectangle(frame, (0, 0), (frame.shape[1], box_h), (0, 0, 0), -1)
        for i, line in enumerate(lines):
            cv2.putText(
                frame,
                line,
                (pad, pad + 18 + i * line_h),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color if i == 0 else (230, 230, 230),
                1 if i else 2,
                cv2.LINE_AA,
            )

    def _process_frame(self, frame):
        msg = self._new_msg()
        if self.mirror_image:
            frame = cv2.flip(frame, 1)

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.holistic.process(frame_rgb)
        debug_frame = frame.copy()

        palms_together = False
        crossed = False
        left_score = -1.0
        right_score = -1.0

        if res.pose_landmarks:
            palms_together, crossed, left_score, right_score = self._fill_pose_fields(msg, res)

            if self.show_debug:
                self.mp_drawing.draw_landmarks(
                    debug_frame,
                    res.pose_landmarks,
                    self.mp_holistic.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style(),
                )
                if res.left_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        debug_frame,
                        res.left_hand_landmarks,
                        self.mp_holistic.HAND_CONNECTIONS,
                    )
                if res.right_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        debug_frame,
                        res.right_hand_landmarks,
                        self.mp_holistic.HAND_CONNECTIONS,
                    )
                self._draw_debug_overlay(
                    debug_frame, msg, palms_together, crossed, left_score, right_score
                )
        else:
            self._stable_mode = MODE_STANDBY
            self._candidate_mode = MODE_STANDBY
            self._candidate_count = 0
            if self.show_debug:
                cv2.rectangle(debug_frame, (0, 0), (debug_frame.shape[1], 58), (0, 0, 0), -1)
                cv2.putText(
                    debug_frame,
                    "NO PERSON | MODE: STANDBY",
                    (12, 38),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

        return msg, debug_frame

    def _read_frame(self):
        if self.use_ros_topic:
            self._last_frame_time = max(0.0, (rospy.Time.now() - self._start_time).to_sec())
            return True, self.current_frame

        ok, frame = self.cap.read()
        if ok and frame is not None:
            if self.video_path and self._video_fps > 0.0:
                self._last_frame_time = self._frame_index / self._video_fps
                self._frame_index += 1
            else:
                self._last_frame_time = max(0.0, (rospy.Time.now() - self._start_time).to_sec())
            return True, frame

        if self.video_path:
            if self.loop_video:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self._frame_index = 0
                ok, frame = self.cap.read()
                if ok and frame is not None:
                    self._last_frame_time = 0.0
                    self._frame_index = 1
                return ok, frame
            rospy.loginfo("[body_pose_estimation] video finished")
            rospy.signal_shutdown("video finished")
            return False, None

        rospy.logwarn_throttle(5, "[body_pose_estimation] camera read failed")
        return False, None

    def _control_logic_for_msg(self, msg):
        if msg.num_people <= 0:
            return "未检测到人体；输出待机结果，底盘速度归零，机械臂保持。"
        if msg.control_mode == MODE_SAFETY:
            return "双手合十或交叉触发安全锁；teleop_mapper 立即清零线速度和角速度，机械臂桥接暂停响应。"
        if msg.control_mode == MODE_BASE:
            return (
                "左手张开举在肩旁进入底盘模式；右手以右肩附近为零点作为空中摇杆，"
                "linear.x = right_hand_y_norm * max_linear_speed，"
                "左右转向超过阈值后使用固定低角速度。"
            )
        if msg.control_mode == MODE_ARM:
            return (
                "左手握拳举在肩旁进入机械臂模式；进入该模式前必须经过安全锁，"
                "右手上/下离散触发准备/抓取预设姿态，右手握拳闭合夹爪，右手张开打开夹爪。"
            )
        return "左手放下或未形成有效模式；底盘停止，机械臂保持当前位置。"

    def _record_report_sample(self, msg):
        if not self.report_path:
            return
        t = float(self._last_frame_time)
        if (
            self._last_report_sample_time is not None
            and self.report_sample_interval > 0.0
            and t - self._last_report_sample_time < self.report_sample_interval
        ):
            return
        self._last_report_sample_time = t
        self._report_samples.append(
            {
                "time": t,
                "mode": msg.mode_label,
                "mode_cn": MODE_CN.get(int(msg.control_mode), msg.mode_label),
                "action": msg.action_label,
                "logic": self._control_logic_for_msg(msg),
                "people": int(msg.num_people),
                "x": float(msg.right_hand_x_norm),
                "y": float(msg.right_hand_y_norm),
                "depth": float(msg.right_hand_depth_norm),
                "left_open": int(msg.left_palm_open),
                "left_fist": int(msg.left_fist),
                "right_open": int(msg.right_palm_open),
                "right_fist": int(msg.right_fist),
            }
        )

    def _format_time(self, seconds):
        seconds = max(0.0, float(seconds))
        minutes = int(seconds // 60)
        sec = seconds - minutes * 60
        return "{:02d}:{:05.2f}".format(minutes, sec)

    def _build_report_segments(self):
        if not self._report_samples:
            return []
        segments = []
        for sample in self._report_samples:
            key = (sample["mode"], sample["action"], sample["logic"])
            if not segments or segments[-1]["key"] != key:
                segments.append(
                    {
                        "key": key,
                        "start": sample["time"],
                        "end": sample["time"],
                        "samples": [sample],
                    }
                )
            else:
                segments[-1]["end"] = sample["time"]
                segments[-1]["samples"].append(sample)

        frame_dt = 1.0 / self._video_fps if self._video_fps > 0.0 else 0.0
        for idx, segment in enumerate(segments):
            if idx + 1 < len(segments):
                segment["end"] = segments[idx + 1]["start"]
            elif frame_dt > 0.0:
                segment["end"] += frame_dt
        return segments

    def _avg(self, samples, field):
        if not samples:
            return 0.0
        return sum(float(s[field]) for s in samples) / float(len(samples))

    def _write_report(self):
        if not self.report_path or self._report_written:
            return
        self._report_written = True
        os.makedirs(os.path.dirname(self.report_path), exist_ok=True)
        segments = self._build_report_segments()
        video_duration = 0.0
        if self.cap is not None and self.video_path:
            frame_count = self.cap.get(cv2.CAP_PROP_FRAME_COUNT)
            if self._video_fps > 0.0 and frame_count and frame_count > 0:
                video_duration = float(frame_count) / self._video_fps
        if not video_duration and self._report_samples:
            video_duration = self._report_samples[-1]["time"]

        lines = [
            "# 体感动作识别结果",
            "",
            "- 视频: `{}`".format(self.video_path or self.source_label),
            "- 生成时间: {}".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")),
            "- 视频时长: {}".format(self._format_time(video_duration)),
            "- 采样帧数: {}".format(len(self._report_samples)),
            "- 识别段数: {}".format(len(segments)),
            "",
            "## 时间节点与控制结果",
            "",
            "| 时间段 | 模式 | 动作 | 右手 x/y/depth 均值 | 手势 | 控制逻辑 |",
            "|---|---|---|---|---|---|",
        ]

        for segment in segments:
            samples = segment["samples"]
            first = samples[0]
            time_range = "{}-{}".format(
                self._format_time(segment["start"]),
                self._format_time(segment["end"]),
            )
            avg_xyz = "{:+.2f} / {:+.2f} / {:+.2f}".format(
                self._avg(samples, "x"),
                self._avg(samples, "y"),
                self._avg(samples, "depth"),
            )
            gesture = "L(open/fist)={}/{}; R(open/fist)={}/{}".format(
                first["left_open"],
                first["left_fist"],
                first["right_open"],
                first["right_fist"],
            )
            lines.append(
                "| {} | {} ({}) | `{}` | {} | {} | {} |".format(
                    time_range,
                    first["mode"],
                    first["mode_cn"],
                    first["action"],
                    avg_xyz,
                    gesture,
                    first["logic"],
                )
            )

        lines.extend(
            [
                "",
                "## 识别逻辑说明",
                "",
                "1. MediaPipe Holistic 同时检测人体姿态关键点和左右手关键点。",
                "2. 用肩膀、髋部和手腕估计躯干尺度，所有距离都除以躯干尺度，减少人与摄像头远近变化的影响。",
                "3. 左手决定模式：左手放下为 `STANDBY`；左手张开并举在肩旁为 `BASE`；左手握拳并举在肩旁为 `ARM`。",
                "4. 双手在胸前距离很近，或左右手腕交叉靠近对侧肩膀时，判定为 `SAFETY`。",
                "5. `BASE` 与 `ARM` 之间不能直接互切，必须先进入 `SAFETY` 才允许切换到另一种控制模式。",
                "6. 底盘模式下，右手相对右肩附近零点形成空中摇杆：上/下映射前进/后退，左/右超过阈值后固定低角速度转向。",
                "7. 机械臂模式下，右手上/下离散触发准备/抓取预设姿态；右手开掌/握拳控制夹爪开/合。",
                "8. 模式切换有少量稳定帧过滤，避免单帧误识别导致控制跳变。",
                "",
            ]
        )

        with open(self.report_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))
        rospy.loginfo("[body_pose_estimation] report written: %s", self.report_path)

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            ok, frame = self._read_frame()
            if not ok or frame is None:
                rate.sleep()
                continue

            msg, dbg = self._process_frame(frame)
            self.body_pose_pub.publish(msg)
            self._record_report_sample(msg)

            if self.show_debug:
                try:
                    dm = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
                    dm.header = msg.header
                    self.debug_image_pub.publish(dm)
                except Exception as exc:
                    rospy.logerr_throttle(5, "debug image publish failed: %s", exc)
                cv2.imshow("Body action recognition - Q quit", dbg)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
            rate.sleep()
        self.shutdown()

    def shutdown(self):
        self._write_report()
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        self.holistic.close()
        rospy.loginfo("[body_pose_estimation] stopped")


def main():
    try:
        BodyPoseEstimation().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
