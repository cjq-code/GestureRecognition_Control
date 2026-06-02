#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hand_action_demo.py - standalone hand-action recognition demo.

It does not decide robot modes and does not publish motion commands. It only:
- detects whether left/right palms are open,
- reports each wrist position relative to its shoulder origin,
- publishes a debug image and a compact JSON status string.
"""
import json
import os

import cv2
import mediapipe as mp
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _clip(value, low=-1.0, high=1.0):
    return float(np.clip(float(value), low, high))


def _dist2(a, b):
    return float(np.hypot(a.x - b.x, a.y - b.y))


def _finger_extended(lm, tip_i, pip_i):
    return lm[tip_i].y < lm[pip_i].y


def _palm_open_score(lm):
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


def _direction_label(x_norm, y_norm, threshold):
    x = 0 if abs(x_norm) < threshold else (1 if x_norm > 0 else -1)
    y = 0 if abs(y_norm) < threshold else (1 if y_norm > 0 else -1)
    if x == 0 and y == 0:
        return "CENTER"
    if y > 0 and x == 0:
        return "UP"
    if y < 0 and x == 0:
        return "DOWN"
    if x < 0 and y == 0:
        return "LEFT"
    if x > 0 and y == 0:
        return "RIGHT"
    if y > 0 and x < 0:
        return "UP_LEFT"
    if y > 0 and x > 0:
        return "UP_RIGHT"
    if y < 0 and x < 0:
        return "DOWN_LEFT"
    return "DOWN_RIGHT"


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
    return os.path.abspath(os.path.join(os.path.expanduser("~/catkin_ws"), expanded))


def _probe_v4l2_camera_index(max_index=15):
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


class HandActionDemo:
    LEFT_SHOULDER = 11
    RIGHT_SHOULDER = 12
    LEFT_HIP = 23
    RIGHT_HIP = 24

    def __init__(self):
        rospy.init_node("hand_action_demo", anonymous=True)

        self.camera_index = int(rospy.get_param("~camera_index", 0))
        self.video_path = _resolve_input_path(rospy.get_param("~video_path", ""))
        self.loop_video = rospy.get_param("~loop_video", True)
        self.mirror_image = rospy.get_param("~mirror_image", False)
        self.show_debug = rospy.get_param("~show_debug", True)
        self.image_width = int(rospy.get_param("~image_width", 640))
        self.image_height = int(rospy.get_param("~image_height", 480))
        self.publish_rate = float(rospy.get_param("~publish_rate", 30))
        self.open_threshold = float(rospy.get_param("~open_threshold", 0.75))
        self.axis_deadzone = float(rospy.get_param("~axis_deadzone", 0.18))
        self.axis_full_scale = float(rospy.get_param("~axis_full_scale", 0.95))
        self.direction_threshold = float(rospy.get_param("~direction_threshold", 0.20))
        self.status_topic = rospy.get_param("~status_topic", "/hand_action_demo/status")
        self.debug_image_topic = rospy.get_param("~debug_image_topic", "/hand_action_demo/debug_image")

        self.bridge = CvBridge()
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.debug_image_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)

        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5,
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_styles = mp.solutions.drawing_styles

        self.cap = None
        self.source_label = ""
        self._open_input()

        rospy.loginfo("[hand_action_demo] ready")
        rospy.loginfo("  - source: %s", self.source_label)
        rospy.loginfo("  - status_topic: %s", self.status_topic)
        rospy.loginfo("  - debug_image_topic: %s", self.debug_image_topic)

    def _open_input(self):
        if self.video_path:
            if not os.path.exists(self.video_path):
                raise RuntimeError("Video file does not exist: {}".format(self.video_path))
            self.cap = cv2.VideoCapture(self.video_path)
            if not self.cap.isOpened():
                raise RuntimeError("Cannot open video file: {}".format(self.video_path))
            self.source_label = "video {}".format(self.video_path)
            return

        if self.camera_index < 0:
            found = _probe_v4l2_camera_index()
            if found is None:
                raise RuntimeError("No usable camera found")
            self.camera_index = found

        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera index {}".format(self.camera_index))
        self.source_label = "camera index {}".format(self.camera_index)

    def _hand_status(self, name, hand_landmarks, wrist, origin, torso_scale):
        if hand_landmarks:
            score = _palm_open_score(hand_landmarks.landmark)
            palm_open = score >= self.open_threshold
        else:
            score = -1.0
            palm_open = False

        # Use the user's perspective for horizontal direction. With a
        # front-facing camera, image x is mirrored relative to the user.
        raw_x = (origin.x - wrist.x) / torso_scale
        raw_y = (origin.y - wrist.y) / torso_scale
        x_norm = _axis_with_deadzone(raw_x, self.axis_deadzone, self.axis_full_scale)
        y_norm = _axis_with_deadzone(raw_y, self.axis_deadzone, self.axis_full_scale)
        direction = _direction_label(x_norm, y_norm, self.direction_threshold)

        return {
            "hand": name,
            "detected": bool(hand_landmarks),
            "palm_open": bool(palm_open),
            "open_score": round(float(max(score, 0.0)), 3),
            "x_norm": round(float(x_norm), 3),
            "y_norm": round(float(y_norm), 3),
            "direction": direction,
        }

    def _empty_status(self):
        return {
            "stamp": rospy.Time.now().to_sec(),
            "num_people": 0,
            "left": {
                "hand": "left",
                "detected": False,
                "palm_open": False,
                "open_score": 0.0,
                "x_norm": 0.0,
                "y_norm": 0.0,
                "direction": "UNKNOWN",
            },
            "right": {
                "hand": "right",
                "detected": False,
                "palm_open": False,
                "open_score": 0.0,
                "x_norm": 0.0,
                "y_norm": 0.0,
                "direction": "UNKNOWN",
            },
        }

    def _recognize(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        res = self.holistic.process(rgb)
        rgb.flags.writeable = True

        status = self._empty_status()
        if not res.pose_landmarks:
            return res, status

        plm = res.pose_landmarks.landmark
        ls = plm[self.LEFT_SHOULDER]
        rs = plm[self.RIGHT_SHOULDER]
        lh = plm[self.LEFT_HIP]
        rh = plm[self.RIGHT_HIP]
        shoulder_width = max(_dist2(ls, rs), 0.18)
        torso_height = max(abs(((lh.y + rh.y) * 0.5) - ((ls.y + rs.y) * 0.5)), 0.20)
        torso_scale = max(shoulder_width, 0.75 * torso_height, 0.18)

        status["num_people"] = 1
        status["left"] = self._hand_status("left", res.left_hand_landmarks, plm[15], ls, torso_scale)
        status["right"] = self._hand_status("right", res.right_hand_landmarks, plm[16], rs, torso_scale)
        return res, status

    def _draw(self, frame, res, status):
        if res.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                frame,
                res.pose_landmarks,
                self.mp_holistic.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_styles.get_default_pose_landmarks_style(),
            )
        if res.left_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                frame, res.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS
            )
        if res.right_hand_landmarks:
            self.mp_drawing.draw_landmarks(
                frame, res.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS
            )

        lines = [
            "LEFT  open={} score={:.2f} dir={} x={:+.2f} y={:+.2f}".format(
                status["left"]["palm_open"],
                status["left"]["open_score"],
                status["left"]["direction"],
                status["left"]["x_norm"],
                status["left"]["y_norm"],
            ),
            "RIGHT open={} score={:.2f} dir={} x={:+.2f} y={:+.2f}".format(
                status["right"]["palm_open"],
                status["right"]["open_score"],
                status["right"]["direction"],
                status["right"]["x_norm"],
                status["right"]["y_norm"],
            ),
            "origin: left/right shoulder | source: {}".format(self.source_label),
        ]
        for i, line in enumerate(lines):
            y = 28 + i * 28
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, cv2.LINE_AA)
        return frame

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            ok, frame = self.cap.read()
            if not ok:
                if self.video_path and self.loop_video:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                break
            if self.mirror_image:
                frame = cv2.flip(frame, 1)

            res, status = self._recognize(frame)
            self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

            debug = self._draw(frame, res, status)
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))
            if self.show_debug:
                cv2.imshow("hand_action_demo", debug)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            rate.sleep()

        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        HandActionDemo().run()
    except rospy.ROSInterruptException:
        pass
