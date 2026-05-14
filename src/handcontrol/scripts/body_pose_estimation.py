#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
body_pose_estimation.py — MediaPipe Holistic：全身 + 双手
发布 /body_pose（含课程字段）：左手「2」解锁底盘、双手并拢停止、腿部前后/侧向、肘弯、右手张开度。
"""
import rospy
import cv2
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from handcontrol.msg import BodyPose


def _probe_v4l2_camera_index(max_index: int = 15):
    """返回第一个可用 V4L2 设备索引；若无则 None。"""
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


def _angle_deg_at_bend(a, b, c):
    """在点 b 处向量 ba 与 bc 的夹角（度），约 180 为伸直。"""
    ba = np.array([a.x - b.x, a.y - b.y], dtype=np.float64)
    bc = np.array([c.x - b.x, c.y - b.y], dtype=np.float64)
    n1 = np.linalg.norm(ba)
    n2 = np.linalg.norm(bc)
    if n1 < 1e-9 or n2 < 1e-9:
        return 180.0
    cosv = float(np.clip(np.dot(ba, bc) / (n1 * n2), -1.0, 1.0))
    return float(np.degrees(np.arccos(cosv)))


def _finger_extended(lm, tip_i, pip_i):
    """MediaPipe Hand：指尖 y 小于近端关节 y 视为在图像中「向上张开」。"""
    return lm[tip_i].y < lm[pip_i].y


def _is_two_gesture(lm):
    """食指+中指伸直，无名指+小指弯曲 — 数字「2」。"""
    idx = _finger_extended(lm, 8, 6) and _finger_extended(lm, 12, 10)
    ring_down = not _finger_extended(lm, 16, 14)
    pink_down = not _finger_extended(lm, 20, 18)
    return idx and ring_down and pink_down


def _palm_open_score(lm):
    """0~1：四指伸直比例（不含拇指）。"""
    fingers = [
        _finger_extended(lm, 8, 6),
        _finger_extended(lm, 12, 10),
        _finger_extended(lm, 16, 14),
        _finger_extended(lm, 20, 18),
    ]
    return float(sum(1 for f in fingers if f)) / 4.0


def _wrist_dist(lm_l, lm_r):
    dx = lm_l[0].x - lm_r[0].x
    dy = lm_l[0].y - lm_r[0].y
    return float(np.hypot(dx, dy))


class BodyPoseEstimation:
    def __init__(self):
        rospy.init_node("body_pose_estimation", anonymous=True)

        raw_camera = rospy.get_param("~camera_index", 0)
        self.use_ros_topic = rospy.get_param("~use_ros_topic", False)
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.image_width = rospy.get_param("~image_width", 640)
        self.image_height = rospy.get_param("~image_height", 480)
        self.publish_rate = rospy.get_param("~publish_rate", 30)
        self.show_debug = rospy.get_param("~show_debug", True)
        self.use_course_mapping = rospy.get_param("~use_course_mapping", True)
        self.min_detection_confidence = rospy.get_param("~min_detection_confidence", 0.6)
        self.min_tracking_confidence = rospy.get_param("~min_tracking_confidence", 0.5)

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

        self.cap = None
        if not self.use_ros_topic:
            if raw_camera < 0:
                found = _probe_v4l2_camera_index()
                if found is None:
                    raise RuntimeError(
                        "camera_index<0 表示自动探测，但未找到任何可用 V4L2 摄像头。"
                        "请连接摄像头、检查虚拟机透传，或运行: rosrun handcontrol list_usb_cameras.py"
                    )
                self.camera_index = found
                rospy.logwarn(
                    f"[body_pose_estimation] 自动选择摄像头索引 {self.camera_index}（参数 camera_index:=-1）"
                )
            else:
                self.camera_index = int(raw_camera)
            self._init_camera()
        else:
            self.camera_index = int(raw_camera)
            rospy.Subscriber(self.image_topic, Image, self._image_callback)
            self.current_frame = None

        # Pose indices
        self.LEFT_SHOULDER, self.LEFT_ELBOW, self.LEFT_WRIST = 11, 13, 15
        self.RIGHT_SHOULDER, self.RIGHT_ELBOW, self.RIGHT_WRIST = 12, 14, 16
        self.NOSE = 0
        self.LEFT_HIP, self.RIGHT_HIP = 23, 24
        self.LEFT_KNEE, self.RIGHT_KNEE = 25, 26
        self.LEFT_ANKLE, self.RIGHT_ANKLE = 27, 28

        # 课程门闩：左手「2」稳定后解锁；双手并拢关闭
        self._two_stable = 0
        self._course_armed = False
        self._two_needed = rospy.get_param("~two_gesture_stable_frames", 10)
        self._palms_dist_max = rospy.get_param("~palms_together_max_dist", 0.16)

        rospy.loginfo("[body_pose_estimation] Holistic 节点初始化")
        rospy.loginfo(f"  - use_course_mapping: {self.use_course_mapping}")
        if self.use_ros_topic:
            src = f"ROS 话题 {self.image_topic}"
        else:
            src = f"V4L2 索引 {self.camera_index}"
        rospy.loginfo(f"  - 图像源: {src}")

    def _init_camera(self):
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap.release()
            self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头索引 {self.camera_index}")
        rospy.loginfo(f"[body_pose_estimation] 摄像头已打开 (索引: {self.camera_index})")

    def _image_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"图像转换失败: {e}")

    def _calculate_arm_angle(self, shoulder, elbow, wrist):
        upper = np.array([elbow.x - shoulder.x, elbow.y - shoulder.y])
        if np.linalg.norm(upper) < 1e-6:
            return 0.0, False
        angle_se = float(
            np.degrees(np.arctan2(-(elbow.y - shoulder.y), elbow.x - shoulder.x))
        )
        is_arm_up = wrist.y < elbow.y
        return angle_se, is_arm_up

    def _detect_state(self, landmarks, left_up, right_up):
        left_wrist = landmarks[self.LEFT_WRIST]
        right_wrist = landmarks[self.RIGHT_WRIST]
        nose = landmarks[self.NOSE]
        if left_wrist.y < nose.y and right_wrist.y < nose.y:
            return 2
        if left_up or right_up:
            return 1
        return 0

    def _process_frame(self, frame):
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
        msg.course_base_motion_enabled = 1 if self._course_armed else 0
        msg.left_hand_two_detected = 0
        msg.palms_together = 0
        msg.right_palm_open_score = 0.0
        msg.avg_elbow_bend_deg = 0.0
        msg.leg_forward_norm = 0.0
        msg.leg_lateral_norm = 0.0

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.holistic.process(frame_rgb)
        debug_frame = frame.copy()

        if res.pose_landmarks:
            plm = res.pose_landmarks.landmark
            msg.num_people = 1

            left_angle, left_up = self._calculate_arm_angle(
                plm[self.LEFT_SHOULDER], plm[self.LEFT_ELBOW], plm[self.LEFT_WRIST]
            )
            right_angle, right_up = self._calculate_arm_angle(
                plm[self.RIGHT_SHOULDER], plm[self.RIGHT_ELBOW], plm[self.RIGHT_WRIST]
            )
            msg.left_arm_angle = float(left_angle)
            msg.right_arm_angle = float(right_angle)
            msg.left_arm_up = 1 if left_up else 0
            msg.right_arm_up = 1 if right_up else 0
            cx = (plm[self.LEFT_SHOULDER].x + plm[self.RIGHT_SHOULDER].x) / 2
            cy = (plm[self.LEFT_SHOULDER].y + plm[self.LEFT_HIP].y) / 2
            msg.body_center_x = float(cx)
            msg.body_center_y = float(cy)
            msg.state = self._detect_state(plm, left_up, right_up)

            lk = _angle_deg_at_bend(plm[self.LEFT_HIP], plm[self.LEFT_KNEE], plm[self.LEFT_ANKLE])
            rk = _angle_deg_at_bend(plm[self.RIGHT_HIP], plm[self.RIGHT_KNEE], plm[self.RIGHT_ANKLE])
            left_bend = max(0.0, 175.0 - lk)
            right_bend = max(0.0, 175.0 - rk)
            msg.leg_forward_norm = float(np.clip(max(left_bend, right_bend) / 55.0, 0.0, 1.0))
            msg.leg_lateral_norm = float(np.clip((right_bend - left_bend) / 45.0, -1.0, 1.0))

            lelb = _angle_deg_at_bend(plm[self.LEFT_SHOULDER], plm[self.LEFT_ELBOW], plm[self.LEFT_WRIST])
            relb = _angle_deg_at_bend(plm[self.RIGHT_SHOULDER], plm[self.RIGHT_ELBOW], plm[self.RIGHT_WRIST])
            msg.avg_elbow_bend_deg = float(max(0.0, 180.0 - (lelb + relb) / 2.0))

            if self.use_course_mapping:
                two_now = 0
                if res.left_hand_landmarks:
                    lhl = res.left_hand_landmarks.landmark
                    if _is_two_gesture(lhl):
                        two_now = 1
                msg.left_hand_two_detected = two_now
                if two_now:
                    self._two_stable += 1
                else:
                    self._two_stable = 0
                if self._two_stable >= self._two_needed:
                    self._course_armed = True

                palms = 0
                if res.left_hand_landmarks and res.right_hand_landmarks:
                    d = _wrist_dist(
                        res.left_hand_landmarks.landmark,
                        res.right_hand_landmarks.landmark,
                    )
                    if d < self._palms_dist_max:
                        palms = 1
                msg.palms_together = palms
                if palms:
                    self._course_armed = False
                    self._two_stable = 0

                if res.right_hand_landmarks:
                    rhl = res.right_hand_landmarks.landmark
                    msg.right_palm_open_score = float(np.clip(_palm_open_score(rhl), 0.0, 1.0))

                msg.course_base_motion_enabled = 1 if self._course_armed else 0

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
                arm_txt = "ARMED" if self._course_armed else "DISARMED"
                cv2.rectangle(debug_frame, (8, 8), (520, 118), (0, 0, 0), -1)
                cv2.putText(
                    debug_frame,
                    f"course: {arm_txt} | two(L): {msg.left_hand_two_detected} | palms: {msg.palms_together}",
                    (16, 36),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    debug_frame,
                    f"leg F/L: {msg.leg_forward_norm:.2f} / {msg.leg_lateral_norm:+.2f} | elbow bend: {msg.avg_elbow_bend_deg:.0f}",
                    (16, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (220, 220, 220),
                    1,
                )
                cv2.putText(
                    debug_frame,
                    f"state(legacy): {msg.state} | palm open R: {msg.right_palm_open_score:.2f}",
                    (16, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (200, 200, 200),
                    1,
                )
        else:
            self._two_stable = 0
            if self.show_debug:
                cv2.putText(
                    debug_frame,
                    "NO PERSON",
                    (20, 45),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2,
                )

        return msg, debug_frame

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            frame = None
            if self.use_ros_topic:
                frame = self.current_frame
            else:
                ret, frame = self.cap.read()
                if not ret:
                    rospy.logwarn_throttle(5, "[body_pose_estimation] 摄像头读取失败")
                    rate.sleep()
                    continue
            if frame is None:
                rate.sleep()
                continue
            msg, dbg = self._process_frame(frame)
            self.body_pose_pub.publish(msg)
            if self.show_debug:
                try:
                    dm = self.bridge.cv2_to_imgmsg(dbg, encoding="bgr8")
                    dm.header = msg.header
                    self.debug_image_pub.publish(dm)
                except Exception as e:
                    rospy.logerr_throttle(5, f"调试图像发布失败: {e}")
                cv2.imshow("Holistic body — Q quit", dbg)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            rate.sleep()
        self.shutdown()

    def shutdown(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        self.holistic.close()
        rospy.loginfo("[body_pose_estimation] 已关闭")


def main():
    try:
        BodyPoseEstimation().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
