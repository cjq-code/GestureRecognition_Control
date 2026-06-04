#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision_target_detector.py - simple color target detector for visual grasping.

Step 1 only: detect a colored object in the robot RGB camera image. This node
does not publish cmd_vel and does not command the manipulator.
"""
import json

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

try:
    from PIL import Image as PILImage
    from PIL import ImageDraw, ImageFont
except Exception:
    PILImage = None
    ImageDraw = None
    ImageFont = None


COLOR_RANGES = {
    "red": [
        ((0, 30, 30), (12, 255, 255)),
        ((168, 30, 30), (180, 255, 255)),
    ],
    "orange": [((5, 50, 50), (25, 255, 255))],
    "yellow": [((20, 50, 50), (38, 255, 255))],
    "green": [((35, 50, 50), (85, 255, 255))],
    "blue": [((90, 50, 50), (135, 255, 255))],
}


class VisionTargetDetector:
    def __init__(self):
        rospy.init_node("vision_target_detector", anonymous=True)

        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.target_color = rospy.get_param("~target_color", "red").lower()
        self.min_area_norm = float(rospy.get_param("~min_area_norm", 0.0005))
        self.blur_kernel = int(rospy.get_param("~blur_kernel", 5))
        self.morph_kernel = int(rospy.get_param("~morph_kernel", 5))
        self.center_deadzone = float(rospy.get_param("~center_deadzone", 0.12))
        self.debug_image_topic = rospy.get_param("~debug_image_topic", "/vision_grasp/debug_image")
        self.align_status_topic = rospy.get_param("~align_status_topic", "/vision_grasp/align_status")
        self.process_rate = float(rospy.get_param("~process_rate", 8.0))
        self.debug_rate = float(rospy.get_param("~debug_rate", 4.0))
        self.image_scale = float(rospy.get_param("~image_scale", 0.5))
        self.debug_text = rospy.get_param("~debug_text", True)
        self.debug_text_panel = rospy.get_param("~debug_text_panel", True)
        self.debug_panel_width = int(rospy.get_param("~debug_panel_width", 220))
        self.debug_panel_position = str(rospy.get_param("~debug_panel_position", "bottom")).strip().lower()
        self.debug_state_only = rospy.get_param("~debug_state_only", True)
        self.debug_font_scale = float(rospy.get_param("~debug_font_scale", 0.24))
        self.debug_line_step = int(rospy.get_param("~debug_line_step", 18))
        self.debug_cjk_font_size = int(rospy.get_param("~debug_cjk_font_size", 14))
        self.debug_gripper_front_y_norm = float(rospy.get_param("~debug_gripper_front_y_norm", -0.35))
        self.show_debug_window = rospy.get_param("~show_debug_window", False)
        self.debug_window_active_only = rospy.get_param("~debug_window_active_only", True)
        self.debug_window_name = str(rospy.get_param("~debug_window_name", "vision_grasp_debug"))
        self.active_topic = rospy.get_param("~active_topic", "/vision_grasp/active")
        self.last_process_time = rospy.Time(0)
        self.last_debug_time = rospy.Time(0)
        self.align_state = ""
        self.grasp_done = False
        self.auto_active = False
        self._debug_window_open = False
        self._debug_window_warned = False
        self._font = self._load_cjk_font()

        if self.target_color not in COLOR_RANGES:
            rospy.logwarn(
                "unknown target_color=%s, fallback to red. valid=%s",
                self.target_color,
                sorted(COLOR_RANGES.keys()),
            )
            self.target_color = "red"

        self.bridge = CvBridge()
        self.visible_pub = rospy.Publisher("/vision_grasp/target_visible", Bool, queue_size=10)
        self.x_pub = rospy.Publisher("/vision_grasp/target_x_norm", Float32, queue_size=10)
        self.y_pub = rospy.Publisher("/vision_grasp/target_y_norm", Float32, queue_size=10)
        self.front_y_pub = rospy.Publisher("/vision_grasp/target_front_y_norm", Float32, queue_size=10)
        self.bottom_y_pub = rospy.Publisher("/vision_grasp/target_bottom_y_norm", Float32, queue_size=10)
        self.area_pub = rospy.Publisher("/vision_grasp/target_area_norm", Float32, queue_size=10)
        self.status_pub = rospy.Publisher("/vision_grasp/status", String, queue_size=10)
        self.debug_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1)
        rospy.Subscriber(self.align_status_topic, String, self._align_status_callback, queue_size=1)
        rospy.Subscriber(self.active_topic, Bool, self._active_callback, queue_size=1)

        rospy.loginfo("[vision_target_detector] ready")
        rospy.loginfo("  - image_topic: %s", self.image_topic)
        rospy.loginfo("  - target_color: %s", self.target_color)
        rospy.loginfo("  - min_area_norm: %.4f", self.min_area_norm)
        rospy.loginfo("  - process_rate: %.1f Hz", self.process_rate)
        rospy.loginfo("  - debug_rate: %.1f Hz", self.debug_rate)
        rospy.loginfo("  - image_scale: %.2f", self.image_scale)
        rospy.loginfo("  - show_debug_window: %s", self.show_debug_window)
        rospy.loginfo("  - debug_window_active_only: %s", self.debug_window_active_only)

    def _load_cjk_font(self):
        if ImageFont is None:
            return None
        for path in (
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
        ):
            try:
                return ImageFont.truetype(path, self.debug_cjk_font_size)
            except Exception:
                pass
        return None

    def _align_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.align_state = str(data.get("state", ""))
        self.grasp_done = bool(data.get("grasp_done", False))

    def _active_callback(self, msg):
        self.auto_active = bool(msg.data)
        if not self.auto_active and self._debug_window_open and self.debug_window_active_only:
            self._close_debug_window()

    def _close_debug_window(self):
        try:
            cv2.destroyWindow(self.debug_window_name)
        except Exception:
            pass
        self._debug_window_open = False

    def _make_mask(self, bgr):
        img = bgr
        if self.blur_kernel >= 3:
            k = self.blur_kernel if self.blur_kernel % 2 == 1 else self.blur_kernel + 1
            img = cv2.GaussianBlur(img, (k, k), 0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = None
        for lower, upper in COLOR_RANGES[self.target_color]:
            part = cv2.inRange(hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        if self.morph_kernel >= 3:
            k = np.ones((self.morph_kernel, self.morph_kernel), dtype=np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def _detect(self, bgr):
        h, w = bgr.shape[:2]
        mask = self._make_mask(bgr)
        contours, _hier = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return mask, None, {"contour_count": 0, "max_area_norm": 0.0}

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        area_norm = area / float(max(w * h, 1))
        diag = {"contour_count": len(contours), "max_area_norm": area_norm}
        if area_norm < self.min_area_norm:
            return mask, None, diag

        x, y, bw, bh = cv2.boundingRect(contour)
        cx = x + bw * 0.5
        cy = y + bh * 0.5
        front_y = y
        bottom_y = y + bh
        x_norm = float(np.clip((cx - w * 0.5) / (w * 0.5), -1.0, 1.0))
        y_norm = float(np.clip((h * 0.5 - cy) / (h * 0.5), -1.0, 1.0))
        front_y_norm = float(np.clip((h * 0.5 - front_y) / (h * 0.5), -1.0, 1.0))
        bottom_y_norm = float(np.clip((h * 0.5 - bottom_y) / (h * 0.5), -1.0, 1.0))
        if abs(x_norm) <= self.center_deadzone:
            alignment = "CENTER"
        elif x_norm < 0:
            alignment = "LEFT"
        else:
            alignment = "RIGHT"

        return mask, {
            "visible": True,
            "x_norm": x_norm,
            "y_norm": y_norm,
            "front_y_norm": front_y_norm,
            "bottom_y_norm": bottom_y_norm,
            "area_norm": area_norm,
            "bbox": [int(x), int(y), int(bw), int(bh)],
            "alignment": alignment,
        }, diag

    def _draw_debug(self, bgr, detection):
        out = bgr.copy()
        h, w = out.shape[:2]
        cv2.line(out, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)
        dead = int(self.center_deadzone * w * 0.5)
        cv2.line(out, (w // 2 - dead, 0), (w // 2 - dead, h), (0, 255, 255), 1)
        cv2.line(out, (w // 2 + dead, 0), (w // 2 + dead, h), (0, 255, 255), 1)
        gripper_y = int((0.5 - self.debug_gripper_front_y_norm * 0.5) * h)
        cv2.line(out, (0, gripper_y), (w, gripper_y), (255, 255, 0), 1)

        if detection:
            x, y, bw, bh = detection["bbox"]
            cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cx = x + bw // 2
            cy = y + bh // 2
            cv2.circle(out, (cx, cy), 4, (0, 255, 255), -1)
            cv2.line(out, (x, y), (x + bw, y), (255, 0, 255), 1)
            cv2.line(out, (x, y + bh), (x + bw, y + bh), (255, 0, 255), 1)
            lines = [
                "target={} visible=True align={}".format(self.target_color, detection["alignment"]),
                "x={:+.2f} y={:+.2f} front={:+.2f} bottom={:+.2f} area={:.4f}".format(
                    detection["x_norm"],
                    detection["y_norm"],
                    detection["front_y_norm"],
                    detection["bottom_y_norm"],
                    detection["area_norm"],
                ),
            ]
        else:
            lines = [
                "target={} visible=False".format(self.target_color),
                "move closer or tune color/area",
            ]
        if self.debug_state_only:
            if self.grasp_done:
                lines = ["已抓取"]
            else:
                lines = ["state={}".format(self.align_state or "WAITING")]
        else:
            if self.align_state:
                lines.append("state={}".format(self.align_state))
            if self.grasp_done:
                lines.insert(0, "已抓取")

        if self.debug_text:
            if self.debug_text_panel:
                out = self._append_text_panel(out, lines)
            else:
                out = self._draw_text_lines(out, lines)
        elif self.grasp_done:
            out = self._draw_grasped_label(out)
        return out

    def _append_text_panel(self, bgr, lines):
        h, w = bgr.shape[:2]
        panel_h = max(24, 8 + max(1, len(lines)) * self.debug_line_step)
        panel = np.zeros((panel_h, w, 3), dtype=np.uint8)
        panel[:, :] = (36, 36, 36)
        if self.debug_panel_position == "top":
            combined = np.vstack((panel, bgr))
            x0 = 8
            y_base = 6
        else:
            combined = np.vstack((bgr, panel))
            x0 = 8
            y_base = h + 6
        if PILImage is None or ImageDraw is None or self._font is None:
            for i, line in enumerate(lines):
                y0 = y_base + 12 + i * self.debug_line_step
                cv2.putText(combined, line, (x0, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
            return combined
        rgb = cv2.cvtColor(combined, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(rgb)
        draw = ImageDraw.Draw(pil)
        for i, line in enumerate(lines):
            color = (0, 255, 0) if line == "已抓取" else (255, 255, 0)
            draw.text((x0, y_base + i * self.debug_line_step), line, font=self._font, fill=color)
        return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)

    def _draw_text_lines(self, bgr, lines):
        if PILImage is None or ImageDraw is None or self._font is None:
            for i, line in enumerate(lines):
                y0 = 12 + i * self.debug_line_step
                cv2.putText(bgr, line, (6, y0), cv2.FONT_HERSHEY_SIMPLEX, self.debug_font_scale, (0, 255, 255), 1, cv2.LINE_AA)
            return bgr
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(rgb)
        draw = ImageDraw.Draw(pil)
        for i, line in enumerate(lines):
            draw.text((4, 2 + i * self.debug_line_step), line, font=self._font, fill=(255, 255, 0))
        return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)

    def _draw_grasped_label(self, bgr):
        if PILImage is None or ImageDraw is None or self._font is None:
            cv2.putText(bgr, "GRASPED", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1, cv2.LINE_8)
            return bgr
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(rgb)
        draw = ImageDraw.Draw(pil)
        text = "已抓取"
        draw.text((4, 2), text, font=self._font, fill=(0, 255, 0))
        return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)

    def _publish_status(self, detection, diag=None):
        diag = diag or {"contour_count": 0, "max_area_norm": 0.0}
        if detection:
            visible = True
            x_norm = detection["x_norm"]
            y_norm = detection["y_norm"]
            front_y_norm = detection["front_y_norm"]
            bottom_y_norm = detection["bottom_y_norm"]
            area_norm = detection["area_norm"]
            status = detection.copy()
        else:
            visible = False
            x_norm = 0.0
            y_norm = 0.0
            front_y_norm = 0.0
            bottom_y_norm = 0.0
            area_norm = 0.0
            status = {
                "visible": False,
                "x_norm": 0.0,
                "y_norm": 0.0,
                "front_y_norm": 0.0,
                "bottom_y_norm": 0.0,
                "area_norm": 0.0,
                "bbox": [0, 0, 0, 0],
                "alignment": "LOST",
            }
        status["target_color"] = self.target_color
        status["stamp"] = rospy.Time.now().to_sec()
        status["contour_count"] = int(diag.get("contour_count", 0))
        status["max_area_norm"] = round(float(diag.get("max_area_norm", 0.0)), 6)
        status["min_area_norm"] = self.min_area_norm

        self.visible_pub.publish(Bool(data=visible))
        self.x_pub.publish(Float32(data=float(x_norm)))
        self.y_pub.publish(Float32(data=float(y_norm)))
        self.front_y_pub.publish(Float32(data=float(front_y_norm)))
        self.bottom_y_pub.publish(Float32(data=float(bottom_y_norm)))
        self.area_pub.publish(Float32(data=float(area_norm)))
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def _image_callback(self, msg):
        now = rospy.Time.now()
        if self.process_rate > 0.0 and self.last_process_time != rospy.Time(0):
            if (now - self.last_process_time).to_sec() < 1.0 / self.process_rate:
                return
        self.last_process_time = now
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logerr("image conversion failed: %s", exc)
            return
        if 0.0 < self.image_scale < 1.0:
            bgr = cv2.resize(bgr, None, fx=self.image_scale, fy=self.image_scale, interpolation=cv2.INTER_AREA)
        _mask, detection, diag = self._detect(bgr)
        self._publish_status(detection, diag)
        if self.debug_rate <= 0.0:
            return
        if self.last_debug_time != rospy.Time(0):
            if (now - self.last_debug_time).to_sec() < 1.0 / self.debug_rate:
                return
        self.last_debug_time = now
        debug = self._draw_debug(bgr, detection)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))
        self._show_debug_window(debug)

    def _show_debug_window(self, debug):
        if not self.show_debug_window:
            return
        if self.debug_window_active_only and not self.auto_active:
            return
        try:
            cv2.imshow(self.debug_window_name, debug)
            cv2.waitKey(1)
            self._debug_window_open = True
        except Exception as exc:
            if not self._debug_window_warned:
                rospy.logwarn("[vision_target_detector] cannot show debug window: %s", exc)
                self._debug_window_warned = True

    def run(self):
        try:
            rospy.spin()
        finally:
            if self._debug_window_open:
                self._close_debug_window()


if __name__ == "__main__":
    try:
        VisionTargetDetector().run()
    except rospy.ROSInterruptException:
        pass
