#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
列出本机 V4L2 设备节点，并用 OpenCV（优先 CAP_V4L2）探测可打开的 camera index。
用法（与 ROS 节点相同解释器，避免 conda / 系统混用）:
  /usr/bin/python3 $(rospack find handcontrol)/scripts/list_usb_cameras.py
或:
  rosrun handcontrol list_usb_cameras.py
"""
from __future__ import annotations

import glob
import sys


def main() -> int:
    devs = sorted(glob.glob("/dev/video*"))
    print("=== /dev/video* (V4L2 设备节点) ===")
    if not devs:
        print(
            "(无)\n"
            "  说明: 未检测到任何视频设备。常见原因:\n"
            "  - 未接 USB 摄像头 / 笔记本摄像头被禁用\n"
            "  - 在虚拟机里未把摄像头透传给客户机\n"
            "  - 无权限: 将用户加入 video 组后重新登录: sudo usermod -aG video $USER\n"
        )
    else:
        for d in devs:
            print(" ", d)

    print("\n=== OpenCV VideoCapture(index), 优先 CAP_V4L2, index 0..15 ===")
    try:
        import cv2
    except ImportError as e:
        print("无法 import cv2:", e)
        print("请安装: /usr/bin/python3 -m pip install --user opencv-python")
        return 1

    ok = []
    for i in range(16):
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(i)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            ok.append((i, w, h))
        cap.release()

    if not ok:
        print("(无可用 index — 与 body_pose_estimation 里 camera_index=0 失败一致)")
        print("\n可选方案:")
        print("  1) 接好摄像头后重跑本脚本")
        print("  2) 用仿真相机话题: roslaunch ... use_ros_topic:=true image_topic:=/camera/rgb/image_raw")
        return 0

    for i, w, h in ok:
        print(f"  index {i:2d}: 可打开, 默认分辨率约 {w}x{h}")
    print("\n启动时在 launch 里传入第一个可用 index, 例如:")
    print(f"  camera_index:={ok[0][0]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
