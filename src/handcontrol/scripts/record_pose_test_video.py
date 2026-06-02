#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Record raw camera video for body-pose recognition tests.

Examples:
  rosrun handcontrol record_pose_test_video.py
  rosrun handcontrol record_pose_test_video.py --camera-index -1 --duration 60
  rosrun handcontrol record_pose_test_video.py --output ~/pose_test.mp4 --mirror
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from datetime import datetime


def _import_cv2():
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        print("无法 import cv2:", exc, file=sys.stderr)
        print("请安装: /usr/bin/python3 -m pip install --user opencv-python", file=sys.stderr)
        return None
    return cv2


def _probe_v4l2_camera_index(cv2, max_index: int = 15):
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


def _open_camera(cv2, index: int, width: int, height: int, fps: float):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(index)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"无法打开摄像头索引 {index}")
    return cap


def _output_path(output: str | None, output_dir: str) -> str:
    if output:
        return os.path.abspath(os.path.expanduser(output))

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"body_pose_test_{stamp}.mp4"
    return os.path.abspath(os.path.expanduser(os.path.join(output_dir, filename)))


def _writer_for_path(cv2, path: str, fps: float, size: tuple[int, int], codec: str):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*codec)
    writer = cv2.VideoWriter(path, fourcc, fps, size)
    if writer.isOpened():
        return writer, path

    writer.release()
    fallback_path = os.path.splitext(path)[0] + ".avi"
    fallback_fourcc = cv2.VideoWriter_fourcc(*"XVID")
    writer = cv2.VideoWriter(fallback_path, fallback_fourcc, fps, size)
    if writer.isOpened():
        print(f"mp4 写入器不可用，已改用: {fallback_path}")
        return writer, fallback_path

    writer.release()
    raise RuntimeError("无法创建视频写入器，请检查 OpenCV 编码器支持")


def _draw_overlay(cv2, frame, lines: list[str]):
    h, w = frame.shape[:2]
    pad = 10
    line_h = 24
    box_h = pad * 2 + line_h * len(lines)
    cv2.rectangle(frame, (0, 0), (w, box_h), (0, 0, 0), -1)
    for i, line in enumerate(lines):
        y = pad + 18 + i * line_h
        cv2.putText(
            frame,
            line,
            (pad, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    cv2.putText(
        frame,
        "SPACE pause/resume | q/ESC stop",
        (pad, h - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="录制体感识别测试视频，默认保存原始摄像头画面。"
    )
    parser.add_argument("-i", "--camera-index", type=int, default=0, help="摄像头索引；-1 自动探测")
    parser.add_argument("-o", "--output", default=None, help="输出文件路径，默认按时间戳生成 mp4")
    parser.add_argument(
        "--output-dir",
        default="~/catkin_ws/pose_test_videos",
        help="未指定 --output 时的视频保存目录",
    )
    parser.add_argument("--width", type=int, default=640, help="采集宽度")
    parser.add_argument("--height", type=int, default=480, help="采集高度")
    parser.add_argument("--fps", type=float, default=30.0, help="采集和写入帧率")
    parser.add_argument("--duration", type=float, default=30.0, help="录制秒数；0 表示手动停止")
    parser.add_argument("--countdown", type=float, default=3.0, help="开始录制前倒计时秒数")
    parser.add_argument("--codec", default="mp4v", help="OpenCV fourcc 编码，默认 mp4v")
    parser.add_argument("--mirror", action="store_true", help="水平翻转画面，适合前置摄像头预览")
    parser.add_argument("--record-overlay", action="store_true", help="把时间信息也写入视频")
    parser.add_argument("--no-preview", action="store_true", help="不显示预览窗口")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    cv2 = _import_cv2()
    if cv2 is None:
        return 1

    camera_index = args.camera_index
    if camera_index < 0:
        found = _probe_v4l2_camera_index(cv2)
        if found is None:
            print("未找到可用摄像头。可先运行: rosrun handcontrol list_usb_cameras.py", file=sys.stderr)
            return 1
        camera_index = found
        print(f"自动选择摄像头 index {camera_index}")

    output_path = _output_path(args.output, args.output_dir)
    actual_output_path = output_path
    cap = None
    writer = None
    window_name = "record body pose test video"
    frames_written = 0

    try:
        cap = _open_camera(cv2, camera_index, args.width, args.height, args.fps)

        ok, frame = cap.read()
        if not ok or frame is None:
            raise RuntimeError("摄像头已打开，但无法读取首帧")

        if args.mirror:
            frame = cv2.flip(frame, 1)
        frame_size = (int(frame.shape[1]), int(frame.shape[0]))
        writer, actual_output_path = _writer_for_path(cv2, output_path, args.fps, frame_size, args.codec)
        actual_path = writer.getBackendName() if hasattr(writer, "getBackendName") else ""

        print("=== 体感识别测试视频录制 ===")
        print(f"摄像头: index {camera_index}")
        print(f"分辨率: {frame_size[0]}x{frame_size[1]} @ {args.fps:g} fps")
        print(f"保存到: {actual_output_path}")
        if actual_path:
            print(f"VideoWriter backend: {actual_path}")
        print("按 SPACE 暂停/继续，按 q 或 ESC 结束。")

        countdown_end = time.monotonic() + max(0.0, float(args.countdown))
        record_start = None
        pause_start = None
        paused_total = 0.0
        paused = False
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("摄像头读取失败，录制结束。", file=sys.stderr)
                break

            if args.mirror:
                frame = cv2.flip(frame, 1)

            now = time.monotonic()
            if record_start is None and now >= countdown_end:
                record_start = now

            display = frame.copy()
            if record_start is None:
                remain = int(math.ceil(max(0.0, countdown_end - now)))
                _draw_overlay(cv2, display, [f"STARTING IN {remain}s", "Prepare body-pose gesture test"])
            else:
                active_pause = (now - pause_start) if paused and pause_start is not None else 0.0
                elapsed = now - record_start - paused_total - active_pause
                status = "PAUSED" if paused else "REC"
                lines = [
                    f"{status} {elapsed:05.1f}s / {args.duration:g}s" if args.duration > 0 else f"{status} {elapsed:05.1f}s",
                    f"camera index {camera_index} | frame {frames_written}",
                ]
                _draw_overlay(cv2, display, lines)

                if not paused:
                    writer.write(display if args.record_overlay else frame)
                    frames_written += 1

                if args.duration > 0 and elapsed >= args.duration:
                    break

            if not args.no_preview:
                cv2.imshow(window_name, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord(" "):
                    if record_start is None:
                        continue
                    if paused:
                        paused_total += now - pause_start
                        pause_start = None
                        paused = False
                    else:
                        pause_start = now
                        paused = True
            elif args.duration <= 0 and record_start is not None:
                time.sleep(1.0 / max(args.fps, 1.0))

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，录制结束。")
    except Exception as exc:
        print(f"录制失败: {exc}", file=sys.stderr)
        return 1
    finally:
        if writer is not None:
            writer.release()
        if cap is not None:
            cap.release()
        if cv2 is not None:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

    if frames_written > 0:
        print(f"已保存: {actual_output_path}")
    else:
        print("未写入视频帧。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
