#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘速度融合：订阅体感 /cmd_vel_body 与激光 /scan，在 course_base_armed 为真时做简单前向避障，
输出到 /cmd_vel（Gazebo diff_drive）。
"""
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class CmdVelScanAvoid:
    def __init__(self):
        rospy.init_node("cmd_vel_scan_avoid", anonymous=True)
        self.in_topic = rospy.get_param("~cmd_vel_in", "/cmd_vel_body")
        self.out_topic = rospy.get_param("~cmd_vel_out", "/cmd_vel")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.armed_topic = rospy.get_param("~armed_topic", "/course_base_armed")
        self.front_deg = rospy.get_param("~front_sector_deg", 55.0)
        self.safe_dist = rospy.get_param("~safe_distance", 0.42)
        self.slow_dist = rospy.get_param("~slow_distance", 0.75)
        self.avoid_gain = rospy.get_param("~avoid_angular_gain", 0.9)

        self._scan = None
        self._cmd = Twist()
        self._armed = False

        rospy.Subscriber(self.scan_topic, LaserScan, self._scan_cb, queue_size=1)
        rospy.Subscriber(self.in_topic, Twist, self._cmd_cb, queue_size=1)
        rospy.Subscriber(self.armed_topic, Bool, self._armed_cb, queue_size=1)
        self._pub = rospy.Publisher(self.out_topic, Twist, queue_size=10)

        rospy.loginfo(
            f"[cmd_vel_scan_avoid] in={self.in_topic} out={self.out_topic} scan={self.scan_topic}"
        )

    def _armed_cb(self, msg):
        self._armed = bool(msg.data)

    def _scan_cb(self, msg):
        self._scan = msg

    def _cmd_cb(self, msg):
        self._cmd = msg

    def _front_min_range(self, scan):
        """LDS: angle 0 为机器人前方；取左右对称扇区最小有效距离。"""
        n = len(scan.ranges)
        if n == 0:
            return float("inf"), 0.0
        a0 = scan.angle_min
        inc = scan.angle_increment
        half = math.radians(self.front_deg * 0.5)
        idx_lo = int(max(0, math.floor((-half - a0) / inc)))
        idx_hi = int(min(n - 1, math.ceil((half - a0) / inc)))
        chunk = np.array(scan.ranges[idx_lo : idx_hi + 1], dtype=np.float64)
        chunk = chunk[np.isfinite(chunk)]
        chunk = chunk[(chunk > scan.range_min + 1e-3) & (chunk < scan.range_max)]
        if chunk.size == 0:
            return float("inf"), 0.0
        i_local = int(np.argmin(chunk))
        idx = idx_lo + i_local
        ang = a0 + idx * inc
        return float(np.min(chunk)), float(ang)

    def _lateral_balance(self, scan):
        """比较前向扇区左、右半边的平均距离，用于微调转向。"""
        n = len(scan.ranges)
        if n < 8:
            return 0.0
        k = max(1, int(n * (self.front_deg / 360.0) / 2))
        left = np.array(scan.ranges[1 : k + 1], dtype=np.float64)
        right = np.array(scan.ranges[max(1, n - k - 1) : n], dtype=np.float64)

        def _clean(a):
            a = a[np.isfinite(a)]
            return a[(a > scan.range_min + 1e-3) & (a < scan.range_max)]

        left = _clean(left)
        right = _clean(right)
        if left.size == 0 or right.size == 0:
            return 0.0
        ml = float(np.mean(left))
        mr = float(np.mean(right))
        return float(np.clip((mr - ml) / 1.0, -1.0, 1.0))

    def run(self):
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = self._cmd.linear.x
            twist.angular.z = self._cmd.angular.z

            if self._armed and self._scan is not None:
                dmin, _ = self._front_min_range(self._scan)
                lat = self._lateral_balance(self._scan)
                v = twist.linear.x
                if dmin < self.safe_dist:
                    twist.linear.x = min(v, 0.04)
                    twist.angular.z += self.avoid_gain * lat
                elif dmin < self.slow_dist and v > 0:
                    scale = float(np.clip((dmin - self.safe_dist) / (self.slow_dist - self.safe_dist), 0.15, 1.0))
                    twist.linear.x = v * scale
                    twist.angular.z += 0.35 * self.avoid_gain * lat

            twist.angular.z = float(np.clip(twist.angular.z, -2.2, 2.2))
            twist.linear.x = float(np.clip(twist.linear.x, -0.35, 0.35))
            self._pub.publish(twist)
            rate.sleep()


def main():
    try:
        CmdVelScanAvoid().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
