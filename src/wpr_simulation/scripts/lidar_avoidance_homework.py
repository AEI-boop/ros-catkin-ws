#!/usr/bin/env python3
# coding=utf-8

import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LidarAvoidanceNode:
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.safe_distance = rospy.get_param("~safe_distance", 0.80)
        self.warn_distance = rospy.get_param("~warn_distance", 1.20)
        self.linear_speed = rospy.get_param("~linear_speed", 0.15)
        self.turn_speed = rospy.get_param("~turn_speed", 0.70)
        self.slow_linear_speed = rospy.get_param("~slow_linear_speed", 0.06)

        self.vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self.lidar_callback, queue_size=10)

        rospy.loginfo("Lidar avoidance node started.")
        rospy.loginfo("scan_topic=%s cmd_vel_topic=%s", self.scan_topic, self.cmd_vel_topic)
        rospy.loginfo(
            "safe_distance=%.2f warn_distance=%.2f linear_speed=%.2f turn_speed=%.2f",
            self.safe_distance,
            self.warn_distance,
            self.linear_speed,
            self.turn_speed,
        )

    @staticmethod
    def _is_valid_range(value):
        return math.isfinite(value) and value > 0.0

    def _sector_min_distance(self, scan, center_deg, half_width_deg):
        center = math.radians(center_deg)
        half_width = math.radians(half_width_deg)

        min_dist = float("inf")
        for i, r in enumerate(scan.ranges):
            if not self._is_valid_range(r):
                continue

            angle = scan.angle_min + i * scan.angle_increment
            diff = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if abs(diff) <= half_width:
                if scan.range_min <= r <= scan.range_max:
                    min_dist = min(min_dist, r)

        return min_dist

    def lidar_callback(self, scan):
        front = self._sector_min_distance(scan, center_deg=0.0, half_width_deg=20.0)
        left = self._sector_min_distance(scan, center_deg=45.0, half_width_deg=20.0)
        right = self._sector_min_distance(scan, center_deg=-45.0, half_width_deg=20.0)

        cmd = Twist()

        if front < self.safe_distance:
            cmd.linear.x = 0.0
            if left >= right:
                cmd.angular.z = self.turn_speed
                turn_dir = "left"
            else:
                cmd.angular.z = -self.turn_speed
                turn_dir = "right"
            rospy.logwarn(
                "Obstacle ahead: front=%.2f m, left=%.2f m, right=%.2f m -> turn %s",
                front,
                left,
                right,
                turn_dir,
            )
        elif front < self.warn_distance:
            cmd.linear.x = self.slow_linear_speed
            cmd.angular.z = 0.3 if left >= right else -0.3
            rospy.loginfo(
                "Approaching obstacle: front=%.2f m -> slow and adjust",
                front,
            )
        else:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0

        self.vel_pub.publish(cmd)


def main():
    rospy.init_node("lidar_avoidance_homework")
    LidarAvoidanceNode()
    rospy.spin()


if __name__ == "__main__":
    main()
