#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class GreenBottleMoveDistanceNode:
    def __init__(self):
        rospy.init_node("green_bottle_move_distance", anonymous=True)

        self.bridge = CvBridge()
        self.rgb_img = None
        self.depth_img = None
        self.depth_encoding = None

        # Depth and detection parameters
        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))
        self.min_valid_depth_m = float(rospy.get_param("~min_valid_depth_m", 0.1))
        self.max_valid_depth_m = float(rospy.get_param("~max_valid_depth_m", 10.0))
        self.green_area_min = float(rospy.get_param("~green_area_min", 350.0))
        self.change_threshold_m = float(rospy.get_param("~change_threshold_m", 0.1))

        # Motion parameters
        self.motion_enable = bool(rospy.get_param("~motion_enable", True))
        self.forward_speed = float(rospy.get_param("~forward_speed", 0.12))
        self.turn_speed = float(rospy.get_param("~turn_speed", 0.45))
        self.avoid_dist_m = float(rospy.get_param("~avoid_dist_m", 0.60))
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.show_debug = bool(rospy.get_param("~show_debug", False))

        self.rgb_topic = rospy.get_param("~rgb_topic", "/kinect2/hd/image_color_rect")
        self.depth_topic = rospy.get_param("~depth_topic", "/kinect2/sd/image_depth_rect")

        self.last_green_dist_m = None
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber(self.rgb_topic, Image, self.rgb_cb, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)

        self.motion_start_t = rospy.Time.now()
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("green_bottle_move_distance started")
        rospy.loginfo(
            "motion_enable=%s forward=%.2f turn=%.2f avoid_dist=%.2f",
            self.motion_enable,
            self.forward_speed,
            self.turn_speed,
            self.avoid_dist_m,
        )
        rospy.loginfo("rgb_topic=%s depth_topic=%s", self.rgb_topic, self.depth_topic)

    def _on_shutdown(self):
        self.cmd_pub.publish(Twist())
        if self.show_debug:
            cv2.destroyAllWindows()

    def rgb_cb(self, msg):
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "rgb convert failed: %s", str(e))

    def depth_cb(self, msg):
        try:
            self.depth_encoding = msg.encoding
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "depth convert failed: %s", str(e))

    def _depth_to_meters(self, depth_arr):
        if self.depth_encoding in ("32FC1", "64FC1"):
            return depth_arr.astype(np.float32)
        return depth_arr.astype(np.float32) * self.depth_scale

    def _valid_mask(self, depth_m):
        safe = np.where(np.isfinite(depth_m), depth_m, -1.0)
        return (safe >= self.min_valid_depth_m) & (safe <= self.max_valid_depth_m)

    def _nearest_obstacle_distance(self):
        if self.depth_img is None or self.depth_encoding is None:
            return None
        depth_m = self._depth_to_meters(self.depth_img)
        valid = self._valid_mask(depth_m)
        if not np.any(valid):
            return None
        near_m = float(np.min(depth_m[valid]))
        return near_m if math.isfinite(near_m) else None

    def _detect_green(self):
        if self.rgb_img is None:
            return None

        hsv = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2HSV)

        if not hasattr(self, "_color_check_logged"):
            sat_mean = float(np.mean(hsv[:, :, 1]))
            if sat_mean < 1.0:
                rospy.logwarn(
                    "当前彩色图像几乎是灰度(sat_mean=%.3f)，颜色阈值检测会失败。"
                    "请确认使用了真正彩色话题或更换场景。",
                    sat_mean,
                )
            self._color_check_logged = True

        # Green threshold, tuned for gazebo bottle color
        lower_green = np.array([35, 70, 50], dtype=np.uint8)
        upper_green = np.array([90, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_green, upper_green)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        max_cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_cnt)
        if area < self.green_area_min:
            return None

        m = cv2.moments(max_cnt)
        if m["m00"] == 0:
            return None

        cx = int(m["m10"] / m["m00"])
        cy = int(m["m01"] / m["m00"])
        return {"cx": cx, "cy": cy, "contour": max_cnt, "area": area}

    def _mapped_depth_distance(self, cx_rgb, cy_rgb):
        if self.depth_img is None or self.rgb_img is None or self.depth_encoding is None:
            return None

        h_rgb, w_rgb = self.rgb_img.shape[:2]
        h_d, w_d = self.depth_img.shape[:2]

        cx_d = int(np.clip(int(cx_rgb * w_d / w_rgb), 0, w_d - 1))
        cy_d = int(np.clip(int(cy_rgb * h_d / h_rgb), 0, h_d - 1))

        # Median in a small patch for robustness
        r = 2
        x0, x1 = max(0, cx_d - r), min(w_d, cx_d + r + 1)
        y0, y1 = max(0, cy_d - r), min(h_d, cy_d + r + 1)
        patch = self.depth_img[y0:y1, x0:x1]
        if patch.size == 0:
            return None

        patch_m = self._depth_to_meters(patch)
        valid = self._valid_mask(patch_m)
        if not np.any(valid):
            return None

        dist_m = float(np.median(patch_m[valid]))
        if (not math.isfinite(dist_m)) or dist_m < self.min_valid_depth_m or dist_m > self.max_valid_depth_m:
            return None
        return dist_m

    def _motion_cmd(self, near_m):
        cmd = Twist()

        if not self.motion_enable:
            return cmd

        # Simple safety: obstacle too near -> rotate in place
        if near_m is not None and near_m < self.avoid_dist_m:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
            return cmd

        # Patrol cycle: forward 4s, turn left 2s, forward 4s, turn right 2s
        t = (rospy.Time.now() - self.motion_start_t).to_sec() % 12.0
        if t < 4.0:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
        elif t < 6.0:
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_speed
        elif t < 10.0:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = -self.turn_speed

        return cmd

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            if self.depth_encoding is not None and not hasattr(self, "_depth_info_logged"):
                rospy.loginfo("depth encoding=%s", self.depth_encoding)
                self._depth_info_logged = True

            near_m = self._nearest_obstacle_distance()
            cmd = self._motion_cmd(near_m)
            self.cmd_pub.publish(cmd)

            green = self._detect_green()
            if green is None:
                rospy.logwarn_throttle(2.0, "未检测到绿色瓶子")
                rate.sleep()
                continue

            dist_m = self._mapped_depth_distance(green["cx"], green["cy"])
            if dist_m is None:
                rospy.logwarn_throttle(2.0, "检测到绿色瓶子，但深度无效")
            else:
                if self.last_green_dist_m is None:
                    rospy.loginfo("绿色瓶子初始距离: %.2f m", dist_m)
                    self.last_green_dist_m = dist_m
                elif abs(dist_m - self.last_green_dist_m) > self.change_threshold_m:
                    rospy.loginfo("绿色瓶子距离更新: %.2f m", dist_m)
                    self.last_green_dist_m = dist_m

            if self.show_debug and self.rgb_img is not None:
                dbg = self.rgb_img.copy()
                cv2.drawContours(dbg, [green["contour"]], -1, (0, 255, 255), 2)
                cv2.circle(dbg, (green["cx"], green["cy"]), 5, (255, 0, 0), -1)
                if dist_m is not None:
                    cv2.putText(
                        dbg,
                        "green_dist=%.2fm" % dist_m,
                        (green["cx"] + 10, green["cy"] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )
                cv2.imshow("green_bottle_debug", dbg)
                cv2.waitKey(1)

            rate.sleep()


if __name__ == "__main__":
    try:
        GreenBottleMoveDistanceNode().run()
    except rospy.ROSInterruptException:
        pass
