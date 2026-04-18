#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class RedBottleDistanceNode:
    def __init__(self):
        rospy.init_node("red_bottle_smart", anonymous=True)

        self.bridge = CvBridge()
        self.rgb_img = None
        self.depth_img = None
        self.depth_encoding = None

        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))
        self.min_valid_depth_m = float(rospy.get_param("~min_valid_depth_m", 0.1))
        self.max_valid_depth_m = float(rospy.get_param("~max_valid_depth_m", 10.0))
        self.red_area_min = float(rospy.get_param("~red_area_min", 500.0))
        self.change_threshold_m = float(rospy.get_param("~change_threshold_m", 0.1))
        self.near_change_threshold_m = float(rospy.get_param("~near_change_threshold_m", 0.05))
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.show_debug = bool(rospy.get_param("~show_debug", False))

        self.last_red_dist_m = None
        self.last_near_dist_m = None

        rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self.rgb_cb, queue_size=1)
        rospy.Subscriber("/kinect2/sd/image_depth_rect", Image, self.depth_cb, queue_size=1)

        rospy.loginfo("red_bottle_smart started")
        rospy.loginfo("params: area_min=%.1f change_th=%.2f near_change_th=%.2f", self.red_area_min, self.change_threshold_m, self.near_change_threshold_m)

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

    def _depth_to_meters_array(self, depth_arr):
        if self.depth_encoding in ("32FC1", "64FC1"):
            depth_m = depth_arr.astype(np.float32)
        else:
            depth_m = depth_arr.astype(np.float32) * self.depth_scale
        return depth_m

    def _valid_depth_mask(self, depth_m):
        # Replace non-finite depth with -1 first, then threshold to avoid numpy invalid warnings.
        safe_depth = np.where(np.isfinite(depth_m), depth_m, -1.0)
        return (safe_depth >= self.min_valid_depth_m) & (safe_depth <= self.max_valid_depth_m)

    def detect_red_region(self):
        if self.rgb_img is None:
            return None

        hsv = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2HSV)

        lower_red_1 = np.array([0, 120, 70], dtype=np.uint8)
        upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
        lower_red_2 = np.array([170, 120, 70], dtype=np.uint8)
        upper_red_2 = np.array([180, 255, 255], dtype=np.uint8)

        mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        max_cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_cnt)
        if area < self.red_area_min:
            return None

        m = cv2.moments(max_cnt)
        if m["m00"] == 0:
            return None

        cx = int(m["m10"] / m["m00"])
        cy = int(m["m01"] / m["m00"])

        return {
            "cx": cx,
            "cy": cy,
            "area": area,
            "contour": max_cnt,
            "mask": mask,
        }

    def depth_at_mapped_pixel(self, cx_rgb, cy_rgb):
        if self.depth_img is None or self.rgb_img is None or self.depth_encoding is None:
            return None

        h_rgb, w_rgb = self.rgb_img.shape[:2]
        h_d, w_d = self.depth_img.shape[:2]

        cx_d = int(cx_rgb * w_d / w_rgb)
        cy_d = int(cy_rgb * h_d / h_rgb)
        cx_d = int(np.clip(cx_d, 0, w_d - 1))
        cy_d = int(np.clip(cy_d, 0, h_d - 1))

        r = 2
        x0 = max(0, cx_d - r)
        x1 = min(w_d, cx_d + r + 1)
        y0 = max(0, cy_d - r)
        y1 = min(h_d, cy_d + r + 1)

        patch = self.depth_img[y0:y1, x0:x1]
        if patch.size == 0:
            return None

        patch_m = self._depth_to_meters_array(patch)
        valid = self._valid_depth_mask(patch_m)
        if not np.any(valid):
            return None

        dist_m = float(np.median(patch_m[valid]))
        if (not math.isfinite(dist_m)) or dist_m < self.min_valid_depth_m or dist_m > self.max_valid_depth_m:
            return None

        return dist_m

    def nearest_obstacle_distance(self):
        if self.depth_img is None or self.depth_encoding is None:
            return None

        depth_m = self._depth_to_meters_array(self.depth_img)
        valid = self._valid_depth_mask(depth_m)
        if not np.any(valid):
            return None

        near_m = float(np.min(depth_m[valid]))
        if not math.isfinite(near_m):
            return None
        return near_m

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            if self.depth_encoding is not None and not hasattr(self, "_depth_info_logged"):
                rospy.loginfo("depth encoding=%s", self.depth_encoding)
                self._depth_info_logged = True

            near_m = self.nearest_obstacle_distance()
            if near_m is not None:
                if self.last_near_dist_m is None:
                    rospy.loginfo("最近障碍物初始距离: %.2f m", near_m)
                    self.last_near_dist_m = near_m
                elif abs(near_m - self.last_near_dist_m) > self.near_change_threshold_m:
                    rospy.loginfo("最近障碍物距离更新: %.2f m", near_m)
                    self.last_near_dist_m = near_m

            red = self.detect_red_region()
            if red is None:
                rospy.logwarn_throttle(2.0, "未检测到红色瓶子")
                if self.show_debug and self.rgb_img is not None:
                    cv2.imshow("red_bottle_debug", self.rgb_img)
                    cv2.waitKey(1)
                rate.sleep()
                continue

            dist_m = self.depth_at_mapped_pixel(red["cx"], red["cy"])
            if dist_m is None:
                rospy.logwarn_throttle(2.0, "检测到红瓶，但该像素深度无效")
            else:
                if self.last_red_dist_m is None:
                    rospy.loginfo("红瓶初始距离: %.2f m", dist_m)
                    self.last_red_dist_m = dist_m
                elif abs(dist_m - self.last_red_dist_m) > self.change_threshold_m:
                    rospy.loginfo("红瓶距离更新: %.2f m", dist_m)
                    self.last_red_dist_m = dist_m

            if self.show_debug and self.rgb_img is not None:
                dbg = self.rgb_img.copy()
                cv2.drawContours(dbg, [red["contour"]], -1, (0, 255, 255), 2)
                cv2.circle(dbg, (red["cx"], red["cy"]), 5, (255, 0, 0), -1)
                if dist_m is not None:
                    cv2.putText(dbg, "dist=%.2fm" % dist_m, (red["cx"] + 10, red["cy"] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("red_bottle_debug", dbg)
                cv2.waitKey(1)

            rate.sleep()

        if self.show_debug:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        RedBottleDistanceNode().run()
    except rospy.ROSInterruptException:
        pass
