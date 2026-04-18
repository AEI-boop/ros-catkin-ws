#!/usr/bin/env python3
import math
import os
import struct

import rospy
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header

FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
]


class RGBD2PointCloudSaver:
    def __init__(self):
        rospy.init_node("get_pointcloud3", anonymous=True)
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.depth_encoding = None
        self.info = None

        self.sample_step = max(1, int(rospy.get_param("~sample_step", 2)))
        self.max_depth_m = float(rospy.get_param("~max_depth_m", 5.0))
        self.depth_scale = float(rospy.get_param("~depth_scale", 0.001))
        self.publish_rate = float(rospy.get_param("~publish_rate", 15.0))

        self.save_pcd = bool(rospy.get_param("~save_pcd", True))
        self.pcd_output = os.path.expanduser(rospy.get_param("~pcd_output", "~/output_cloud.pcd"))
        self.saved_once = False

        self.sub_rgb = rospy.Subscriber("/kinect2/hd/image_color_rect", Image, self.cb_rgb)
        self.sub_depth = rospy.Subscriber("/kinect2/sd/image_depth_rect", Image, self.cb_depth)
        self.sub_info = rospy.Subscriber("/kinect2/sd/camera_info", CameraInfo, self.cb_info)

        self.pub = rospy.Publisher("/pointcloud_output", PointCloud2, queue_size=1)
        rospy.loginfo(
            "get_pointcloud3 started: step=%d max_depth=%.2fm save_pcd=%s file=%s",
            self.sample_step,
            self.max_depth_m,
            self.save_pcd,
            self.pcd_output,
        )

    def cb_rgb(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def cb_depth(self, msg):
        self.depth_encoding = msg.encoding
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def cb_info(self, msg):
        self.info = msg

    @staticmethod
    def _pack_rgb_uint32(b, g, r):
        return struct.unpack("I", struct.pack("BBBB", int(b), int(g), int(r), 255))[0]

    def _save_pcd_binary(self, points):
        if not points:
            return

        output_dir = os.path.dirname(self.pcd_output)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z rgb\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F U\n"
            "COUNT 1 1 1 1\n"
            f"WIDTH {len(points)}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {len(points)}\n"
            "DATA binary\n"
        )

        with open(self.pcd_output, "wb") as f:
            f.write(header.encode("ascii"))
            for x, y, z, rgb_uint in points:
                f.write(struct.pack("<fffI", float(x), float(y), float(z), int(rgb_uint)))

        self.saved_once = True
        rospy.loginfo("PCD saved: %s (points=%d)", self.pcd_output, len(points))

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            if self.rgb is None or self.depth is None or self.info is None or self.depth_encoding is None:
                rate.sleep()
                continue

            if not hasattr(self, "_depth_info_logged"):
                rospy.loginfo("depth encoding=%s", self.depth_encoding)
                self._depth_info_logged = True

            h_d, w_d = self.depth.shape
            h_rgb, w_rgb = self.rgb.shape[:2]

            fx, fy = self.info.K[0], self.info.K[4]
            cx, cy = self.info.K[2], self.info.K[5]

            points = []
            step = self.sample_step

            for v in range(0, h_d, step):
                for u in range(0, w_d, step):
                    d = self.depth[v, u]

                    if self.depth_encoding in ("32FC1", "64FC1"):
                        z = float(d)
                        if (not math.isfinite(z)) or z <= 0.0 or z > self.max_depth_m:
                            continue
                    else:
                        if int(d) == 0:
                            continue
                        z = float(d) * self.depth_scale
                        if z <= 0.0 or z > self.max_depth_m:
                            continue

                    x = (u - cx) * z / fx
                    y = -(v - cy) * z / fy

                    u_rgb = int(u * w_rgb / w_d)
                    v_rgb = int(v * h_rgb / h_d)
                    u_rgb = max(0, min(u_rgb, w_rgb - 1))
                    v_rgb = max(0, min(v_rgb, h_rgb - 1))

                    b, g, r = self.rgb[v_rgb, u_rgb]
                    rgb = self._pack_rgb_uint32(b, g, r)
                    points.append([x, y, z, rgb])

            rospy.loginfo_throttle(1.0, "points=%d", len(points))

            if points:
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "kinect2_camera_frame"

                cloud = pcl2.create_cloud(header, FIELDS, points)
                self.pub.publish(cloud)

                if self.save_pcd and not self.saved_once:
                    self._save_pcd_binary(points)

            rate.sleep()


if __name__ == "__main__":
    try:
        RGBD2PointCloudSaver().run()
    except rospy.ROSInterruptException:
        pass
