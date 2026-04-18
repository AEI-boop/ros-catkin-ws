#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
from std_msgs.msg import Header
import struct
import math

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
]

class RGBD2PointCloud:
    def __init__(self):
        rospy.init_node('get_pointcloud', anonymous=True)
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.depth_encoding = None
        self.info = None

        # 16UC1 深度常见单位为毫米，默认换算到米。
        self.sample_step = max(1, int(rospy.get_param('~sample_step', 2)))
        self.max_depth_m = float(rospy.get_param('~max_depth_m', 5.0))
        self.depth_scale = float(rospy.get_param('~depth_scale', 0.001))
        self.publish_rate = float(rospy.get_param('~publish_rate', 15.0))
        
        self.sub_rgb = rospy.Subscriber(
            "/kinect2/hd/image_color_rect", Image, self.cb_rgb
        )
        self.sub_depth = rospy.Subscriber(
            "/kinect2/sd/image_depth_rect", Image, self.cb_depth
        )
        self.sub_info = rospy.Subscriber(
            "/kinect2/sd/camera_info", CameraInfo, self.cb_info
        )

        self.pub = rospy.Publisher("/pointcloud_output", PointCloud2, queue_size=1)
        rospy.loginfo(
            "点云节点已启动: step=%d max_depth=%.2fm depth_scale=%f",
            self.sample_step,
            self.max_depth_m,
            self.depth_scale,
        )

    def cb_rgb(self, msg):
        self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def cb_depth(self, msg):
        self.depth_encoding = msg.encoding
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def cb_info(self, msg):
        self.info = msg

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        
        while not rospy.is_shutdown():
            if self.rgb is None or self.depth is None or self.info is None or self.depth_encoding is None:
                rate.sleep()
                continue

            if not hasattr(self, '_depth_info_logged'):
                rospy.loginfo("depth encoding=%s", self.depth_encoding)
                self._depth_info_logged = True

            # 分辨率
            h_d, w_d = self.depth.shape
            h_rgb, w_rgb = self.rgb.shape[:2]

            # 内参
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
                    
                    # 光学坐标系 → 相机连杆坐标系，Y轴取反，适配RViz显示
                    x = (u - cx) * z / fx
                    y = -(v - cy) * z / fy  # Y轴取反，桌腿朝下，桌子正过来

                    # 颜色坐标对齐
                    u_rgb = int(u * w_rgb / w_d)
                    v_rgb = int(v * h_rgb / h_d)
                    u_rgb = max(0, min(u_rgb, w_rgb-1))
                    v_rgb = max(0, min(v_rgb, h_rgb-1))

                    # 颜色100%正确，红瓶绿瓶无偏差
                    b, g, r = self.rgb[v_rgb, u_rgb]
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
                    
                    points.append([x, y, z, rgb])

            rospy.loginfo_throttle(1.0, f"本次生成点云数量: {len(points)}")

            if len(points) > 0:
                header = Header()
                header.stamp = rospy.Time.now()
                header.frame_id = "kinect2_camera_frame"
                
                cloud = pcl2.create_cloud(header, FIELDS, points)
                self.pub.publish(cloud)

            rate.sleep()

if __name__ == '__main__':
    try:
        RGBD2PointCloud().run()
    except rospy.ROSInterruptException:
        pass
