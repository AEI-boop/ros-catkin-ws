#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

current_frame = None

# 空回调函数，用于滑动条
def nothing(x):
    pass

# 鼠标回调函数：点击打印HSV值
def mouse_click(event, x, y, flags, param):
    global current_frame
    if event == cv2.EVENT_LBUTTONDOWN and current_frame is not None:
        hsv_image = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        h, s, v = hsv_image[y, x]
        print(f"坐标({x},{y}) 当前点 HSV -> H:{h}, S:{s}, V:{v}")

def image_callback(msg):
    global current_frame
    bridge = CvBridge()
    try:
        # 将 ROS 图像消息转换为 OpenCV 格式 (BGR8)
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        current_frame = cv_image.copy()
        
        # 1. 颜色空间转换：RGB -> HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 获取滑动条的值
        h_min = cv2.getTrackbarPos('H Min', 'Trackbars')
        s_min = cv2.getTrackbarPos('S Min', 'Trackbars')
        v_min = cv2.getTrackbarPos('V Min', 'Trackbars')
        h_max = cv2.getTrackbarPos('H Max', 'Trackbars')
        s_max = cv2.getTrackbarPos('S Max', 'Trackbars')
        v_max = cv2.getTrackbarPos('V Max', 'Trackbars')

        # 设定阈值的上下限数组
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])

        # 2. 二值化掩膜操作，分割提取目标物
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

        # 对掩膜进行形态学操作（可选），消除噪点
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 与原图进行按位与操作，提取出带原色的目标区域
        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # 3. 计算目标实质的轮廓和质心坐标
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # 找到最大的轮廓（假设最大的就是目标球）
            max_contour = max(contours, key=cv2.contourArea)
            
            # 计算质心的图像矩
            M = cv2.moments(max_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # 在原图画出质心标记和文字
                cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(cv_image, f"Center: ({cx}, {cy})", (cx - 50, cy - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                # 画出目标的边界框
                x, y, w, h = cv2.boundingRect(max_contour)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # 显示各个窗口
        cv2.imshow("RGB Camera", cv_image)
        cv2.imshow("Mask Result", mask)
        cv2.imshow("Color Extraction", result)
        
        # 绑定鼠标事件进行取色
        cv2.setMouseCallback("RGB Camera", mouse_click)
        
        cv2.waitKey(1)

    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")

def main():
    rospy.init_node('hsv_color_tracker', anonymous=True)

    # 创建调整阈值的滑动条控制面板
    cv2.namedWindow('Trackbars')
    cv2.resizeWindow("Trackbars", 400, 300)
    
    # 默认寻找绿色的 HSV 阈值推荐值
    cv2.createTrackbar('H Min', 'Trackbars', 40, 179, nothing)
    cv2.createTrackbar('H Max', 'Trackbars', 80, 179, nothing)
    cv2.createTrackbar('S Min', 'Trackbars', 100, 255, nothing)
    cv2.createTrackbar('S Max', 'Trackbars', 255, 255, nothing)
    cv2.createTrackbar('V Min', 'Trackbars', 100, 255, nothing)
    cv2.createTrackbar('V Max', 'Trackbars', 255, 255, nothing)

    # 订阅 Kinect2 高清彩色图像话题
    image_topic = "/kinect2/hd/image_color_rect"
    rospy.Subscriber(image_topic, Image, image_callback, queue_size=1)
    
    rospy.loginfo("开始运行基于 HSV 的颜色追踪节点！")
    rospy.loginfo("您可以使用 Trackbars 窗口调节阈值，并在 RGB Camera 窗口点击鼠标左键获取目标 HSV 色值。")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("节点已关闭")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
