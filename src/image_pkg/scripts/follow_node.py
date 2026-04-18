#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# --- 跟踪配置 ---
# 目标颜色 HSV 阈值 (修改为蓝色识别)
H_MIN, S_MIN, V_MIN = 100, 100, 100
H_MAX, S_MAX, V_MAX = 140, 255, 255

# 画面宽度中心目标
IMAGE_CENTER_X = 960 / 2  # Kinect2 HD的分辨率宽大概1920，中心是960（具体看实际消息）

# 比例控制参数 (P控制)
KP_STEER = 0.0015   # 转向速度比例系数：减小以防止甩头太猛
FORWARD_SPEED = 0.2 # 适度降低前进速度，慢慢逼近
MIN_AREA_THRESHOLD = 500  # 球的最小面积阈值，防止把噪点当成球

cmd_pub = None
bridge = None

def image_callback(msg):
    global IMAGE_CENTER_X
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"CV Bridge Error: {e}")
        return

    # 动态获取画面实际宽度中心
    height, width, _ = cv_image.shape
    IMAGE_CENTER_X = width / 2

    # HSV 颜色提取
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([H_MIN, S_MIN, V_MIN])
    upper_bound = np.array([H_MAX, S_MAX, V_MAX])
    
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # 找轮廓提取质心
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    twistMsg = Twist() # 默认全是0（即停止）

    if len(contours) > 0:
        # 找最大的色块
        max_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(max_contour)
        
        # 面积必须大于阈值，防止看到背景一点点绿就疯跑
        if area > MIN_AREA_THRESHOLD:
            M = cv2.moments(max_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00']) # 球的水平中点
                cy = int(M['m01'] / M['m00']) 
                
                # 可视化：画准星
                cv2.circle(cv_image, (cx, cy), 10, (0,0,255), -1)
                
                # --- 核心：比例跟随控制 ---
                # 设定：偏差 = 图片中心 - 球心。 
                # (如果球在画面左半边，则 cx < 画面中心，误差为正，导致正向角速度，即左转寻找)
                error_x = IMAGE_CENTER_X - cx
                
                # 发布前进速度
                twistMsg.linear.x = FORWARD_SPEED 
                
                # 旋转速度 = P系数 * 画面水平偏差，并做限幅防止剧烈甩飞
                angular_speed = KP_STEER * error_x
                twistMsg.angular.z = max(min(angular_speed, 1.0), -1.0) # 最大允许1.0 rad/s

                cv2.putText(cv_image, f"Dir: {twistMsg.angular.z:.2f}", (cx - 30, cy - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
    else:
        # 视野里没求，或者色块太小 -> 停下！
        cv2.putText(cv_image, "LOST TARGET!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)

    # 通过cmd_pub发布底层驱动消息给小车
    cmd_pub.publish(twistMsg)

    # 渲染小车看到加工后的最终画面
    # 显示小车前行瞄准线
    cv2.line(cv_image, (int(IMAGE_CENTER_X), 0), (int(IMAGE_CENTER_X), height), (0, 255, 0), 1) 
    cv2.imshow("Robot Follow View", cv_image)
    cv2.waitKey(3)


def main():
    global cmd_pub, bridge
    rospy.init_node('follow_node', anonymous=True)
    bridge = CvBridge()
    
    # 订阅相机图像
    rospy.Subscriber("/kinect2/hd/image_color_rect", Image, image_callback, queue_size=1)
    # 发布底盘移动消息
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.loginfo("开始颜色跟随！（通过发布 cmd_vel 控制底盘）")
    
    rospy.spin()
    
    # 程序退出时必须发布零速让车停下，不然它会保持最后一个指令的速度一直跑
    cmd_pub.publish(Twist())
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
