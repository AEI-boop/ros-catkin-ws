#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# 适配不同版本的 OpenCV，直接从 Linux 系统默认路径中去加载层联模型
possible_paths = [
    '/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml',
    '/usr/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml'
]

cascade_path = None
for p in possible_paths:
    if os.path.exists(p):
        cascade_path = p
        break

if not cascade_path:
    print("Error: 找不到系统内置的 haarcascade_frontalface_default.xml 文件")
    
face_cascade = cv2.CascadeClassifier(cascade_path)

bridge = None
cmd_pub = None
IMAGE_CENTER_X = 960 / 2
KP_STEER = 0.0008     # 【修改】进一步降低转向灵敏度，缓解左右摆动(震荡)
FORWARD_SPEED = 0.08  # 【修改】降低前进速度，慢慢靠近
photo_count = 0

# 添加一个缓存机制机制：防止识别算法偶尔一两帧没检测到导致的“卡顿”和频繁丢失
lost_frames = 0
MAX_LOST_FRAMES = 15  # 允许最多丢失15帧（约0.5秒）还可以凭惯性继续追踪
last_twist = Twist()

def image_callback(msg):
    global IMAGE_CENTER_X, photo_count, lost_frames, last_twist
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        return

    height, width, _ = cv_image.shape
    IMAGE_CENTER_X = width / 2

    # 1. 转为灰度图做人脸识别
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    # 降低 minNeighbors (例如从5降到4) 可以放宽识别条件，对侧脸或边缘更敏感，减少突然跟丢的概率
    faces = face_cascade.detectMultiScale(gray, 1.1, 4, minSize=(30, 30))

    twistMsg = Twist()
    
    if len(faces) > 0:
        lost_frames = 0 # 找到了，清空丢失计数器
        
        # 如果有多张脸，只取最大（最近）的那张脸
        max_face = max(faces, key=lambda f: f[2] * f[3])
        x, y, w, h = max_face
        
        # 脸的质心
        cx = x + w // 2
        cy = y + h // 2
        
        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)

        # 比例跟随：通过转向对准人脸
        error_x = IMAGE_CENTER_X - cx
        
        # 【修改】增加“死区(Deadzone)”：如果脸已经在画面中央附近(偏差小于20像素)，就不要再转了，防止左右鬼畜微调
        if abs(error_x) < 20:
            error_x = 0

        angular_speed = KP_STEER * error_x
        # 限制最大转向速度，并且将最大速度进一步压缩到0.5避免猛摆
        twistMsg.angular.z = max(min(angular_speed, 0.5), -0.5) 

        # 进一步缩减靠近的比例（改大系数分母 /6.0），保持更安全的远距离拍照
        if h < height / 6.0: 
            twistMsg.linear.x = FORWARD_SPEED
        else:
            twistMsg.linear.x = 0.0
            cv2.putText(cv_image, "PERFECT DISTANCE!", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3)

        # 简单逻辑：如果发现目标几乎在画布正中央（对准了），说明是个好侧写时刻，触发拍照
        if abs(error_x) < 30 and photo_count < 3 and twistMsg.linear.x == 0.0:
            filename = f"/home/starjie/catkin_ws/src/image_pkg/scripts/handsome_guy_{photo_count}.jpg"
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"📸 咔嚓！拍到了帅哥的一张写真！保存在: {filename}")
            photo_count += 1
            # 加个巨大的滤镜动画反馈说拍了
            cv2.putText(cv_image, "* FLASH *", (cx - 50, cy - 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,255), 4)

        # 记录下这次的有效移动指令，以备不时之需
        last_twist = twistMsg

    else:
        # 如果当前帧没检测到脸，但还在允许的容忍范围内，就继续执行上一帧的动作（惯性），假装还在跟
        if lost_frames < MAX_LOST_FRAMES:
            lost_frames += 1
            twistMsg = last_twist
            cv2.putText(cv_image, "Tracking... (Buffering)", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,165,255), 2)
        else:
            # 彻底跟丢了
            cv2.putText(cv_image, "LOST TARGET!", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)
            # 停止运动
            twistMsg = Twist()

    cmd_pub.publish(twistMsg)

    cv2.imshow("Paparazzi Bot", cv_image)
    cv2.waitKey(3)

def main():
    global cmd_pub, bridge
    rospy.init_node('face_follow_paparazzi', anonymous=True)
    bridge = CvBridge()
    
    rospy.Subscriber("/kinect2/hd/image_color_rect", Image, image_callback, queue_size=1)
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.loginfo("🤖 狗仔机器人已启动！目标：寻找帅哥并拍摄写真！")
    
    rospy.spin()
    
    cmd_pub.publish(Twist())
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()