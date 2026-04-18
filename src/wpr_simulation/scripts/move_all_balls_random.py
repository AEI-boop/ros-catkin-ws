#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import random
from geometry_msgs.msg import Twist

def all_balls_random_move():
    rospy.init_node('all_balls_random_move', anonymous=True)

    # 创建所有球的发布者
    publishers = {
        "orange_ball": rospy.Publisher("orange_ball_vel", Twist, queue_size=10),
        "red_ball": rospy.Publisher("red_ball_vel", Twist, queue_size=10),
        "green_ball": rospy.Publisher("green_ball_vel", Twist, queue_size=10),
        "blue_ball": rospy.Publisher("blue_ball_vel", Twist, queue_size=10)
    }
    
    rospy.sleep(0.5)

    rate = rospy.Rate(0.2) # 每5秒改变一次速度
    rospy.loginfo("🚀 所有小球开始随机运动！")

    while not rospy.is_shutdown():
        # 给每个球生成独立的随机速度并发布
        for ball_name, pub in publishers.items():
            x_vel = random.uniform(-0.3, 0.3)
            y_vel = random.uniform(-0.3, 0.3)

            vel_cmd = Twist()
            vel_cmd.linear.x = x_vel
            vel_cmd.linear.y = y_vel
            vel_cmd.angular.z = 0.0

            pub.publish(vel_cmd)
        
        rospy.loginfo("已更新所有小球的随机速度")
        rate.sleep()

if __name__ == '__main__':
    try:
        all_balls_random_move()
    except rospy.ROSInterruptException:
        pass
