#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import random
from geometry_msgs.msg import Twist
import sys

def ball_random_move():
    rospy.init_node('ball_random_move', anonymous=True)

    if len(sys.argv) > 1:
        ball_name = sys.argv[1]
        # 用 {小球名}_vel，选定移动的ball
        vel_topic = f"{ball_name}_vel"
    else:
        # 默认控制橙球
        vel_topic = "orange_ball_vel"

    vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=10)
    rospy.sleep(0.5)

    rate = rospy.Rate(0.2)
    rospy.loginfo(f"✅ 控制小球：{vel_topic}")

    while not rospy.is_shutdown():
        x_vel = random.uniform(-0.3, 0.3)
        y_vel = random.uniform(-0.3, 0.3)

        vel_cmd = Twist()
        vel_cmd.linear.x = x_vel
        vel_cmd.linear.y = y_vel
        vel_cmd.angular.z = 0.0

        vel_pub.publish(vel_cmd)
        rospy.loginfo(f"(%.2f , %.2f) -> %s", x_vel, y_vel, vel_topic)
        rate.sleep()

if __name__ == '__main__':
    try:
        ball_random_move()
    except rospy.ROSInterruptException:
        pass
