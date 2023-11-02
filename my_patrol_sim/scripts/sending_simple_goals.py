#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('pubpose')
    turtle_vel_pub = rospy.Publisher('robot_1/move_base_simple/goal', PoseStamped, queue_size=1)
    
    mypose=PoseStamped()
    turtle_vel_pub.publish(mypose)  # 先发送一个空位置，试探一下，否则第一个包容易丢
    time.sleep(1)
    
    mypose=PoseStamped()
    mypose.header.frame_id='map'  # 设置自己的目标
    mypose.pose.position.x = 2.325
    mypose.pose.position.y = 16.675
    mypose.pose.position.z=0
    mypose.pose.orientation.x=0
    mypose.pose.orientation.y=0
    mypose.pose.orientation.z=1
    mypose.pose.orientation.w=0
    
    turtle_vel_pub.publish(mypose)  # 发送自己设置的目标点

    time.sleep(5)
