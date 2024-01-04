#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class Transfer_Goal():
    def __init__(self):
        rospy.init_node("transfer_goal")
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_callback, queue_size=1)
        rospy.Subscriber("/robot_reach_goal_str", String, self.reach_goal_callback, queue_size=10)
        self.node_xy_cumberland = np.array( [ [0, 31, 289],
                                [1, 82, 287],
                                [2, 82, 160],
                                [3, 143, 197],
                                [4, 143, 160],
                                [5, 200, 366],
                                [6, 221, 165],
                                [7, 226, 301],
                                [8, 255, 365],
                                [9, 287, 334],
                                [10, 295, 367],
                                [11, 303, 232],
                                [12, 304, 293],
                                [13, 308, 160],
                                [14, 368, 228],
                                [15, 373, 161],
                                [16, 377, 350],
                                [17, 438, 138],
                                [18, 453, 160],
                                [19, 455, 350],
                                [20, 455, 314],
                                [21, 467, 213],
                                [22, 492, 160],
                                [23, 508, 350],
                                [24, 515, 220],
                                [25, 537, 460],
                                [26, 537, 426],
                                [27, 542, 43],
                                [28, 546, 152],
                                [29, 559, 350],
                                [30, 618, 324],
                                [31, 624, 289],
                                [32, 624, 217],
                                [33, 624, 152],
                                [34, 629, 88],
                                [35, 647, 289],
                                [36, 657, 152],
                                [37, 658, 233],
                                [38, 664, 91],
                                [39, 669, 289] ] , dtype=np.float32)
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.resolution = 0.075
        self.node_xy_cumberland[:,1] = self.node_xy_cumberland[:,1] * self.resolution + self.origin_x
        self.node_xy_cumberland[:,2] = self.node_xy_cumberland[:,2] * self.resolution + self.origin_y
        print(self.node_xy_cumberland)
        # self.robot_prefix = "/robot_1"
        # robot goal publisher
        self.robot_0_goal_pub = rospy.Publisher("/robot_0/move_base_simple/goal", PoseStamped, queue_size=1)
        self.robot_1_goal_pub = rospy.Publisher("/robot_1/move_base_simple/goal", PoseStamped, queue_size=1)
        self.robot_2_goal_pub = rospy.Publisher("/robot_2/move_base_simple/goal", PoseStamped, queue_size=1)
        self.robot_3_goal_pub = rospy.Publisher("/robot_3/move_base_simple/goal", PoseStamped, queue_size=1)
        # robot node list
        self.robot_0_node_list = [35, 37, 39, 37, 32, 31, 32, 33, 34, 38, 34, 33, 36, 33, 28, 27, 34, 33, 32, 37]
        self.robot_1_node_list = [0, 2, 1, 2, 4, 3, 4, 6, 7, 8, 7, 5, 7, 6, 4, 2]
        self.robot_2_node_list = [12, 16, 9, 16, 10, 16, 19, 20, 19, 23, 26, 25, 26, 29, 30, 29, 23, 19, 16]
        self.robot_3_node_list = [14, 15, 13, 11, 13, 15, 17, 18, 22, 18, 21, 24, 21, 18, 17, 15]
        # robot node index
        self.robot_0_index = 1
        self.robot_1_index = 1
        self.robot_2_index = 1
        self.robot_3_index = 1
        # robot_pose wait to be published
        self.robot_pose=PoseStamped()
        self.robot_0_goal_pub.publish(self.robot_pose)  # 先发送一个空位置，试探一下，否则第一个包容易丢
        self.robot_1_goal_pub.publish(self.robot_pose)  # 先发送一个空位置，试探一下，否则第一个包容易丢
        self.robot_2_goal_pub.publish(self.robot_pose)  # 先发送一个空位置，试探一下，否则第一个包容易丢
        self.robot_3_goal_pub.publish(self.robot_pose)  # 先发送一个空位置，试探一下，否则第一个包容易丢
        time.sleep(1)
        self.robot_pose.header.frame_id='map'  # 设置自己的目标
        self.robot_pose.pose.position.x=0
        self.robot_pose.pose.position.y=0
        self.robot_pose.pose.position.z=0
        self.robot_pose.pose.orientation.x=0
        self.robot_pose.pose.orientation.y=0
        self.robot_pose.pose.orientation.z=1
        self.robot_pose.pose.orientation.w=0
        # publish the first node
        self.robot_pose.pose.position.x = self.node_xy_cumberland[self.robot_0_node_list[0]][1]
        self.robot_pose.pose.position.y = self.node_xy_cumberland[self.robot_0_node_list[0]][2]
        self.robot_0_goal_pub.publish(self.robot_pose)  # 发送设置的目标点
        self.robot_pose.pose.position.x = self.node_xy_cumberland[self.robot_1_node_list[0]][1]
        self.robot_pose.pose.position.y = self.node_xy_cumberland[self.robot_1_node_list[0]][2]
        self.robot_1_goal_pub.publish(self.robot_pose)  # 发送设置的目标点
        self.robot_pose.pose.position.x = self.node_xy_cumberland[self.robot_2_node_list[0]][1]
        self.robot_pose.pose.position.y = self.node_xy_cumberland[self.robot_2_node_list[0]][2]
        self.robot_2_goal_pub.publish(self.robot_pose)  # 发送设置的目标点
        self.robot_pose.pose.position.x = self.node_xy_cumberland[self.robot_3_node_list[0]][1]
        self.robot_pose.pose.position.y = self.node_xy_cumberland[self.robot_3_node_list[0]][2]
        self.robot_3_goal_pub.publish(self.robot_pose)  # 发送设置的目标点


    def goal_pose_callback(self, msg):
        self.robot_1_goal_pub.publish(msg)

    def reach_goal_callback(self, msg):
        if int(msg.data) == 0:
            self.robot_pose.pose.position.x = self.node_xy_cumberland[self.robot_0_node_list[self.robot_0_index]][1]
            self.robot_pose.pose.position.y = self.node_xy_cumberland[self.robot_0_node_list[self.robot_0_index]][2]
            self.robot_0_goal_pub.publish(self.robot_pose)  # 发送设置的目标点
            self.robot_0_index += 1
            self.robot_0_index = self.robot_0_index % len(self.robot_0_node_list)
        elif int(msg.data) == 1:
            self.robot_pose.pose.position.x = self.node_xy_cumberland[self.robot_1_node_list[self.robot_1_index]][1]
            self.robot_pose.pose.position.y = self.node_xy_cumberland[self.robot_1_node_list[self.robot_1_index]][2]
            self.robot_1_goal_pub.publish(self.robot_pose)  # 发送设置的目标点
            self.robot_1_index += 1
            self.robot_1_index = self.robot_1_index % len(self.robot_1_node_list)
        elif int(msg.data) == 2:
            self.robot_pose.pose.position.x = self.node_xy_cumberland[self.robot_2_node_list[self.robot_2_index]][1]
            self.robot_pose.pose.position.y = self.node_xy_cumberland[self.robot_2_node_list[self.robot_2_index]][2]
            self.robot_2_goal_pub.publish(self.robot_pose)  # 发送设置的目标点
            self.robot_2_index += 1
            self.robot_2_index = self.robot_2_index % len(self.robot_2_node_list)
        elif int(msg.data) == 3:
            self.robot_pose.pose.position.x = self.node_xy_cumberland[self.robot_3_node_list[self.robot_3_index]][1]
            self.robot_pose.pose.position.y = self.node_xy_cumberland[self.robot_3_node_list[self.robot_3_index]][2]
            self.robot_3_goal_pub.publish(self.robot_pose)  # 发送设置的目标点
            self.robot_3_index += 1
            self.robot_3_index = self.robot_3_index % len(self.robot_3_node_list)
        else:
            pass


if __name__ == '__main__':
    time.sleep(15)  # wait because of roslaunch
    Transfer_Goal()
    rospy.spin()
