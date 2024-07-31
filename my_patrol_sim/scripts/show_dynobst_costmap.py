#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import copy
import cv2
from nav_msgs.msg import OccupancyGrid
from dynamic_obstacle_detector.msg import DynamicObstacles
from sensor_msgs.msg import LaserScan
# import message_filters

class DynObstCostMap(object):
    """
    This is a class to get the Global Cost Map.
    """

    def __init__(self):
        """
        Constructor for Global Cost map class.
        :param configs: configurations for costmap
        :type configs: OccupancyGrid
        """
        self.occupancygrid_msg = None
        self.map_data = None
        self.width = None
        self.height = None
        self.dynamicobstacles_msg = None

        rospy.init_node("dynobst_costmap")

        rospy.Subscriber("/map", OccupancyGrid, self.get_cost_map)
        rospy.Subscriber("/dynamic_obstacles", DynamicObstacles, self.get_dynamic_obstacles)
        rospy.Subscriber("/robot_0/base_scan", LaserScan, self.laserscan_callback, queue_size=1)

    def get_dynamic_obstacles(self, msg):
        self.dynamicobstacles_msg = msg
        if self.occupancygrid_msg is None:
            pass
        else:
            for index in range(len(msg.obstacles)):
                ob_x = msg.obstacles[index].position.x
                ob_y = msg.obstacles[index].position.y
                ob_vx = msg.obstacles[index].velocity.x
                ob_vy = msg.obstacles[index].velocity.y
                print(ob_x, ob_y, ob_vx, ob_vy)
            print(len(msg.obstacles))

    def laserscan_callback(self, scan):
        self.scan_msg = scan

    def get_cost_map(self, msg):
        """
        costmap subscriber's callback function.
        """
        self.occupancygrid_msg = msg
        self.map_data = msg.data
        self.width = msg.info.width
        self.height = msg.info.height


    def get_map(self):
        """
        This function returns the size and 2D costmap.
        :rtype: [int, int], np.ndarray
        """
        while(not self.occupancygrid_msg):
            continue
        while(not self.dynamicobstacles_msg):
            continue
        
        width = copy.deepcopy(self.width)
        height = copy.deepcopy(self.height)
        map_size = [width, height]
        map_data = copy.deepcopy(self.map_data)
        dynobst_num = len(self.dynamicobstacles_msg.obstacles)
        dynobst_msg = copy.deepcopy(self.dynamicobstacles_msg)
        scan_msg = copy.deepcopy(self.scan_msg)

        return map_size, map_data, dynobst_num, dynobst_msg, scan_msg

def draw_map(map_size, map_data, dynobst_num, dynobst_msg, scan_msg):
    """
        This function returns the RGB image.
        :rtype: np.ndarray or None
    """
    row, col = map_size[1], map_size[0]  # 一维转二维，注意对应关系
    my_costmap = np.array(map_data)
    my_costmap = my_costmap.reshape((row, col))
    img = np.zeros((row, col, 3))
    for i in range(row):
        for j in range(col):
        	# 判断无障碍点、未知点和障碍点 画不同颜色
            if my_costmap[i][j] == 0:
                img[i][j] = [255, 255, 255]
            elif my_costmap[i][j] == -1:
                img[i][j] = [255, 0, 0]
            else:
                img[i][j] = [0, 0, 0]
    for index in range(dynobst_num):
        ob_x = dynobst_msg.obstacles[index].position.x
        ob_y = dynobst_msg.obstacles[index].position.y
        ob_vx = dynobst_msg.obstacles[index].velocity.x
        ob_vy = dynobst_msg.obstacles[index].velocity.y
        mx = (int)((ob_x) / 0.05)
        my = (int)((ob_y) / 0.05)
        print(my, mx)
        img[my][mx] = [0, 0, 255]
    print(row, col)
    rp = [2.0, 3.0, 0.0]
    cached_cos = np.cos( np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment) + rp[2] )
    cached_sin = np.sin( np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment) + rp[2] )
    # Convert scan from polar to cartesian coordinate system
    pc_x = scan_msg.ranges * cached_cos + rp[0]
    pc_y = scan_msg.ranges * cached_sin + rp[1]
    for i in range(len(pc_x)):
        sx = (int)(pc_x[i] / 0.05)
        sy = (int)(pc_y[i] / 0.05)
        img[sy][sx] = [0, 255, 0]

    cv2.imshow('map',img)
    cv2.waitKey(0)


def main():
    print('Start')
    dynobst_costmap = DynObstCostMap()

    rospy.sleep(5)

    print('Load...')
    map_size, map_data, dynobst_num, dynobst_msg, scan_msg = dynobst_costmap.get_map()

    draw_map(map_size, map_data, dynobst_num, dynobst_msg, scan_msg)
    print('Finished!')


if __name__ == "__main__":
    main()
