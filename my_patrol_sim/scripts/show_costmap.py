#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import copy
import cv2
from nav_msgs.msg import OccupancyGrid
# import message_filters

class CostMap(object):
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
        
        rospy.init_node('subcostmap')

        # 刚刚通过rostopic list查阅出来的话题名
        costmap_topic = '/robot_1/move_base_node/local_costmap/costmap'
        # 编写一个subscriber 参数分别为：
        # topic名
        # 消息类型
        # 绑定自己重命名的callback function
        self.costmap = rospy.Subscriber(costmap_topic, OccupancyGrid, self.get_cost_map)

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
        
        width = copy.deepcopy(self.width)
        height = copy.deepcopy(self.height)
        map_size = [width, height]
        map_data = copy.deepcopy(self.map_data)
        
        return map_size, map_data

def draw_map(map_size, map_data):
    """
        This function returns the RGB image perceived by the camera.
        :rtype: np.ndarray or None
    """
    row, col = map_size[1], map_size[0]  # 一维转二维，注意对应关系
    costmap = np.array(map_data)
    costmap = costmap.reshape((row, col))
    img = np.zeros((row, col, 3))
    for i in range(row):
        for j in range(col):
        	# 判断无障碍点、未知点和障碍点 画不同颜色
            if costmap[i][j] == 0:
                img[i][j] = [255, 255, 255]
            elif costmap[i][j] == -1:
                img[i][j] = [255, 0, 0]
            else:
                img[i][j] = [0, 0, 0]
    cv2.imshow('map',img)
    cv2.waitKey(0)


def main():
    print('Start')
    costmap = CostMap()
    
    print('Load...')
    map_size, map_data = costmap.get_map()
	
    draw_map(map_size, map_data)
    print('Finished!')


if __name__ == "__main__":
    main()
