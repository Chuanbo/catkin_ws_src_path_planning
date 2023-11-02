#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PIL import Image, ImageDraw

import scanf

# 需要读取的 .graph 和 .ppm 文件的名称 
name_only = "maps/grid/grid"
# 保存机器人路径（由一系列位置点组成）的文件名称 
robotpath_filename = "results/grid_4_robotpath.txt"

# 读取.graph文件，初始化地图中节点的数目dimension 
graph_file = open( name_only + '.graph', 'r' )
read_tuple = scanf.fscanf(graph_file, "%d %d %d %f")
dimension = read_tuple[0]
width_px = read_tuple[1]
height_px = read_tuple[2]
resolution = read_tuple[3]
graph_file.close()

mapImage = Image.open( name_only + '_info.ppm' )
draw = ImageDraw.Draw(mapImage)  # 定义画图对象 

robotpath_file = open(robotpath_filename, 'r')

for line in robotpath_file:
    line = line.rstrip()  # remove end-of-line
    if len(line) == 0:
        continue  # 跳过空行 
    parts = line.split(' ')
    that_time = float(parts[0])
    robot_id = int(parts[1])
    x_robot = float(parts[2])
    y_robot = float(parts[3])
    th_robot = float(parts[4])
    # 设置机器人占据的区域，以供画图 
    x_robot_px = int(x_robot/resolution)
    y_robot_px = int(height_px-y_robot/resolution)
    robot_radius = 3  # 画图时的机器人半径，单位像素pixel 
    box = ( x_robot_px-robot_radius, y_robot_px-robot_radius, x_robot_px+robot_radius, y_robot_px+robot_radius )
    if robot_id == 0:
        # blue 
        draw.ellipse( box, outline=(0,0,255), fill=(255,255,255) )  # 将机器人位置用圆形表示画在地图上 
    elif robot_id == 1:
        # red 
        draw.ellipse( box, outline=(255,0,0), fill=(255,255,255) )  # 将机器人位置用圆形表示画在地图上 
    elif robot_id == 2:
        # green 
        draw.ellipse( box, outline=(0,255,0), fill=(255,255,255) )  # 将机器人位置用圆形表示画在地图上 
    else:
        # magenta 
        draw.ellipse( box, outline=(255,0,255), fill=(255,255,255) )  # 将机器人位置用圆形表示画在地图上 

robotpath_file.close()

del draw  # 释放画图对象所占内存 

mapImage.show()

mapImage.save("results/grid_4_robotpath.ppm")
