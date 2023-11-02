#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Tkinter
import tkMessageBox
from PIL import Image, ImageTk, ImageDraw

import sys
import os
import time

# 目前仅用到rospy的time 
# import rospy

import scanf

import math


# 全局变量 
patrol_is_start = False  # 巡逻是否开始 

# 需要读取的 .graph 和 -graph.png 文件的名称 
mapImage_filename = "maps/grid/grid_info.ppm"
mapGraph_filename = "maps/grid/grid.graph"

# 保存机器人路径（由一系列位置点组成）的文件名称 
robotpath_filename = "results/grid_4_robotpath.txt"

# 临时的初值，读取graph文件后会赋值为与地图相匹配的值 
dimension = 1
width_px = 1
height_px = 1
resolution = 0.05


# 程序界面的类的定义 
class MyApp:

    def __init__(self, master):
        
        self.root = master
    
        textLabel = Tkinter.Label(master, text='Topological Map')
        textLabel.pack()
        
        imageFrame = Tkinter.Frame(master, relief=Tkinter.RAISED, borderwidth=1)
        imageFrame.pack(side=Tkinter.TOP)
        global mapImage_filename
        self.mapImage = Image.open( mapImage_filename )
        mapPhoto = ImageTk.PhotoImage(self.mapImage)
        self.imageLabel = Tkinter.Label(imageFrame, image=mapPhoto)
        self.imageLabel.image = mapPhoto  # keep a reference! 
        self.imageLabel.pack()
        
        buttonFrame = Tkinter.Frame(master)
        buttonFrame.pack(side=Tkinter.BOTTOM)
        self.startButton = Tkinter.Button(buttonFrame, text="Start", command=self.send_start)
        self.startButton.pack(side=Tkinter.LEFT, padx=10, pady=10)
        self.stopButton = Tkinter.Button(buttonFrame, text="Stop", command=self.send_stop)
        self.stopButton.pack(side=Tkinter.RIGHT, padx=10, pady=10)
        
        # 读取.graph文件，初始化地图中节点的数目dimension 
        global mapGraph_filename
        graph_file = open( mapGraph_filename, 'r' )
        read_tuple = scanf.fscanf(graph_file, "%d %d %d %f")
        global dimension
        global width_px
        global height_px
        global resolution
        dimension = read_tuple[0]
        width_px = read_tuple[1]
        height_px = read_tuple[2]
        resolution = read_tuple[3]
        graph_file.close()
        # 目前，对graph文件后面的各节点信息并未加以利用 
        
        # 打开记录机器人路径的文件 
        global robotpath_filename
        self.robotpath_file = open(robotpath_filename, 'r')
        
        mapImage = self.mapImage.copy()  # 复制图像，类的成员变量self.mapImage存储着地图，将机器人位置用圆形表示画在地图上 
        draw = ImageDraw.Draw(mapImage)  # 定义画图对象 
        
        for i in range(0,4):
            line = self.robotpath_file.readline()
            line = line.rstrip()  # remove end-of-line 
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
                draw.ellipse( box, outline=(0,0,255), fill=(0,0,255) )  # 将机器人位置用圆形表示画在地图上 
            elif robot_id == 1:
                # red 
                draw.ellipse( box, outline=(255,0,0), fill=(255,0,0) )  # 将机器人位置用圆形表示画在地图上 
            elif robot_id == 2:
                # green 
                draw.ellipse( box, outline=(0,255,0), fill=(0,255,0) )  # 将机器人位置用圆形表示画在地图上 
            else:
                # magenta 
                draw.ellipse( box, outline=(255,0,255), fill=(255,0,255) )  # 将机器人位置用圆形表示画在地图上 
        
        del draw  # 释放画图对象所占内存 
        mapPhoto = ImageTk.PhotoImage(mapImage)
        self.imageLabel.configure(image=mapPhoto)  # 更新Label显示的图像 
        self.imageLabel.image = mapPhoto  # keep a reference! 
        
        
    def send_start(self):
        
        global patrol_is_start
        patrol_is_start = True
        
        self.update_image()  # 更新机器人的位置信息，并在界面上显示 
        
        tkMessageBox.showinfo('Message','Patrol Start')

    
    def send_stop(self):
        
        global patrol_is_start
        patrol_is_start = False
        
        # 关闭记录机器人路径的文件 
        self.robotpath_file.close()
        
        tkMessageBox.showinfo('Message','Patrol Stop')
    
    
    # 更新机器人的位置信息，并在界面上显示 
    def update_image(self):
    
        global patrol_is_start
        if patrol_is_start == False:
            self.root.after(1000, self.update_image)  # 1000ms(1s)后，再次调用self.update_image函数 
            return
        
        mapImage = self.mapImage.copy()  # 复制图像，类的成员变量self.mapImage存储着地图，将机器人位置用圆形表示画在地图上 
        draw = ImageDraw.Draw(mapImage)  # 定义画图对象 
        
        for i in range(0,4):
            line = self.robotpath_file.readline()
            if line == '':
                # 已到文档结尾，停止刷新图像 
                self.send_stop()
                return
            line = line.rstrip()  # remove end-of-line 
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
                draw.ellipse( box, outline=(0,0,255), fill=(0,0,255) )  # 将机器人位置用圆形表示画在地图上 
            elif robot_id == 1:
                # red 
                draw.ellipse( box, outline=(255,0,0), fill=(255,0,0) )  # 将机器人位置用圆形表示画在地图上 
            elif robot_id == 2:
                # green 
                draw.ellipse( box, outline=(0,255,0), fill=(0,255,0) )  # 将机器人位置用圆形表示画在地图上 
            else:
                # magenta 
                draw.ellipse( box, outline=(255,0,255), fill=(255,0,255) )  # 将机器人位置用圆形表示画在地图上 
        
        del draw  # 释放画图对象所占内存 
        mapPhoto = ImageTk.PhotoImage(mapImage)
        self.imageLabel.configure(image=mapPhoto)  # 更新Label显示的图像 
        self.imageLabel.image = mapPhoto  # keep a reference! 
        
        self.root.after(1000, self.update_image)  # 1000ms(1s)后，再次调用self.update_image函数 
    

# 主函数 
def main():
    
    root = Tkinter.Tk()
    root.title('Patrol Control')
    myapp = MyApp(root)
    root.mainloop()
    

if __name__ == '__main__':
    main()
