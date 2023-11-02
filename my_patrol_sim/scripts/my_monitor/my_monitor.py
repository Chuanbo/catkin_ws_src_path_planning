#!/usr/bin/env python
# -*- coding: utf-8 -*-

import tkinter
import tkinter.messagebox
from PIL import Image, ImageTk, ImageDraw

import threading

import select

import lcm
from exlcm import command_t
from exlcm import status_t
from exlcm import idleness_t

import sys
import os
import time

# 目前仅用到rospy的time 
import rospy

import math


# 全局变量 
myLockStatus = threading.RLock()  # 机器人状态的互斥锁 
statusDic = {}  # 用字典表示机器人状态，机器人编号为字典的键值 

patrol_is_start = False  # 巡逻是否开始 

myLockIdleness = threading.RLock()  # 节点空闲率的互斥锁 
lastVisitList = []  # 用列表表示节点最近一次被访问的时间 
instantIdlenessList = []  # 用列表表示节点的瞬时空闲率 
visitCountList = []  # 用列表表示节点被访问的次数 
maxIdlenessList = []  # 用列表表示节点的最大瞬时空闲率 
meanIdlenessList = []  # 用列表表示节点的平均空闲率 
M2IdlenessList = []  # 用列表表示节点空闲率的M2 
stdDevIdlenessList = []  # 用列表表示节点空闲率的标准差 

idlenessRecordDic = {}  # 用字典存储各节点瞬时空闲率的历史信息 

# 需要读取的 .graph 和 -graph.png 文件的名称 
mapImage_filename = "maps/grid/grid_info.ppm"
mapGraph_filename = "maps/grid/grid.graph"
# 保存巡逻结果的文件名称 
results_filename = "results/grid_results.txt"
# 保存机器人路径（由一系列位置点组成）的文件名称 
robotpath_filename = "results/grid_robotpath.txt"
# 保存图中各节点的瞬时空闲率随时间的变化 
timeidleness_filename = "results/grid_timeidleness.txt"

# 临时的初值，读取graph文件后会赋值为与地图相匹配的值 
dimension = 1
width_px = 1
height_px = 1
resolution = 0.05


def readvalue_line(graph_file):
    # 读取文件中的数值(返回值为字符串)，每行只有一个数值 
    line = graph_file.readline()
    line = line.rstrip()  # remove end-of-line
    if len(line) == 0:
        return readvalue_line(graph_file)  # 递归调用,跳过空行 
    return line


# lcm消息处理线程的类的定义 
class myThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self._stop = threading.Event()


    def stop(self):
        self._stop.set()


    def run(self):
        
        # 读取.graph文件，初始化地图中节点的数目dimension 
        graph_file = open( mapGraph_filename, 'r' )
        global dimension
        global width_px
        global height_px
        global resolution
        dimension = int(readvalue_line(graph_file))
        width_px = int(readvalue_line(graph_file))
        height_px = int(readvalue_line(graph_file))
        resolution = float(readvalue_line(graph_file))
        graph_file.close()
        # 目前，对graph文件后面的各节点信息并未加以利用 
        
        # 打开记录机器人路径的文件 
        global robotpath_filename
        self.robotpath_file = open(robotpath_filename, 'w')
        
        # lastVisitList 等全局变量的初始化在 MyApp 中的 send_start 函数中执行 
        
        lc = lcm.LCM()
        subscription_status = lc.subscribe("ROBOTSTATUS", self.my_handler_status)
        subscription_idleness = lc.subscribe("VERTEXIDLENESS", self.my_handler_idleness)
        
        try:
            timeout = 0.5  # amount of time to wait, in seconds 
            while True:
                rfds, wfds, efds = select.select([lc.fileno()], [], [], timeout)
                if rfds:
                    lc.handle()
                else:
                    if self._stop.isSet():
                        break
        except KeyboardInterrupt:
            pass
        
        lc.unsubscribe(subscription_status)
        lc.unsubscribe(subscription_idleness)
        
        # 关闭记录机器人路径的文件 
        self.robotpath_file.close()


    # 机器人状态消息处理函数 
    def my_handler_status(self, channel, data):
        msg = status_t.decode(data)
        global myLockStatus
        global statusDic
        global height_px
        global resolution
        # 将机器人状态存储在全局变量(字典)里 
        myLockStatus.acquire()
        # 此处将机器人位置由 世界坐标系(rviz坐标系)(y轴向上) 转换到 图像坐标系(画图时使用)(y轴向下) 
        statusDic[msg.robot_id] = [ int(msg.x_robot/resolution), int(height_px-msg.y_robot/resolution) ]
        myLockStatus.release()
        
        global patrol_is_start
        if patrol_is_start == False:
            return
        
        self.robotpath_file.write('%f ' % (time.time()))
        self.robotpath_file.write('%d ' % (msg.robot_id))
        self.robotpath_file.write('%f ' % (msg.x_robot))
        self.robotpath_file.write('%f ' % (msg.y_robot))
        self.robotpath_file.write('%f\n' % (msg.th_robot))
    
    
    # 节点空闲率消息处理函数 
    def my_handler_idleness(self, channel, data):
        msg = idleness_t.decode(data)
        
        global patrol_is_start
        if patrol_is_start == False:
            return
        
        global dimension
        global myLockIdleness
        global lastVisitList
        global instantIdlenessList
        global visitCountList
        global maxIdlenessList
        global meanIdlenessList
        global M2IdlenessList
        global stdDevIdlenessList
        myLockIdleness.acquire()
        if msg.visit_time > lastVisitList[msg.vertex_id]:
            index = msg.vertex_id
            if visitCountList[index] < 0:
                # 前几次的节点访问不记入统计，达到稳定巡逻状态时再开始统计 
                visitCountList[index] += 1
            else:
                lastIdleness = msg.visit_time - lastVisitList[index]
                delta = lastIdleness - meanIdlenessList[index]
                visitCountList[index] += 1
                meanIdlenessList[index] += delta / visitCountList[index]
                M2IdlenessList[index] += delta * (lastIdleness - meanIdlenessList[index])
#                stdDevIdlenessList[index] = math.sqrt(M2IdlenessList[index] / visitCountList[index])
                if lastIdleness > maxIdlenessList[index]:
                    maxIdlenessList[index] = lastIdleness
                idlenessRecordDic[index].append(lastIdleness)
            
            lastVisitList[msg.vertex_id] = msg.visit_time
            for i in range(0, dimension):
                instantIdlenessList[i] = msg.visit_time - lastVisitList[i]
            print (lastVisitList)
#            print (instantIdlenessList)
            print (visitCountList)
        myLockIdleness.release()


# 程序界面的类的定义 
class MyApp:

    def __init__(self, master):
    
        # 目前仅用到rospy的time 
        rospy.init_node('my_monitor', anonymous=True, log_level=rospy.INFO, disable_signals=True)
        
        self.root = master
    
        textLabel = tkinter.Label(master, text='Topological Map')
        textLabel.pack()
        
        imageFrame = tkinter.Frame(master, relief=tkinter.RAISED, borderwidth=1)
        imageFrame.pack(side=tkinter.TOP)
        self.mapImage = Image.open( mapImage_filename )
        mapPhoto = ImageTk.PhotoImage(self.mapImage)
        self.imageLabel = tkinter.Label(imageFrame, image=mapPhoto)
        self.imageLabel.image = mapPhoto  # keep a reference! 
        self.imageLabel.pack()
        
        buttonFrame = tkinter.Frame(master)
        buttonFrame.pack(side=tkinter.BOTTOM)
        self.startButton = tkinter.Button(buttonFrame, text="Start", command=self.send_start)
        self.startButton.pack(side=tkinter.LEFT, padx=10, pady=10)
        self.stopButton = tkinter.Button(buttonFrame, text="Stop", command=self.send_stop)
        self.stopButton.pack(side=tkinter.RIGHT, padx=10, pady=10)
        
        self.lc = lcm.LCM()
        
        self.update_image()  # 更新机器人的位置信息，并在界面上显示 
        
        
    def send_start(self):
        
        # 初始化节点空闲率列表 
        global dimension
        global myLockIdleness
        global lastVisitList
        global instantIdlenessList
        global visitCountList
        global maxIdlenessList
        global meanIdlenessList
        global M2IdlenessList
        global stdDevIdlenessList
        seconds = rospy.get_time()  # Get the current time in float seconds 
        myLockIdleness.acquire()
        lastVisitList = [ seconds ] * dimension
        instantIdlenessList = [ 0.0 ] * dimension
        visitCountList = [ -1 ] * dimension  # 前几次的节点访问不记入统计，达到稳定巡逻状态时再开始统计 
        maxIdlenessList = [ 0.0 ] * dimension
        meanIdlenessList = [ 0.0 ] * dimension
        M2IdlenessList = [ 0.0 ] * dimension
        stdDevIdlenessList = [ 0.0 ] * dimension
        for i in range(0, dimension):
            idlenessRecordDic[i] = []
        myLockIdleness.release()
        
        # 打开记录图中各节点的瞬时空闲率随时间变化的文件 
        global timeidleness_filename
        self.timeidleness_file = open(timeidleness_filename, 'w')
        
        global patrol_is_start
        patrol_is_start = True
        
        # 向巡逻机器人发布开始命令 
        msg = command_t()
        msg.is_start = True
        msg.is_stop = False
        self.lc.publish("ROBOTCOMMAND", msg.encode())
        
        tkinter.messagebox.showinfo('Message','Send Start Command')

    
    def send_stop(self):
        
        # 向巡逻机器人发布停止命令 
        msg = command_t()
        msg.is_start = False
        msg.is_stop = True
        self.lc.publish("ROBOTCOMMAND", msg.encode())
        
        global patrol_is_start
        patrol_is_start = False
        
        tkinter.messagebox.showinfo('Message','Send Stop Command')
        
        # 将巡逻统计结果保存到文件 
        if os.path.exists(results_filename):
            outputfile = open(results_filename, 'a')
        else:
            outputfile = open(results_filename, 'w')
#        outputfile.write(time.ctime())
        outputfile.write(time.strftime("%Y-%m-%d %X", time.localtime()))
        outputfile.write('\n')
        global dimension
        global myLockIdleness
        global lastVisitList
        global instantIdlenessList
        global visitCountList
        global maxIdlenessList
        global meanIdlenessList
        global M2IdlenessList
        global stdDevIdlenessList
        myLockIdleness.acquire()
        outputfile.write('VertexID ')
        for i in range(0, dimension):
            outputfile.write('%d ' % (i))
        outputfile.write('\n')
        outputfile.write('VisitCount ')
        for i in range(0, dimension):
            outputfile.write('%d ' % (visitCountList[i]))
        outputfile.write('\n')
        outputfile.write('MaxIdleness ')
        for i in range(0, dimension):
            outputfile.write('%.3f ' % (maxIdlenessList[i]))
        outputfile.write('\n')
        outputfile.write('MeanIdleness ')
        for i in range(0, dimension):
            outputfile.write('%.3f ' % (meanIdlenessList[i]))
        outputfile.write('\n')
        for i in range(0, dimension):
            if visitCountList[i] > 1:
                stdDevIdlenessList[i] = math.sqrt( M2IdlenessList[i] / visitCountList[i] )
        outputfile.write('StdDevIdleness ')
        for i in range(0, dimension):
            outputfile.write('%.3f ' % (stdDevIdlenessList[i]))
        outputfile.write('\n')
        outputfile.write('IdlenessRecord\n')
        for i in range(0, dimension):
            outputfile.write('(VertexID%d) ' % (i))
            for listelement in idlenessRecordDic[i]:
                outputfile.write('%.3f ' % (listelement))
            outputfile.write('\n')
        myLockIdleness.release()
        outputfile.write('\n')
        outputfile.close()
        
        # 关闭记录图中各节点的瞬时空闲率随时间变化的文件 
        self.timeidleness_file.close()
        
        tkinter.messagebox.showinfo('Message','Results File Saved')
    
    
    # 更新机器人的位置信息，并在界面上显示 
    def update_image(self):
    
        global myLockStatus
        global statusDic
        mapImage = self.mapImage.copy()  # 复制图像，类的成员变量self.mapImage存储着地图，将机器人位置用圆形表示画在地图上 
        draw = ImageDraw.Draw(mapImage)  # 定义画图对象 
        robot_radius = 3  # 画图时的机器人半径，单位像素pixel 
        
        myLockStatus.acquire()
        # 遍历存储机器人位置信息的字典 
        for robot_id, robot_status in statusDic.items():
            # 设置机器人占据的区域，以供画图 
            box = ( robot_status[0]-robot_radius, robot_status[1]-robot_radius, robot_status[0]+robot_radius, robot_status[1]+robot_radius )
            draw.ellipse( box, outline=(0,255,0), fill=(0,255,0) )  # 将机器人位置用圆形表示画在地图上 
        myLockStatus.release()
        
        del draw  # 释放画图对象所占内存 
        mapPhoto = ImageTk.PhotoImage(mapImage)
        self.imageLabel.configure(image=mapPhoto)  # 更新Label显示的图像 
        self.imageLabel.image = mapPhoto  # keep a reference! 
        
        global patrol_is_start
        if patrol_is_start == True:
            global dimension
            global myLockIdleness
            global lastVisitList
            global instantIdlenessList
            seconds = rospy.get_time()  # Get the current time in float seconds 
            myLockIdleness.acquire()
            idlenessSum = 0
            idlenessMax = 0
            for i in range(0, dimension):
                instantIdlenessList[i] = seconds - lastVisitList[i]
                idlenessSum += instantIdlenessList[i]
                if instantIdlenessList[i] > idlenessMax :
                    idlenessMax = instantIdlenessList[i]
            self.timeidleness_file.write('%f ' % (seconds))
            self.timeidleness_file.write('%f ' % (idlenessSum/dimension))
            self.timeidleness_file.write('%f\n' % (idlenessMax))
            myLockIdleness.release()
#            print ("timeidleness")
        
        self.root.after(1000, self.update_image)  # 1000ms(1s)后，再次调用self.update_image函数 
    

# 主函数 
def main():
    if len(sys.argv) < 3:
        print ("Use: python ", sys.argv[0], " <map_name> <robot_num>")
        sys.exit(0)
    
    mapName = sys.argv[1]
    robotNum = sys.argv[2]
    
    global mapImage_filename
    global mapGraph_filename
    global results_filename
    global robotpath_filename
    global timeidleness_filename
    mapImage_filename = "maps/" + mapName + "/" + mapName + "_info.ppm"
    mapGraph_filename = "maps/" + mapName + "/" + mapName + ".graph"
    results_filename = "results/" + mapName + "_" + robotNum + "_results.txt"
    robotpath_filename = "results/" + mapName + "_" + robotNum + "_robotpath.txt"
    timeidleness_filename = "results/" + mapName + "_" + robotNum + "_timeidleness.txt"

    thread_listener = myThread()
    thread_listener.start()
    
    root = tkinter.Tk()
    root.title('Patrol Control')
    myapp = MyApp(root)
    root.mainloop()
    
    thread_listener.stop()
    

if __name__ == '__main__':
    main()
