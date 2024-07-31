#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
import tf
import time
import copy
import math
import sys
import numpy as np
import A_star_path_finding_improved as A_star
import new_Dynamic_Window_Approaches as DWA
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from dynamic_obstacle_detector.msg import DynamicObstacles

"""代码参考
        https://blog.csdn.net/a819096127/article/details/89552223
        https://github.com/FansaOrz/intelligence_engineering
        https://blog.csdn.net/weixin_42301220/article/details/127769819
        https://github.com/CHH3213/chhRobotics
"""

class robot_Astar_DWA():
    def __init__(self, argv):
        if len(argv) == 1:
            print("No parameter")
            self.robot_id = ""
            self.robot_prefix = ""
        else:
            print("robot_id = ", argv[1])
            self.robot_id = argv[1]
            self.robot_prefix = "/robot_" + argv[1]
        print(self.robot_prefix)
        rospy.init_node("robot_" + self.robot_id + "_Astar_DWA")
        self.start_time = 0
        self.end_time = 0
        # self.path_pub = rospy.Publisher("/path_my_A_star", Path, queue_size=1)
        # self.path_pub_changed = rospy.Publisher("/path_my_A_star_changed", Path, queue_size=1)
        self.line_database = []  # 用于路径规划的分割合并
        # 关于地图的一些变量
        self.origin_x = 0
        self.origin_y = 0
        self.resolution = 0
        self.width = 0
        self.height = 0
        # self.map_test_pub = rospy.Publisher("/map_test", OccupancyGrid, queue_size=1)
        self.map_sub_once = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        # new add
        self.reach_goal_pub = rospy.Publisher("/robot_reach_goal_str", String, queue_size=10)
        self.laser_sub = rospy.Subscriber(self.robot_prefix + "/base_scan", LaserScan, self.laserscan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.robot_prefix + "/odom", Odometry, self.odom_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.robot_prefix + "/cmd_vel", Twist, queue_size=1)
        self.scan_msg = None
        self.rp = np.zeros(3)
        self.crv = 0.0
        self.crw = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_x_delta = 0.0
        self.odom_y_delta = 0.0
        self.odom_yaw_delta = 0.0
        self.listener = tf.TransformListener()
        rospy.sleep(1)
        # 起始点和目标点
        self.start_map_point = []
        self.goal_map_point = []
        # 地图上的路径
        self.path_map = []
        self.path_world = []
        # 是否要寻找路径的开关
        self.if_start_find_path = False
        self.goal_pose = PoseStamped()
        self.init_pose = PoseWithCovarianceStamped()
        self.goal_pose_sub = rospy.Subscriber(self.robot_prefix + "/move_base_simple/goal", PoseStamped, self.goal_pose_callback, queue_size=1)
        self.last_time = rospy.get_rostime()
        self.start_find_path()
        # new add local map
        self.local_width = 0
        self.local_height = 0
        self.occupancy_array = None
        # new add dynamic obstacle or velocity obstacle
        self.dynamicobstacles_msg = None
        self.dynobst_sub = rospy.Subscriber("/dynamic_obstacles", DynamicObstacles, self.dynamic_obstacles_callback)
        # new add dwa
        self.ob = np.array([[-100,-100],[-200,-200]])  # 先固定好格式，后续会继续具体赋值
        self.if_start_dwa_navigation = False
        dwa_config = DWA.Config()
        self.robot_radius = dwa_config.robot_radius  # Astar与DWA中的机器人半径robot_radius保持一致
        self.dwa = DWA.DWA(dwa_config)
        self.sub_goal = None
        rate = rospy.Rate(5)  # change from 10Hz to 5Hz
        while not rospy.is_shutdown():
            if not self.if_start_dwa_navigation:
                pass
            else:
                # 否则，进行dwa导航
                if self.sub_goal is None:
                    if len(self.path_map_be) == 0:
                        self.if_start_dwa_navigation = False
                    else:
                        temp_point = self.path_map_be.pop()
                        print(temp_point)
                        # path from Astar is [y,x] not [x,y]
                        goal_x, goal_y = self.mapToWorld(temp_point[1], temp_point[0])
                        self.sub_goal = np.array([goal_x, goal_y])
                        self.getrobotpose()  # update robot position
                if self.sub_goal is not None:
                    # check reaching goal
                    dist_to_sub_goal = math.hypot(self.rp[0] - self.sub_goal[0], self.rp[1] - self.sub_goal[1])
                    if (dist_to_sub_goal < 10 * dwa_config.xy_goal_tolerance) and (len(self.path_map_be) > 0):
                        # 提前抛弃当前子目标点，切换到下一个子目标点，以避免接近子目标点时机器人的减速
                        self.sub_goal = None
                        print("approach sub goal !")
                    elif (dist_to_sub_goal < dwa_config.xy_goal_tolerance) and (len(self.path_map_be) == 0):
                        self.sub_goal = None
                        # 机器人已到达目标点，发布消息告知上层控制端，以获取新的导航目标点
                        reach_goal_msg = String()
                        reach_goal_msg.data = self.robot_id
                        self.reach_goal_pub.publish(reach_goal_msg)
                        self.publish_cmd_twist(0.0, 0.0)
                        print("reach final goal !")
                    else:
                        state = np.array([self.rp[0], self.rp[1], self.rp[2], self.crv, self.crw])
                        self.update_ob_from_scan()  # update self.ob from self.scan_msg
                        dynobst_msg = copy.deepcopy(self.dynamicobstacles_msg)
                        # 与静态地图 map 对比，过滤掉 dynobst_msg 中误判的动态障碍物
                        self.update_local_map()
                        dynobst_msg_new = self.update_dynobst(dynobst_msg)
                        # print(len(dynobst_msg.obstacles))
                        # print("ss")
                        # print(len(dynobst_msg_new.obstacles))
                        u, predicted_trajectory = self.dwa.dwa_control(state, self.sub_goal, self.ob, dynobst_msg_new)
                        # 如果只有一行，则程序未能正确求解
                        # print(np.size(predicted_trajectory, 0))
                        self.publish_cmd_twist(u[0], u[1])
                        # print("v = ", u[0], "    w = ", u[1])
            rate.sleep()
        rospy.spin()


    def update_dynobst(self, dynobst_msg):
        dynobst_num = len(dynobst_msg.obstacles)
        for index in reversed(range(dynobst_num)):  # 逆序遍历
            ob_x = dynobst_msg.obstacles[index].position.x
            ob_y = dynobst_msg.obstacles[index].position.y
            ob_vx = dynobst_msg.obstacles[index].velocity.x
            ob_vy = dynobst_msg.obstacles[index].velocity.y
            dynobst_point = self.WorldTomap(ob_x, ob_y)
            # print(dynobst_point)
            dx = self.occupancy_array[:, 0] - dynobst_point[0]
            dy = self.occupancy_array[:, 1] - dynobst_point[1]
            r = np.hypot(dx, dy)
            if np.min(r) < 4:  # 如果识别出的动态障碍物距离静态障碍物过于接近，则舍弃之
                del dynobst_msg.obstacles[index]
        return dynobst_msg


    def update_local_map(self):
        self.local_width = int(4.0 / self.resolution)  # pixel
        self.local_height = int(4.0 / self.resolution)
        robot_point = self.WorldTomap(self.rp[0], self.rp[1])
        i_min = int(max(0, robot_point[1]-self.local_height))
        i_max = int(min(self.height, robot_point[1]+self.local_height))
        j_min = int(max(0, robot_point[0]-self.local_width))
        j_max = int(min(self.width, robot_point[0]+self.local_width))
        occupancy_x_list = []  # 静态障碍物占据的栅格点坐标构成的列表
        occupancy_y_list = []
        for i in range(i_min, i_max):
            for j in range(j_min, j_max):
                if self.map[i][j] > 0:
                    occupancy_x_list.append(j)
                    occupancy_y_list.append(i)
        self.occupancy_array = np.vstack((occupancy_x_list, occupancy_y_list)).T


    def update_ob_from_scan(self):
        if self.scan_msg is None:
            print("No scan from laser!")
        else:
            scan = copy.deepcopy(self.scan_msg)
            cached_cos = np.cos( np.arange(scan.angle_min, scan.angle_max, scan.angle_increment) + self.rp[2] )
            cached_sin = np.sin( np.arange(scan.angle_min, scan.angle_max, scan.angle_increment) + self.rp[2] )
            # Convert scan from polar to cartesian coordinate system
            pc_x = scan.ranges * cached_cos + self.rp[0]
            pc_y = scan.ranges * cached_sin + self.rp[1]
            # 如果激光测距点的间距/地图分辨率与机器人半径相比要小很多，则可以适当地降采样以降低计算复杂度
            self.ob = np.stack([pc_x, pc_y]).T
            # print("self.ob.shape = ", self.ob.shape)
            # print(self.rp)
            # print(self.ob)


    # 贝塞尔插值 not used
    def smooth(self, pts):
        new_pts = []
        n_spline = 15

        idx = 1
        while idx + 1 < len(pts):
            m1 = (pts[idx - 1] + pts[idx]) / 2.
            m2 = (pts[idx] + pts[idx + 1]) / 2.
            k = float(np.sum(np.square(pts[idx - 1] - pts[idx]))) / np.sum(np.square(pts[idx] - pts[idx + 1]))
            m = (m1 + k * m2) / (1 + k)
            m1 += pts[idx] - m
            m2 += pts[idx] - m

            if idx == 1:
                for i in range(n_spline):
                    t = 1. * i / n_spline
                    p = ((1 - t) ** 2) * pts[0] + 2 * t * (1 - t) * m1 + (t ** 2) * pts[1]
                    new_pts.append(p)
            else:
                for i in range(n_spline):
                    t = 1. * i / n_spline
                    p = ((1 - t) ** 3) * pts[idx - 1] + 3 * t * ((1 - t) ** 2) * last_m2 + 3 * (t ** 2) * (
                                1 - t) * m1 + (t ** 3) * pts[idx]
                    new_pts.append(p)

            if idx == len(pts) - 2:
                for i in range(n_spline + 1):
                    t = 1. * i / n_spline
                    p = ((1 - t) ** 2) * pts[idx] + 2 * t * (1 - t) * m2 + (t ** 2) * pts[idx + 1]
                    new_pts.append(p)

            last_m2 = m2
            idx += 1

        return np.array(new_pts)


    def start_find_path(self):
        if self.if_start_find_path:

            print('\033[32m[I] : Start find path with A* \033[0m')
            # 初始化路径规划类的对象（局部变量，用后即删）
            Dij_find_path = A_star.find_path(self.map, robot_size=8, inflation_size=2)
            # 获取规划的路径
            # Astar use [y,x] not [x,y]
            self.path_map, open_list_index = Dij_find_path.start_find(np.flip(self.start_map_point), np.flip(self.goal_map_point))
            # 把遍历的open表节点更新到地图中
            self.path_map_be = self.path_map
            # self.update_map(open_list_index)
            # 分割合并算法，提取关键拐点
            self.split(self.path_map_be, 0.3)
            self.line_database = self.merge(self.line_database, 2.4)
            # 把关键拐点添加到新的path中
            self.path_map_be = []
            # 提取关键拐点
            for i in range(len(self.line_database)):
                self.path_map_be.append(np.array(self.line_database[i][0]))
            end_length = len(self.line_database[len(self.line_database) - 1]) - 1
            self.path_map_be.append(np.array(self.line_database[len(self.line_database) - 1][end_length]))
            self.line_database = []  # 用于路径规划的分割合并  本次Astar路径规划已完成，清空line_database，以备下次Astar路径规划
            # 下面这个while循环如果没有被注释，则抛弃所有中间点，只保留最终目标点，即不使用Astar算法，仅使用DWA算法进行导航
            while (len(self.path_map_be) > 1):
                self.path_map_be.pop()

            print("-----------------------------------------")
            print(len(self.path_map_be))
            print(len(self.path_map_be[0]))
            print(self.path_map_be)    # self.path_map_be未经过贝塞尔插值，不是连续且平滑的路径，仅是提取的Astar规划路径的关键拐点，等待后续的轨迹连接+平滑+避障操作
            # print(self.path_map_be)
            # print(self.path_map)
            # 发布规划的路径
            # self.publish_astar_path()
        else:
            rospy.sleep(1)
            print('\033[33m[W] : Please set goal pose\033[0m')
            return


    # 将轨迹信息写到地图数据上 not used (在rviz上直观显示Astar规划路径，后续计算中并未使用)
    def update_map(self, index):
        temp = list(self.map_msg.data)
        for i in range(len(index)):
            temp_x = index[i][0][0] - 1
            temp_y = index[i][1][0] - 1
            temp[temp_x * self.width + temp_y] = 50
        self.map_msg.data = tuple(temp)
        time.sleep(3)
        self.map_test_pub.publish(self.map_msg)
        print("-----show_changed_map-----")

    def laserscan_callback(self, scan):
        self.scan_msg = scan

    def dynamic_obstacles_callback(self, msg):
        self.dynamicobstacles_msg = msg
        # for print
        # for index in range(len(msg.obstacles)):
        #     ob_x = msg.obstacles[index].position.x
        #     ob_y = msg.obstacles[index].position.y
        #     ob_vx = msg.obstacles[index].velocity.x
        #     ob_vy = msg.obstacles[index].velocity.y
        #     print(ob_x, ob_y, ob_vx, ob_vy)
        # print("dynamic obstacles Number =  ", len(msg.obstacles))


    # 回调函数系列
    def goal_pose_callback(self, msg):
        # 获取机器人的当前位置，作为路径规划的起点
        while not self.getrobotpose():
            rospy.sleep(1)
        # 获取目标点位置，作为路径规划的终点
        self.path_map = []
        self.goal_pose = msg
        self.if_start_find_path = True
        # print(msg)
        self.goal_map_point =  self.WorldTomap(msg.pose.position.x, msg.pose.position.y)
        print("-----------------goal point---------------",self.goal_map_point)
        if self.goal_map_point == [-1, -1]:
            print("\033[30m[E] : Please set the valid goal point\033[0m")
            return
        else:
            self.start_find_path()
            self.if_start_dwa_navigation = True
            self.if_start_find_path = False


    def mapToWorld(self, mx, my):
        # print("mapToWorld")
        wx = self.origin_x + mx * self.resolution
        wy = self.origin_y + my * self.resolution
        return [wx, wy]

    def WorldTomap(self, wx, wy):
        # 返回-1，-1就是有问题
        # print(wx, wy)
        # print(self.origin_x, self.origin_y)
        if wx < self.origin_x or wy < self.origin_y:
            # print("<<<<<<<")
            return [-1, -1]
        mx = (int)((wx - self.origin_x) / self.resolution)
        my = (int)((wy - self.origin_y) / self.resolution)
        if mx < self.width and my < self.height:
            # return [my, mx]
            return [mx, my]
        return [-1, -1]


    def map_callback(self, msg):
        # map的回调函数，调用一次更新地图后，即取消回调函数
        print(msg.header)
        print("------")
        print(msg.info)
        print("------")

        # 初始化map里的参数值
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        print("-----width------",self.width)
        # 把map里的消息存下来
        self.map_msg = msg
        raw = np.array(msg.data, dtype=np.int8)
        self.map = raw.reshape((self.height, self.width))
        self.map_sub_once.unregister()  # unsubscribe to a topic
        # test local map
        # self.update_local_map()
        # print(self.occupancy_array)
        # print(self.occupancy_array.shape)


    def getrobotpose(self):
        try:
            (trans,rot) = self.listener.lookupTransform("/map", self.robot_prefix + "/base_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False
        self.rp[0] = trans[0]
        self.rp[1] = trans[1]
        r,p,y = tf.transformations.euler_from_quaternion(rot)
        self.rp[2] = y
        # 更新tf变化得到的机器人位置与odom位置之间的差值/修正值
        self.odom_x_delta = trans[0] - self.odom_x
        self.odom_y_delta = trans[1] - self.odom_y
        self.odom_yaw_delta = y - self.odom_yaw

        # 将机器人当前位置保存到self.start_map_point，作为路径规划的起点
        self.start_map_point =  self.WorldTomap(trans[0], trans[1])
        # print("----------------start point----------------",self.start_map_point)
        # print("value = ", self.map[self.start_map_point[0]][self.start_map_point[1]])
        if self.start_map_point == [-1, -1]:
            print("\033[0;31m[E] : Please set the valid goal point\033[0m")
        return True

    def odom_callback(self, data):
        # 通过odom更新机器人的速度crv和角速度crw
        self.crv = data.twist.twist.linear.x
        self.crw = data.twist.twist.angular.z
        # 记录odom消息中的机器人位置信息，用于快速更新机器人位置 x,y,yaw
        self.odom_x = data.pose.pose.position.x
        self.odom_y = data.pose.pose.position.y
        rot = data.pose.pose.orientation
        r,p,y = tf.transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        self.odom_yaw = y
        # ww
        self.rp[0] = self.odom_x + self.odom_x_delta
        self.rp[1] = self.odom_y + self.odom_y_delta
        self.rp[2] = self.odom_yaw + self.odom_yaw_delta


    def publish_cmd_twist(self, v, w):
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)


    def merge(self, line_dataset, threshold):
        new_line_dataset = [line_dataset[0]]
        # 遍历第二条直线到最后一条直线
        for i in range(1, len(line_dataset)):
            # 将相邻的两条直线拟合成一条,前一条直线的第一个点，后一条直线的最后一个点
            first_point = new_line_dataset[len(new_line_dataset) - 1][0]
            end_point = line_dataset[i][len(line_dataset[i]) - 1]
            # 计算直线方程
            x1 = first_point[0]
            y1 = first_point[1]
            x2 = end_point[0]
            y2 = end_point[1]
            a = y2 - y1
            b = x1 - x2
            c = y1 * x2 - y2 * x1
            # 声明最大距离变量
            max_dis = 0
            # 遍历第一条直线
            # print("22222")
            for j1 in range(len(new_line_dataset[len(new_line_dataset) - 1]) - 1):
                # print("111111")
                temp_point = new_line_dataset[len(new_line_dataset) - 1][j1]
                x = temp_point[0]
                y = temp_point[1]
                # 计算距离直线的距离
                current_dis = abs(a * x + b * y + c) / math.sqrt(math.pow(a, 2) + math.pow(b, 2))
                if current_dis > max_dis:
                    max_dis = current_dis
            # 遍历第二条直线
            for j2 in range(len(line_dataset[i]) - 1):
                temp_point = line_dataset[i][j2]
                x = temp_point[0]
                y = temp_point[1]
                # 计算距离直线的距离
                current_dis = abs(a * x + b * y + c) / math.sqrt(math.pow(a, 2) + math.pow(b, 2))
                if current_dis > max_dis:
                    max_dis = current_dis
            # 小于阈值就可以合并
            if max_dis < threshold:
                # print("1111111111", len(new_line_dataset))
                # 只需记录起点和终点
                temp_line = [first_point, end_point]
                # 如果合并了，就把最后一条直线删掉
                new_line_dataset[len(new_line_dataset) - 1] = temp_line
                # 把新的合并完的直线加到new里面
                # new_line_dataset.append(temp_line)
            else:
                new_line_dataset.append(line_dataset[i])
        return new_line_dataset

    def split(self, points, threshold):
        # print("split")
        points_length = len(points)
        # 假设直线方程为a*x+b*y+c=0,计算直线方程
        x1 = points[0][0]
        y1 = points[0][1]
        x2 = points[points_length - 1][0]
        y2 = points[points_length - 1][1]
        a = y2 - y1
        b = x1 - x2
        c = y1 * x2 - y2 * x1
        # 声明最大距离变量
        max_dis = 0
        max_dis_index = 0
        # 遍历当前路径中的所有节点，寻找最远距离的点
        for temp_point_index in range(points_length):
            x = points[temp_point_index][0]
            y = points[temp_point_index][1]
            current_dis = abs(a * x + b * y + c) / math.sqrt(math.pow(a, 2) + math.pow(b, 2))
            # 找到一个更大的，就更新最大值和下标
            if current_dis > max_dis:
                max_dis = current_dis
                max_dis_index = temp_point_index
        if max_dis > threshold:
            # print("more split")
            if max_dis_index - 0 == 1:
                self.line_database.append(points)
            else:
                self.split(points[0:max_dis_index + 1], threshold)
            if points_length - 1 - max_dis_index == 1:
                self.line_database.append(points)
            else:
                self.split(points[max_dis_index:points_length], threshold)
        else:
            # 如果不用分割了，就把当前这一条直线存到直线库中
            self.line_database.append(points)
            return


    def publish_astar_path(self):
        # print("total path_length===============", len(self.path_map))
        time = 1
        self.current_path = Path()
        for i in range(len(self.path_map)):
            current_pose = PoseStamped()
            current_pose.pose.position.x, current_pose.pose.position.y= self.mapToWorld(self.path_map[i][1], self.path_map[i][0])
            # self.path_world.append(self.mapToWorld(self.path_map[i][1], self.path_map[i][0]))
            current_pose.pose.position.z = 0.0
            current_pose.pose.orientation.x = 0.0
            current_pose.pose.orientation.y = 0.0
            current_pose.pose.orientation.z = 0.0
            current_pose.pose.orientation.w = 1.0
            self.current_path.poses.append(current_pose)
            time += 1
        current_time = rospy.get_rostime()
        self.current_path.header.stamp = current_time
        self.current_path.header.frame_id = "map"
        self.path_pub.publish(self.current_path)
        self.last_time = current_time
        # rospy.sleep(1)

        # print("total path_length_be===============", len(self.path_map_be))
        time = 1
        self.current_path_changed = Path()
        for i in range(len(self.path_map_be) - 1, -1, -1):
            current_pose = PoseStamped()
            current_pose.pose.position.x, current_pose.pose.position.y= self.mapToWorld(self.path_map_be[i][1], self.path_map_be[i][0])
            # self.path_world.append(self.mapToWorld(self.path_map[i][1], self.path_map[i][0]))
            current_pose.pose.position.z = 0.0
            current_pose.pose.orientation.x = 0.0
            current_pose.pose.orientation.y = 0.0
            current_pose.pose.orientation.z = 0.0
            current_pose.pose.orientation.w = 1.0
            self.current_path_changed.poses.append(current_pose)
            time += 1
        current_time = rospy.get_rostime()
        self.current_path_changed.header.stamp = current_time
        self.current_path_changed.header.frame_id = "map"
        self.path_pub_changed.publish(self.current_path_changed)
        self.last_time = current_time
        # rospy.sleep(1)


if __name__ == '__main__':
    robot_Astar_DWA(sys.argv)
