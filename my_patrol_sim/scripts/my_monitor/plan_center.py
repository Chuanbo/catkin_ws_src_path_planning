#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import time

import lcm
from exlcm import plan_t


# 主函数 
def main():
    if len(sys.argv) < 3:
        print "Use: python ", sys.argv[0], " <map_name> <robot_num>"
        sys.exit(0)
    
    mapName = sys.argv[1]
    robotNum = int(sys.argv[2])  # int 
    
    plan_filename = "plan_files/" + mapName + "/plan_" + mapName + "_" + str(robotNum) + ".txt"
    print plan_filename
    plan_file = open( plan_filename, 'r' )
    
    lc = lcm.LCM()
    
    for robot_i in range(0, robotNum):
        path_list = eval( plan_file.readline() )
        print path_list
        
        msg = plan_t()
        
        msg.robot_id = robot_i
        msg.robot_num = robotNum
        msg.num_step = len(path_list)
        msg.steps = path_list
        
        lc.publish("PLANVERTEXLIST", msg.encode())
        
        time.sleep(0.5)
        
    
    plan_file.close()
    

if __name__ == '__main__':
    main()
