#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time, os
import math
import ConfigParser


colors = ['blue', 'red', 'cyan', 'magenta', 'blue', 'red', 'cyan', 'magenta', 'blue', 'red', 'cyan', 'magenta']

initPoses = {}


# load initial poses from configuration file 
def loadInitPoses():
    global initPoses
    try:
        ConfigIP = ConfigParser.ConfigParser()
        ConfigIP.read("../maps/initial_poses.txt")
        for option in ConfigIP.options("InitialPoses"):
            # print option 
            initPoses[option] = ConfigIP.get("InitialPoses", option)
    except:
        print "Could not load initial poses file"


# write initial poses to the "robots.inc" file 
def setInitPoses(mapName, ipStr):
    ipList = eval(ipStr)
    n = len(ipList) / 2
    
    robotsFileName = '../maps/' + mapName + '/robots.inc'
    robotsFile = open(robotsFileName, 'w')
    global colors
    
    for i in range(0,n):
        x = ipList[i*2]
        y = ipList[i*2+1]
        th = 90
        robotsFile.write('turtlebot( pose [ '+str(x)+'  '+str(y)+'  0  '+str(th)+' ]   name "robot'+str(i)+'"  color "'+colors[i]+'")\n')
    
    if n==1:
        # inactive robot for having correct namespaces 
        robotsFile.write('turtlebot( pose [ -2.0   -2.0    0   0.0 ]   name "robot1"  color "red")\n')
    
    robotsFile.close()


# write launch files 
def setLaunchFile(mapName, robotNum):
    # in this function, the type of robotNum is int 
    navFileName = '../launch/navigation_multi_robot.launch'
    navFile = open(navFileName, 'w')
    navFile.write('<launch>\n')
    navFile.write('  <master auto="start"/>\n')
    navFile.write('  <param name="/use_sim_time" value="true"/>\n')
    navFile.write('\n')
    navFile.write('  <include file="$(find my_patrol_sim)/launch/includes/stage_and_map.launch.xml">\n')
    navFile.write('    <arg name="map_name" value="' + mapName + '" />\n')
    navFile.write('  </include>\n')
    navFile.write('\n')
    
    for i in range(0,robotNum):
        navFile.write('  <include file="$(find my_patrol_sim)/launch/includes/one_robot.launch.xml">\n')
        navFile.write('    <arg name="robot_name" value="robot_' + str(i) + '" />\n')
        navFile.write('  </include>\n')
    
    navFile.write('\n')
    navFile.write('</launch>')
    navFile.close()
    
    patFileName = '../launch/patrol_multi_robot.launch'
    patFile = open(patFileName, 'w')
    patFile.write('<launch>\n')
    patFile.write('\n')
    
    for i in range(0,robotNum):
        patFile.write('  <node name="robot_'+str(i)+'" pkg="my_patrol_sim" type="my_patrol_sim" args="'+str(i)+' $(find my_patrol_sim)/maps/'+mapName+'/'+mapName+'.graph" output="screen" >\n')
        patFile.write('    <param name="tf_prefix" value="robot_' + str(i) + '" />\n')
        patFile.write('  </node>\n')
    
    patFile.write('\n')
    patFile.write('</launch>')
    patFile.close()


def main():
    if len(sys.argv) < 3:
        print "Use: python ", sys.argv[0], " <map_name> <robot_num>"
        sys.exit(0)
    
    mapName = sys.argv[1]
    robotNum = sys.argv[2]
    
    loadInitPoses()
    scenario = mapName + "_" + robotNum
    global initPoses
    ipStr = initPoses[scenario.lower()]
    
    setInitPoses(mapName, ipStr)
    
    setLaunchFile(mapName, int(robotNum))
    
    print "Initialize complete.   map_name:", mapName, " robot_num:", robotNum


if __name__ == '__main__':
    main()
