# catkin_ws_src_path_planning

### 1. run stage simulator with 4 robots, control robot_1

`roslaunch my_patrol_sim stage_multi_robot.launch`

### 2. run python script, Astar + DWA navigation for robot_1

`/bin/python3 /home/ychb/catkin_ws/src/my_patrol_sim/scripts/robot_1_Astar_DWA.py`

/my_patrol_sim/scripts/ A_star_path_finding_improved.py Dynamic_Window_Approaches.py robot_1_Astar_DWA.py

### 3. run rviz with robot_1.rviz and send simple goal

`rosrun rviz rviz`

