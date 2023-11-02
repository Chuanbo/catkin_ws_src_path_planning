# my_patrol_sim
Multi-robot patrol simulation

This package is modified from [patrolling_sim](https://github.com/davidbsp/patrolling_sim) <br />
The main differences are as follows: <br />
  [LCM](https://github.com/lcm-proj/lcm), a library independent of ROS, is used to exchange messages between robots in both simulations and real robot experiments <br />
  A GUI monitoring program is implemented to visualize the position of robots and control the start as well as stop of the simulation <br />
  The expected reactive (ER) algorithm for multi-robot patrol is added in this package

Required <br />
  Ubuntu 12.04 <br />
  Boost 1.46 (Debian’s default Boost version, installed with Ubuntu 12.04 already) <br />
  ROS Hydro and package: [stage_ros](http://wiki.ros.org/stage_ros) <br />
  [LCM v1.0.0](https://github.com/lcm-proj/lcm) (Lightweight Communications and Marshalling) <br />
  Python 2.7 and package: [PIL(Python Imaging Library)](http://pythonware.com/products/pil/) or [pillow(The friendly PIL fork)](https://python-pillow.org/)

To run the simulation <br />
  1 use 'catkin_make' to build the package; <br />
  2 run 'roslaunch my_patrol_sim navigation_multi_robot.launch' in a terminal; <br />
  3 run 'roslaunch my_patrol_sim patrol_multi_robot.launch' in a new terminal; <br />
  4 start the Python script 'my_monitor.py' (in 'scripts/my_monitor' folder) to control the simulation.

To configure parameters, use the script 'initialize_sim.py'.

The results are stored in 'scripts/my_monitor/results' folder.

Detailed information can be found in the file "documentation.docx".
