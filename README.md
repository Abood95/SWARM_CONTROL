# SwarmRoboticsResearch

#Dependencies:

Navigation kit: sudo apt-get ros-indigo-navigation


- To start simulation platform:

`roslaunch multi_navigation multi_navigation_establish.launch`

- To get ready for desired trajectory:

'roslaunch multi_navigation multi_navigation_send_goal.launch'

-To start controller:

`rosrun swarm_control_algorithm swarm_control_algorithm_main`