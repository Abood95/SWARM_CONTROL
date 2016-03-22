cmake_minimum_required(VERSION 2.8.3)
project(swarm_control)

find_package(catkin_simple REQUIRED)

catkin_simple()


cs_add_executable(swarm_control_main src/swarm_control_main.cpp)
cs_add_executable(swarm_control_main2 src/swarm_control_main2.cpp)

cs_install()
cs_export()
    