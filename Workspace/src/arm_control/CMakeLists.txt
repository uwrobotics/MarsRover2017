cmake_minimum_required(VERSION 2.8.3)
project(arm_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs can_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(arm_control src/arm_control.cpp)
target_link_libraries(arm_control ${catkin_LIBRARIES})
