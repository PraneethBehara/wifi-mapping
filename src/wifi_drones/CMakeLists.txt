cmake_minimum_required(VERSION 2.8.3)
project(drones_move)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  message_generation
  manual_measurements
)

## Generate messages in the 'msg' folder
 add_message_files(
)

## Generate services in the 'srv' folder
 add_service_files(
  FILES
  MoveTurtlebot.srv
)

## Generate actions in the 'action' folder
 add_action_files(
  FILES
  trace.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  manual_measurements
  move_base_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
