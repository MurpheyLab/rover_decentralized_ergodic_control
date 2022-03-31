cmake_minimum_required(VERSION 2.8.3)
project(rover_decentralized_ergodic_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
  move_base_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  tablet.msg
)

## Generate services in the 'srv' folder
add_service_files(
 FILES RosErgodicControl.srv ClearMapTablet.srv ClearMapStomp.srv
)

## Generate added messages and services with any dependencies
generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs move_base_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs actionlib_msgs move_base_msgs
)
