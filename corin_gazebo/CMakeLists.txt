cmake_minimum_required(VERSION 2.8.3)
project(corin_gazebo)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  corin_description
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES corin_gazebo
#  CATKIN_DEPENDS gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control mcorin_description
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)