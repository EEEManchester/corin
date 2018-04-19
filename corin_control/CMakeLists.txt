cmake_minimum_required(VERSION 2.8.3)
project(corin_control)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  gazebo_msgs
  geometry_msgs
  message_filters
  message_generation
  robotis_controller_msgs
  roscpp
  rospy
  std_msgs
  tf
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   JointState.srv
   Imu.srv
   UiState.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES       # Or other packages containing msgs
   actionlib_msgs
   control_msgs
   geometry_msgs
   robotis_controller_msgs
   sensor_msgs
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories( ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(jointStateServer cpp_script/service/jointStateServer.cpp)
add_dependencies(jointStateServer corin_control_gencpp)
target_link_libraries(jointStateServer ${catkin_LIBRARIES})