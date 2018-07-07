cmake_minimum_required(VERSION 2.8.3)
project(corin_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

################################################################################
# Packages
################################################################################
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  message_generation
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
  visualization_msgs
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
 add_message_files(
   FILES
   MotionPlan.msg
   GridMap.msg
   GridMapInfo.msg
  )

 add_service_files(
   FILES
   PlanPath.srv   # Grid path planning
   GetGridMap.srv    # Grid map
   GenericString.srv # Generic string service, SISO
   JointState.srv
   Imu.srv
   UiState.srv    # user interface
   RigidBody.srv  # CRBI and CoM
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES       # Or other packages containing msgs
   geometry_msgs
   sensor_msgs
   std_msgs
   nav_msgs
   trajectory_msgs
   visualization_msgs
 )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  sensor_msgs
  std_msgs
  message_runtime
  DEPENDS Boost Eigen
)

###########
## Build ##
###########

