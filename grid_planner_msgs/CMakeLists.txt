cmake_minimum_required(VERSION 2.8.3)
project(grid_planner_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

################################################################################
# Packages
################################################################################
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  nav_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
  visualization_msgs
)

# find_package(Eigen REQUIRED)

# catkin_python_setup()

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
    GenericString.srv
 )

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
  message_runtime
  DEPENDS Boost Eigen
)

###########
## Build ##
###########

include_directories( ${catkin_INCLUDE_DIRS} 
                      ${Boost_INCLUDE_DIRS} 
                      ${Eigen_INCLUDE_DIRS})
