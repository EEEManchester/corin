cmake_minimum_required(VERSION 2.8.3)
project(corin_joint_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamixel_sdk
  robotis_controller_msgs
  robotis_device
  robotis_controller
  robotis_framework_common
  cmake_modules
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(corin_manager src/corin_manager.cpp)
target_link_libraries(corin_manager
  ${catkin_LIBRARIES}
)