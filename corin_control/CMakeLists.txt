cmake_minimum_required(VERSION 2.8.3)
project(corin_control)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## add the project source path to the CMake module path so that
## the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

################################################################################
# Packages
################################################################################
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  # dynamic_reconfigure 
  actionlib
  cmake_modules
  control_msgs
  gazebo_msgs
  geometry_msgs
  nav_msgs
  message_filters
  message_generation
  robotis_controller_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
  visualization_msgs
  #grid_planner_core
  # NOTE:   Originally grid_planner_core was not commented out. This resulted in build errors as catkin failed to locate the package. Upon commenting this out for testing, the build succeeded and strangely grid_planner_core was built also. I'm unsure why this is, but since it seemed to work I am leaving this commented out for now. -DSJ
)
#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
# generate_dynamic_reconfigure_options(
#   cfg/dynreconfig.cfg
#   #...
# )
# make sure configure headers are built before any node using them
# add_dependencies(main ${PROJECT_NAME}_gencfg)

# find_package(RBDL REQUIRED)
find_package(Eigen REQUIRED)

catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################
 # add_message_files(
 #   FILES
 #  )

 # add_service_files(
 #   FILES
 #   JointState.srv
 #   Imu.srv
 #   UiState.srv    # user interface
 #   RigidBody.srv  # CRBI and CoM
 # )

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
  actionlib
  DEPENDS Boost Eigen #rbdl
)

###########
## Build ##
###########

include_directories( ${catkin_INCLUDE_DIRS} 
                      ${Boost_INCLUDE_DIRS} 
                      ${Eigen_INCLUDE_DIRS} 
                      #${RBDL_INCLUDE_DIR}
                      )

# add_executable(jointStateServer cpp_script/service/jointStateServer.cpp)
# add_dependencies(jointStateServer corin_control_gencpp)
# target_link_libraries(jointStateServer ${catkin_LIBRARIES})

# Service Server to compute CoM and CRBI
# add_executable(rigid_body_server cpp_script/service/rigid_body_server.cpp
#                                   cpp_script/service/robots/mcorin/transforms.cpp 
#                                   cpp_script/service/robots/mcorin/inertia_properties.cpp 
#                                   cpp_script/service/robots/mcorin/inverse_dynamics.cpp
#                                   cpp_script/service/robots/mcorin/jacobians.cpp
#                                   cpp_script/service/robots/mcorin/miscellaneous.cpp)
# target_link_libraries(rigid_body_server ${catkin_LIBRARIES})
# add_dependencies(rigid_body_server corin_control_gencpp)


# add_executable(main_node cpp_script/main_node.cpp cpp_script/force_distribution.cpp
#                           cpp_script/robots/mcorin/transforms.cpp 
#                           cpp_script/robots/mcorin/inertia_properties.cpp 
#                           cpp_script/robots/mcorin/inverse_dynamics.cpp
#                           cpp_script/robots/mcorin/jacobians.cpp
#                           cpp_script/robots/mcorin/miscellaneous.cpp)
# add_dependencies(main_node corin_control_gencpp)
# target_link_libraries(main_node ${catkin_LIBRARIES} ${RBDL_LIBRARY})

# add_executable(test_spline cpp_script/polynomial_spline.cpp)
# add_dependencies(test_spline corin_control_gencpp)
# target_link_libraries(test_spline ${catkin_LIBRARIES})

####################
##  Test Scripts  ##
####################



# add_executable(test_eigen_quadprog cpp_script/test/test_eigen_quadprog.cpp)
# add_dependencies(test_eigen_quadprog corin_control_gencpp)
# target_link_libraries(test_eigen_quadprog ${catkin_LIBRARIES})

# add_executable(rcg_test cpp_script/rcg_test.cpp
#                         cpp_script/robots/mcorin/transforms.cpp 
#                         cpp_script/robots/mcorin/inertia_properties.cpp 
#                         cpp_script/robots/mcorin/inverse_dynamics.cpp
#                         cpp_script/robots/mcorin/jacobians.cpp)
# add_dependencies(rcg_test corin_control_gencpp)
# target_link_libraries(rcg_test ${catkin_LIBRARIES})