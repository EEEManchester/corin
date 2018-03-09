# Install script for directory: /home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/iit/robots/dlm_yp/declarations.h;/usr/local/include/iit/robots/dlm_yp/link_data_map.h;/usr/local/include/iit/robots/dlm_yp/joint_data_map.h;/usr/local/include/iit/robots/dlm_yp/transforms.h;/usr/local/include/iit/robots/dlm_yp/kinematics_parameters.h;/usr/local/include/iit/robots/dlm_yp/jacobians.h;/usr/local/include/iit/robots/dlm_yp/traits.h;/usr/local/include/iit/robots/dlm_yp/jsim.h;/usr/local/include/iit/robots/dlm_yp/inverse_dynamics.h;/usr/local/include/iit/robots/dlm_yp/forward_dynamics.h;/usr/local/include/iit/robots/dlm_yp/inertia_properties.h;/usr/local/include/iit/robots/dlm_yp/dynamics_parameters.h;/usr/local/include/iit/robots/dlm_yp/miscellaneous.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/iit/robots/dlm_yp" TYPE FILE FILES
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./declarations.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./link_data_map.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./joint_data_map.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./transforms.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./kinematics_parameters.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./jacobians.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./traits.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./jsim.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./inverse_dynamics.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./forward_dynamics.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./inertia_properties.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./dynamics_parameters.h"
    "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/./miscellaneous.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libiitgendlm_yp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libiitgendlm_yp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libiitgendlm_yp.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libiitgendlm_yp.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/build/libiitgendlm_yp.so")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libiitgendlm_yp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libiitgendlm_yp.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libiitgendlm_yp.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/wilson/catkin_ws/src/lcorin/lcorin_control/cpp_script/robots/dlm_yp/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
