cmake_minimum_required(VERSION 3.10)
project(_Legged_Robotics_CPP_BINDINGS)

set(CMAKE_CXX_STANDARD 17)
set(PYBIND11_PYTHON_VERSION 3.6)

set(PYTHON_EXECUTABLE "/home/daniel/.conda/envs/Corin2024/bin/python")

set(CMAKE_PREFIX_PATH "/home/daniel/.conda/envs/Corin2024/share/cmake/pybind11" ${CMAKE_PREFIX_PATH})

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the DQ Robotics library as a subdirectory
add_subdirectory(/home/daniel/catkin_ws/src/corin/cpp/dqrobotics_legged_robots_build)

# Add the target for the C++ library, but don't include posture_calc.cpp here
add_library(${PROJECT_NAME} MODULE python_wrapper.cpp)

# Include the headers
target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE /home/daniel/catkin_ws/src/corin/cpp/dqrobotics_legged_robots_build/include)

# link with DQ Robotics
target_link_libraries(${PROJECT_NAME} PUBLIC dqrobotics)

# Link the pybind11 module
target_link_libraries(${PROJECT_NAME} PRIVATE pybind11::module)