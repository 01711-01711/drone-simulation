cmake_minimum_required(VERSION 3.10)
project(thrust_plugin)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Gazebo and Ignition
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(thrust_plugin SHARED src/ThrustPlugin.cpp)
target_link_libraries(thrust_plugin ${GAZEBO_LIBRARIES})
target_include_directories(thrust_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(thrust_plugin rclcpp std_msgs)

install(TARGETS thrust_plugin
  LIBRARY DESTINATION lib)

ament_package()
