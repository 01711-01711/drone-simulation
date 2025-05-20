cmake_minimum_required(VERSION 3.10)
project(joint_effort_plugin)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(joint_effort_plugin SHARED src/JointEffortPlugin.cpp)
ament_target_dependencies(joint_effort_plugin rclcpp std_msgs)

target_link_libraries(joint_effort_plugin ${GAZEBO_LIBRARIES})

install(TARGETS joint_effort_plugin
  LIBRARY DESTINATION lib)

ament_package()
