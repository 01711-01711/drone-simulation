cmake_minimum_required(VERSION 3.5)
project(drone_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Install launch and rviz config
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  drone_bringup/init_joint_positions.py
  drone_bringup/pid_control_node.py
  drone_bringup/slider_control_service.py
  drone_bringup/dummy_joint_state_publisher.py
  drone_bringup/dummy_imu_publisher.py
  drone_bringup/scripts/toggle_mechanism_service.py
  drone_bringup/scripts/mechanism_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
