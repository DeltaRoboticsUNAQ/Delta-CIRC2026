cmake_minimum_required(VERSION 3.8)
project(circ_chassis_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(xbox_teleop src/xbox_teleop.cpp)
ament_target_dependencies(xbox_teleop rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  xbox_teleop
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
