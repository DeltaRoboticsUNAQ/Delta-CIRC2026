cmake_minimum_required(VERSION 3.8)
project(circ_rover_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(circ_chassis_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(aeb_controller src/aeb_controller.cpp)
ament_target_dependencies(aeb_controller rclcpp sensor_msgs geometry_msgs circ_chassis_msgs)

add_executable(adaptive_speed_controller src/adaptive_speed_controller.cpp)
ament_target_dependencies(adaptive_speed_controller rclcpp sensor_msgs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(unified_safety_controller src/unified_safety_controller.cpp)
ament_target_dependencies(unified_safety_controller rclcpp sensor_msgs geometry_msgs tf2 tf2_geometry_msgs circ_chassis_msgs)

install(TARGETS
  aeb_controller
  adaptive_speed_controller
  unified_safety_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
