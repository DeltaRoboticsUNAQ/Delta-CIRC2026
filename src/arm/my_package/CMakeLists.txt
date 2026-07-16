cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


# find arm packages
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

 

add_executable(my_node src/my_node.cpp)
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install meshes
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Gazebo worlds
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install scripts
install(PROGRAMS
  scripts/keyboard_arduino.py
  scripts/arm_keyboard_teleop.py
  scripts/arm_hw_teleop_bridge.py
  scripts/lector_stm32.py
  scripts/arm_hid_teleop.py
  scripts/arm_hardware_bridge.py
  scripts/stm32_hardware_bridge.py
  scripts/fake_stm32.py
  scripts/moveit_hw_bridge.py
  scripts/robot_description_publisher.py
  scripts/gazebo_set_initial_joint_positions.py
  scripts/arm_moveit_teleop_actions.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install firmware documentation
install(DIRECTORY firmware/
  DESTINATION share/${PROJECT_NAME}/firmware
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
