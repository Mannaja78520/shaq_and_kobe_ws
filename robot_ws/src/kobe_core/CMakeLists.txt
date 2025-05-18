################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(kobe_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(nav2_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(behaviortree_cpp_v3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch scripts config
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Python package installation
################################################################################
ament_python_install_package(${PROJECT_NAME})

################################################################################
# Install Programs
################################################################################
install(PROGRAMS
# scripts/helloworld.py
  # scripts/robot_main_state.py
  # scripts/keyboard_control.py
  # scripts/test_sub_cmd_vel.py
  scripts/joystick_control.py
  scripts/cmd_vel_to_motor_speed.py
  # scripts/servo.py 
  scripts/slider.py
  scripts/hoop_detection.py
  scripts/cmd_vel_auto_aim.py
  scripts/cameracapture.py
  scripts/apriltag_auto_aim.py
  scripts/joystickaing.py
  scripts/cmd_move_omni.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  kobe_core/controller.py
  kobe_core/utilize.py
  DESTINATION lib/${PROJECT_NAME}/src
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

################################################################################
# Macro for ament package
################################################################################
ament_package()
