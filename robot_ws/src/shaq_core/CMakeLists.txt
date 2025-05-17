################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(shaq_core)

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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(tensorflow-lite REQUIRED)
#find_package(nav2_msgs REQUIRED)
#find_package(behaviortree_cpp_v3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

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
# Install Python Programs
################################################################################
install(PROGRAMS
# scripts/helloworld.py
# scripts/robot_main_state.py
  # scripts/keyboard_control.py
  # scripts/test_sub_cmd_vel.py
  # scripts/encoder_graph.py
  # scripts/cmd_koby_task.py
  # scripts/apriltag_distance.py
  # scripts/both_detect.py
  scripts/cmd_vel_auto_aim.py
  scripts/cmd_vel_to_motor_speed.py
  scripts/joystick_control.py
  scripts/slider.py

  scripts_camera/cameracapture.py
  scripts_camera/hoop_detection.py
  scripts_camera/apriltag_auto_aim.py
  
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/controller.py
  src/utilize.py
  DESTINATION lib/${PROJECT_NAME}/src
)

################################################################################
# TensorFlow Lite
################################################################################

# Adjust the following to match your path (if you didn't install to /usr/local)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

################################################################################
# Install C++ Programs
################################################################################

add_executable(apriltag_auto_aim scripts_camera/apriltag_auto_aim.cpp)
add_executable(hoop_detection_onnx scripts_camera/hoop_detection_onnx.cpp)
add_executable(hoop_detection_TFLite scripts_camera/hoop_detection_TFLite.cpp)

ament_target_dependencies(apriltag_auto_aim 
  rclcpp
  sensor_msgs 
  geometry_msgs 
  cv_bridge
)

ament_target_dependencies(hoop_detection_onnx
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  ament_index_cpp
)

ament_target_dependencies(hoop_detection_TFLite
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  ament_index_cpp
)

target_include_directories(hoop_detection_TFLite 
  PUBLIC 
    /usr/local/include
    ${PROJECT_SOURCE_DIR}/include  # Optional: Add your own headers
)

target_link_libraries(apriltag_auto_aim ${OpenCV_LIBS} apriltag)
target_link_libraries(hoop_detection_onnx  ${OpenCV_LIBS})
target_link_libraries(hoop_detection_TFLite
  ${OpenCV_LIBS}
  tensorflowlite
)


install(TARGETS
  apriltag_auto_aim
  hoop_detection_onnx
  hoop_detection_TFLite
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Install Picture Model
################################################################################
install(DIRECTORY 
  models/
  DESTINATION share/${PROJECT_NAME}/models
)

################################################################################

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
