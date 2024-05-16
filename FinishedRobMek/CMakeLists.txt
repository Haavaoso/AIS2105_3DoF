cmake_minimum_required(VERSION 3.8)
project(node_hell)

# Set compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set global C++ standard and PIC
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)
find_package(xacro REQUIRED)

# Define executables and their dependencies
add_executable(create_xacro src/create_xacro.cpp)
ament_target_dependencies(create_xacro rclcpp)

add_executable(platform_simulator src/platform_simulator.cpp)
ament_target_dependencies(platform_simulator rclcpp geometry_msgs)

add_executable(preprocess src/preprocess.cpp)
ament_target_dependencies(preprocess rclcpp sensor_msgs cv_bridge image_transport vision_msgs OpenCV)

add_executable(hough src/hough.cpp)
ament_target_dependencies(hough rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge image_transport vision_msgs OpenCV)

add_executable(contour src/contour.cpp)
ament_target_dependencies(contour rclcpp sensor_msgs cv_bridge image_transport vision_msgs geometry_msgs OpenCV)

add_library(arduino_driver SHARED src/arduino_driver.cpp)
 #Specify include directories
target_include_directories(arduino_driver PRIVATE include)

ament_target_dependencies(arduino_driver
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  vision_msgs
  OpenCV
  trajectory_msgs
  hardware_interface
  controller_manager
  pluginlib
  serial) 

add_executable(regulator src/regulator.cpp)
ament_target_dependencies(regulator rclcpp geometry_msgs std_msgs)

add_executable(camera_kin src/camera_kinematics.cpp)
ament_target_dependencies(camera_kin rclcpp sensor_msgs cv_bridge image_transport vision_msgs geometry_msgs OpenCV)

add_executable(launcher src/launcher.cpp)
ament_target_dependencies(launcher rclcpp std_msgs sensor_msgs cv_bridge image_transport vision_msgs OpenCV)

add_executable(yeeter src/joint_publisher.cpp)
ament_target_dependencies(yeeter rclcpp trajectory_msgs std_msgs)

add_executable(tester src/Joint_tester.cpp)
ament_target_dependencies(tester rclcpp std_msgs)

# Install executable targets
install(TARGETS 
  create_xacro
  platform_simulator
  preprocess
  hough
  contour
  arduino_driver
  regulator
  camera_kin
  launcher
  yeeter
  tester
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and other resources
install(DIRECTORY
  config
  launch
  ros2_control
  urdf
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY include/
  DESTINATION include)
# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

