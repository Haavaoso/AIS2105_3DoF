cmake_minimum_required(VERSION 3.8)
project(node_hell)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_lint_auto_find_test_dependencies())
endif()

add_executable(preprocess src/preprocess.cpp)
ament_target_dependencies(preprocess 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  OpenCV)

add_executable(hough src/hough.cpp)
ament_target_dependencies(hough 
  rclcpp 
  sensor_msgs
  geometry_msgs
  std_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  OpenCV)



add_executable(contour src/contour.cpp)
ament_target_dependencies(contour 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  geometry_msgs
  OpenCV) 


add_executable(regulator src/regulator.cpp)
ament_target_dependencies(regulator 
rclcpp
std_msgs)

add_executable(camera_kin src/camera_kinematics.cpp)
ament_target_dependencies(camera_kin 
rclcpp 
sensor_msgs 
cv_bridge 
image_transport
vision_msgs
geometry_msgs
OpenCV)


add_executable(launcher src/launcher.cpp)
ament_target_dependencies(launcher 
  rclcpp
  std_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  OpenCV)

install(TARGETS 
  preprocess
  hough
  regulator
  contour
  camera_kin
  launcher
  DESTINATION lib/${PROJECT_NAME})

ament_package()

