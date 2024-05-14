cmake_minimum_required(VERSION 3.8)
project(dof_platform)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)

# Executable for preprocessing images
add_executable(preprocess src/preprocess.cpp)
ament_target_dependencies(preprocess 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  geometry_msgs
  OpenCV)

# Executable for Contour transform
add_executable(contourPub src/contourPublish.cpp)
ament_target_dependencies(contourPub 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  geometry_msgs
  OpenCV)
  
  # Executable for Hough transform
add_executable(hough src/hough.cpp)
ament_target_dependencies(hough 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  geometry_msgs
  OpenCV)
   
# Installation
install(TARGETS 
  preprocess
  contourPub
  hough
  DESTINATION lib/${PROJECT_NAME})

# Package setup
ament_package()
