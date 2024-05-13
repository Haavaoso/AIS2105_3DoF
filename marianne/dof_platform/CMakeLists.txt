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
find_package(OpenCV 4 REQUIRED)

# Conditional linting options
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_lint_auto_find_test_dependencies())
endif()

# Executable for preprocessing images
add_executable(preprocess src/preprocess.cpp)
ament_target_dependencies(preprocess 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  OpenCV)

# Executable for Hough transform
add_executable(hough_transform src/hough.cpp)
ament_target_dependencies(hough_transform 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  OpenCV)

# Executable for Canny edge detection
add_executable(canny_edge src/canny_edge_detection.cpp)
ament_target_dependencies(canny_edge 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  OpenCV)

# Executable for camera topic handling
add_executable(camera_topic src/camera_topic.cpp)
ament_target_dependencies(camera_topic 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  OpenCV)

# Executable for object tracking
#add_executable(object_tracker src/object_tracker.cpp)
#ament_target_dependencies(object_tracker 
#  rclcpp 
#  sensor_msgs 
#  cv_bridge 
#  image_transport 
#  vision_msgs
#  OpenCV)

   
# Installation
install(TARGETS 
  preprocess
  hough_transform
  canny_edge
  camera_topic
  #object_tracker
  DESTINATION lib/${PROJECT_NAME})

# Package setup
ament_package()

