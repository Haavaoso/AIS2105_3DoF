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
  std_msgs 
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


  add_executable(regulator src/regulator.cpp)
ament_target_dependencies(regulator 
rclcpp
std_msgs)


add_executable(launcher src/launcher.cpp)
ament_target_dependencies(launcher 
  rclcpp
  std_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport
  vision_msgs
  OpenCV)

# Add launcher to installation targets
install(TARGETS 
  preprocess
  hough_transform
  canny_edge
  regulator
  launcher
  DESTINATION lib/${PROJECT_NAME})

   
# Package setup
ament_package()

