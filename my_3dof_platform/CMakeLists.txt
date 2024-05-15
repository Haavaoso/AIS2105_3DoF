cmake_minimum_required(VERSION 3.8)
project(my_3dof_platform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable for creating the xacro file
add_executable(create_xacro src/create_xacro.cpp)

# Add the executable for the platform simulator
add_executable(platform_simulator src/platform_simulator.cpp)

# Link the necessary libraries
ament_target_dependencies(platform_simulator rclcpp geometry_msgs)

# Install the executables
install(TARGETS
  create_xacro
  platform_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

