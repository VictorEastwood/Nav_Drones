cmake_minimum_required(VERSION 3.8)
project(nav_drone_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_ros_com REQUIRED)

#################
# Setup targets #
#################

# include directories
include_directories(include)

# nav_drone_control_node
add_executable(nav_drone_control_node src/nav_drone_control.cpp)
ament_target_dependencies(nav_drone_control_node rclcpp px4_msgs px4_ros_com)
install(TARGETS nav_drone_control_node DESTINATION lib/${PROJECT_NAME})

# nav_drone_control_srv_node
add_executable(nav_drone_control_srv_node src/nav_drone_control_srv.cpp)
ament_target_dependencies(nav_drone_control_srv_node rclcpp px4_msgs px4_ros_com)
install(TARGETS nav_drone_control_srv_node DESTINATION lib/${PROJECT_NAME})

# # frame_transforms
# target_link_libraries(nav_drone_control_node
#   frame_transforms
# )
############
# Install ##
############
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

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
