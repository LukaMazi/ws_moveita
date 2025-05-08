cmake_minimum_required(VERSION 3.8)
project(franka_hybrid_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

# Include headers
include_directories(
  include  # This makes <franka_hybrid_control/xxx.hpp> available
  ${other_dependencies_INCLUDE_DIRS}
  )

# Declare a C++ library
add_library(franka_hybrid_controller SHARED
  src/franka_hybrid_controller.cpp
)

target_include_directories(franka_hybrid_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(franka_hybrid_controller
  rclcpp
  controller_interface
  franka_msgs
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  std_msgs
)

# Declare a C++ executable
add_executable(force_position_node src/force_position_node.cpp)

target_include_directories(force_position_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(force_position_node
  rclcpp
  franka_msgs
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

target_link_libraries(force_position_node
  franka_hybrid_controller
)

# Install targets
install(TARGETS
  franka_hybrid_controller
  force_position_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch/config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Export stuff
ament_export_include_directories(include)
ament_export_libraries(franka_hybrid_controller)
ament_export_dependencies(
  rclcpp
  controller_interface
  franka_msgs
  sensor_msgs
  trajectory_msgs
  geometry_msgs
  moveit_ros_planning_interface
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  std_msgs
)

ament_package()
