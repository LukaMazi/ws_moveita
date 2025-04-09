cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# Add include directories for MoveIt
include_directories(${moveit_ros_planning_interface_INCLUDE_DIRS})

add_executable(collision_detection src/collision_detection.cpp)
target_include_directories(collision_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)
target_compile_features(collision_detection PUBLIC c_std_99 cxx_std_17)  # Changed to collision_detection
ament_target_dependencies(
  collision_detection
  rclcpp
  moveit_ros_planning_interface
  # Removed hello_moveit (package name shouldn't be here)
)

install(TARGETS collision_detection
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()