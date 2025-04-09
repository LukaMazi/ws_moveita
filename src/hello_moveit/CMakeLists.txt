cmake_minimum_required(VERSION 3.8)
project(hello_moveit)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)  # ADDED: For collision detection
find_package(moveit_msgs REQUIRED)  # ADDED: For collision messages
find_package(geometric_shapes REQUIRED)  # ADDED: For collision geometries
find_package(tf2_ros REQUIRED)  # ADDED: For transforms

# Original executable
add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${moveit_ros_planning_interface_INCLUDE_DIRS}
)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
 hello_moveit
 rclcpp
 moveit_ros_planning_interface
)

# ADDED: New collision detection executable
add_executable(collision_detection src/collision_detection.cpp)
target_include_directories(collision_detection PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${moveit_ros_planning_interface_INCLUDE_DIRS}
 ${moveit_core_INCLUDE_DIRS}
)
target_compile_features(collision_detection PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
 collision_detection
 rclcpp
 moveit_ros_planning_interface
 moveit_core
 moveit_msgs
 geometric_shapes
 tf2_ros
)

install(TARGETS hello_moveit collision_detection  # MODIFIED: Added collision_detection
 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
 DESTINATION share/${PROJECT_NAME}
)
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