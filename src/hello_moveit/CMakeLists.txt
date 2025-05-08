cmake_minimum_required(VERSION 3.8)
project(hello_moveit)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
 add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
# Add include directories for MoveIt
include_directories(${moveit_ros_planning_interface_INCLUDE_DIRS})
# Executable 1: collision_detection
add_executable(collision_detection src/collision_detection.cpp)
target_include_directories(collision_detection PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
${moveit_ros_planning_interface_INCLUDE_DIRS}
)
target_compile_features(collision_detection PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
 collision_detection
 rclcpp
 moveit_ros_planning_interface
 moveit_visual_tools
)
# ✅ Executable 2: hello_moveit
add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
${moveit_ros_planning_interface_INCLUDE_DIRS}
)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
 hello_moveit
 rclcpp
 moveit_ros_planning_interface
 moveit_visual_tools
)
# ✅ Executable 3: mojmoj
add_executable(mojmoj src/mojmoj.cpp)
target_include_directories(mojmoj PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
${moveit_ros_planning_interface_INCLUDE_DIRS}
)
target_compile_features(mojmoj PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
 mojmoj
 rclcpp
 moveit_ros_planning_interface
 moveit_visual_tools
)
# ✅ Executable 4: srcek
add_executable(srcek src/srcek.cpp)
target_include_directories(srcek PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
${moveit_ros_planning_interface_INCLUDE_DIRS}
)
target_compile_features(srcek PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
 srcek
 rclcpp
 moveit_ros_planning_interface
 moveit_visual_tools
)

# ✅ Executable 5: simple_mover
add_executable(simple_mover src/simple_mover.cpp)
target_include_directories(simple_mover PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
${moveit_ros_planning_interface_INCLUDE_DIRS}
)
target_compile_features(simple_mover PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
 simple_mover
 rclcpp
 moveit_ros_planning_interface
 moveit_visual_tools
)
# Install all targets
install(TARGETS
 collision_detection
 hello_moveit
 mojmoj
 srcek
 simple_mover
 DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()