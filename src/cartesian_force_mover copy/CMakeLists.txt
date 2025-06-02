cmake_minimum_required(VERSION 3.5)
project(cartesian_force_mover_copy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bare_mover src/bare_mover.cpp)
ament_target_dependencies(bare_mover rclcpp geometry_msgs std_msgs)

install(TARGETS bare_mover DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()