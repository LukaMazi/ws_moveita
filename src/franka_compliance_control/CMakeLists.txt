cmake_minimum_required(VERSION 3.8)
project(franka_compliance_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) # Za FrankaState in WrenchStamped (za adapter)
find_package(franka_msgs REQUIRED) # Za FrankaState (za adapter)

# --- VOZLIŠČE: HybridReferencePublisher ---
add_executable(hybrid_reference_publisher src/hybrid_reference_publisher.cpp)
ament_target_dependencies(hybrid_reference_publisher
  rclcpp
  std_msgs
  geometry_msgs
)

# --- VOZLIŠČE: FrankaFTDataAdapter (morate še implementirati) ---
add_executable(franka_ft_data_adapter src/franka_ft_data_adapter.cpp)
ament_target_dependencies(franka_ft_data_adapter
  rclcpp
  sensor_msgs  # Za WrenchStamped
  franka_msgs  # Za FrankaState
  geometry_msgs # Za morebitne transformacije
)

# Install executables
install(TARGETS
  hybrid_reference_publisher
  franka_ft_data_adapter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch, config, and urdf files
# Ustvarite te mape v vašem paketu: launch, config, urdf
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies (minimalno)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  franka_msgs
)

ament_package()