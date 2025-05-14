#include "franka_hybrid_control/force_position_interface.hpp"
#include <mutex>
#include "std_msgs/msg/float32.hpp"  // Make sure this is included

namespace franka_hybrid_control
{

std::mutex control_mutex_;

ForcePositionInterface::ForcePositionInterface()
: Node("force_position_interface"),  // Use a consistent name
  force_axis_(2),  // Default to Z-axis
  target_force_(0.0),
  params_updated_(false)
{
  // Declare parameters
  this->declare_parameter<std::string>("robot_description", "");
  this->declare_parameter<std::string>("robot_description_semantic", "");
  this->declare_parameter<std::string>("kinematics_config", "");
  this->declare_parameter<std::string>("force_control_params", "");

  // Get parameters with error handling
  std::string robot_description = this->get_parameter("robot_description").as_string();
  if (robot_description.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_description' is missing or empty!");
    rclcpp::shutdown();
    return;
  }
  std::string robot_description_semantic = this->get_parameter("robot_description_semantic").as_string();
  std::string kinematics_config = this->get_parameter("kinematics_config").as_string();
  std::string force_control_params = this->get_parameter("force_control_params").as_string();

  RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
  RCLCPP_INFO(this->get_logger(), "  - robot_description: %s", robot_description.c_str());
  RCLCPP_INFO(this->get_logger(), "  - robot_description_semantic: %s", robot_description_semantic.c_str());
  RCLCPP_INFO(this->get_logger(), "  - kinematics_config: %s", kinematics_config.c_str());
  RCLCPP_INFO(this->get_logger(), "  - force_control_params: %s", force_control_params.c_str());

  // Create the controller
  controller_ = std::make_shared<FrankaHybridController>();

  // Create subscribers
  force_axis_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/hybrid_controller/force_axis", 10,
    std::bind(&ForcePositionInterface::force_axis_callback, this, std::placeholders::_1));

  force_target_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/hybrid_controller/force_target", 10,
    std::bind(&ForcePositionInterface::force_target_callback, this, std::placeholders::_1));

  pose_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/hybrid_controller/pose_target", 10,
    std::bind(&ForcePositionInterface::pose_target_callback, this, std::placeholders::_1));

  // Set up timer for checking if we need to execute
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // Adjust as needed
    std::bind(&ForcePositionInterface::execute_control, this));

  RCLCPP_INFO(this->get_logger(), "Force Position Interface initialized");
  RCLCPP_INFO(this->get_logger(), "Publish targets to:");
  RCLCPP_INFO(this->get_logger(), "  - /hybrid_controller/force_axis (Int32, 0=X, 1=Y, 2=Z)");
  RCLCPP_INFO(this->get_logger(), "  - /hybrid_controller/force_target (Float64, in Newtons)");
  RCLCPP_INFO(this->get_logger(), "  - /hybrid_controller/pose_target (PoseStamped)");
}

void ForcePositionInterface::force_axis_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(control_mutex_);
  if (msg->data >= 0 && msg->data <= 2) {
    force_axis_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Force axis set to %d", force_axis_);
    params_updated_ = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid force axis: %d (must be 0, 1, or 2)", msg->data);
  }
}

void ForcePositionInterface::force_target_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  target_force_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "Force target set to %.2f N", target_force_);
  params_updated_ = true;
}

void ForcePositionInterface::pose_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  target_pose_ = *msg;
  RCLCPP_INFO(this->get_logger(), "Pose target updated");
  params_updated_ = true;
}

void ForcePositionInterface::execute_control()
{
  std::lock_guard<std::mutex> lock(control_mutex_);
  if (params_updated_) {
    params_updated_ = false;

    // Check if we have a valid pose
    if (target_pose_.header.frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(), "No pose target set yet, waiting...");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "Executing hybrid control with force axis=%d, target=%.2f N",
      force_axis_, target_force_);

    // Execute the controller
    if (!controller_->execute(force_axis_, target_force_, target_pose_)) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to execute hybrid control. Force axis: %d, Target force: %.2f N, Pose frame_id: %s",
        force_axis_, target_force_, target_pose_.header.frame_id.c_str());
    }
  }
}

} // namespace franka_hybrid_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto force_interface = std::make_shared<franka_hybrid_control::ForcePositionInterface>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(force_interface);  // Only add the main node
  executor.spin();

  rclcpp::shutdown();
  return 0;
}