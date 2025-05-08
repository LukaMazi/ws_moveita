#include "franka_hybrid_control/force_position_interface.hpp"

namespace franka_hybrid_control
{

ForcePositionInterface::ForcePositionInterface()
: Node("force_position_interface"),
  force_axis_(2),  // Default to Z-axis
  target_force_(0.0),
  params_updated_(false)
{
  // Create the controller
  controller_ = std::make_shared<FrankaHybridController>();
  
  // Create subscribers
  force_axis_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/hybrid_controller/force_axis", 10,
    std::bind(&ForcePositionInterface::force_axis_callback, this, std::placeholders::_1));
    
  force_target_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/hybrid_controller/force_target", 10,
    std::bind(&ForcePositionInterface::force_target_callback, this, std::placeholders::_1));
    
  pose_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/hybrid_controller/pose_target", 10,
    std::bind(&ForcePositionInterface::pose_target_callback, this, std::placeholders::_1));
    
  // Set up timer for checking if we need to execute
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), 
    std::bind(&ForcePositionInterface::execute_control, this));
    
  RCLCPP_INFO(this->get_logger(), "Force Position Interface initialized");
  RCLCPP_INFO(this->get_logger(), "Publish targets to:");
  RCLCPP_INFO(this->get_logger(), "  - /hybrid_controller/force_axis (Int32, 0=X, 1=Y, 2=Z)");
  RCLCPP_INFO(this->get_logger(), "  - /hybrid_controller/force_target (Float64, in Newtons)");
  RCLCPP_INFO(this->get_logger(), "  - /hybrid_controller/pose_target (PoseStamped)");
}

void ForcePositionInterface::force_axis_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (msg->data >= 0 && msg->data <= 2) {
    force_axis_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Force axis set to %d", force_axis_);
    params_updated_ = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Invalid force axis: %d (must be 0, 1, or 2)", msg->data);
  }
}

void ForcePositionInterface::force_target_callback(const std_msgs::msg::Float64::SharedPtr msg)
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
      RCLCPP_ERROR(this->get_logger(), "Failed to execute hybrid control");
    }
  }
}

} // namespace franka_hybrid_control

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto force_interface = std::make_shared<franka_hybrid_control::ForcePositionInterface>();
  auto hybrid_controller = force_interface->get_controller();  // Add a getter for controller_

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(force_interface);
  executor.add_node(hybrid_controller);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}