#include "franka_hybrid_control/franka_hybrid_controller.hpp"

namespace franka_hybrid_control
{

FrankaHybridController::FrankaHybridController()
: Node("franka_hybrid_controller"),
  force_axis_(2),  // Default to Z-axis
  target_force_(0.0),
  force_kp_(0.001),
  force_ki_(0.0001),
  force_kd_(0.0),
  force_error_integral_(0.0),
  prev_force_error_(0.0),
  stop_thread_(false),
  connected_to_robot_(false),
  executing_(false)
{
  this->declare_parameter("force_kp", 0.001);
  this->declare_parameter("force_ki", 0.0001);
  this->declare_parameter("force_kd", 0.0);
  
  force_kp_ = this->get_parameter("force_kp").as_double();
  force_ki_ = this->get_parameter("force_ki").as_double();
  force_kd_ = this->get_parameter("force_kd").as_double();
  
  // Create subscribers
  franka_state_sub_ = this->create_subscription<franka_msgs::msg::FrankaRobotState>(
    "/franka_state_controller/franka_states", 10, 
    std::bind(&FrankaHybridController::franka_state_callback, this, std::placeholders::_1));
    
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    std::bind(&FrankaHybridController::joint_state_callback, this, std::placeholders::_1));
    
  // Create publishers
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/cartesian_velocity_controller/cartesian_velocity", 10);
    
  force_error_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/hybrid_controller/force_error", 10);
    
  // Initialize TF listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // Initialize control thread
  control_thread_ = std::thread(&FrankaHybridController::control_loop, this);
  
  RCLCPP_INFO(this->get_logger(), "Franka Hybrid Controller initialized");
}

void FrankaHybridController::franka_state_callback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg)
{
  current_franka_state_ = msg;
  
  // Extract external force from robot state
  current_wrench_[0] = msg->o_f_ext_hat_k.wrench.force.x;  // X force
  current_wrench_[1] = msg->o_f_ext_hat_k.wrench.force.y;  // Y force
  current_wrench_[2] = msg->o_f_ext_hat_k.wrench.force.z;  // Z force
  current_wrench_[3] = msg->o_f_ext_hat_k.wrench.torque.x; // X torque
  current_wrench_[4] = msg->o_f_ext_hat_k.wrench.torque.y; // Y torque
  current_wrench_[5] = msg->o_f_ext_hat_k.wrench.torque.z; // Z torque
}

void FrankaHybridController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joint_state_ = msg;
}

bool FrankaHybridController::initialize_connection()
{
  if (connected_to_robot_) {
    return true;
  }
  
  try {
    // Connect to MoveIt
    RCLCPP_INFO(this->get_logger(), "Connecting to MoveIt instance");
    
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(
        std::make_shared<rclcpp::Node>("franka_move_group_node")), "fr3_arm");
    
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create MoveGroupInterface");
      return false;
    }
    
    move_group_interface_->setMaxVelocityScalingFactor(0.3);
    move_group_interface_->setMaxAccelerationScalingFactor(0.3);
    
    // Wait for initial robot state
    RCLCPP_INFO(this->get_logger(), "Waiting for robot state...");
    auto start_time = this->now();
    while (!current_franka_state_ && (this->now() - start_time).seconds() < 5.0) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    if (!current_franka_state_) {
      RCLCPP_ERROR(this->get_logger(), "Timed out waiting for robot state");
      return false;
    }
    
    connected_to_robot_ = true;
    RCLCPP_INFO(this->get_logger(), "Connected to robot");
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception during connection: %s", e.what());
    return false;
  }
}

void FrankaHybridController::update_force_control()
{
  if (!current_franka_state_) {
    return;
  }
  
  // Calculate force error
  double current_force = current_wrench_[force_axis_];
  double force_error = target_force_ - current_force;
  
  // PID control for force
  force_error_integral_ += force_error * 0.01;  // dt = 0.01 (100Hz)
  double force_error_derivative = (force_error - prev_force_error_) / 0.01;
  prev_force_error_ = force_error;
  
  // Anti-windup for integral term
  if (force_error_integral_ > 1.0) force_error_integral_ = 1.0;
  if (force_error_integral_ < -1.0) force_error_integral_ = -1.0;
  
  // Calculate velocity command for force-controlled axis
  double velocity_command = force_kp_ * force_error + 
                           force_ki_ * force_error_integral_ + 
                           force_kd_ * force_error_derivative;
  
  // Create twist message
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = this->now();
  twist_msg.header.frame_id = "panda_link0";
  
  // Set velocities - only command velocity on force-controlled axis
  switch (force_axis_) {
    case 0:  // X-axis
      twist_msg.twist.linear.x = velocity_command;
      break;
    case 1:  // Y-axis
      twist_msg.twist.linear.y = velocity_command;
      break;
    case 2:  // Z-axis
      twist_msg.twist.linear.z = velocity_command;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid force axis: %d", force_axis_);
      return;
  }
  
  // Publish twist command
  velocity_pub_->publish(twist_msg);
  
  // Publish force error for debugging
  std_msgs::msg::Float64 error_msg;
  error_msg.data = force_error;
  force_error_pub_->publish(error_msg);
}

void FrankaHybridController::control_loop()
{
  rclcpp::Rate rate(100);  // 100Hz control loop
  
  while (rclcpp::ok() && !stop_thread_) {
    if (executing_ && connected_to_robot_) {
      update_force_control();
    }
    
    rate.sleep();
  }
}

bool FrankaHybridController::execute(int force_axis, 
                                     double target_force, 
                                     const geometry_msgs::msg::PoseStamped& target_pose)
{
  if (!initialize_connection()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize connection to robot");
    return false;
  }
  
  force_axis_ = force_axis;
  target_force_ = target_force;
  target_pose_ = target_pose;
  
  // Reset force control parameters
  force_error_integral_ = 0.0;
  prev_force_error_ = 0.0;
  
  // Move to initial position using MoveIt (for position-controlled axes)
  RCLCPP_INFO(this->get_logger(), 
    "Moving to initial position while preparing for force control on axis %d", force_axis_);
  
  // Create a pose target that only controls the non-force axes
  geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose().pose;
  geometry_msgs::msg::Pose goal_pose = target_pose_.pose;
  
  // Only update the position-controlled axes
  switch (force_axis_) {
    case 0:  // X force controlled, Y/Z position controlled
      goal_pose.position.x = current_pose.position.x;  // Keep current X
      break;
    case 1:  // Y force controlled, X/Z position controlled
      goal_pose.position.y = current_pose.position.y;  // Keep current Y
      break;
    case 2:  // Z force controlled, X/Y position controlled
      goal_pose.position.z = current_pose.position.z;  // Keep current Z
      break;
  }
  
  // Set the target pose for MoveIt
  move_group_interface_->setPoseTarget(goal_pose);
  
  // Create a plan
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Planning failed");
    return false;
  }
  
  // Execute the plan
  executing_ = true;
  success = (move_group_interface_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Execution failed");
    executing_ = false;
    return false;
  }
  
  RCLCPP_INFO(this->get_logger(), 
    "Position goal reached, now maintaining hybrid force-position control");
  
  // Continue with force control in the background
  // The control loop will maintain the force on the specified axis
  
  return true;
}

} // namespace franka_hybrid_control