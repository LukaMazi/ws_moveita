// franka_heart_trajectory.cpp
// ROS2 node for moving Franka Research 3 robot in a heart-shaped trajectory
// With continuous motion and arm extension at the end

#include <memory>
#include <chrono>
#include <math.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class FrankaHeartTrajectory : public rclcpp::Node
{
public:
  FrankaHeartTrajectory() : Node("franka_heart_trajectory")
  {
    // Initialize parameters
    this->declare_parameter("planning_time", 10.0);
    this->declare_parameter("num_points", 75);  // Moderate number of points for smooth trajectory
    this->declare_parameter("heart_size", 0.1);  // Smaller heart size to avoid desk collision
    this->declare_parameter("heart_center_x", 0.5);  // Heart center position in robot frame
    this->declare_parameter("heart_center_y", 0.0);
    this->declare_parameter("heart_center_z", 0.4);
    this->declare_parameter("move_group_name", "fr3_arm");
    this->declare_parameter("velocity_scaling", 0.3);  // Moderate velocity for smooth motion
    this->declare_parameter("acceleration_scaling", 0.3);  // Moderate acceleration
    
    // Use a timer to allow ROS system to initialize before running trajectory
    timer_ = this->create_wall_timer(
      5s, std::bind(&FrankaHeartTrajectory::execute_trajectory, this));
      
    RCLCPP_INFO(this->get_logger(), "FrankaHeartTrajectory node initialized, waiting to connect to existing MoveIt instance");
  }

private:
  void execute_trajectory()
  {
    // Cancel the timer after first execution
    timer_->cancel();
    
    try {
      RCLCPP_INFO(this->get_logger(), "Connecting to existing MoveIt instance");
      
      // Get the move_group name from parameter
      std::string move_group_name = this->get_parameter("move_group_name").as_string();
      
      // Connect to the existing MoveIt instance
      auto move_group_node = std::make_shared<rclcpp::Node>(
        "franka_heart_trajectory_moveit_client",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
      );
      
      // Use a separate thread for the move_group_node to avoid blocking
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(move_group_node);
      std::thread executor_thread([&executor]() { executor.spin(); });
      
      // Initialize MoveGroupInterface with the separate node
      moveit::planning_interface::MoveGroupInterface move_group(move_group_node, move_group_name);
      
      // Set planning time and velocity constraints
      move_group.setPlanningTime(this->get_parameter("planning_time").as_double());
      move_group.setMaxVelocityScalingFactor(this->get_parameter("velocity_scaling").as_double());
      move_group.setMaxAccelerationScalingFactor(this->get_parameter("acceleration_scaling").as_double());
      
      // Get parameters
      int num_points = this->get_parameter("num_points").as_int();
      double heart_size = this->get_parameter("heart_size").as_double();
      double heart_center_x = this->get_parameter("heart_center_x").as_double();
      double heart_center_y = this->get_parameter("heart_center_y").as_double();
      double heart_center_z = this->get_parameter("heart_center_z").as_double();
      
      // Create orientation quaternion for front plane heart
      tf2::Quaternion orientation;
      orientation.setRPY(0, M_PI_2, 0);  // Roll=0, Pitch=90deg, Yaw=0 - Looking forward
      orientation.normalize();
      
      // Get current state as starting point
      move_group.setStartStateToCurrentState();
      
      // Generate heart shape points - in YZ plane (front plane)
      std::vector<geometry_msgs::msg::Pose> waypoints;
      generate_heart_waypoints_front_plane(waypoints, num_points, heart_size, 
                                          heart_center_x, heart_center_y, heart_center_z, 
                                          orientation);
      
      // Display info
      RCLCPP_INFO(this->get_logger(), "Reference frame: %s", move_group.getPlanningFrame().c_str());
      RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
      RCLCPP_INFO(this->get_logger(), "Drawing heart with %zu waypoints", waypoints.size());
      
      // Plan and execute the heart trajectory as a single continuous cartesian path
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;  // Disable jump threshold
      const double eef_step = 0.005;  // Small step size for smooth trajectory
      
      double path_fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      
      RCLCPP_INFO(this->get_logger(), "Planned %.2f%% of the heart trajectory", path_fraction * 100.0);
      
      if (path_fraction > 0.95) {  // At least 95% of the path was computed
        moveit::planning_interface::MoveGroupInterface::Plan heart_plan;
        heart_plan.trajectory_ = trajectory;
        
        RCLCPP_INFO(this->get_logger(), "Executing heart trajectory");
        move_group.execute(heart_plan);
        RCLCPP_INFO(this->get_logger(), "Heart trajectory execution completed");
        
        // Now extend the arm fully up in the air while keeping the hand orientation
        RCLCPP_INFO(this->get_logger(), "Planning arm extension fully upwards");

        // Get the current joint values to avoid unexpected movements
        std::vector<double> current_joint_values = move_group.getCurrentJointValues();

        // Define the target joint values for a fully extended arm upwards
        std::map<std::string, double> target_joint_values;

        // Adjust these values based on your FR3's joint configuration for "fully up"
        // You might need to experiment with these values to achieve the desired pose.
        // It's crucial to stay within the joint limits of your robot.
        target_joint_values["fr3_joint1"] = 0.0;     // Base rotation (adjust as needed)
        target_joint_values["fr3_joint2"] = -1.57;  // Shoulder abduction (approx. -90 degrees)
        target_joint_values["fr3_joint3"] = 0.0;     // Elbow rotation (adjust as needed)
        target_joint_values["fr3_joint4"] = -1.57;  // Wrist 1 flexion/extension (approx. -90 degrees)
        target_joint_values["fr3_joint5"] = 0.0;     // Wrist 2 rotation (adjust as needed)
        target_joint_values["fr3_joint6"] = 1.57;   // Wrist 3 flexion/extension (approx. +90 degrees)
        target_joint_values["fr3_joint7"] = 0.0;     // Hand rotation (adjust as needed)

        // Move to the target joint values
        move_group.setJointValueTarget(target_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan extension_plan;

        bool extension_success = (move_group.plan(extension_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (extension_success) {
          RCLCPP_INFO(this->get_logger(), "Executing arm extension upwards");
          move_group.execute(extension_plan);
          RCLCPP_INFO(this->get_logger(), "Extended arm fully upwards");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to plan arm extension upwards");
        }

      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute enough of the heart trajectory (%.2f%%)", path_fraction * 100.0);
      }
      
      // Clean up
      executor.cancel();
      executor_thread.join();
      
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception occurred: %s", e.what());
    }
  }
  
  void generate_heart_waypoints_front_plane(
    std::vector<geometry_msgs::msg::Pose>& waypoints,
    int num_points,
    double size,
    double center_x,
    double center_y,
    double center_z,
    const tf2::Quaternion& orientation)
  {
    // Ensure we start with an empty vector
    waypoints.clear();
    
    // Generate heart waypoints in a single continuous loop
    // Using angle parameter from 0 to 2π
    for (int i = 0; i <= num_points; ++i) {
      double t = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_points);
      
      // Heart curve parametric equations
      // x = 16 * sin(t)^3
      // y = 13 * cos(t) - 5 * cos(2t) - 2 * cos(3t) - cos(4t)
      double param_x = 16.0 * pow(sin(t), 3);
      double param_y = 13.0 * cos(t) - 5.0 * cos(2*t) - 2.0 * cos(3*t) - cos(4*t);
      
      // Scale and position the heart in YZ plane (front plane)
      // X stays constant (depth), heart is drawn in Y-Z plane
      double x = center_x;  // Fixed X position
      double y = param_x * size / 16.0 + center_y;  // param_x maps to Y axis
      double z = param_y * size / 16.0 + center_z;  // param_y maps to Z axis
      
      // Create pose
      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      
      // Convert tf2 quaternion to geometry_msgs quaternion
      pose.orientation = tf2::toMsg(orientation);
      
      waypoints.push_back(pose);
    }
    
    // Make sure to close the loop by adding the first point again
    if (!waypoints.empty()) {
      waypoints.push_back(waypoints.front());
    }
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrankaHeartTrajectory>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}