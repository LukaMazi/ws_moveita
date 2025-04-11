#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("franka_squiggly_line_follower");
  auto logger = node->get_logger();

  // Wait for connections to establish
  RCLCPP_INFO(logger, "Waiting for ROS connections to establish...");
  rclcpp::sleep_for(5s);
  
  // Create MoveIt components
  RCLCPP_INFO(logger, "Creating MoveIt interfaces...");
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "fr3_arm");
  auto planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  
  // Configure MoveIt
  move_group->setPlanningTime(15.0);
  move_group->setMaxVelocityScalingFactor(0.2);
  move_group->setMaxAccelerationScalingFactor(0.2);
  move_group->setGoalPositionTolerance(0.01);  // 1cm position tolerance
  move_group->setGoalOrientationTolerance(0.05);  // ~3 degrees orientation tolerance
  
  // Wait for robot state
  RCLCPP_INFO(logger, "Waiting for current robot state...");
  bool state_received = false;
  int attempts = 0;
  auto current_pose = geometry_msgs::msg::Pose();
  
  while (!state_received && attempts < 5) {
    try {
      current_pose = move_group->getCurrentPose().pose;
      if (current_pose.position.x != 0.0 || 
          current_pose.position.y != 0.0 || 
          current_pose.position.z != 0.0) {
        state_received = true;
        RCLCPP_INFO(logger, "Robot state received!");
      } else {
        RCLCPP_WARN(logger, "Zero position received. Waiting...");
        rclcpp::sleep_for(2s);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger, "Error getting current pose: %s", e.what());
      rclcpp::sleep_for(2s);
    }
    attempts++;
  }
  
  if (!state_received) {
    RCLCPP_ERROR(logger, "Failed to get valid robot state after %d attempts. Using default position.", attempts);
    // Set a default pose for simulation if unable to get real robot state
    current_pose.position.x = 0.5;
    current_pose.position.y = 0.0;
    current_pose.position.z = 0.4;
    current_pose.orientation.x = 0.0;
    current_pose.orientation.y = 0.707;
    current_pose.orientation.z = 0.0;
    current_pose.orientation.w = 0.707;
  }
  
  RCLCPP_INFO(logger, "Current position: x=%.3f, y=%.3f, z=%.3f",
              current_pose.position.x, current_pose.position.y, current_pose.position.z);
  
  // Path parameters
  const int num_points = 50;
  const double amplitude = 0.03;  // Reduced amplitude for easier planning
  const double frequency = 2.0;
  const double length = 0.3;  // Shorter path for better success rate
  
  // Generate waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose = current_pose;
  
  // Set fixed orientation (Franka default)
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 0.707;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.707;
  
  // Generate squiggly path
  for (int i = 0; i < num_points; ++i) {
    double t = static_cast<double>(i) / (num_points - 1);
    // Start from a position relative to current position
    target_pose.position.x = current_pose.position.x + t * length;
    target_pose.position.y = current_pose.position.y + amplitude * sin(t * M_PI * 2 * frequency);
    target_pose.position.z = current_pose.position.z;
    waypoints.push_back(target_pose);
    RCLCPP_DEBUG(logger, "Waypoint %d: [%.3f, %.3f, %.3f]",
                i, target_pose.position.x, target_pose.position.y, target_pose.position.z);
  }
  
  // Plan and execute in smaller segments for better success
  const int segment_size = 10;
  const int num_segments = (num_points + segment_size - 1) / segment_size;  // Ceiling division
  
  for (int segment = 0; segment < num_segments; ++segment) {
    RCLCPP_INFO(logger, "Planning segment %d of %d...", segment + 1, num_segments);
    
    // Extract segment waypoints
    std::vector<geometry_msgs::msg::Pose> segment_waypoints;
    int start_idx = segment * segment_size;
    int end_idx = std::min((segment + 1) * segment_size, num_points);
    
    for (int i = start_idx; i < end_idx; ++i) {
      segment_waypoints.push_back(waypoints[i]);
    }
    
    // Cartesian path planning
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double step = 0.01;
    const double jump_threshold = 0.0;
    
    double fraction = move_group->computeCartesianPath(
      segment_waypoints, step, jump_threshold, trajectory);
    
    RCLCPP_INFO(logger, "Planned %.2f%% of segment %d path", fraction * 100.0, segment + 1);
    
    if (fraction > 0.8) {  // Lowered threshold for success
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      RCLCPP_INFO(logger, "Executing segment %d trajectory...", segment + 1);
      
      auto exec_result = move_group->execute(plan);
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "Segment %d executed successfully", segment + 1);
      } else {
        RCLCPP_ERROR(logger, "Failed to execute segment %d", segment + 1);
        break;  // Stop if execution fails
      }
    } else {
      RCLCPP_ERROR(logger, "Planning failed for segment %d (%.1f%% completed)", 
                  segment + 1, fraction * 100.0);
      break;  // Stop if planning fails
    }
    
    // Short pause between segments
    rclcpp::sleep_for(500ms);
  }
  
  RCLCPP_INFO(logger, "Motion sequence completed");
  rclcpp::shutdown();
  return 0;
}