

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

  // Create MoveIt components
  auto move_group = moveit::planning_interface::MoveGroupInterface(node, "fr3_arm");
  auto planning_scene_interface = moveit::planning_interface::PlanningSceneInterface();

  // Configure MoveIt
  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // Get current pose
  auto current_pose = move_group.getCurrentPose().pose;
  RCLCPP_INFO(logger, "Current position: x=%.3f, y=%.3f, z=%.3f", 
              current_pose.position.x, current_pose.position.y, current_pose.position.z);

  // Path parameters
  const int num_points = 50;
  const double amplitude = 0.05;
  const double frequency = 2.0;
  const double length = 0.4;

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
    
    target_pose.position.x = 0.5 + t * length;
    target_pose.position.y = 0.0 + amplitude * sin(t * M_PI * 2 * frequency);
    target_pose.position.z = 0.4;

    waypoints.push_back(target_pose);
    RCLCPP_DEBUG(logger, "Waypoint %d: [%.3f, %.3f, %.3f]", 
                 i, target_pose.position.x, target_pose.position.y, target_pose.position.z);
  }

  // Cartesian path planning
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double step = 0.01;
  const double jump_threshold = 0.0;
  
  double fraction = move_group.computeCartesianPath(
    waypoints, step, jump_threshold, trajectory);

  RCLCPP_INFO(logger, "Planned %.2f%% of the path", fraction * 100.0);

  if (fraction > 0.9) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    
    RCLCPP_INFO(logger, "Executing trajectory...");
    move_group.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed (%.1f%% completed)", fraction * 100.0);
  }

  rclcpp::shutdown();
  return 0;
}