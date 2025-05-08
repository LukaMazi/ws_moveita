#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("simple_franka_mover");
  auto const logger = node->get_logger();

  // Create the Move Group Interface
  static const std::string PLANNING_GROUP = "fr3_arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // Get the name of the end-effector link
  const std::string end_effector_link = move_group.getEndEffectorLink();
  RCLCPP_INFO(logger, "End effector link: %s", end_effector_link.c_str());

  // Plan to a Cartesian Pose goal
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 0.707;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.707;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.5;

  RCLCPP_INFO(logger, "Setting target pose...");
  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(logger, "Planning successful");
    RCLCPP_INFO(logger, "Moving to target pose...");
    move_group.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed");
  }

  // Second target pose
  geometry_msgs::msg::Pose target_pose2;
  target_pose2.orientation.x = 0.0;
  target_pose2.orientation.y = 0.707;
  target_pose2.orientation.z = 0.0;
  target_pose2.orientation.w = 0.707;
  target_pose2.position.x = 0.3;
  target_pose2.position.y = 0.2;
  target_pose2.position.z = 0.4;

  RCLCPP_INFO(logger, "Setting second target pose...");
  move_group.setPoseTarget(target_pose2);

  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  success = (move_group.plan(plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    RCLCPP_INFO(logger, "Planning for second pose successful");
    RCLCPP_INFO(logger, "Moving to second target pose...");
    move_group.execute(plan2);
  } else {
    RCLCPP_ERROR(logger, "Planning for second pose failed");
  }

  rclcpp::shutdown();
  return 0;
}