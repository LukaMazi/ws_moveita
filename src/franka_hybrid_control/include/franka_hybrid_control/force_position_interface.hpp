#ifndef FRANKA_HYBRID_CONTROL__FORCE_POSITION_INTERFACE_HPP_
#define FRANKA_HYBRID_CONTROL__FORCE_POSITION_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "franka_hybrid_control/franka_hybrid_controller.hpp"

namespace franka_hybrid_control
{

/**
 * @brief Interface node for hybrid force-position control
 * 
 * This node provides a user-friendly interface for controlling
 * the Franka robot using hybrid force-position control.
 */
class ForcePositionInterface : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for ForcePositionInterface
   */
  ForcePositionInterface();

  std::shared_ptr<FrankaHybridController> get_controller() const {
    return controller_;
  }
  

private:
  // Callbacks
  void force_axis_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void force_target_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void pose_target_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  // Execute hybrid control
  void execute_control();
  
  // Force-position controller
  std::shared_ptr<FrankaHybridController> controller_;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr force_axis_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr force_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_sub_;
  
  // Control parameters
  int force_axis_;
  double target_force_;
  geometry_msgs::msg::PoseStamped target_pose_;
  
  // Control flag
  bool params_updated_;
  
  // Timer for execution
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace franka_hybrid_control

#endif // FRANKA_HYBRID_CONTROL__FORCE_POSITION_INTERFACE_HPP_