#ifndef FRANKA_HYBRID_CONTROL__FRANKA_HYBRID_CONTROLLER_HPP_
#define FRANKA_HYBRID_CONTROL__FRANKA_HYBRID_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <franka_msgs/franka_msgs/msg/franka_robot_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace franka_hybrid_control
{

/**
 * @brief Hybrid force-position controller for Franka FR3 robot
 * 
 * This class implements a controller that can perform position control
 * on two Cartesian axes while performing force control on the third axis.
 */
class FrankaHybridController : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for FrankaHybridController
   */
  FrankaHybridController();

  /**
   * @brief Execute the hybrid force-position control
   * 
   * @param force_axis The axis to control with force (0=X, 1=Y, 2=Z)
   * @param target_force The desired force in Newtons
   * @param target_pose The target pose for position-controlled axes
   * @return true if execution was successful
   */
  bool execute(int force_axis, 
               double target_force, 
               const geometry_msgs::msg::PoseStamped& target_pose);

private:
  // Callbacks
  void franka_state_callback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  // Control methods
  void control_loop();
  bool initialize_connection();
  void update_force_control();
  
  // MoveIt interface
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  
  // Subscribers
  rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr franka_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_error_pub_;
  
  // Control parameters
  int force_axis_;  // 0=X, 1=Y, 2=Z
  double target_force_;
  double force_kp_;  // Proportional gain for force control
  double force_ki_;  // Integral gain for force control
  double force_kd_;  // Derivative gain for force control
  double force_error_integral_;
  double prev_force_error_;
  
  // Current state
  franka_msgs::msg::FrankaRobotState::SharedPtr current_franka_state_;
  sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
  geometry_msgs::msg::PoseStamped target_pose_;
  geometry_msgs::msg::PoseStamped current_pose_;
  std::array<double, 6> current_wrench_;
  
  // Thread for control loop
  std::thread control_thread_;
  bool stop_thread_;
  
  // Control flags
  bool connected_to_robot_;
  bool executing_;
  
  // TF listener for transforming forces
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace franka_hybrid_control

#endif // FRANKA_HYBRID_CONTROL__FRANKA_HYBRID_CONTROLLER_HPP_