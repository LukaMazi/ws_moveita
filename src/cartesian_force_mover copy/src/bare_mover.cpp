#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

class MinimalMover : public rclcpp::Node {
public:
  MinimalMover() : Node("bare_mover"), x_(0.4), commanded_force_z_(-20.0) {
    // Publishers for commands
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_frame", 10);
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("target_wrench", 10);

    // Subscriber for force feedback
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "force_torque_sensor_broadcaster/wrench", 10,
      std::bind(&MinimalMover::wrench_feedback_callback, this, std::placeholders::_1));

    // Timer for publishing commands
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&MinimalMover::publish_targets, this));

    // Initialize measured forces
    measured_force_x_ = 0.0;
    measured_force_y_ = 0.0;
    measured_force_z_ = 0.0;
    measured_torque_x_ = 0.0;
    measured_torque_y_ = 0.0;
    measured_torque_z_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "Force Feedback Mover initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: force_torque_sensor_broadcaster/wrench");
    RCLCPP_INFO(this->get_logger(), "Publishing to: target_frame, target_wrench");
  }

private:
  void wrench_feedback_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    // Store the measured forces and torques
    measured_force_x_ = msg->wrench.force.x;
    measured_force_y_ = msg->wrench.force.y;
    measured_force_z_ = msg->wrench.force.z;
    measured_torque_x_ = msg->wrench.torque.x;
    measured_torque_y_ = msg->wrench.torque.y;
    measured_torque_z_ = msg->wrench.torque.z;
  }

  void publish_targets() {
    auto now = this->get_clock()->now();

    // Publish pose command
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = x_;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.3;
    pose.pose.orientation.w = 1.0;
    pose_pub_->publish(pose);

    // Publish force command
    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = now;
    wrench.header.frame_id = "tool0";
    wrench.wrench.force.z = commanded_force_z_;
    wrench_pub_->publish(wrench);

    // Move slowly in X direction
    x_ += 0.001;

    // Log both commanded and measured forces
    RCLCPP_INFO(this->get_logger(), 
      "COMMANDED - Pose: x=%.3f, z=%.3f | Force: Fz=%.1f", 
      x_, pose.pose.position.z, commanded_force_z_);
    
    RCLCPP_INFO(this->get_logger(), 
      "MEASURED  - Forces: Fx=%.2f, Fy=%.2f, Fz=%.2f | Torques: Tx=%.2f, Ty=%.2f, Tz=%.2f", 
      measured_force_x_, measured_force_y_, measured_force_z_,
      measured_torque_x_, measured_torque_y_, measured_torque_z_);

    // Calculate force error for Z-axis (most important for your application)
    double force_error_z = commanded_force_z_ - measured_force_z_;
    RCLCPP_INFO(this->get_logger(), 
      "FORCE ERROR - Z-axis: %.2f N (commanded: %.1f, measured: %.2f)", 
      force_error_z, commanded_force_z_, measured_force_z_);
    
    RCLCPP_INFO(this->get_logger(), "---");
  }

  // Position and force commands
  double x_;
  double commanded_force_z_;
  
  // Measured force/torque values
  double measured_force_x_, measured_force_y_, measured_force_z_;
  double measured_torque_x_, measured_torque_y_, measured_torque_z_;

  // ROS2 components
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalMover>());
  rclcpp::shutdown();
  return 0;
}