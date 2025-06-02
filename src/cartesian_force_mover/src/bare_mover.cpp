#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

class MinimalMover : public rclcpp::Node {
public:
  MinimalMover() : Node("bare_mover"), x_(0.4) {
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_frame", 10);
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("target_wrench", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&MinimalMover::publish_targets, this));
  }

private:
  void publish_targets() {
    auto now = this->get_clock()->now();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = x_;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.3;
    pose.pose.orientation.w = 1.0;
    pose_pub_->publish(pose);

    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = now;
    wrench.header.frame_id = "tool0";
    wrench.wrench.force.z = -20.0;
    wrench_pub_->publish(wrench);

    x_ += 0.001;
    RCLCPP_INFO(this->get_logger(), "Sending pose x=%.3f z=%.3f with Fz=-5.0", x_, pose.pose.position.z);
  }

  double x_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalMover>());
  rclcpp::shutdown();
  return 0;
}