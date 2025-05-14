#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class CustomPublisher : public rclcpp::Node
{
public:
  CustomPublisher()
  : Node("custom_publisher")
  {
    force_axis_pub_ = this->create_publisher<std_msgs::msg::Int32>("/hybrid_controller/force_axis", 10);
    force_target_pub_ = this->create_publisher<std_msgs::msg::Float32>("/hybrid_controller/force_target", 10);
    pose_target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/hybrid_controller/pose_target", 10);

    this->declare_parameter<double>("initial_delay", 5.0);
    this->declare_parameter<double>("publish_interval", 5.0);

    double initial_delay = this->get_parameter("initial_delay").as_double();
    publish_interval_ = this->get_parameter("publish_interval").as_double();

    RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds before starting to publish messages", initial_delay);
    std::this_thread::sleep_for(std::chrono::duration<double>(initial_delay));

    check_system_status();

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(publish_interval_),
      std::bind(&CustomPublisher::publish_sequence, this)
    );

    RCLCPP_INFO(this->get_logger(), "Custom publisher initialized and ready to publish");
  }

private:
  void check_system_status()
  {
    // Note: rclcpp does not provide a direct API to list all topics synchronously at startup.
    // For a real check, you could use rclcpp::Node::get_topic_names_and_types(), but it may not be up-to-date at startup.
    // Here, we just log a warning as in your Python code.
    std::vector<std::string> required_topics = {
      "/franka_state_controller/franka_states",
      "/joint_states",
      "/fr3_arm_controller/state"
    };
    // For simplicity, just log that we are not checking topics in C++
    RCLCPP_WARN(this->get_logger(), "Topic existence check not implemented in C++ version. Will publish anyway.");
  }

  void publish_sequence()
  {
    publish_force_axis();
    std::this_thread::sleep_for(0.5s);
    publish_force_target();
    std::this_thread::sleep_for(0.5s);
    publish_pose_target();
  }

  void publish_force_axis()
  {
    auto msg = std_msgs::msg::Int32();
    msg.data = 2;
    force_axis_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published force_axis: %d", msg.data);
  }

  void publish_force_target()
  {
    auto msg = std_msgs::msg::Float32();
    msg.data = 2.0;
    force_target_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published force_target: %.2f", msg.data);
  }

  void publish_pose_target()
  {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.frame_id = "fr3_link0";
    msg.header.stamp = this->now();

    msg.pose.position.x = 0.311;
    msg.pose.position.y = 0.020;
    msg.pose.position.z = 0.640;

    msg.pose.orientation.x = 0.929;
    msg.pose.orientation.y = -0.368;
    msg.pose.orientation.z = -0.020;
    msg.pose.orientation.w = 0.024;

    pose_target_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published pose_target");
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr force_axis_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr force_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double publish_interval_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomPublisher>());
  rclcpp::shutdown();
  return 0;
}