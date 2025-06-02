#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

class CartesianDemo : public rclcpp::Node
{
public:
    CartesianDemo() : Node("simple_publisher"), x_(0.3)
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_frame", 10);
        force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/target_wrench", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CartesianDemo::publish, this));
    }

private:
    void publish()
    {
        auto now = this->get_clock()->now();
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.header.stamp = now;
        pose.pose.position.x = x_;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.3;
        pose.pose.orientation.w = 1.0;
        pose_pub_->publish(pose);
        
        geometry_msgs::msg::WrenchStamped wrench;
        wrench.header.stamp = now;
        wrench.header.frame_id = "base_link";  // Changed to match pose frame
        wrench.wrench.force.z = -5.0;
        force_pub_->publish(wrench);
        

    }
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianDemo>());
  rclcpp::shutdown();
  return 0;
}
