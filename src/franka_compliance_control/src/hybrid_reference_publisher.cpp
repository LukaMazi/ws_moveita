#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class HybridReferencePublisher : public rclcpp::Node
{
public:
    HybridReferencePublisher() : Node("hybrid_reference_publisher")
    {
        // Publishers za cartesian_compliance_controller
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cartesian_compliance_controller/reference_pose", 10);
            
        force_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/cartesian_compliance_controller/target_wrench", 10);
            
        // Možnost za dinamično spreminjanje stiffness
        stiffness_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/cartesian_compliance_controller/stiffness", 10);
        
        // Timer za periodično pošiljanje
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HybridReferencePublisher::publish_references, this));
            
        RCLCPP_INFO(this->get_logger(), "Hybrid Reference Publisher initialized");
        
        // Nastavi začetne vrednosti
        setup_initial_targets();
    }

private:
    void setup_initial_targets()
    {
        // Pozicijska referenca (X, Y kontrolirana, Z bo kontroliran preko sile)
        target_pose_.header.frame_id = "fr3_link0";
        target_pose_.pose.position.x = 0.311;  // Želena X pozicija
        target_pose_.pose.position.y = 0.020;  // Želena Y pozicija
        target_pose_.pose.position.z = 0.640;  // Z se bo prilagajal glede na silo
        
        // Orientacija
        target_pose_.pose.orientation.w = 1.0;
        target_pose_.pose.orientation.x = 0.0;
        target_pose_.pose.orientation.y = 0.0;
        target_pose_.pose.orientation.z = 0.0;
        
        // Ciljna sila (samo Z komponenta)
        target_wrench_.header.frame_id = "fr3_hand_tcp";
        target_wrench_.wrench.force.x = 0.0;
        target_wrench_.wrench.force.y = 0.0;
        target_wrench_.wrench.force.z = -5.0;  // 5N navzdol
        target_wrench_.wrench.torque.x = 0.0;
        target_wrench_.wrench.torque.y = 0.0;
        target_wrench_.wrench.torque.z = 0.0;
    }
    
    void publish_references()
    {
        auto now = this->get_clock()->now();
        
        // Posodobi časovne oznake
        target_pose_.header.stamp = now;
        target_wrench_.header.stamp = now;
        
        // Objavi reference
        pose_pub_->publish(target_pose_);
        force_pub_->publish(target_wrench_);
        
        // Opcijsko: dinamično spreminjanje stiffness
        // publish_dynamic_stiffness();
    }
    
    void publish_dynamic_stiffness()
    {
        // Če želite dinamično spreminjati, katera os je force/position controlled
        std_msgs::msg::Float64MultiArray stiffness_msg;
        stiffness_msg.data = {
            1500.0,  // trans_x - visoka stiffness (pozicijska kontrola)
            1500.0,  // trans_y - visoka stiffness (pozicijska kontrola)
            10.0,    // trans_z - nizka stiffness (sila kontrola)
            300.0,   // rot_x
            300.0,   // rot_y
            300.0    // rot_z
        };
        stiffness_pub_->publish(stiffness_msg);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr stiffness_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::PoseStamped target_pose_;
    geometry_msgs::msg::WrenchStamped target_wrench_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HybridReferencePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}