// src/franka_ft_data_adapter.cpp
#include <memory> // Za std::make_shared
#include <string> // Za std::string

#include "rclcpp/rclcpp.hpp"
#include "franka_msgs/msg/franka_state.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

class FrankaFTDataAdapter : public rclcpp::Node
{
public:
    FrankaFTDataAdapter() : Node("franka_ft_data_adapter") // Ime vozlišča
    {
        // Parameter za ime teme vhodnih FrankaState sporočil
        this->declare_parameter<std::string>("franka_state_topic", "/franka_state_controller/franka_states");
        std::string franka_state_topic = this->get_parameter("franka_state_topic").as_string();

        // Parameter za ime teme izhodnih WrenchStamped sporočil
        this->declare_parameter<std::string>("output_wrench_topic", "/cartesian_compliance_controller/ft_sensor_wrench");
        std::string output_wrench_topic = this->get_parameter("output_wrench_topic").as_string();

        // Parameter za ime osnovnega koordinatnega sistema robota
        // To mora biti isti frame_id, kot ga pričakuje CartesianComplianceController
        // in v katerem so sile O_F_ext_hat_K dejansko podane.
        this->declare_parameter<std::string>("robot_base_frame_id", "fr3_link0");
        robot_base_frame_id_ = this->get_parameter("robot_base_frame_id").as_string();

        // Kakovost storitve (QoS) - uporaba sistemskih privzetih nastavitev je običajno v redu za stanje robota
        auto qos = rclcpp::SystemDefaultsQoS();

        franka_state_subscriber_ = this->create_subscription<franka_msgs::msg::FrankaState>(
            franka_state_topic,
            qos, // Uporaba QoS profila
            std::bind(&FrankaFTDataAdapter::frankaStateCallback, this, std::placeholders::_1));

        wrench_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
            output_wrench_topic,
            qos); // Uporaba QoS profila

        RCLCPP_INFO(this->get_logger(), "FrankaFTDataAdapter se je zagnal.");
        RCLCPP_INFO(this->get_logger(), "Poslušam na temi: %s", franka_state_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Objavljam na temi: %s", output_wrench_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Uporabljen frame_id za objavljeno silo: %s", robot_base_frame_id_.c_str());
    }

private:
    void frankaStateCallback(const franka_msgs::msg::FrankaState::SharedPtr msg)
    {
        // Preverimo, ali so podatki o silah veljavni (niso NaN)
        // Polje O_F_ext_hat_K je array tipa double[6]
        for (int i = 0; i < 6; ++i) {
            if (std::isnan(msg->o_f_ext_hat_k[i])) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, // Opozori največ enkrat na 5 sekund
                                     "Prejeti NaN v podatkih o zunanji sili (O_F_ext_hat_K). Sporočilo se ne objavlja.");
                return;
            }
        }

        auto wrench_msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();

        // Nastavi časovno oznako sporočila. Uporaba this->now() je boljša praksa,
        // če ni stroge zahteve po času iz senzorja in če so ure sinhronizirane.
        wrench_msg->header.stamp = this->now();
        wrench_msg->header.frame_id = robot_base_frame_id_;

        // Sila: O_F_ext_hat_K[0], O_F_ext_hat_K[1], O_F_ext_hat_K[2]
        wrench_msg->wrench.force.x = msg->o_f_ext_hat_k[0];
        wrench_msg->wrench.force.y = msg->o_f_ext_hat_k[1];
        wrench_msg->wrench.force.z = msg->o_f_ext_hat_k[2];

        // Navor: O_F_ext_hat_K[3], O_F_ext_hat_K[4], O_F_ext_hat_K[5]
        wrench_msg->wrench.torque.x = msg->o_f_ext_hat_k[3];
        wrench_msg->wrench.torque.y = msg->o_f_ext_hat_k[4];
        wrench_msg->wrench.torque.z = msg->o_f_ext_hat_k[5];

        wrench_publisher_->publish(std::move(wrench_msg));
    }

    rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr franka_state_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_publisher_;
    std::string robot_base_frame_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaFTDataAdapter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}