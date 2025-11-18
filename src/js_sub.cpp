#include <memory>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStateSubscriber : public rclcpp::Node
{
public:
    JointStateSubscriber()
    : Node("joint_state_subscriber")
    {
        // Enable fake hardware equivalent
        use_fake_hardware_ = true;

        // Initialize joint names (same order as Python)
        joint_names_ = {
            "J1_left", "J2_left", "J3_left", "J4_left", "J5_left", "J6_left", "L_link7_to_flange",
            "J1_right", "J2_right", "J3_right", "J4_right", "J5_right", "J6_right", "R_link7_to_flange",
            "neck_joint", "head_joint"
        };

        joint_state_actual_.resize(joint_names_.size(), std::numeric_limits<double>::quiet_NaN());

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&JointStateSubscriber::jsCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "JointState subscriber started.");
    }

private:

    void jsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!use_fake_hardware_) return;

        // Build map: name → position  (same as Python dict(zip()))
        std::unordered_map<std::string, double> name_to_position;
        for (size_t i = 0; i < msg->name.size(); i++) {
            name_to_position[msg->name[i]] = msg->position[i];
        }

        // Fill joint_state_actual_ in correct order
        for (size_t i = 0; i < joint_names_.size(); i++) {
            auto it = name_to_position.find(joint_names_[i]);
            if (it != name_to_position.end()) {
                joint_state_actual_[i] = it->second;
            } else {
                joint_state_actual_[i] = std::numeric_limits<double>::quiet_NaN();
            }
        }

        joint_callback_status_ = true;

        // (Optional debug print)
        RCLCPP_INFO(this->get_logger(), "Updated joint_state_actual");
    }

    // --- Member Variables ---

    bool use_fake_hardware_{false};
    bool joint_callback_status_{false};

    std::vector<std::string> joint_names_;
    std::vector<double> joint_state_actual_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateSubscriber>());
    rclcpp::shutdown();
    return 0;
}
