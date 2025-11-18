#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class DesiredJointStatePublisher : public rclcpp::Node
{
public:
    DesiredJointStatePublisher()
    : Node("desired_joint_state_publisher")
    {
        // Create publisher
        desired_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "desired/joint_states",
            10
        );

        // Create a timer at 100 Hz (10 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),   // 100 Hz
            std::bind(&DesiredJointStatePublisher::timerCallback, this)
        );

        // Initialize some dummy desired joint state values (example)
        desired_values_.resize(16, 0.0);

        RCLCPP_INFO(this->get_logger(), "Desired Joint State Publisher running at 100 Hz");
    }

private:
    void timerCallback()
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = desired_values_;

        desired_pub_->publish(msg);

        // Optional debug
        // RCLCPP_INFO(this->get_logger(), "Published desired joint state");
    }

    // Member variables
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr desired_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> desired_values_;   // Your desired joint states
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DesiredJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
