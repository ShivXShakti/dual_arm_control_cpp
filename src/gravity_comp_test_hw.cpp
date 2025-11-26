#include <memory>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>

#include <darm_msgs/msg/ui_command.hpp>
#include <darm_msgs/msg/ui_status.hpp>

class JointStateSubscriber : public rclcpp::Node
{
public:
    JointStateSubscriber()
    : Node("joint_state_subscriber")
    {
        pub = this->create_publisher<darm_msgs::msg::UiCommand>("/svaya/ui/command", 10);
        sub = this->create_subscription<darm_msgs::msg::UiStatus>("/svaya/ui/status", 10, 
            std::bind(&JointStateSubscriber::callback, this, std::placeholders::_1)); 
    
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),   // 1000 Hz
            std::bind(&JointStateSubscriber::timerCallback, this)
        );
        RCLCPP_INFO(this->get_logger(), "JointState subscriber started.");
    }
private:
    void timerCallback()
    {
        if (joint_callback_status_){
            std::ifstream urdf_file("/home/cstar/Documents/dual_arm_ws/src/robot_description/urdf/darm_with_gripper.urdf");
            std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                                    std::istreambuf_iterator<char>());
            urdf::Model model;
            if (!model.initString(urdf_string)) {
                std::cerr << "Failed to parse URDF" << std::endl;
            }
            KDL::Tree tree;
            if (!kdl_parser::treeFromUrdfModel(model, tree)) {
                std::cerr << "Failed to convert URDF to KDL tree" << std::endl;
            }
            //    ACTUAL JOINT STATES 
            Eigen::VectorXd current_pos(16);
            Eigen::VectorXd target_pos(16);
            Eigen::VectorXd target_torq(16);
            target_torq.setZero();
            auto msg = darm_msgs::msg::UiCommand();
            msg.developer_command.enable = true;
            msg.developer_command.command.resize(16);
            for (int i = 0; i < 7; i++){
                current_pos[i] = joint_msg->left_arm.position[i];
                current_pos[i + 7] = joint_msg->right_arm.position[i];
                if (i < 2)
                {
                    current_pos[i + 14] = joint_msg->head.position[i];
                }
            }

            //      ======== LEFT HAND ============
            KDL::Chain chain_l;
            if (!tree.getChain("torso", "L_delto_base_link", chain_l)) {
                std::cerr << "Failed to extract chain" << std::endl;
            }
            KDL::Vector gravity(0.0, 0.0, -9.81);

            KDL::ChainDynParam dyn_paraml(chain_l, gravity);
            KDL::JntArray ql(chain_l.getNrOfJoints());
            for (unsigned int i = 0; i < chain_l.getNrOfJoints(); i++) {
                ql(i) = current_pos[i]; // radians
            }
            KDL::JntArray Gl(chain_l.getNrOfJoints());
            dyn_paraml.JntToGravity(ql, Gl);

            //              GRIPPER LEFT
            std::vector<double> joint_state_actual_lGF(11);
            for (unsigned int i = 0; i < chain_l.getNrOfJoints(); i++) {
                joint_state_actual_lGF[i] = current_pos[i]; 
            }
            
            KDL::Chain chain_lGF1;
            if (!tree.getChain("torso", "L_F1_04", chain_lGF1)) {
                std::cerr << "Failed to extract chain" << std::endl;
            }
            KDL::ChainDynParam dyn_paramlGF1(chain_lGF1, gravity);
            KDL::JntArray qlGF1(chain_lGF1.getNrOfJoints());
            for (unsigned int i = 0; i < chain_lGF1.getNrOfJoints(); i++) {
                qlGF1(i) = joint_state_actual_lGF[i]; 
            }
            KDL::JntArray GlGF1(chain_lGF1.getNrOfJoints());
            dyn_paramlGF1.JntToGravity(qlGF1, GlGF1);

            KDL::Chain chain_lGF2;
            if (!tree.getChain("torso", "L_F2_04", chain_lGF2)) {
                std::cerr << "Failed to extract chain" << std::endl;
            }
            KDL::ChainDynParam dyn_paramlGF2(chain_lGF2, gravity);
            KDL::JntArray qlGF2(chain_lGF2.getNrOfJoints());
            for (unsigned int i = 0; i < chain_lGF2.getNrOfJoints(); i++) {
                qlGF2(i) = joint_state_actual_lGF[i]; 
            }
            KDL::JntArray GlGF2(chain_lGF2.getNrOfJoints());
            dyn_paramlGF2.JntToGravity(qlGF2, GlGF2);

            KDL::Chain chain_lGF3;
            if (!tree.getChain("torso", "L_F3_04", chain_lGF3)) {
                std::cerr << "Failed to extract chain" << std::endl;
            }
            KDL::ChainDynParam dyn_paramlGF3(chain_lGF3, gravity);
            KDL::JntArray qlGF3(chain_lGF3.getNrOfJoints());
            for (unsigned int i = 0; i < chain_lGF3.getNrOfJoints(); i++) {
                qlGF3(i) = joint_state_actual_lGF[i]; // radians
            }
            KDL::JntArray GlGF3(chain_lGF3.getNrOfJoints());
            dyn_paramlGF3.JntToGravity(qlGF3, GlGF3);
           

            std::vector<double> final_torque_l = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            for (unsigned int i = 0; i <chain_l.getNrOfJoints(); i++) {
                final_torque_l[i] += GlGF1(i) + GlGF2(i) + GlGF3(i) - 3*Gl(i);
                final_torque_l[i] = final_torque_l[i] + Gl(i);
            }

            //          ========== RIGHT HAND ===========
            KDL::Chain chain_r;
            if (!tree.getChain("torso", "R_delto_base_link", chain_r)) {
                std::cerr << "Failed to extract chain" << std::endl;
            }
            KDL::ChainDynParam dyn_paramr(chain_r, gravity);
            KDL::JntArray qr(chain_r.getNrOfJoints());
            for (unsigned int i = 0; i < chain_r.getNrOfJoints(); i++) {
                qr(i) = current_pos[i+7]; // radians
            }
            KDL::JntArray Gr(chain_r.getNrOfJoints());
            dyn_paramr.JntToGravity(qr, Gr);

            //              GRIPPER right
            std::vector<double> joint_state_actual_rGF(11);
            for (unsigned int i = 0; i < chain_r.getNrOfJoints(); i++) {
                joint_state_actual_rGF[i] = 0.0;//current_pos[i+7]; 
            }
            
            KDL::Chain chain_rGF1;
            if (!tree.getChain("torso", "R_F1_04", chain_rGF1)) {
                std::cerr << "Failed to extract chain" << std::endl;
            }
            KDL::ChainDynParam dyn_paramrGF1(chain_rGF1, gravity);
            KDL::JntArray qrGF1(chain_rGF1.getNrOfJoints());
            for (unsigned int i = 0; i < chain_rGF1.getNrOfJoints(); i++) {
                qrGF1(i) = joint_state_actual_rGF[i]; 
            }
            KDL::JntArray GrGF1(chain_rGF1.getNrOfJoints());
            dyn_paramrGF1.JntToGravity(qrGF1, GrGF1);

            KDL::Chain chain_rGF2;
            if (!tree.getChain("torso", "R_F2_04", chain_rGF2)) {
                std::cerr << "Failed to extract chain" << std::endl;
            }
            KDL::ChainDynParam dyn_paramrGF2(chain_rGF2, gravity);
            KDL::JntArray qrGF2(chain_rGF2.getNrOfJoints());
            for (unsigned int i = 0; i < chain_rGF2.getNrOfJoints(); i++) {
                qrGF2(i) = joint_state_actual_rGF[i]; 
            }
            KDL::JntArray GrGF2(chain_rGF2.getNrOfJoints());
            dyn_paramrGF2.JntToGravity(qrGF2, GrGF2);

            KDL::Chain chain_rGF3;
            if (!tree.getChain("torso", "R_F3_04", chain_rGF3)) {
                std::cerr << "Failed to extract chain" << std::endl;
            }
            KDL::ChainDynParam dyn_paramrGF3(chain_rGF3, gravity);
            KDL::JntArray qrGF3(chain_rGF3.getNrOfJoints());
            for (unsigned int i = 0; i < chain_rGF3.getNrOfJoints(); i++) {
                qrGF3(i) = joint_state_actual_rGF[i]; // radians
            }
            KDL::JntArray GrGF3(chain_rGF3.getNrOfJoints());
            dyn_paramrGF3.JntToGravity(qrGF3, GrGF3);
           

            std::vector<double> final_torque_r = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            for (unsigned int i = 0; i <chain_r.getNrOfJoints(); i++) {
                final_torque_r[i] += GrGF1(i) + GrGF2(i) + GrGF3(i) - 3*Gr(i);
                final_torque_r[i] = final_torque_r[i] + Gr(i);
            }

            
            //std::vector<double> desired_tau {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            for (unsigned int i = 0; i < chain_l.getNrOfJoints(); i++) {
                target_torq[i] = final_torque_l[i]; 
                target_torq[i+7] = final_torque_r[i]; 
                if (i<2){
                    target_torq[i+14] = 0.0;
                }
            }
            
            //   ========= TARGET TORQUE ==========
            for (size_t i = 0; i < 16; i++){
                msg.developer_command.command[i].torque = target_torq[i];
                msg.developer_command.command[i].position = current_pos[i];
            }
            pub->publish(msg);

            for (unsigned int i = 0; i <target_torq.size(); i++) {
                std::cout << "target_torq " << i+1 << ": " << target_torq[i] << " Nm" << std::endl;
            }
        }
            
    }


    void callback(darm_msgs::msg::UiStatus::SharedPtr msg){
        joint_msg = msg;
        joint_callback_status_ = true;
    }

    // --- Member Variables ---
    bool joint_callback_status_{false};
    std::vector<double> joint_state_actual_GF;
    std::vector<double> final_torque_l;
    std::vector<double> final_torque_r;
    std::string urdf_string;
    rclcpp::TimerBase::SharedPtr timer_;

    darm_msgs::msg::UiStatus::SharedPtr joint_msg;
    Eigen::VectorXd current_pos;
    Eigen::VectorXd target_pos;
    Eigen::VectorXd target_torq;

    rclcpp::Subscription<darm_msgs::msg::UiStatus>::SharedPtr sub;
    rclcpp::Publisher<darm_msgs::msg::UiCommand>::SharedPtr pub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateSubscriber>());
    rclcpp::shutdown();
    return 0;
}
