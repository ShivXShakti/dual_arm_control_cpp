#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <iostream>
#include <fstream>
#include <string>

int main() {
    // Load URDF from file
    std::ifstream urdf_file("/home/scg/Documents/surya_ws/src/robot_description/urdf/dual_arm.urdf");
    std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                             std::istreambuf_iterator<char>());

    urdf::Model model;
    if (!model.initString(urdf_string)) {
        std::cerr << "Failed to parse URDF" << std::endl;
        return -1;
    }

    // Parse URDF into KDL tree
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
        std::cerr << "Failed to convert URDF to KDL tree" << std::endl;
        return -1;
    }

    // Extract chain (replace with your actual link names)
    KDL::Chain chain;
    if (!tree.getChain("torso", "R_delto_base_link", chain)) {
        std::cerr << "Failed to extract chain" << std::endl;
        return -1;
    }

    // Gravity vector
    KDL::Vector gravity(0.0, 0.0, -9.81);

    // Dynamics solver
    KDL::ChainDynParam dyn_param(chain, gravity);

    // Example joint positions
    KDL::JntArray q(chain.getNrOfJoints());
    for (unsigned int i = 0; i < chain.getNrOfJoints(); i++) {
        q(i) = 0.0; // radians
    }

    // Compute gravity torques
    KDL::JntArray G(chain.getNrOfJoints());
    dyn_param.JntToGravity(q, G);

    std::cout << "Gravity compensation torques:" << std::endl;
    for (unsigned int i = 0; i < chain.getNrOfJoints(); i++) {
        std::cout << "Joint " << i+1 << ": " << G(i) << " Nm" << std::endl;
    }

    return 0;
}
