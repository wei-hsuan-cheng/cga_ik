#include "rclcpp/rclcpp.hpp"
#include "cga_ik/cga_utils.hpp"
#include "cga_ik_spherical_robot/cga_ik_spherical_robot.hpp"
#include "robot_math_utils/robot_math_utils_v1_9.hpp"
#include <iostream>

using RM = RMUtils;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cga_spherical_robot_validate");
    std::cout << "\n----- Starting CGA spherical robot validation -----\n" << std::endl;

    rclcpp::shutdown();
    return 0;
}