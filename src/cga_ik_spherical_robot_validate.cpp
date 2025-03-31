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

    // Example usage: pick some angles to test
    float theta0 = 0.0f;
    float theta1 = 0.0f;
    float theta2 = 0.0f;

    // Robot geometry
    float r_b = 0.5f;
    float r_e = 0.4f;

    // Compute the IK:
    auto result = cga_ik_spherical_robot::computeSphericalRobotIK(theta0, theta1, theta2, r_b, r_e);

    // Print out the angles
    auto rad2deg = [](float r){ return r * 180.0f / float(M_PI); };
    std::cout << "\nMotor 0 angle = " << result.angle0 << " rad ("
              << rad2deg(result.angle0) << " deg)";
    std::cout << "\nMotor 1 angle = " << result.angle1 << " rad ("
              << rad2deg(result.angle1) << " deg)";
    std::cout << "\nMotor 2 angle = " << result.angle2 << " rad ("
              << rad2deg(result.angle2) << " deg)\n";

    // For debugging, we can log some of the CGA objects:
    std::cout << "\nElbow positions:\n";
    // std::cout << "elb0: "; result.elb0.log();
    // std::cout << "elb1: "; result.elb1.log();
    // std::cout << "elb2: "; result.elb2.log();

    std::cout << "\nEnd point:\n";
    result.endpoint.log();

    rclcpp::shutdown();
    return 0;
}