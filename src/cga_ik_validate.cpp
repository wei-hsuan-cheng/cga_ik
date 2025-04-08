#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "cga/cga_utils.hpp"
#include "cga_ik_cobot_6dof/cga_ik_cobot_6dof.hpp"
#include "robot_math_utils/robot_math_utils_v1_9.hpp"

using RM = RMUtils;

int main(int argc, char ** argv) {
    // Initialize ROS 2.
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cga_ik_validate");
    std::cout << "\n----- Starting CGA IK validation -----\n" << std::endl;

    // Create the DH table for the 6-DoF arm.
    DHTable dh_table = cga_ik::loadDHTable();

    // Print the entire D-H table
    std::cout << "\n----- D-H table (alpha_{i-1}, a_{i-1}, d_{i}, theta_{i}) -----" << std::endl;
    std::cout << dh_table << std::endl;

    double d1 = dh_table.dh_table(0, 2);
    double a2 = dh_table.dh_table(2, 1);
    double a3 = dh_table.dh_table(3, 1);
    double d4 = dh_table.dh_table(3, 2);
    double d5 = dh_table.dh_table(4, 2);
    double d6 = dh_table.dh_table(5, 2);

    Vector6d target_pose(a3 + d5, d4, d1 + a2 - d6, M_PI, M_PI / 4.0, M_PI);
    // Vector6d target_pose(a3 + d5, d4 * 100.0, d1 + a2 - d6, M_PI, M_PI / 4.0, M_PI);
    // Vector6d target_pose(1.0, 0.0, 0.0, 0.0, 0.0, M_PI / 2);

    Vector3d pos_offset(0.0, 0.0, 0.0);
    double f_motion = 0.001;
    double t_ = 35.0;
    Vector6d pose_offset = RM::PosQuat2R6Pose( PosQuat(pos_offset, 
                                                       RM::Quatz(0.5 * M_PI* sin(2.0 * M_PI * f_motion * t_))
                                                      ) 
                                              );

    target_pose = RM::TransformR6Poses({target_pose, pose_offset});

    Vector6d target_pose_prime = cga_ik::CGAKinematicsPose2R6Pose( cga_ik::R6Pose2CGAKinematicsPose(target_pose) );
    Vector6d target_pose_res = RM::R6Poses2RelativeR6Pose(target_pose_prime, target_pose);

    // RM::PrintVec(target_pose, "\nIK target_pose [m, rad]");   
    // RM::PrintVec(target_pose_prime, "\ntarget_pose_prime [m, rad]");
    RM::PrintVec(target_pose_res, "\nPose residual [m, rad]");


    cga_ik::CGAIKRobotConfig robot_config = cga_ik::setRobotConfig(1, -1, 1);
    std::cout << "\nRobot configuration: " << robot_config.kud << ", " << robot_config.klr << ", " << robot_config.kfn << std::endl;

    cga_ik::SolveNullPoints(target_pose, dh_table, robot_config);
    // std::cout << "\nCGA IK null points [m] =" << std::endl;
    // cga_ik::null_points_g3.print();

    // If reachable, solve the joint angles and return the null points.
    if (cga_ik::reachable) {
        cga_ik::SolveJointAngles();
        RM::PrintVec(cga_ik::joints * RM::r2d, "\nJoint angles [deg]");
    } else {
        std::cerr << "\nTarget target_pose is not reachable!" << std::endl;
    }

    // Validate the IK solution by FK
    Vector6d resulting_pose = cga_ik::CGAFK(cga_ik::joints, dh_table);

    // Pose residual
    Vector6d pose_res = RM::R6Poses2RelativeR6Pose(resulting_pose, target_pose);
    RM::PrintVec(pose_res, "\nPose residual [m, rad]");




    

    rclcpp::shutdown();
    return 0;
}
