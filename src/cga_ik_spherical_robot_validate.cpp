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

    std::cout << "elb0: "; down(result.elb0).log();
    std::cout << "elb1: "; down(result.elb1).log();
    std::cout << "elb2: "; down(result.elb2).log();

    std::cout << "\nEnd point:\n";
    result.endpoint.log();

    rclcpp::shutdown();
    return 0;
}





// #include <vector>
// #include <iostream>
// #include <tuple>
// #include "cga_ik/cga_utils.hpp"
// #include "robot_math_utils/robot_math_utils_v1_9.hpp"

// #include <rclcpp/rclcpp.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <std_msgs/msg/float64_multi_array.hpp>
// #include <geometry_msgs/msg/quaternion.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>

// #include <Eigen/Dense>
// #include <Eigen/Geometry>
// #include <chrono>
// #include <memory>
// #include <cmath>    // for std::acos

// using cga_utils::up;
// using cga_utils::down;

// using RM = RMUtils;

// // A small helper to compute the Euclidean angle between two 3D vectors:
// float angleBetween(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
// {
//     float dotVal = v1.dot(v2);
//     float norms  = v1.norm() * v2.norm();
//     if (norms < 1e-9f) {
//         return 0.0f; // or handle degenerately
//     }
//     float cosVal = dotVal / norms;
//     // Clamp numerical error for safe acos:
//     if (cosVal > 1.0f)  cosVal = 1.0f;
//     if (cosVal < -1.0f) cosVal = -1.0f;
//     return std::acos(cosVal); // returns angle in [0, π]
// }

// int main(int argc, char ** argv) {
//     // Initialize ROS 2.
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("cga_ik_spherical_robot_validate");
//     std::cout << "\n----- Starting CGA spherical robot validation -----\n" << std::endl;

//     // ------------------------------------------
//     // Step 1: Define the robot properties in CGA form
//     // ------------------------------------------

//     // Each motor axis is set in the e1-e2 plane, separated by 120 degrees.
//     CGA s0 = 1.0f * e1; 
//     CGA s1 = (-0.5f * e1) + ((std::sqrt(3.0f) / 2.0f) * e2);
//     CGA s2 = (-0.5f * e1) - ((std::sqrt(3.0f) / 2.0f) * e2);

//     // Radii:
//     float r_b = 0.5f;  // outer sphere radius
//     float r_e = 0.4f;  // end plate circle radius

//     // Rotation centre: -(1/3)*r_b * e3
//     CGA rotation_centre = -(1.0f / 3.0f) * r_b * e3;
//     // up-project to conformal point
//     CGA up_rotation_centre = up(rotation_centre);

//     {
//         // Motor sphere radius
//         CGA temp = (r_b * s0) - rotation_centre;
//         float dotVal = (temp | temp)[0];
//         float l = std::sqrt(std::fabs(dotVal));

//         // The starting distance of the end plate from the e12 plane
//         float d = std::sqrt(l*l - r_e*r_e);

//         // A fixed sphere about the rotation centre (grade-4 sphere)
//         CGA S = !( up_rotation_centre - 0.5f * (l*l) * ni );

//         // Generate the fixed planes perpendicular to the axes of the motors:
//         CGA Tr0 = CGA(1.0f, 0) - 0.5f * (((-r_b * s0) + rotation_centre) ^ ni);
//         CGA a0_full = Tr0 * ( ((up(r_b*s0) | S) ^ ni) ) * ~Tr0;
//         CGA a0 = cga_utils::Grade(a0_full, 4);

//         CGA Tr1 = CGA(1.0f, 0) - 0.5f * (((-r_b * s1) + rotation_centre) ^ ni);
//         CGA a1_full = Tr1 * ( ((up(r_b*s1) | S) ^ ni) ) * ~Tr1;
//         CGA a1 = cga_utils::Grade(a1_full, 4);

//         CGA Tr2 = CGA(1.0f, 0) - 0.5f * (((-r_b * s2) + rotation_centre) ^ ni);
//         CGA a2_full = Tr2 * ( ((up(r_b*s2) | S) ^ ni) ) * ~Tr2;
//         CGA a2 = cga_utils::Grade(a2_full, 4);

//         // Fixed circles
//         CGA k0 = a0 & S;
//         CGA k1 = a1 & S;
//         CGA k2 = a2 & S;

//         // Print logs for demonstration
//         std::cout << "s0: "; s0.log();
//         std::cout << "s1: "; s1.log();
//         std::cout << "s2: "; s2.log();

//         std::cout << "\nrotation_centre (CGA): ";
//         rotation_centre.log();

//         std::cout << "up_rotation_centre: ";
//         up_rotation_centre.log();

//         std::cout << "\nComputed l = " << l << ", d = " << d << std::endl;

//         std::cout << "\nSphere S: ";
//         S.log();

//         std::cout << "\nPlane a0: ";
//         a0.log();
//         std::cout << "Plane a1: ";
//         a1.log();
//         std::cout << "Plane a2: ";
//         a2.log();

//         std::cout << "\nFixed circle k0: ";
//         k0.log();
//         std::cout << "Fixed circle k1: ";
//         k1.log();
//         std::cout << "Fixed circle k2: ";
//         k2.log();
//     }

//     // -------------------------------------------------------------
//     // Step 2: For demonstration, do a single orientation of the end-effector
//     // -------------------------------------------------------------

//     // e^(theta2/2 * e13) * e^(theta1/2 * e23) * e^(theta0/2 * e12)
//     float theta0 = 0.0f;  
//     float theta1 = 0.0f;  
//     float theta2 = 0.0f;

//     CGA R2 = cga_utils::rot(cga::e13, theta2);
//     CGA R1 = cga_utils::rot(cga::e23, theta1);
//     CGA R0 = cga_utils::rot(cga::e12, theta0);
//     CGA R_ee = (R2 * R1 * R0).normalized();

//     // Recompute the same values (l, d, sphere S, planes a0,a1,a2) from above
//     CGA temp2 = (r_b * s0) - rotation_centre;
//     float dotVal2 = (temp2 | temp2)[0];
//     float l_demo = std::sqrt(std::fabs(dotVal2));
//     float d_demo = std::sqrt(l_demo*l_demo - r_e*r_e);

//     CGA S_demo = !( up_rotation_centre - 0.5f * (l_demo*l_demo) * ni );

//     CGA Tr0_demo = CGA(1.0f, 0) 
//                  - 0.5f * (((-r_b * s0) + rotation_centre) ^ ni);
//     CGA a0_full_demo = Tr0_demo * ( ((up(r_b*s0) | S_demo) ^ ni) ) * ~Tr0_demo;
//     CGA a0_demo = cga_utils::Grade(a0_full_demo, 4);

//     CGA Tr1_demo = CGA(1.0f, 0) 
//                  - 0.5f * (((-r_b * s1) + rotation_centre) ^ ni);
//     CGA a1_full_demo = Tr1_demo * ( ((up(r_b*s1) | S_demo) ^ ni) ) * ~Tr1_demo;
//     CGA a1_demo = cga_utils::Grade(a1_full_demo, 4);

//     CGA Tr2_demo = CGA(1.0f, 0) 
//                  - 0.5f * (((-r_b * s2) + rotation_centre) ^ ni);
//     CGA a2_full_demo = Tr2_demo * ( ((up(r_b*s2) | S_demo) ^ ni) ) * ~Tr2_demo;
//     CGA a2_demo = cga_utils::Grade(a2_full_demo, 4);

//     // End-plate corners (in 3D, each corner is at r_e * s_i - d * e3, 
//     // then transformed by the rotor, plus the rotation_centre offset)
//     CGA x0 = r_e * s0 - d_demo * e3;
//     CGA x1 = r_e * s1 - d_demo * e3;
//     CGA x2 = r_e * s2 - d_demo * e3;

//     CGA y0 = up(R_ee * x0 * ~R_ee + rotation_centre);
//     CGA y1 = up(R_ee * x1 * ~R_ee + rotation_centre);
//     CGA y2 = up(R_ee * x2 * ~R_ee + rotation_centre);

//     // Construct the moving circles p0,p1,p2
//     CGA Tx0 = CGA(1.0f,0) + 0.5f * ( (R_ee*x0*~R_ee) ^ ni );
//     CGA planeFactor0 = ((S_demo|y0) ^ ni).normalized();
//     CGA p0_full = Tx0 * planeFactor0 * ~Tx0;
//     CGA p0 = cga_utils::Grade(p0_full, 4);

//     CGA Tx1 = CGA(1.0f,0) + 0.5f * ( (R_ee*x1*~R_ee) ^ ni );
//     CGA planeFactor1 = ((S_demo|y1) ^ ni).normalized();
//     CGA p1_full = Tx1 * planeFactor1 * ~Tx1;
//     CGA p1 = cga_utils::Grade(p1_full, 4);

//     CGA Tx2 = CGA(1.0f,0) + 0.5f * ( (R_ee*x2*~R_ee) ^ ni );
//     CGA planeFactor2 = ((S_demo|y2) ^ ni).normalized();
//     CGA p2_full = Tx2 * planeFactor2 * ~Tx2;
//     CGA p2 = cga_utils::Grade(p2_full, 4);

//     CGA c0 = p0 & S_demo;
//     CGA c1 = p1 & S_demo;
//     CGA c2 = p2 & S_demo;

//     // Intersect circles to get elbow positions
//     CGA T0_full = (a0_demo & p0) & S_demo;
//     CGA T0_graded = cga_utils::Grade(T0_full, 2);
//     CGA T0 = T0_graded.normalized();
//     float T0sqVal = (T0*T0)[0];
//     float invLenT0 = 1.0f / std::sqrt(std::fabs(T0sqVal));
//     CGA elb0 = ( CGA(1.0f,0) + T0 * invLenT0 ) * (T0 | ni);

//     CGA T1_full = (a1_demo & p1) & S_demo;
//     CGA T1_graded = cga_utils::Grade(T1_full, 2);
//     CGA T1 = T1_graded.normalized();
//     float T1sqVal = (T1*T1)[0];
//     float invLenT1 = 1.0f / std::sqrt(std::fabs(T1sqVal));
//     CGA elb1 = ( CGA(1.0f,0) + T1 * invLenT1 ) * (T1 | ni);

//     CGA T2_full = (a2_demo & p2) & S_demo;
//     CGA T2_graded = cga_utils::Grade(T2_full, 2);
//     CGA T2 = T2_graded.normalized();
//     float T2sqVal = (T2*T2)[0];
//     float invLenT2 = 1.0f / std::sqrt(std::fabs(T2sqVal));
//     CGA elb2 = ( CGA(1.0f,0) + T2 * invLenT2 ) * (T2 | ni);

//     // endpoint = ((y0 ^ y1 ^ y2 ^ ni) * up_rotation_centre * (y0 ^ y1 ^ y2 ^ ni))
//     CGA triProduct = (y0 ^ y1 ^ y2 ^ ni);
//     CGA endpoint = (triProduct * up_rotation_centre) * triProduct;

//     // -----------------------------
//     // Step 3: IK angle calculation for each motor
//     // We interpret “the angle between lines from elbow point to rotation_centre 
//     // and from rotation_centre to motor pivot” as the “motor angle”.
//     // -----------------------------
//     // We'll define motor pivot i:  pivot_i = up(r_b*s_i + rotation_centre).
//     // Then we “down” everything to R^3 to do the angle.
//     std::cout << "\n--- Single orientation results ---\n";
//     std::cout << "Rotors: R_ee: "; R_ee.log();

//     // Let's down() the rotation_centre itself (for 3D location):
//     Eigen::Vector3f rc3 = (down(up(rotation_centre))).vector().head<3>();

//     // A helper lambda to get the 3D vector from the elbow to the rotation centre:
//     auto getL1 = [&](const CGA &elb) -> Eigen::Vector3f {
//         Eigen::Vector3f e = (down(elb)).vector().head<3>();
//         return rc3 - e; // from elbow to rotation_centre
//     };
//     // A helper lambda to get the 3D vector from the elbow to the motor pivot:
//     auto getL2 = [&](const CGA &elb, const CGA &s_i) -> Eigen::Vector3f {
//         // Motor pivot as a conformal point:
//         CGA pivotCga = up(r_b*s_i + rotation_centre);
//         Eigen::Vector3f pivot3 = (down(pivotCga)).vector().head<3>();
//         Eigen::Vector3f e = (down(elb)).vector().head<3>();
//         return pivot3 - e; // from elbow to pivot
//     };

//     // For each elbow i, compute angle_i:
//     Eigen::Vector3f L10 = getL1(elb0);
//     Eigen::Vector3f L20 = getL2(elb0, s0);
//     float angle0 = angleBetween(L10, L20);

//     Eigen::Vector3f L11 = getL1(elb1);
//     Eigen::Vector3f L21 = getL2(elb1, s1);
//     float angle1 = angleBetween(L11, L21);

//     Eigen::Vector3f L12 = getL1(elb2);
//     Eigen::Vector3f L22 = getL2(elb2, s2);
//     float angle2 = angleBetween(L12, L22);

//     // Print angles in degrees:
//     auto rad2deg = [](float r){ return r * 180.0f / float(M_PI); };

//     std::cout << "\nMotor 0 angle = " << angle0 << " rad (" << rad2deg(angle0) << " deg)";
//     std::cout << "\nMotor 1 angle = " << angle1 << " rad (" << rad2deg(angle1) << " deg)";
//     std::cout << "\nMotor 2 angle = " << angle2 << " rad (" << rad2deg(angle2) << " deg)\n";

//     std::cout << "\nEnd-plate corners (y0,y1,y2):\n";
//     y0.log(); 
//     y1.log(); 
//     y2.log();

//     std::cout << "\nElbow positions (elb0, elb1, elb2):\n";
//     elb0.log();
//     elb1.log();
//     elb2.log();

//     std::cout << "\nEnd point:";
//     endpoint.log();

//     // shutdown
//     rclcpp::shutdown();
//     return 0;
// }