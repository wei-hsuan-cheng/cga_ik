// This algorithm is based on: https://slides.com/hugohadfield/game2020

#ifndef CGA_IK_SPHERICAL_ROBOT_HPP
#define CGA_IK_SPHERICAL_ROBOT_HPP

#include <Eigen/Dense>
#include "cga_ik/cga_utils.hpp"
#include "robot_math_utils/robot_math_utils_v1_9.hpp"

using cga_utils::up;
using cga_utils::down;
using cga_utils::Grade;
using RM = RMUtils;

namespace cga_ik_spherical_robot {

// A small helper to compute the Euclidean angle between two 3D vectors.
inline float angleBetweenVecs(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
{
    float dotVal = v1.dot(v2);
    float norms  = v1.norm() * v2.norm();
    if (norms < 1e-9f) {
        return 0.0f; // handle degenerate case
    }
    return (float) RM::ArcCos( (double) dotVal / norms ); // returns angle in [0, π]
}

float solveJointAngle(const CGA &o, const CGA &m, const CGA &rc, const CGA &epl)
{
    Eigen::Vector3f normal_cmo = 
    ( cga_utils::G2R( down(o) ) - cga_utils::G2R(m) ).cross( cga_utils::G2R(rc) - cga_utils::G2R(m) );
    normal_cmo = normal_cmo.normalized();

    Eigen::Vector3f normal_ymc = 
    ( cga_utils::G2R(rc) - cga_utils::G2R(m) ).cross( cga_utils::G2R( down(epl) ) - cga_utils::G2R(m) );
    normal_ymc = normal_ymc.normalized();
    
    // Compute axis-angle representation
    Eigen::Vector3f axis_cross = normal_cmo.cross(normal_ymc);
    float ang_cross = angleBetweenVecs(normal_cmo, normal_ymc);
    // Check if axis_cross (axis of rotation) is in the same direction as l_vec (from motor to centre)
    Eigen::Vector3f l_vec = (cga_utils::G2R(rc) - cga_utils::G2R(m)).normalized();
    float ang = axis_cross(0) / l_vec(0) >= 0 ? ang_cross : -ang_cross;
    
    // // Debugging
    // std::cout << "normal_cmo: " << normal_cmo.transpose() << std::endl;
    // std::cout << "normal_ymc: " << normal_ymc.transpose() << std::endl;
    // std::cout << "axis_cross: " << axis_cross.transpose() << std::endl;
    // Eigen::Vector3f axis_div = axis_cross.cwiseQuotient(l_vec); // entry-wise division for axis_cross and l_vec
    // std::cout << "axis_div: " << axis_div.transpose() << std::endl;
    // std::cout << "ang_cross [deg] = " << ang_cross * RM::r2d << std::endl;
    // std::cout << "ang [deg] = " << ang * RM::r2d << std::endl;

    return ang;
}

// This struct holds the final output of the single-orientation IK.
struct SphericalRobotIKResult {
    float r_b, r_e;
    Eigen::Vector3f m_0, m_1, m_2;           // motor (pivot) positions
    Eigen::Vector3f rot_cen;      // rotation centre of the robot
    float r_s, d;
    Eigen::Vector3f epl_0, epl_1, epl_2, epl_c;           // end-plate corners and centre
    Eigen::Vector3f elb_0, elb_1, elb_2;     // elbow positions
    float th_0, th_1, th_2;  // motor angles [rad]
};

// A function that takes in the quaternion
// and returns the full IK results of the 3-DoF spherical robot.
inline SphericalRobotIKResult computeSphericalRobotIK(
    // Principal axis of the 3-DoF spherical robot
    CGA e_principal,
    // Rotation centre
    CGA rot_cen,
    // Two basis vectors on the base plane
    CGA s_0, CGA s_1,
    // Quaternion orientation
    Eigen::Quaternionf quat,
    // geometry parameters:
    float r_b, float r_e) 
{
    // Compute s_2 based on s_0 and s_1
    CGA s_2 = (-1.0f) * s_0 + (-1.0f) * s_1;


    // Outer sphere radius
    CGA temp = (r_b * s_0) - rot_cen;
    float dotVal = (temp | temp)[0];
    float r_s = std::sqrt( std::fabs(dotVal) ); // Radius of the outer sphere
    float d = std::sqrt( r_s*r_s - r_e*r_e ); // Distance from the rotation centre to the end-plate


    // Create the “fixed” sphere S about rot_cen
    CGA S = !( up(rot_cen) - 0.5f * (r_s*r_s) * ni );


    // Generate the “fixed” planes (a_0, a_1, a_2)
    CGA Tr_0 = CGA(1.0f,0) - 0.5f * ( ((-r_b * s_0) + rot_cen) ^ ni );
    CGA a_0_full = Tr_0 * ( ( up(r_b*s_0) | S ) ^ ni ) * ~Tr_0;
    CGA a_0 = Grade(a_0_full, 4);

    CGA Tr_1 = CGA(1.0f,0) - 0.5f * ( ((-r_b * s_1) + rot_cen) ^ ni );
    CGA a_1_full = Tr_1 * ( ( up(r_b*s_1) | S ) ^ ni ) * ~Tr_1;
    CGA a_1 = Grade(a_1_full, 4);

    CGA Tr_2 = CGA(1.0f,0) - 0.5f * ( ((-r_b * s_2) + rot_cen) ^ ni );
    CGA a_2_full = Tr_2 * ( ( up(r_b*s_2) | S ) ^ ni ) * ~Tr_2;
    CGA a_2 = Grade(a_2_full, 4);


    // Build the rotor R_ee
    Eigen::Vector3f zyx_euler = RM::Quat2zyxEuler( quat.cast<double>() ).cast<float>();
    CGA R_z = cga_utils::rot(e1 * e2, zyx_euler(0));
    CGA R_y = cga_utils::rot(e3 * e1, zyx_euler(1));
    CGA R_x = cga_utils::rot(e2 * e3, zyx_euler(2));
    CGA R_ee = (R_z * R_y * R_x).normalized();


    // End-plate corners in R^3
    CGA x_0 = r_e * s_0 + d * e_principal;
    CGA x_1 = r_e * s_1 + d * e_principal;
    CGA x_2 = r_e * s_2 + d * e_principal;

    CGA epl_0 = up(R_ee * x_0 * ~R_ee + rot_cen);
    CGA epl_1 = up(R_ee * x_1 * ~R_ee + rot_cen);
    CGA epl_2 = up(R_ee * x_2 * ~R_ee + rot_cen);


    // End-plate centre
    CGA pos_epl_0_epl_c =  R_ee * (-r_e * s_0) * ~R_ee;
    CGA Tr_epl_0_epl_c = cga_utils::trans( pos_epl_0_epl_c );
    CGA epl_c = Tr_epl_0_epl_c * epl_0 * ~Tr_epl_0_epl_c;


    // Build translator for each corner
    CGA Tx_0 = CGA(1.0f,0) + 0.5f * ((R_ee*x_0*~R_ee) ^ ni);
    CGA planeFactor_0 = ((S|epl_0) ^ ni).normalized();
    CGA p_0_full = Tx_0 * planeFactor_0 * ~Tx_0;
    CGA p_0 = Grade(p_0_full, 4);

    CGA Tx_1 = CGA(1.0f,0) + 0.5f * ((R_ee*x_1*~R_ee) ^ ni);
    CGA planeFactor_1 = ((S|epl_1) ^ ni).normalized();
    CGA p_1_full = Tx_1 * planeFactor_1 * ~Tx_1;
    CGA p_1 = Grade(p_1_full, 4);

    CGA Tx_2 = CGA(1.0f,0) + 0.5f * ((R_ee*x_2*~R_ee) ^ ni);
    CGA planeFactor_2 = ((S|epl_2) ^ ni).normalized();
    CGA p_2_full = Tx_2 * planeFactor_2 * ~Tx_2;
    CGA p_2 = Grade(p_2_full, 4);


    // Intersect the circles for the elbows
    CGA T_0_full = (a_0 & p_0) & S;
    CGA T_0_graded = Grade(T_0_full, 2);
    CGA T_0 = T_0_graded.normalized();
    float T_0_sqVal = (T_0*T_0)[0];
    float invLen_T_0 = 1.0f / std::sqrt(std::fabs(T_0_sqVal));
    CGA elb_0 = (CGA(1.0f,0) + T_0 * invLen_T_0) * (T_0 | ni);

    CGA T_1_full = (a_1 & p_1) & S;
    CGA T_1_graded = Grade(T_1_full, 2);
    CGA T_1 = T_1_graded.normalized();
    float T_1_sqVal = (T_1*T_1)[0];
    float invLen_T_1 = 1.0f / std::sqrt(std::fabs(T_1_sqVal));
    CGA elb_1 = (CGA(1.0f,0) + T_1 * invLen_T_1) * (T_1 | ni);

    CGA T_2_full = (a_2 & p_2) & S;
    CGA T_2_graded = Grade(T_2_full, 2);
    CGA T_2 = T_2_graded.normalized();
    float T_2_sqVal = (T_2*T_2)[0];
    float invLen_T_2 = 1.0f / std::sqrt(std::fabs(T_2_sqVal));
    CGA elb_2 = (CGA(1.0f,0) + T_2 * invLen_T_2) * (T_2 | ni);


    // Now compute the IK solutions (motor angles)
    // Cross product to compute the angle between two planes
    float th_0 = solveJointAngle(no, r_b * s_0, rot_cen, epl_0);
    float th_1 = solveJointAngle(no, r_b * s_1, rot_cen, epl_1);
    float th_2 = solveJointAngle(no, r_b * s_2, rot_cen, epl_2);


    // Assemble results:
    SphericalRobotIKResult result;
    result.r_b = r_b;
    result.r_e = r_e;
    result.m_0 = cga_utils::G2R(r_b * s_0);
    result.m_1 = cga_utils::G2R(r_b * s_1);
    result.m_2 = cga_utils::G2R(r_b * s_2);
    result.rot_cen = cga_utils::G2R(rot_cen);
    result.d = d;
    result.r_s = r_s;
    result.epl_0 = cga_utils::G2R( down(epl_0) );
    result.epl_1 = cga_utils::G2R( down(epl_1) );
    result.epl_2 = cga_utils::G2R( down(epl_2) );
    result.epl_c = cga_utils::G2R( down(epl_c) );
    result.elb_0 = cga_utils::G2R( down(elb_0) );
    result.elb_1 = cga_utils::G2R( down(elb_1) );
    result.elb_2 = cga_utils::G2R( down(elb_2) );
    result.th_0 = th_0;
    result.th_1 = th_1;
    result.th_2 = th_2;

    return result;
}

} // namespace cga_ik_spherical_robot

#endif // CGA_IK_SPHERICAL_ROBOT_HPP