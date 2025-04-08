// This IK solution is originally adopted from: https://slides.com/hugohadfield/game2020
// A JavaScript implementation of this code: https://enkimute.github.io/ganja.js/examples/coffeeshop.html#2DmBscfSXO

#ifndef CGA_IK_SPHERICAL_ROBOT_HPP
#define CGA_IK_SPHERICAL_ROBOT_HPP

#include <Eigen/Dense>
#include "cga/cga_utils.hpp"
#include "robot_math_utils/robot_math_utils_v1_9.hpp"

using Eigen::Quaternionf;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::MatrixXf;

using Matrix6f = Eigen::Matrix<float, 6, 6>;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Vector6f = Eigen::Matrix<float, 6, 1>;
using Vector7f = Eigen::Matrix<float, 7, 1>;
using Eigen::VectorXf;

using cga_utils::up;
using cga_utils::down;
using cga_utils::Grade;
using RM = RMUtils;

namespace cga_ik_spm_3dof {

// Helper: compute the outer sphere radius
float computeOuterSphereRadius(const CGA &m, const CGA &rc)
{
    CGA temp = m - rc;
    float dotVal = (temp | temp)[0];
    return std::sqrt( std::fabs(dotVal) );
}

CGA constructSphere(const float &r_s, const CGA &rc, const int &grade)
{
    // Construct the outer sphere S with radius r_s centred at rc
    CGA s = up(rc) - 0.5f * (r_s * r_s) * ni;
    switch (grade) {
        case 1:
            return s;
        case 4:
            return !s;
        default:
            throw std::invalid_argument("[cga_ik_spm_3dof::constructSphere() ERROR] Invalid grade for CGA sphere construction");
    }
    
}

// Helper: convert zyx Euler angles to rotor
CGA zyxEuler2Rotor(const Vector3f &zyx_euler)
{
    CGA R_z = cga_utils::rot(e1 * e2, zyx_euler(0));
    CGA R_y = cga_utils::rot(e3 * e1, zyx_euler(1));
    CGA R_x = cga_utils::rot(e2 * e3, zyx_euler(2));
    return (R_z * R_y * R_x).normalized();
}

// Helper: Three basis vectors to quaternion orientation
Quaternionf computeQuatFromBasis(const CGA &x_hat, const CGA &y_hat, const CGA &z_hat)
{
    Matrix3f RMat;
    RMat << (x_hat | e1)[0], (y_hat | e1)[0], (z_hat | e1)[0],
            (x_hat | e2)[0], (y_hat | e2)[0], (z_hat | e2)[0],
            (x_hat | e3)[0], (y_hat | e3)[0], (z_hat | e3)[0];
    return Quaternionf(RMat);
}

// Helper: compute the initial quaternion orientation of the motor.
Quaternionf computeMotorInitalQuat(const CGA &m, const CGA &rc)
{
    CGA z_hat = (rc - m).normalized();
    CGA vec_m_o = (down(no) - m).normalized();
    CGA x_hat_i = ( (-1) * e123 * (vec_m_o ^ z_hat) ).normalized(); // Dual 
    CGA y_hat_i = ( (-1) * e123 * (z_hat ^ x_hat_i) ).normalized(); // Dual
    return computeQuatFromBasis(x_hat_i, y_hat_i, z_hat);
}

// Helper: compute the quaternion orientation of the motor.
Quaternionf computeMotorQuat(const CGA &m, const CGA &epl, const CGA &rc)
{
    CGA z_hat = (rc - m).normalized();
    CGA vec_m_epl = (down(epl) - m).normalized();
    CGA x_hat = ( (-1) * e123 * (z_hat ^ vec_m_epl) ).normalized(); // Dual 
    CGA y_hat = ( (-1) * e123 * (z_hat ^ x_hat) ).normalized(); // Dual
    return computeQuatFromBasis(x_hat, y_hat, z_hat);
}

// Helper: compute the end-plate corner offset.
CGA computeEndPlateCornerOffset(const float &r_e, const CGA &s, const float &d, const CGA &e_p)
{
    // The offset from the rotation centre to the end-plate corners
    return r_e * s + d * e_p;
}

// Helper: compute the end-plate corner position.
CGA computeEndPlateCornerPosition(const CGA &x, const CGA &R_ee, const CGA &rc)
{
    return up(R_ee * x * ~R_ee + rc);
}

// Helper: compute the end-plate centre position.
CGA computeEndPlateCentrePosition(const CGA &epl_0, const CGA &R_ee, float r_e, const CGA &s_0)
{
    CGA pos_epl_0_epl_c =  R_ee * (-r_e * s_0) * ~R_ee;
    CGA Tr_epl_0_epl_c = cga_utils::trans( pos_epl_0_epl_c );
    return Tr_epl_0_epl_c * epl_0 * ~Tr_epl_0_epl_c;
}

// Helper: compute the quaternion orientation of the end-plate corner.
Quaternionf computeEndPlateCornerQuat(const CGA &epl, const CGA &epl_c, const CGA &rc)
{
    CGA z_hat = (rc - down(epl)).normalized();
    CGA vec_epl_epl_c = (down(epl_c) - down(epl)).normalized();
    CGA x_hat = ( (-1) * e123 * (z_hat ^ vec_epl_epl_c) ).normalized(); // Dual 
    CGA y_hat = ( (-1) * e123 * (z_hat ^ x_hat) ).normalized(); // Dual
    return computeQuatFromBasis(x_hat, y_hat, z_hat);
}

// Helper: compute the fixed planes.
CGA computeFixedPlanes(const CGA &m, const CGA &rc, const CGA &S)
{
    CGA Tr = CGA(1.0f, 0) - 0.5f * ( ((-1) * m + rc) ^ ni );
    CGA a_full = Tr * ( ( up(m) | S ) ^ ni ) * ~Tr;
    return Grade(a_full, 4);
}
// Helper: compute the moving planes.
CGA computeMovingPlanes(const CGA &x, const CGA &R_ee, const CGA &epl, const CGA &S)
{
    CGA Tx = CGA(1.0f,0) + 0.5f * ((R_ee * x * ~R_ee) ^ ni);
    CGA planeFactor = ((S | epl) ^ ni).normalized();
    CGA p_full = Tx * planeFactor * ~Tx;
    return Grade(p_full, 4);
}

// Helper: compute the elbow positions.
CGA computeElbowPositions(const CGA &a, const CGA &p, const CGA &S)
{
    // Intersect the circles for the elbows
    CGA T_full = (a & p) & S;
    CGA T_graded = Grade(T_full, 2);
    CGA T = T_graded.normalized();
    float T_sqVal = (T*T)[0];
    float invLen_T = 1.0f / std::sqrt(std::fabs(T_sqVal));

    return (CGA(1.0f,0) + T * invLen_T) * (T | ni);
}

// Helper: compute the quaternion orientation of the elbow.
Quaternionf computeElbowQuat(const CGA &epl, const CGA &elb, const CGA &rc)
{
    CGA z_hat = (rc - down(elb)).normalized();
    CGA vec_elb_epl = (down(epl) - down(elb)).normalized();
    CGA y_hat = ( (-1) * e123 * (z_hat ^ vec_elb_epl) ).normalized(); // Dual 
    CGA x_hat = ( (-1) * e123 * (y_hat ^ z_hat) ).normalized(); // Dual
    return computeQuatFromBasis(x_hat, y_hat, z_hat);
}

// Helper: compute relative z-angle between two quaternions
float computeRelativeZAngle(const Quaternionf & quat_0_1, const Quaternionf & quat_0_2)
{
    Quaterniond quat_1_2 = RM::InvQuat( quat_0_1.cast<double>() ) * quat_0_2.cast<double>();
    Vector4d axis_ang_1_2 = RM::AxisAng3( RM::Quat2so3(quat_1_2) );
    return axis_ang_1_2(2) > 0 ? axis_ang_1_2(3) : -axis_ang_1_2(3);
}


// This struct holds the final output of the single-orientation IK.
struct SPM3DoFIKResult {
    float r_b, r_e;
    float r_s, d;
    
    Vector3f rot_cen;      // rotation centre of the robot
    Quaternionf quat_rot_cen; // quaternion for rotation centre

    // Relative poses w.r.t the rotation centre
    Vector3f pos_rot_cen_m_0, pos_rot_cen_m_1, pos_rot_cen_m_2;           // motor positions
    Quaternionf quat_rot_cen_m_0, quat_rot_cen_m_1, quat_rot_cen_m_2; // quaternions for motor orientations
    Quaternionf quat_rot_cen_m_0_i, quat_rot_cen_m_1_i, quat_rot_cen_m_2_i; // initial quaternions for motor orientations

    Vector3f pos_rot_cen_epl_0, pos_rot_cen_epl_1, pos_rot_cen_epl_2, pos_rot_cen_epl_c, pos_rot_cen_ept; // end-plate corners and centre, and the end-point on the outer sphere
    Quaternionf quat_rot_cen_epl_0, quat_rot_cen_epl_1, quat_rot_cen_epl_2, quat_rot_cen_epl_c, quat_rot_cen_ept; // quaternions for end-plate corners and centre, and the end-point on the outer sphere
    
    Vector3f pos_rot_cen_elb_0, pos_rot_cen_elb_1, pos_rot_cen_elb_2; // elbow positions
    Quaternionf quat_rot_cen_elb_0, quat_rot_cen_elb_1, quat_rot_cen_elb_2; // quaternions for elbow orientations
    
    float th_0, th_1, th_2;  // motor angles [rad]
};


// Main function: compute IK solution of the 3-DoF spherical robot.
inline SPM3DoFIKResult computeSPM3DoFIK(
    // geometry parameters
    const float &r_b, const float &r_e,
    // Principal axis of the 3-DoF spherical robot
    const CGA &e_principal,
    // Rotation centre
    const CGA &rot_cen,
    // Two basis vectors on the base plane
    const CGA &s_0, const CGA &s_1,
    // Target quaternion orientation for the end-plate
    const Quaternionf &target_quat_epl) 
{
    // These are the invariant parameters (can be defined as global variables)
    // Compute s_2 based on s_0 and s_1
    CGA s_2 = (-1.0f) * s_0 + (-1.0f) * s_1;

    // Motor positions
    CGA m_0 = r_b * s_0;
    CGA m_1 = r_b * s_1;
    CGA m_2 = r_b * s_2;

    // Construct the “fixed” outer sphere S about rot_cen (grade-4 sphere)
    float r_s = computeOuterSphereRadius(m_0, rot_cen);
    CGA S = constructSphere(r_s, rot_cen, 4);

    // Distance from the rotation centre to the end-plate
    float d = std::sqrt( r_s * r_s - r_e * r_e ); 

    // Construct the “fixed” planes (a_0, a_1, a_2) (grade-4 planes)
    CGA a_0 = computeFixedPlanes(m_0, rot_cen, S);
    CGA a_1 = computeFixedPlanes(m_1, rot_cen, S);
    CGA a_2 = computeFixedPlanes(m_2, rot_cen, S);

    
    // These are the variant parameters
    // Build the rotor R_ee
    Vector3f zyx_euler = RM::Quat2zyxEuler( target_quat_epl.cast<double>() ).cast<float>();
    CGA R_ee = zyxEuler2Rotor(zyx_euler);

    // Positions of the end-plate corners (G41 vectors)
    CGA offset_rot_cen_epl_0 = computeEndPlateCornerOffset(r_e, s_0, d, e_principal); // invariant
    CGA epl_0 = computeEndPlateCornerPosition(offset_rot_cen_epl_0, R_ee, rot_cen);
    CGA offset_rot_cen_epl_1 = computeEndPlateCornerOffset(r_e, s_1, d, e_principal);
    CGA epl_1 = computeEndPlateCornerPosition(offset_rot_cen_epl_1, R_ee, rot_cen);
    CGA offset_rot_cen_epl_2 = computeEndPlateCornerOffset(r_e, s_2, d, e_principal);
    CGA epl_2 = computeEndPlateCornerPosition(offset_rot_cen_epl_2, R_ee, rot_cen);

    // End-plate centre (G41 vector)
    CGA epl_c = computeEndPlateCentrePosition(epl_0, R_ee, r_e, s_0);

    // End-point on the outer sphere (G41 vector)
    CGA ept = up( rot_cen + r_s * (down(epl_c) - rot_cen).normalized() );

    // Construct the moving planes (p_0, p_1, p_2) (grade-4 planes)
    CGA p_0 = computeMovingPlanes(offset_rot_cen_epl_0, R_ee, epl_0, S);
    CGA p_1 = computeMovingPlanes(offset_rot_cen_epl_1, R_ee, epl_1, S);
    CGA p_2 = computeMovingPlanes(offset_rot_cen_epl_2, R_ee, epl_2, S);

    // Compute the elbow positions (G41 vectors)
    CGA elb_0 = computeElbowPositions(a_0, p_0, S);
    CGA elb_1 = computeElbowPositions(a_1, p_1, S);
    CGA elb_2 = computeElbowPositions(a_2, p_2, S);

    // Quaternion orientation for each motor
    Quaternionf quat_m_0_i = computeMotorInitalQuat(m_0, rot_cen);
    Quaternionf quat_m_0 = computeMotorQuat(m_0, epl_0, rot_cen);
    Quaternionf quat_m_1_i = computeMotorInitalQuat(m_1, rot_cen);
    Quaternionf quat_m_1 = computeMotorQuat(m_1, epl_1, rot_cen);
    Quaternionf quat_m_2_i = computeMotorInitalQuat(m_2, rot_cen);
    Quaternionf quat_m_2 = computeMotorQuat(m_2, epl_2, rot_cen);

    // Quaternion orientation for each end-plate corner
    Quaternionf quat_epl_0 = computeEndPlateCornerQuat(epl_0, epl_c, rot_cen);
    Quaternionf quat_epl_1 = computeEndPlateCornerQuat(epl_1, epl_c, rot_cen);
    Quaternionf quat_epl_2 = computeEndPlateCornerQuat(epl_2, epl_c, rot_cen);
    Quaternionf quat_epl_c = target_quat_epl;
    Quaternionf quat_ept = target_quat_epl;

    // Quaternion orientation for each elbow
    Quaternionf quat_elb_0 = computeElbowQuat(epl_0, elb_0, rot_cen);
    Quaternionf quat_elb_1 = computeElbowQuat(epl_1, elb_1, rot_cen);
    Quaternionf quat_elb_2 = computeElbowQuat(epl_2, elb_2, rot_cen);

    // Compute the IK solutions (motor angles)
    // Compute angle between two motor orientations
    float th_0 = computeRelativeZAngle(quat_m_0_i, quat_m_0);
    float th_1 = computeRelativeZAngle(quat_m_1_i, quat_m_1);
    float th_2 = computeRelativeZAngle(quat_m_2_i, quat_m_2);



    // Compute the relative poses
    Quaternionf quat_rot_cen = Quaternionf::Identity(); // rot_cen w.r.t. base
    
    Quaternionf quat_rot_cen_m_0_i = quat_rot_cen.inverse() * quat_m_0_i;
    Quaternionf quat_rot_cen_m_1_i = quat_rot_cen.inverse() * quat_m_1_i;
    Quaternionf quat_rot_cen_m_2_i = quat_rot_cen.inverse() * quat_m_2_i;

    CGA pos_rot_cen_m_0 = m_0 - rot_cen;
    CGA pos_rot_cen_m_1 = m_1 - rot_cen;
    CGA pos_rot_cen_m_2 = m_2 - rot_cen;
    Quaternionf quat_rot_cen_m_0 = quat_rot_cen.inverse() * quat_m_0;
    Quaternionf quat_rot_cen_m_1 = quat_rot_cen.inverse() * quat_m_1;
    Quaternionf quat_rot_cen_m_2 = quat_rot_cen.inverse() * quat_m_2;

    CGA pos_rot_cen_epl_0 = down(epl_0) - rot_cen;
    CGA pos_rot_cen_epl_1 = down(epl_1) - rot_cen;
    CGA pos_rot_cen_epl_2 = down(epl_2) - rot_cen;
    CGA pos_rot_cen_epl_c = down(epl_c) - rot_cen;
    CGA pos_rot_cen_ept = down(ept) - rot_cen;
    Quaternionf quat_rot_cen_epl_0 = quat_rot_cen.inverse() * quat_epl_0;
    Quaternionf quat_rot_cen_epl_1 = quat_rot_cen.inverse() * quat_epl_1;
    Quaternionf quat_rot_cen_epl_2 = quat_rot_cen.inverse() * quat_epl_2;
    Quaternionf quat_rot_cen_epl_c = quat_rot_cen.inverse() * quat_epl_c;
    Quaternionf quat_rot_cen_ept = quat_rot_cen.inverse() * quat_ept;

    CGA pos_rot_cen_elb_0 = down(elb_0) - rot_cen;
    CGA pos_rot_cen_elb_1 = down(elb_1) - rot_cen;
    CGA pos_rot_cen_elb_2 = down(elb_2) - rot_cen;
    Quaternionf quat_rot_cen_elb_0 = quat_rot_cen.inverse() * quat_elb_0;
    Quaternionf quat_rot_cen_elb_1 = quat_rot_cen.inverse() * quat_elb_1;
    Quaternionf quat_rot_cen_elb_2 = quat_rot_cen.inverse() * quat_elb_2;




    // Assemble results:
    SPM3DoFIKResult result;

    // Spherical robot geometry parameters
    result.r_b = r_b;
    result.r_e = r_e;
    result.r_s = r_s;
    result.d = d;

    // Relative poses w.r.t the base centre frame
    result.rot_cen = cga_utils::G2R(rot_cen);
    result.quat_rot_cen = quat_rot_cen;

    // Relative poses w.r.t the rotation centre frame
    result.quat_rot_cen_m_0_i = quat_rot_cen_m_0_i;
    result.quat_rot_cen_m_1_i = quat_rot_cen_m_1_i;
    result.quat_rot_cen_m_2_i = quat_rot_cen_m_2_i;

    result.pos_rot_cen_m_0 = cga_utils::G2R(pos_rot_cen_m_0);
    result.pos_rot_cen_m_1 = cga_utils::G2R(pos_rot_cen_m_1);
    result.pos_rot_cen_m_2 = cga_utils::G2R(pos_rot_cen_m_2);
    result.quat_rot_cen_m_0 = quat_rot_cen_m_0;
    result.quat_rot_cen_m_1 = quat_rot_cen_m_1;
    result.quat_rot_cen_m_2 = quat_rot_cen_m_2;

    result.pos_rot_cen_epl_0 = cga_utils::G2R(pos_rot_cen_epl_0);
    result.pos_rot_cen_epl_1 = cga_utils::G2R(pos_rot_cen_epl_1);
    result.pos_rot_cen_epl_2 = cga_utils::G2R(pos_rot_cen_epl_2);
    result.pos_rot_cen_epl_c = cga_utils::G2R(pos_rot_cen_epl_c);
    result.pos_rot_cen_ept = cga_utils::G2R(pos_rot_cen_ept);
    result.quat_rot_cen_epl_0 = quat_rot_cen_epl_0;
    result.quat_rot_cen_epl_1 = quat_rot_cen_epl_1;
    result.quat_rot_cen_epl_2 = quat_rot_cen_epl_2;
    result.quat_rot_cen_epl_c = quat_rot_cen_epl_c;
    result.quat_rot_cen_ept = quat_rot_cen_ept;

    result.pos_rot_cen_elb_0 = cga_utils::G2R( pos_rot_cen_elb_0 );
    result.pos_rot_cen_elb_1 = cga_utils::G2R( pos_rot_cen_elb_1 );
    result.pos_rot_cen_elb_2 = cga_utils::G2R( pos_rot_cen_elb_2 );
    result.quat_rot_cen_elb_0 = quat_rot_cen_elb_0;
    result.quat_rot_cen_elb_1 = quat_rot_cen_elb_1;
    result.quat_rot_cen_elb_2 = quat_rot_cen_elb_2;

    // IK solution (motor angles)
    result.th_0 = th_0;
    result.th_1 = th_1;
    result.th_2 = th_2;

    return result;
}

} // namespace cga_ik_spm_3dof

#endif // CGA_IK_SPHERICAL_ROBOT_HPP