// This IK solution is originally adopted from: https://slides.com/hugohadfield/game2020
// A JavaScript implementation of this code: https://enkimute.github.io/ganja.js/examples/coffeeshop.html#2DmBscfSXO

#ifndef CGA_IK_SPM_3DOF_HPP
#define CGA_IK_SPM_3DOF_HPP

#include <Eigen/Dense>
#include "cga/cga_utils.hpp"
#include "robot_math_utils/robot_math_utils_v1_9.hpp"

using cga_utils::Grade;
using cga_utils::up;
using cga_utils::down;
using cga_utils::cross;
using cga_utils::zyxEuler2Rotor;
using cga_utils::sphere;
using RM = RMUtils;

namespace cga_ik_spm_3dof {

// Helper: compute the outer sphere radius
float computeOuterSphereRadius(const CGA &pos_rc_m)
{
    return std::sqrt( std::fabs( (pos_rc_m | pos_rc_m)[0] ) );
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
Quaternionf computeMotorInitalQuat(const CGA &rc, const CGA &pos_rc_m)
{
    CGA z_hat = ((-1) * pos_rc_m).normalized();
    CGA vec_rc = rc.normalized();
    CGA x_hat_i = ( cross(z_hat, vec_rc) ).normalized();
    CGA y_hat_i = ( cross(z_hat, x_hat_i) ).normalized(); // Dual    
    return computeQuatFromBasis(x_hat_i, y_hat_i, z_hat);
}

// Helper: compute the quaternion orientation of the motor.
Quaternionf computeMotorQuat(const CGA &pos_rc_epl, const CGA &pos_rc_m)
{
    CGA z_hat = ((-1) * pos_rc_m).normalized();
    CGA vec_m_epl = ( down(pos_rc_epl) - pos_rc_m ).normalized();
    CGA x_hat = ( cross(z_hat, vec_m_epl) ).normalized();
    CGA y_hat = ( cross(z_hat, x_hat) ).normalized(); // Dual
    return computeQuatFromBasis(x_hat, y_hat, z_hat);
}

// Helper: compute the end-plate corner offset.
CGA computeEndPlateCornerOffset(const float &r_e, const CGA &s, const float &d, const CGA &e_p)
{
    // The offset from the rotation centre to the end-plate corners
    return r_e * s + d * e_p;
}

// Helper: compute the end-plate corner position.
CGA computeEndPlateCornerPosition(const CGA &x, const CGA &R_rot_cen_ee)
{
    return up(R_rot_cen_ee * x * ~R_rot_cen_ee);
}

// Helper: compute the end-plate centre position.
CGA computeEndPlateCentrePosition(const CGA &pos_rot_cen_epl_0, const CGA &R_rot_cen_ee, float r_e, const CGA &s_0)
{
    CGA pos_epl_0_epl_c =  R_rot_cen_ee * (-r_e * s_0) * ~R_rot_cen_ee;
    CGA Tr_epl_0_epl_c = cga_utils::trans( pos_epl_0_epl_c );
    return Tr_epl_0_epl_c * pos_rot_cen_epl_0 * ~Tr_epl_0_epl_c;
}

// Helper: compute the quaternion orientation of the end-plate corner.
Quaternionf computeEndPlateCornerQuat(const CGA &pos_rc_epl, const CGA &pos_rc_epl_c)
{
    CGA z_hat = ( (-1) * down(pos_rc_epl) ).normalized();
    CGA vec_epl_epl_c = (down(pos_rc_epl_c) - down(pos_rc_epl)).normalized();
    CGA x_hat = ( cross(z_hat, vec_epl_epl_c) ).normalized();
    CGA y_hat = ( cross(z_hat, x_hat) ).normalized(); // Dual
    return computeQuatFromBasis(x_hat, y_hat, z_hat);
}

// Helper: compute the fixed planes.
CGA computeFixedPlanes(const CGA &pos_rc_m, const CGA &S)
{
    CGA Tr = CGA(1.0f, 0) - 0.5f * ( ((-1) * pos_rc_m) ^ ni );
    CGA a_full = Tr * ( ( up(pos_rc_m) | S ) ^ ni ) * ~Tr;
    return Grade(a_full, 4);
}

// Helper: compute the moving planes.
CGA computeMovingPlanes(const CGA &x, const CGA &R_rot_cen_ee, const CGA &pos_rc_epl, const CGA &S)
{
    CGA Tx = CGA(1.0f,0) + 0.5f * ((R_rot_cen_ee * x * ~R_rot_cen_ee) ^ ni);
    CGA planeFactor = ((S | pos_rc_epl) ^ ni).normalized();
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
Quaternionf computeElbowQuat(const CGA &pos_rc_epl, const CGA &pos_rc_elb)
{
    CGA z_hat = ( (-1) * down(pos_rc_elb) ).normalized();
    CGA vec_elb_epl = (down(pos_rc_epl) - down(pos_rc_elb)).normalized();
    CGA y_hat = ( cross(z_hat, vec_elb_epl) ).normalized();
    CGA x_hat = ( cross(y_hat, z_hat) ).normalized();
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
    float r_s_out, r_s_in, d;
    
    Vector3f pos_base_rot_cen;      // rotation centre position w.r.t. base
    Quaternionf quat_base_rot_cen; // rotation centre quaternion orientation w.r.t. base

    Vector3f pos_ubase_rot_cen;      // rotation centre position w.r.t. up-shifted base
    Quaternionf quat_ubase_rot_cen; // rotation centre quaternion orientation w.r.t. base

    // Relative poses w.r.t the rotation centre
    Vector3f pos_rot_cen_piv_0, pos_rot_cen_piv_1, pos_rot_cen_piv_2; // pivot positions
    Quaternionf quat_rot_cen_piv_0, quat_rot_cen_piv_1, quat_rot_cen_piv_2; // quaternions for pivot orientations

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
    // Geometric parameters
    const float &r_c,
    const float &ang_b_m,
    const float &r_b,
    const float &r_s_out,
    const float &d,
    const float &r_e,
    const float &r_s_in,
    // Principal axis of the 3-DoF spherical robot
    const CGA &e_principal,
    // Two basis vectors on the base plane
    const CGA &s_0, const CGA &s_1,
    // Target quaternion orientation for the end-plate
    const Quaternionf &target_quat_epl) 
{
    // These are the invariant parameters (can be defined as global variables)

    // Parameteros for the up-shifted base
    float r_c_prime = r_s_in * std::cos(ang_b_m * RM::d2r);
    float r_b_prime = r_s_in * std::sin(ang_b_m * RM::d2r);

    // Rotation centre position w.r.t. up-shifted base
    CGA pos_base_rot_cen = r_c * e_principal; // Rotation centre position w.r.t. up-shifted base
    Quaternionf quat_base_rot_cen = Quaternionf::Identity(); 
    
    // Rotation centre position w.r.t. up-shifted base
    CGA pos_ubase_rot_cen = r_c_prime * e_principal; // Rotation centre position w.r.t. up-shifted base
    Quaternionf quat_ubase_rot_cen = Quaternionf::Identity(); 

    // Compute s_2 based on s_0 and s_1 (G3 vector)
    CGA s_2 = (-1.0f) * s_0 + (-1.0f) * s_1;

    // Pivot positions (G3 vectors)
    CGA pos_rot_cen_piv_0 = r_b * s_0 - pos_base_rot_cen;
    CGA pos_rot_cen_piv_1 = r_b * s_1 - pos_base_rot_cen;
    CGA pos_rot_cen_piv_2 = r_b * s_2 - pos_base_rot_cen;

    // Motor positions (G3 vectors)
    CGA pos_rot_cen_m_0 = r_b_prime * s_0 - pos_ubase_rot_cen;
    CGA pos_rot_cen_m_1 = r_b_prime * s_1 - pos_ubase_rot_cen;
    CGA pos_rot_cen_m_2 = r_b_prime * s_2 - pos_ubase_rot_cen;

    // Construct the “fixed” spheres
    // Outer sphere S_out with radius of r_s_out about rot_cen (grade-4 sphere)
    // Inner sphere S_in with radius of r_s_in about rot_cen (grade-4 sphere)
    CGA S_out = sphere(r_s_out, cga_utils::R2G(Vector3f::Zero()), 4);
    CGA S_in = sphere(r_s_in, cga_utils::R2G(Vector3f::Zero()), 4);

    // Construct the “fixed” planes (a_0, a_1, a_2) (grade-4 planes)
    CGA a_0 = computeFixedPlanes(pos_rot_cen_m_0, S_in);
    CGA a_1 = computeFixedPlanes(pos_rot_cen_m_1, S_in);
    CGA a_2 = computeFixedPlanes(pos_rot_cen_m_2, S_in);

    
    // These are the variant parameters
    // Build the rotor R_rot_cen_ee from the target quaternion orientation
    Quaternionf target_quat_rot_cen_epl = quat_ubase_rot_cen.inverse() * target_quat_epl;
    Vector3f zyx_euler = RM::Quat2zyxEuler( target_quat_rot_cen_epl.cast<double>() ).cast<float>();
    CGA R_rot_cen_ee = zyxEuler2Rotor(zyx_euler);

    // Positions of the end-plate corners (G41 vectors)
    CGA offset_rot_cen_epl_0 = computeEndPlateCornerOffset(r_e, s_0, d, e_principal); // invariant
    CGA pos_rot_cen_epl_0 = computeEndPlateCornerPosition(offset_rot_cen_epl_0, R_rot_cen_ee);
    CGA offset_rot_cen_epl_1 = computeEndPlateCornerOffset(r_e, s_1, d, e_principal);
    CGA pos_rot_cen_epl_1 = computeEndPlateCornerPosition(offset_rot_cen_epl_1, R_rot_cen_ee);
    CGA offset_rot_cen_epl_2 = computeEndPlateCornerOffset(r_e, s_2, d, e_principal);
    CGA pos_rot_cen_epl_2 = computeEndPlateCornerPosition(offset_rot_cen_epl_2, R_rot_cen_ee);

    // End-plate centre (G41 vector)
    CGA pos_rot_cen_epl_c = computeEndPlateCentrePosition(pos_rot_cen_epl_0, R_rot_cen_ee, r_e, s_0);

    // End-point on the outer sphere (G41 vector)
    CGA pos_rot_cen_ept = up( r_s_in * down(pos_rot_cen_epl_c).normalized() );

    // Construct the moving planes (p_0, p_1, p_2) (grade-4 planes)
    CGA p_0 = computeMovingPlanes(offset_rot_cen_epl_0, R_rot_cen_ee, pos_rot_cen_epl_0, S_in);
    CGA p_1 = computeMovingPlanes(offset_rot_cen_epl_1, R_rot_cen_ee, pos_rot_cen_epl_1, S_in);
    CGA p_2 = computeMovingPlanes(offset_rot_cen_epl_2, R_rot_cen_ee, pos_rot_cen_epl_2, S_in);

    // Compute the elbow positions (G41 vectors)
    CGA pos_rot_cen_elb_0 = computeElbowPositions(a_0, p_0, S_in);
    CGA pos_rot_cen_elb_1 = computeElbowPositions(a_1, p_1, S_in);
    CGA pos_rot_cen_elb_2 = computeElbowPositions(a_2, p_2, S_in);

    // Quaternion orientation for each pivot
    Quaternionf quat_rot_cen_piv_0 = computeMotorInitalQuat(pos_base_rot_cen, pos_rot_cen_piv_0);
    Quaternionf quat_rot_cen_piv_1 = computeMotorInitalQuat(pos_base_rot_cen, pos_rot_cen_piv_1);
    Quaternionf quat_rot_cen_piv_2 = computeMotorInitalQuat(pos_base_rot_cen, pos_rot_cen_piv_2);

    // Quaternion orientation for each motor
    Quaternionf quat_rot_cen_m_0_i = computeMotorInitalQuat(pos_ubase_rot_cen, pos_rot_cen_m_0);
    Quaternionf quat_rot_cen_m_0 = computeMotorQuat(pos_rot_cen_epl_0, pos_rot_cen_m_0);
    Quaternionf quat_rot_cen_m_1_i = computeMotorInitalQuat(pos_ubase_rot_cen, pos_rot_cen_m_1);
    Quaternionf quat_rot_cen_m_1 = computeMotorQuat(pos_rot_cen_epl_1, pos_rot_cen_m_1);
    Quaternionf quat_rot_cen_m_2_i = computeMotorInitalQuat(pos_ubase_rot_cen, pos_rot_cen_m_2);
    Quaternionf quat_rot_cen_m_2 = computeMotorQuat(pos_rot_cen_epl_2, pos_rot_cen_m_2);
    
    // Quaternion orientation for each end-plate corner
    Quaternionf quat_rot_cen_epl_0 = computeEndPlateCornerQuat(pos_rot_cen_epl_0, pos_rot_cen_epl_c);
    Quaternionf quat_rot_cen_epl_1 = computeEndPlateCornerQuat(pos_rot_cen_epl_1, pos_rot_cen_epl_c);
    Quaternionf quat_rot_cen_epl_2 = computeEndPlateCornerQuat(pos_rot_cen_epl_2, pos_rot_cen_epl_c);
    Quaternionf quat_rot_cen_epl_c = target_quat_rot_cen_epl;
    Quaternionf quat_rot_cen_ept = target_quat_rot_cen_epl;

    // Quaternion orientation for each elbow
    Quaternionf quat_rot_cen_elb_0 = computeElbowQuat(pos_rot_cen_epl_0, pos_rot_cen_elb_0);
    Quaternionf quat_rot_cen_elb_1 = computeElbowQuat(pos_rot_cen_epl_1, pos_rot_cen_elb_1);
    Quaternionf quat_rot_cen_elb_2 = computeElbowQuat(pos_rot_cen_epl_2, pos_rot_cen_elb_2);

    // Compute the IK solutions (motor angles)
    // Compute angle between two motor orientations
    float th_0 = computeRelativeZAngle(quat_rot_cen_m_0_i, quat_rot_cen_m_0);
    float th_1 = computeRelativeZAngle(quat_rot_cen_m_1_i, quat_rot_cen_m_1);
    float th_2 = computeRelativeZAngle(quat_rot_cen_m_2_i, quat_rot_cen_m_2);


    // Assemble results:
    SPM3DoFIKResult result;

    // Spherical robot geometry parameters
    result.r_s_out = r_s_out;
    result.r_s_in = r_s_in;
    result.d = d;

    // Relative poses w.r.t the base centre frame
    result.pos_base_rot_cen = cga_utils::G2R(pos_base_rot_cen);
    result.quat_base_rot_cen = quat_base_rot_cen;

    // Relative poses w.r.t the up-shifted base centre frame
    result.pos_ubase_rot_cen = cga_utils::G2R(pos_ubase_rot_cen);
    result.quat_ubase_rot_cen = quat_ubase_rot_cen;

    // Relative poses w.r.t the rotation centre frame
    result.pos_rot_cen_piv_0 = cga_utils::G2R(pos_rot_cen_piv_0);
    result.pos_rot_cen_piv_1 = cga_utils::G2R(pos_rot_cen_piv_1);
    result.pos_rot_cen_piv_2 = cga_utils::G2R(pos_rot_cen_piv_2);
    result.quat_rot_cen_piv_0 = quat_rot_cen_piv_0;
    result.quat_rot_cen_piv_1 = quat_rot_cen_piv_1;
    result.quat_rot_cen_piv_2 = quat_rot_cen_piv_2;

    result.pos_rot_cen_m_0 = cga_utils::G2R(pos_rot_cen_m_0);
    result.pos_rot_cen_m_1 = cga_utils::G2R(pos_rot_cen_m_1);
    result.pos_rot_cen_m_2 = cga_utils::G2R(pos_rot_cen_m_2);
    result.quat_rot_cen_m_0_i = quat_rot_cen_m_0_i;
    result.quat_rot_cen_m_1_i = quat_rot_cen_m_1_i;
    result.quat_rot_cen_m_2_i = quat_rot_cen_m_2_i;
    result.quat_rot_cen_m_0 = quat_rot_cen_m_0;
    result.quat_rot_cen_m_1 = quat_rot_cen_m_1;
    result.quat_rot_cen_m_2 = quat_rot_cen_m_2;

    
    result.pos_rot_cen_epl_0 = cga_utils::G2R( down(pos_rot_cen_epl_0) );
    result.pos_rot_cen_epl_1 = cga_utils::G2R( down(pos_rot_cen_epl_1) );
    result.pos_rot_cen_epl_2 = cga_utils::G2R( down(pos_rot_cen_epl_2) );
    result.pos_rot_cen_epl_c = cga_utils::G2R( down(pos_rot_cen_epl_c) );
    result.pos_rot_cen_ept = cga_utils::G2R( down(pos_rot_cen_ept) );
    result.quat_rot_cen_epl_0 = quat_rot_cen_epl_0;
    result.quat_rot_cen_epl_1 = quat_rot_cen_epl_1;
    result.quat_rot_cen_epl_2 = quat_rot_cen_epl_2;
    result.quat_rot_cen_epl_c = quat_rot_cen_epl_c;
    result.quat_rot_cen_ept = quat_rot_cen_ept;

    result.pos_rot_cen_elb_0 = cga_utils::G2R( down(pos_rot_cen_elb_0) );
    result.pos_rot_cen_elb_1 = cga_utils::G2R( down(pos_rot_cen_elb_1) );
    result.pos_rot_cen_elb_2 = cga_utils::G2R( down(pos_rot_cen_elb_2) );
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

#endif // CGA_IK_SPM_3DOF_HPP