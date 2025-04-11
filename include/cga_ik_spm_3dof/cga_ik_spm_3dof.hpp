#ifndef CGA_IK_SPM_3DOF_HPP
#define CGA_IK_SPM_3DOF_HPP

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "cga/cga_utils.hpp"
#include "robot_math_utils/robot_math_utils_v1_10.hpp"

using cga_utils::Grade;
using cga_utils::up;
using cga_utils::down;
using cga_utils::cross;
using cga_utils::zyxEuler2Rotor;
using cga_utils::sphere;
using RM = RMUtils;

namespace cga_ik_spm_3dof
{

/**
 * @brief  A struct that holds the final output of a 3-DoF SPM IK solution.
 */
struct SPM3DoFIKResult
{
    // IK solution (motor angles in [rad])
    float th_0, th_1, th_2;
};

struct SPM3DoFIKResetOrigin
{
    // Nominal quaternion orientation
    Quaternionf quat_ee_nom;

    // Nominal motor angles [rad]
    float th_0_nom, th_1_nom, th_2_nom;
    
};

/**
 * @brief A class that encapsulates the geometry and methods for a 3-DoF SPM IK (Spherical Parallel Manipulator).
 */
class CGAIKSPM3DoF
{
public:
    /**
     * @brief Constructor that sets up the “invariant” geometry parameters and precomputes the “fixed” spheres/planes, storing them as members.
     * @param r_c           rotation-centre offset
     * @param ang_b_m       angle between base-plane pivot and motor pivot [deg]
     * @param r_b           base radius
     * @param d             distance from rotation centre to end-plate plane
     * @param r_e           end-plate circle radius
     * @param r_s_piv       sphere radius for pivot
     * @param r_s_m         sphere radius for motor
     * @param r_s_elb       sphere radius for elbow
     * @param r_s_epl       sphere radius for end-plate
     * @param z_rot_cen_ee  offset for end-effector along principal axis
     * @param krl           sign for left or right elbow configuration (+1 or -1)
     * @param e_principal   principal axis (e.g. e3)
     * @param s_0           first basis vector on the base plane (e.g. e1)
     * @param s_1           second basis vector on the base plane (120 deg from s_0)
     */
    CGAIKSPM3DoF(
        float r_c,
        float ang_b_m,
        float r_b,
        float d,
        float r_e,
        float r_s_piv,
        float r_s_m,
        float r_s_elb,
        float r_s_epl,
        float z_rot_cen_ee,
        int   krl,
        const CGA &e_principal,
        const CGA &s_0,
        const CGA &s_1
    );

    /**
     * @brief Compute the IK for a given target orientation of the end-plate.
     * @param target_quat_ubase_epl The orientation of the end-plate w.r.t. the up-shifted base frame
     * @return SPM3DoFIKResult The IK solution containing keypoints positions, orientations, and motor angles for the 3-DoF SPM.
     */
    float r_s_piv() const { return r_s_piv_; }
    float r_s_m() const { return r_s_m_; }
    float r_s_elb() const { return r_s_elb_; }
    float r_s_epl() const { return r_s_epl_; }
    float z_rot_cen_ee() const { return z_rot_cen_ee_; }

    Vector3f pos_base_rot_cen() const { return cga_utils::G2R(pos_base_rot_cen_); }
    Quaternionf quat_base_rot_cen() const { return quat_base_rot_cen_; }

    // Pivot poses
    Vector3f pos_rot_cen_piv_0() const { return cga_utils::G2R(pos_rot_cen_piv_0_); }
    Vector3f pos_rot_cen_piv_1() const { return cga_utils::G2R(pos_rot_cen_piv_1_); }
    Vector3f pos_rot_cen_piv_2() const { return cga_utils::G2R(pos_rot_cen_piv_2_); }
    Quaternionf quat_rot_cen_piv_0() const { return quat_rot_cen_piv_0_; }
    Quaternionf quat_rot_cen_piv_1() const { return quat_rot_cen_piv_1_; }
    Quaternionf quat_rot_cen_piv_2() const { return quat_rot_cen_piv_2_; }

    // Motor poses
    Vector3f pos_rot_cen_m_0() const { return cga_utils::G2R(pos_rot_cen_m_0_); }
    Vector3f pos_rot_cen_m_1() const { return cga_utils::G2R(pos_rot_cen_m_1_); }
    Vector3f pos_rot_cen_m_2() const { return cga_utils::G2R(pos_rot_cen_m_2_); }
    Quaternionf quat_rot_cen_m_0() const { return quat_rot_cen_m_0_; }
    Quaternionf quat_rot_cen_m_1() const { return quat_rot_cen_m_1_; }
    Quaternionf quat_rot_cen_m_2() const { return quat_rot_cen_m_2_; }

    // End-plate corners/centre, sphere end-point, and end-effector poses
    Vector3f pos_rot_cen_epl_0() const { return cga_utils::G2R(pos_rot_cen_epl_0_); }
    Vector3f pos_rot_cen_epl_1() const { return cga_utils::G2R(pos_rot_cen_epl_1_); }
    Vector3f pos_rot_cen_epl_2() const { return cga_utils::G2R(pos_rot_cen_epl_2_); }
    Vector3f pos_rot_cen_epl_c() const { return cga_utils::G2R(pos_rot_cen_epl_c_); }
    Vector3f pos_rot_cen_ept() const { return cga_utils::G2R(pos_rot_cen_ept_); }
    Vector3f pos_rot_cen_ee() const { return cga_utils::G2R(pos_rot_cen_ee_); }
    Quaternionf quat_rot_cen_epl_0() const { return quat_rot_cen_epl_0_; }
    Quaternionf quat_rot_cen_epl_1() const { return quat_rot_cen_epl_1_; }
    Quaternionf quat_rot_cen_epl_2() const { return quat_rot_cen_epl_2_; }
    Quaternionf quat_rot_cen_epl_c() const { return quat_rot_cen_epl_c_; }
    Quaternionf quat_rot_cen_ept() const { return quat_rot_cen_ept_; }
    Quaternionf quat_rot_cen_ee() const { return quat_rot_cen_ee_; }

    // Elbow poses
    Vector3f pos_rot_cen_elb_0() const { return cga_utils::G2R(pos_rot_cen_elb_0_); }
    Vector3f pos_rot_cen_elb_1() const { return cga_utils::G2R(pos_rot_cen_elb_1_); }
    Vector3f pos_rot_cen_elb_2() const { return cga_utils::G2R(pos_rot_cen_elb_2_); }
    Quaternionf quat_rot_cen_elb_0() const { return quat_rot_cen_elb_0_; }
    Quaternionf quat_rot_cen_elb_1() const { return quat_rot_cen_elb_1_; }
    Quaternionf quat_rot_cen_elb_2() const { return quat_rot_cen_elb_2_; }

    // Main functions
    void solveInvariantGeometry();
    void getTargetRotor(const Quaternionf &target_quat_ubase_epl);
    void solveVariantGeometry();
    void solveAngles();
    SPM3DoFIKResetOrigin resetEEOrigin(const float &th_z_ee_init);
    SPM3DoFIKResult computeIK(const Quaternionf &target_quat_ubase_epl);

private:
    float r_c_;
    float ang_b_m_;         // in [deg]
    float r_b_;
    float d_;
    float r_e_;
    float r_s_piv_;
    float r_s_m_;
    float r_s_elb_;
    float r_s_epl_;
    float z_rot_cen_ee_;
    int   krl_;

    // Reset end-effector origin
    Quaternionf quat_ee_nom_; // Nominal quaternion orientation

    // CGA basis vectors:
    CGA e_principal_;
    CGA s_0_;
    CGA s_1_;
    CGA s_2_;  // s_2 = - (s_0 + s_1)

    // Precomputed “fixed” spheres about rotation centre:
    CGA S_;       ///< Unit sphere radius=1
    CGA S_m_;     ///< Motor sphere radius=r_s_m
    CGA S_elb_;   ///< Elbow sphere radius=r_s_elb
    CGA S_epl_;   ///< End-plate sphere radius=r_s_epl

    // Precomputed “base” and “up-shifted base” frames
    CGA pos_base_rot_cen_;     ///< CGA point for base->rot_cen offset
    Quaternionf quat_base_rot_cen_;
    CGA pos_ubase_rot_cen_;    ///< CGA point for ubase->rot_cen offset
    Quaternionf quat_ubase_rot_cen_;

    // Target orientation of the end-plate w.r.t. rot_cen
    Quaternionf target_quat_rot_cen_epl_;
    CGA R_rot_cen_ee_; 

    // Precomputed directions for the motor pivot lines (“fixed planes” geometry)
    CGA s_2_dir_;             ///< This is just s_2 but normalized if needed
    CGA dir_rot_cen_m_0_;
    CGA dir_rot_cen_m_1_;
    CGA dir_rot_cen_m_2_;

    CGA a_0_, a_1_, a_2_; // Fixed planes for motor_0, motor_1, motor_2

    CGA pos_rot_cen_piv_0_;  // Position for pivot_0
    CGA pos_rot_cen_piv_1_;  // Position for pivot_1
    CGA pos_rot_cen_piv_2_;  // Position for pivot_2
    Quaternionf quat_rot_cen_piv_0_; // Quaternion for pivot_0
    Quaternionf quat_rot_cen_piv_1_; // Quaternion for pivot_1
    Quaternionf quat_rot_cen_piv_2_; // Quaternion for pivot_2

    CGA pos_rot_cen_m_0_; // Position for motor_0
    CGA pos_rot_cen_m_1_; // Position for motor_1
    CGA pos_rot_cen_m_2_; // Position for motor_2
    Quaternionf quat_rot_cen_m_0_i_; // Initial quaternion for motor_0
    Quaternionf quat_rot_cen_m_1_i_; // Initial quaternion for motor_1
    Quaternionf quat_rot_cen_m_2_i_; // Initial quaternion for motor_2
    Quaternionf quat_rot_cen_m_0_; // Quaternion for motor_0
    Quaternionf quat_rot_cen_m_1_; // Quaternion for motor_1
    Quaternionf quat_rot_cen_m_2_; // Quaternion for motor_2

    CGA dir_offset_rot_cen_epl_0_, dir_rot_cen_epl_0_, pos_rot_cen_epl_0_;
    CGA dir_offset_rot_cen_epl_1_, dir_rot_cen_epl_1_, pos_rot_cen_epl_1_;
    CGA dir_offset_rot_cen_epl_2_, dir_rot_cen_epl_2_, pos_rot_cen_epl_2_;
    CGA pos_rot_cen_epl_c_, pos_rot_cen_ept_, pos_rot_cen_ee_;

    CGA p_0_, p_1_, p_2_; // Moving planes for motor_0, motor_1, motor_2
    CGA pos_rot_cen_elb_0_, pos_rot_cen_elb_1_, pos_rot_cen_elb_2_; // Elbow positions

    Quaternionf quat_rot_cen_epl_0_, quat_rot_cen_epl_1_, quat_rot_cen_epl_2_; // End-plate corner quaternions
    Quaternionf quat_rot_cen_epl_c_, quat_rot_cen_ept_, quat_rot_cen_ee_;

    Quaternionf quat_rot_cen_elb_0_, quat_rot_cen_elb_1_, quat_rot_cen_elb_2_; // Elbow quaternions

    // IK solution (motor angles)
    float th_0_, th_1_, th_2_; // Motor angles in [rad]

    inline Quaternionf computeQuatFromBasis(const CGA &x_hat, const CGA &y_hat, const CGA &z_hat) const;
    inline Quaternionf computeMotorInitalQuat(const CGA &rc, const CGA &pos_rc_m) const;
    inline Quaternionf computeMotorQuat(const CGA &pos_rc_epl, const CGA &pos_rc_m) const;

    inline CGA computeEndPlateCornerOffset(float r_e, const CGA &s, float d, const CGA &e_p) const;
    inline CGA computeEndPlateCornerOffsetDirection(float r_e, const CGA &s, float d, const CGA &e_p) const;
    inline CGA computeEndPlateCornerDirection(const CGA &x, const CGA &R_rot_cen_ee) const;
    inline CGA computeEndPlateCentrePosition(const CGA &pos_rot_cen_epl_0, const CGA &R_rot_cen_ee,
                                                 float r_e, const CGA &s_0) const;

    inline Quaternionf computeEndPlateCornerQuat(const CGA &pos_rc_epl, const CGA &pos_rc_epl_c) const;

    inline CGA computeFixedPlanes(const CGA &pos_rc_m, const CGA &S) const;
    inline CGA computeMovingPlanes(const CGA &x, const CGA &R_rot_cen_ee, const CGA &pos_rc_epl, const CGA &S) const;
    inline CGA computeElbowPositions(const CGA &a, const CGA &p, const CGA &S, int krl) const;

    inline Quaternionf computeElbowQuat(const CGA &pos_rc_epl, const CGA &pos_rc_elb) const;
    inline float computeRelativeZAngle(const Quaternionf & quat_0_1, const Quaternionf & quat_0_2) const;
};

//------------------------------------------------------------------------------------------------
// Implementation
//------------------------------------------------------------------------------------------------
inline CGAIKSPM3DoF::CGAIKSPM3DoF(
    float r_c,
    float ang_b_m,
    float r_b,
    float d,
    float r_e,
    float r_s_piv,
    float r_s_m,
    float r_s_elb,
    float r_s_epl,
    float z_rot_cen_ee,
    int   krl,
    const CGA &e_principal,
    const CGA &s_0,
    const CGA &s_1
)
: r_c_(r_c),
  ang_b_m_(ang_b_m),
  r_b_(r_b),
  d_(d),
  r_e_(r_e),
  r_s_piv_(r_s_piv),
  r_s_m_(r_s_m),
  r_s_elb_(r_s_elb),
  r_s_epl_(r_s_epl),
  z_rot_cen_ee_(z_rot_cen_ee),
  krl_(krl),
  e_principal_(e_principal),
  s_0_(s_0),
  s_1_(s_1)
{
    // Reset end-effector origin
    quat_ee_nom_ = Quaternionf::Identity();
    // Solve for the invariant poses of the robot
    solveInvariantGeometry();
    // IK target orientaion of the end-effector w.r.t. the rotation centre
    getTargetRotor(Quaternionf::Identity());
    // Solve for the variant poses of the robot
    solveVariantGeometry();
    // Solve for the motor angles [rad]
    solveAngles();
}


//-----------------------------------------------------------------------------------
// All helper methods inlined below
//-----------------------------------------------------------------------------------
inline Quaternionf CGAIKSPM3DoF::computeQuatFromBasis(const CGA &x_hat,
                                                             const CGA &y_hat,
                                                             const CGA &z_hat) const
{
    Matrix3f RMat;
    // row 0 => dot with e1
    RMat << 
      float((x_hat | e1)[0]), float((y_hat | e1)[0]), float((z_hat | e1)[0]),
      float((x_hat | e2)[0]), float((y_hat | e2)[0]), float((z_hat | e2)[0]),
      float((x_hat | e3)[0]), float((y_hat | e3)[0]), float((z_hat | e3)[0]);

    return Quaternionf(RMat);
}

inline Quaternionf CGAIKSPM3DoF::computeMotorInitalQuat(const CGA &rc,
                                                               const CGA &pos_rc_m) const
{
    CGA z_hat = ((-1) * pos_rc_m).normalized();
    CGA dir_rc = rc.normalized();
    CGA x_hat_i = ( cross(z_hat, dir_rc) ).normalized();
    CGA y_hat_i = ( cross(z_hat, x_hat_i) ).normalized(); 
    return computeQuatFromBasis(x_hat_i, y_hat_i, z_hat);
}

inline Quaternionf CGAIKSPM3DoF::computeMotorQuat(const CGA &pos_rc_epl,
                                                         const CGA &pos_rc_m) const
{
    CGA z_hat = ((-1) * pos_rc_m).normalized();
    CGA dir_m_epl = ( pos_rc_epl - pos_rc_m ).normalized();
    CGA x_hat = ( cross(z_hat, dir_m_epl) ).normalized();
    CGA y_hat = ( cross(z_hat, x_hat) ).normalized();
    return computeQuatFromBasis(x_hat, y_hat, z_hat);
}

inline CGA CGAIKSPM3DoF::computeEndPlateCornerOffset(float r_e,
                                                          const CGA &s,
                                                          float d,
                                                          const CGA &e_p) const
{
    // The offset from the rotation centre to the end-plate corners
    return r_e * s + d * e_p;
}

inline CGA CGAIKSPM3DoF::computeEndPlateCornerOffsetDirection(float r_e,
                                                                   const CGA &s,
                                                                   float d,
                                                                   const CGA &e_p) const
{
    // Just normalized offset
    CGA offset = computeEndPlateCornerOffset(r_e, s, d, e_p);
    return offset.normalized();
}

inline CGA CGAIKSPM3DoF::computeEndPlateCornerDirection(const CGA &x,
                                                             const CGA &R_rot_cen_ee) const
{
    return R_rot_cen_ee * x * ~R_rot_cen_ee;
}

inline CGA CGAIKSPM3DoF::computeEndPlateCentrePosition(const CGA &pos_rot_cen_epl_0,
                                                            const CGA &R_rot_cen_ee,
                                                            float r_e,
                                                            const CGA &s_0) const
{
    CGA pos_epl_0_epl_c =  R_rot_cen_ee * (-r_e * s_0) * ~R_rot_cen_ee;
    CGA Tr_epl_0_epl_c = cga_utils::trans( pos_epl_0_epl_c );
    return down( Tr_epl_0_epl_c * up(pos_rot_cen_epl_0) * ~Tr_epl_0_epl_c );
}

inline Quaternionf CGAIKSPM3DoF::computeEndPlateCornerQuat(const CGA &pos_rc_epl,
                                                                  const CGA &pos_rc_epl_c) const
{
    CGA z_hat = ( (-1) * pos_rc_epl ).normalized();
    CGA dir_epl_epl_c = (pos_rc_epl_c - pos_rc_epl).normalized();
    CGA x_hat = ( cross(z_hat, dir_epl_epl_c) ).normalized();
    CGA y_hat = ( cross(z_hat, x_hat) ).normalized();
    return computeQuatFromBasis(x_hat, y_hat, z_hat);
}

inline CGA CGAIKSPM3DoF::computeFixedPlanes(const CGA &pos_rc_m, const CGA &S) const
{
    CGA Tr = CGA(1.0f, 0) - 0.5f * ( ((-1) * pos_rc_m) ^ ni );
    CGA a_full = Tr * ( ( up(pos_rc_m) | S ) ^ ni ) * ~Tr;
    return Grade(a_full, 4);
}

inline CGA CGAIKSPM3DoF::computeMovingPlanes(const CGA &x,
                                                  const CGA &R_rot_cen_ee,
                                                  const CGA &pos_rc_epl,
                                                  const CGA &S) const
{
    CGA Tx = CGA(1.0f,0) + 0.5f * ((R_rot_cen_ee * x * ~R_rot_cen_ee) ^ ni);
    CGA planeFactor = ( ( S | up(pos_rc_epl) ) ^ ni ).normalized();
    CGA p_full = Tx * planeFactor * ~Tx;
    return Grade(p_full, 4);
}

inline CGA CGAIKSPM3DoF::computeElbowPositions(const CGA &a,
                                                    const CGA &p,
                                                    const CGA &S,
                                                    int krl) const
{
    // Intersect the circles
    CGA T_full = (a & p) & S;
    CGA T_graded = Grade(T_full, 2);
    CGA T = T_graded.normalized();

    float invLen_T = 1.0f / std::sqrt((T * T)[0]);
    CGA elb_g41 = (CGA(1.0f,0) + krl * T * invLen_T) * (T | ni);
    return down(elb_g41);
}

inline Quaternionf CGAIKSPM3DoF::computeElbowQuat(const CGA &pos_rc_epl,
                                                         const CGA &pos_rc_elb) const
{
    CGA z_hat = ( (-1) * pos_rc_elb ).normalized();
    CGA dir_elb_epl = (pos_rc_epl - pos_rc_elb).normalized();
    CGA y_hat = ( cross(z_hat, dir_elb_epl) ).normalized();
    CGA x_hat = ( cross(y_hat, z_hat) ).normalized();
    return computeQuatFromBasis(x_hat, y_hat, z_hat);
}

inline float CGAIKSPM3DoF::computeRelativeZAngle(const Quaternionf &quat_0_1,
                                                 const Quaternionf &quat_0_2) const
{
    Quaterniond dq_1_2 = RM::InvQuat( quat_0_1.cast<double>() ) * quat_0_2.cast<double>();
    Vector4d axis_ang_1_2 = RM::AxisAng3( RM::Quat2so3(dq_1_2) );
    // sign depends on the z-component
    return (axis_ang_1_2(2) > 0.0) ? float(axis_ang_1_2(3)) : float(-axis_ang_1_2(3));
}



// Main functions
inline void CGAIKSPM3DoF::solveInvariantGeometry()
{
    // Compute s_2
    s_2_ = (-1.0f) * s_0_ + (-1.0f) * s_1_;

    // Rotation centre position w.r.t base
    pos_base_rot_cen_   = r_c_ * e_principal_;  // G3 vector
    quat_base_rot_cen_  = Quaternionf::Identity();

    // Rotation centre position w.r.t up-shifted base
    float r_c_prime = r_s_epl_ * std::cos(ang_b_m_ * RM::d2r);
    pos_ubase_rot_cen_  = r_c_prime * e_principal_; // G3 vector
    quat_ubase_rot_cen_ = Quaternionf::Identity();

    // Build the “fixed” spheres about the rotation centre
    // - Unit sphere S with radius=1
    // - Motor sphere S_m with radius=r_s_m
    // - Elbow sphere S_elb with radius=r_s_elb
    // - End-plate sphere S_epl with radius=r_s_epl
    S_     = sphere(1.0f,       down(no), 4);
    S_m_   = sphere(r_s_m_,     down(no), 4);
    S_elb_ = sphere(r_s_elb_,   down(no), 4);
    S_epl_ = sphere(r_s_epl_,   down(no), 4);

    // Motor ray direction for each pivot and motor
    // dir_rot_cen_m_i = (r_b * s_i - pos_base_rot_cen_).normalized();
    dir_rot_cen_m_0_ = (r_b_ * s_0_ - pos_base_rot_cen_).normalized();
    dir_rot_cen_m_1_ = (r_b_ * s_1_ - pos_base_rot_cen_).normalized();
    dir_rot_cen_m_2_ = (r_b_ * s_2_ - pos_base_rot_cen_).normalized();

    // Construct the “fixed” planes a_0, a_1, a_2
    a_0_ = computeFixedPlanes(dir_rot_cen_m_0_, S_);
    a_1_ = computeFixedPlanes(dir_rot_cen_m_1_, S_);
    a_2_ = computeFixedPlanes(dir_rot_cen_m_2_, S_);

    // Pivot poses
    pos_rot_cen_piv_0_ = r_s_piv_ * dir_rot_cen_m_0_;
    pos_rot_cen_piv_1_ = r_s_piv_ * dir_rot_cen_m_1_;
    pos_rot_cen_piv_2_ = r_s_piv_ * dir_rot_cen_m_2_;

    quat_rot_cen_piv_0_ = computeMotorInitalQuat(pos_base_rot_cen_, pos_rot_cen_piv_0_);
    quat_rot_cen_piv_1_ = computeMotorInitalQuat(pos_base_rot_cen_, pos_rot_cen_piv_1_);
    quat_rot_cen_piv_2_ = computeMotorInitalQuat(pos_base_rot_cen_, pos_rot_cen_piv_2_);

    // Motor position
    pos_rot_cen_m_0_ = r_s_m_ * dir_rot_cen_m_0_;
    pos_rot_cen_m_1_ = r_s_m_ * dir_rot_cen_m_1_;
    pos_rot_cen_m_2_ = r_s_m_ * dir_rot_cen_m_2_;

    // Motor quaternion
    // Initial
    quat_rot_cen_m_0_i_ = computeMotorInitalQuat(pos_ubase_rot_cen_, pos_rot_cen_m_0_);
    quat_rot_cen_m_1_i_ = computeMotorInitalQuat(pos_ubase_rot_cen_, pos_rot_cen_m_1_);
    quat_rot_cen_m_2_i_ = computeMotorInitalQuat(pos_ubase_rot_cen_, pos_rot_cen_m_2_);
}

inline void CGAIKSPM3DoF::getTargetRotor(const Quaternionf &target_quat_ubase_epl)
{
    // IK target rotor orientaion of the end-effector w.r.t. the rotation centre
    target_quat_rot_cen_epl_ = quat_ubase_rot_cen_.inverse() * target_quat_ubase_epl;
    Vector3f zyx_euler = RM::Quat2zyxEuler( target_quat_rot_cen_epl_.cast<double>() ).cast<float>();
    R_rot_cen_ee_ = zyxEuler2Rotor(zyx_euler);
}

inline void CGAIKSPM3DoF::solveVariantGeometry()
{
    // Positions of the end-plate corners that lie on S_epl_ (the end-plate sphere).
    // i.e. we do offset = r_e_ * s_i + d_ * e_principal_, then scale to r_s_epl_ in final, 
    // plus the rotor for orientation. 
    // We'll do each corner 0,1,2 similarly.
    // Corner 0:
    dir_offset_rot_cen_epl_0_ = computeEndPlateCornerOffsetDirection(r_e_, s_0_, d_, e_principal_);
    dir_rot_cen_epl_0_ = computeEndPlateCornerDirection(dir_offset_rot_cen_epl_0_, R_rot_cen_ee_);
    pos_rot_cen_epl_0_ = r_s_epl_ * dir_rot_cen_epl_0_;
    // Corner 1:
    dir_offset_rot_cen_epl_1_ = computeEndPlateCornerOffsetDirection(r_e_, s_1_, d_, e_principal_);
    dir_rot_cen_epl_1_ = computeEndPlateCornerDirection(dir_offset_rot_cen_epl_1_, R_rot_cen_ee_);
    pos_rot_cen_epl_1_ = r_s_epl_ * dir_rot_cen_epl_1_;
    // Corner 2:
    dir_offset_rot_cen_epl_2_ = computeEndPlateCornerOffsetDirection(r_e_, s_2_, d_, e_principal_);
    dir_rot_cen_epl_2_ = computeEndPlateCornerDirection(dir_offset_rot_cen_epl_2_, R_rot_cen_ee_);
    pos_rot_cen_epl_2_ = r_s_epl_ * dir_rot_cen_epl_2_;

    // End-plate centre 
    pos_rot_cen_epl_c_ = computeEndPlateCentrePosition(pos_rot_cen_epl_0_, R_rot_cen_ee_, r_e_, s_0_);

    // End-point on the S_epl
    pos_rot_cen_ept_ = r_s_epl_ * (pos_rot_cen_epl_c_).normalized();

    // End-effector position
    pos_rot_cen_ee_ = z_rot_cen_ee_ * (pos_rot_cen_epl_c_).normalized();

    // Now we build the moving planes p_0_, p_1_, p_2_
    // using the previously computed “a_i_” or "S_elb_" is for the elbow but let's see:
    // Actually, a_i_ are the "fixed planes" for the motors, so we do new "p_i" for each corner
    p_0_ = computeMovingPlanes(dir_offset_rot_cen_epl_0_, R_rot_cen_ee_, dir_rot_cen_epl_0_, S_);
    p_1_ = computeMovingPlanes(dir_offset_rot_cen_epl_1_, R_rot_cen_ee_, dir_rot_cen_epl_1_, S_);
    p_2_ = computeMovingPlanes(dir_offset_rot_cen_epl_2_, R_rot_cen_ee_, dir_rot_cen_epl_2_, S_);

    // Compute the elbow positions on S_elb_
    pos_rot_cen_elb_0_ = computeElbowPositions(a_0_, p_0_, S_elb_, krl_);
    pos_rot_cen_elb_1_ = computeElbowPositions(a_1_, p_1_, S_elb_, krl_);
    pos_rot_cen_elb_2_ = computeElbowPositions(a_2_, p_2_, S_elb_, krl_);

    // motor quaternions
    quat_rot_cen_m_0_ = computeMotorQuat(pos_rot_cen_epl_0_, pos_rot_cen_m_0_);
    quat_rot_cen_m_1_ = computeMotorQuat(pos_rot_cen_epl_1_, pos_rot_cen_m_1_);
    quat_rot_cen_m_2_ = computeMotorQuat(pos_rot_cen_epl_2_, pos_rot_cen_m_2_);

    // end-plate corner quaternions
    quat_rot_cen_epl_0_ = computeEndPlateCornerQuat(pos_rot_cen_epl_0_, pos_rot_cen_epl_c_);
    quat_rot_cen_epl_1_ = computeEndPlateCornerQuat(pos_rot_cen_epl_1_, pos_rot_cen_epl_c_);
    quat_rot_cen_epl_2_ = computeEndPlateCornerQuat(pos_rot_cen_epl_2_, pos_rot_cen_epl_c_);

    quat_rot_cen_epl_c_ = target_quat_rot_cen_epl_;
    quat_rot_cen_ept_   = target_quat_rot_cen_epl_;

    // End-effector orientation
    quat_rot_cen_ee_    = target_quat_rot_cen_epl_;

    // Elbow orientation
    quat_rot_cen_elb_0_ = computeElbowQuat(pos_rot_cen_epl_0_, pos_rot_cen_elb_0_);
    quat_rot_cen_elb_1_ = computeElbowQuat(pos_rot_cen_epl_1_, pos_rot_cen_elb_1_);
    quat_rot_cen_elb_2_ = computeElbowQuat(pos_rot_cen_epl_2_, pos_rot_cen_elb_2_);
}

inline void CGAIKSPM3DoF::solveAngles()
{
    // Motor angles
    th_0_ = computeRelativeZAngle(quat_rot_cen_m_0_i_, quat_rot_cen_m_0_);
    th_1_ = computeRelativeZAngle(quat_rot_cen_m_1_i_, quat_rot_cen_m_1_);
    th_2_ = computeRelativeZAngle(quat_rot_cen_m_2_i_, quat_rot_cen_m_2_);
}

inline SPM3DoFIKResetOrigin CGAIKSPM3DoF::resetEEOrigin(const float &th_z_ee_init)
{
    // Nominal quaternion orientation
    quat_ee_nom_ = RM::Quatz(th_z_ee_init * RM::d2r).cast<float>();
    // IK target orientaion of the end-effector w.r.t. the rotation centre
    getTargetRotor(quat_ee_nom_);
    // Solve for the variant poses of the robot
    solveVariantGeometry();
    // Solve for the motor angles [rad]
    solveAngles();

    SPM3DoFIKResetOrigin result;
    result.quat_ee_nom = quat_ee_nom_;
    result.th_0_nom = th_0_;
    result.th_1_nom = th_1_;
    result.th_2_nom = th_2_;
    return result;
}

inline SPM3DoFIKResult CGAIKSPM3DoF::computeIK(const Quaternionf &target_quat_ubase_epl)
{
    // IK target orientaion of the end-effector w.r.t. the rotation centre
    getTargetRotor(target_quat_ubase_epl);
    // Solve for the variant poses of the robot
    solveVariantGeometry();
    // Solve for the motor angles [rad]
    solveAngles();

    SPM3DoFIKResult result;
    result.th_0 = th_0_;
    result.th_1 = th_1_;
    result.th_2 = th_2_;
    return result;
}

} // end namespace cga_ik_spm_3dof

#endif // CGA_IK_SPM_3DOF_HPP
