#ifndef CGA_IK_SPHERICAL_ROBOT_HPP
#define CGA_IK_SPHERICAL_ROBOT_HPP

#include <cmath>        // for std::acos, std::sqrt
#include <Eigen/Dense>
#include "cga_ik/cga_utils.hpp"
#include "robot_math_utils/robot_math_utils_v1_9.hpp"

using cga_utils::up;
using cga_utils::down;
using cga_utils::Grade;
using RM = RMUtils;

namespace cga_ik_spherical_robot {

// A small helper to compute the Euclidean angle between two 3D vectors.
inline float angleBetween(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
{
    float dotVal = v1.dot(v2);
    float norms  = v1.norm() * v2.norm();
    if (norms < 1e-9f) {
        return 0.0f; // handle degenerate case
    }
    float cosVal = dotVal / norms;
    // Clamp numerical error for safe acos:
    if (cosVal > 1.0f)  cosVal = 1.0f;
    if (cosVal < -1.0f) cosVal = -1.0f;
    return std::acos(cosVal); // returns angle in [0, π]
}

// This struct holds elbow data and the “motor angle” for a single motor’s solution.
struct MotorIKResult {
    CGA elbowPoint;  // conformal representation of the elbow
    float angleRad;       // angle at the motor (in radians)
};

// This struct holds the final output of the single-orientation IK.
struct SphericalRobotIKResult {
    float r_b, r_e;
    Eigen::Vector3f rb_s0, rb_s1, rb_s2;           // motor (pivot) positions
    Eigen::Vector3f rotation_centre;      // rotation centre of the robot
    Eigen::Vector3f y0, y1, y2;           // end-plate corners
    Eigen::Vector3f elb0, elb1, elb2;     // elbow positions
    Eigen::Vector3f endpoint;             // final “end point”
    float angle0, angle1, angle2;  // motor angles (in radians)
};

// A function that takes in the rotor angles (theta0, theta1, theta2) 
// and returns the full IK results of the 3-DoF spherical robot.
inline SphericalRobotIKResult computeSphericalRobotIK(
    float theta0, float theta1, float theta2,
    // geometry parameters:
    float r_b, float r_e
) 
{
    // Each motor axis is set in the e1-e3 plane, separated by 120 degrees.
    // (We used to do e1-e2, but now e2 <-> e3 are swapped.)
    CGA s0 = 1.0f * e1; 
    CGA s1 = (-0.5f * e1) + ((std::sqrt(3.0f) / 2.0f) * e3);
    CGA s2 = (-0.5f * e1) - ((std::sqrt(3.0f) / 2.0f) * e3);

    // Rotation centre: -(1/3)*r_b * e2
    CGA rotation_centre = -(1.0f / 3.0f) * r_b * e2;
    CGA up_rotation_centre = up(rotation_centre);

    // Motor sphere radius
    CGA temp = (r_b * s0) - rotation_centre;
    float dotVal = (temp | temp)[0];
    float l = std::sqrt(std::fabs(dotVal));
    float d = std::sqrt(l*l - r_e*r_e);

    // Create the “fixed” sphere S about rotation_centre
    CGA S = !( up_rotation_centre - 0.5f * (l*l) * ni );

    // Generate the “fixed” planes (a0, a1, a2)
    CGA Tr0 = CGA(1.0f,0) - 0.5f * ( ((-r_b * s0) + rotation_centre) ^ ni );
    CGA a0_full = Tr0 * ( ( up(r_b*s0) | S ) ^ ni ) * ~Tr0;
    CGA a0 = Grade(a0_full, 4);

    CGA Tr1 = CGA(1.0f,0) - 0.5f * ( ((-r_b * s1) + rotation_centre) ^ ni );
    CGA a1_full = Tr1 * ( ( up(r_b*s1) | S ) ^ ni ) * ~Tr1;
    CGA a1 = Grade(a1_full, 4);

    CGA Tr2 = CGA(1.0f,0) - 0.5f * ( ((-r_b * s2) + rotation_centre) ^ ni );
    CGA a2_full = Tr2 * ( ( up(r_b*s2) | S ) ^ ni ) * ~Tr2;
    CGA a2 = Grade(a2_full, 4);

    // Build the rotor R_ee
    CGA R2 = cga_utils::rot(e1 * e2, theta2);
    CGA R1 = cga_utils::rot(e3 * e2, theta1);
    CGA R0 = cga_utils::rot(e1 * e3, theta0);

    // Then multiply:
    CGA R_ee = (R2 * R1 * R0).normalized();

    // End-plate corners in R^3
    CGA x0 = r_e * s0 - d * e2;
    CGA x1 = r_e * s1 - d * e2;
    CGA x2 = r_e * s2 - d * e2;

    CGA y0 = up(R_ee * x0 * ~R_ee + rotation_centre);
    CGA y1 = up(R_ee * x1 * ~R_ee + rotation_centre);
    CGA y2 = up(R_ee * x2 * ~R_ee + rotation_centre);

    // Build translator for each corner
    CGA Tx0 = CGA(1.0f,0) + 0.5f * ((R_ee*x0*~R_ee) ^ ni);
    CGA planeFactor0 = ((S|y0) ^ ni).normalized();
    CGA p0_full = Tx0 * planeFactor0 * ~Tx0;
    CGA p0 = Grade(p0_full, 4);

    CGA Tx1 = CGA(1.0f,0) + 0.5f * ((R_ee*x1*~R_ee) ^ ni);
    CGA planeFactor1 = ((S|y1) ^ ni).normalized();
    CGA p1_full = Tx1 * planeFactor1 * ~Tx1;
    CGA p1 = Grade(p1_full, 4);

    CGA Tx2 = CGA(1.0f,0) + 0.5f * ((R_ee*x2*~R_ee) ^ ni);
    CGA planeFactor2 = ((S|y2) ^ ni).normalized();
    CGA p2_full = Tx2 * planeFactor2 * ~Tx2;
    CGA p2 = Grade(p2_full, 4);

    // Intersect the circles for the elbows
    CGA T0_full = (a0 & p0) & S;
    CGA T0_graded = Grade(T0_full, 2);
    CGA T0 = T0_graded.normalized();
    float T0sqVal = (T0*T0)[0];
    float invLenT0 = 1.0f / std::sqrt(std::fabs(T0sqVal));
    CGA elb0 = (CGA(1.0f,0) + T0 * invLenT0) * (T0 | ni);

    CGA T1_full = (a1 & p1) & S;
    CGA T1_graded = Grade(T1_full, 2);
    CGA T1 = T1_graded.normalized();
    float T1sqVal = (T1*T1)[0];
    float invLenT1 = 1.0f / std::sqrt(std::fabs(T1sqVal));
    CGA elb1 = (CGA(1.0f,0) + T1 * invLenT1) * (T1 | ni);

    CGA T2_full = (a2 & p2) & S;
    CGA T2_graded = Grade(T2_full, 2);
    CGA T2 = T2_graded.normalized();
    float T2sqVal = (T2*T2)[0];
    float invLenT2 = 1.0f / std::sqrt(std::fabs(T2sqVal));
    CGA elb2 = (CGA(1.0f,0) + T2 * invLenT2) * (T2 | ni);

    // End point
    CGA triProduct = (y0 ^ y1 ^ y2 ^ ni);
    CGA endpoint = (triProduct * up_rotation_centre) * triProduct;

    // Now compute the angles at each motor
    // We'll define a little helper to get the 3D vector from elbow -> rotation_centre
    // and from elbow -> motor pivot, then find angleBetween.
    Eigen::Vector3f rc3 = cga_utils::G2R( down(up(rotation_centre)) );

    auto getL1 = [&](const CGA &elb) -> Eigen::Vector3f {
        Eigen::Vector3f e = cga_utils::G2R( down(elb) );
        return rc3 - e; // elbow -> rotation_centre
    };
    auto getL2 = [&](const CGA &elb, const CGA &s_i) -> Eigen::Vector3f {
        CGA pivotCga = up(r_b*s_i + rotation_centre);
        Eigen::Vector3f pivot3 = cga_utils::G2R( down(pivotCga) );
        Eigen::Vector3f e = cga_utils::G2R( down(elb) );
        
        return pivot3 - e; // elbow -> motor pivot
    };

    float angle0 = angleBetween(getL1(elb0), getL2(elb0, s0));
    float angle1 = angleBetween(getL1(elb1), getL2(elb1, s1));
    float angle2 = angleBetween(getL1(elb2), getL2(elb2, s2));


    // Assemble results:
    SphericalRobotIKResult result;
    result.r_b = r_b;
    result.r_e = r_e;
    result.rb_s0 = cga_utils::G2R(r_b * s0);
    result.rb_s1 = cga_utils::G2R(r_b * s1);
    result.rb_s2 = cga_utils::G2R(r_b * s2);
    result.rotation_centre = cga_utils::G2R(rotation_centre);
    result.y0 = cga_utils::G2R( down(y0) );
    result.y1 = cga_utils::G2R( down(y1) );
    result.y2 = cga_utils::G2R( down(y2) );
    result.elb0 = cga_utils::G2R( down(elb0) );
    result.elb1 = cga_utils::G2R( down(elb1) );
    result.elb2 = cga_utils::G2R( down(elb2) );
    result.endpoint = cga_utils::G2R( down(endpoint) );
    result.angle0 = angle0;
    result.angle1 = angle1;
    result.angle2 = angle2;

    return result;
}

} // namespace cga_ik_spherical_robot

#endif // CGA_IK_SPHERICAL_ROBOT_HPP






// // Old code 
// #ifndef CGA_IK_SPHERICAL_ROBOT_HPP
// #define CGA_IK_SPHERICAL_ROBOT_HPP

// #include <cmath>        // for std::acos, std::sqrt
// #include <Eigen/Dense>
// #include "cga_ik/cga_utils.hpp"
// #include "robot_math_utils/robot_math_utils_v1_9.hpp"

// using cga_utils::up;
// using cga_utils::down;
// using cga_utils::Grade;
// using RM = RMUtils;

// namespace cga_ik_spherical_robot {

// // A small helper to compute the Euclidean angle between two 3D vectors.
// inline float angleBetween(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
// {
//     float dotVal = v1.dot(v2);
//     float norms  = v1.norm() * v2.norm();
//     if (norms < 1e-9f) {
//         return 0.0f; // handle degenerate case
//     }
//     float cosVal = dotVal / norms;
//     // Clamp numerical error for safe acos:
//     if (cosVal > 1.0f)  cosVal = 1.0f;
//     if (cosVal < -1.0f) cosVal = -1.0f;
//     return std::acos(cosVal); // returns angle in [0, π]
// }

// // This struct holds elbow data and the “motor angle” for a single motor’s solution.
// struct MotorIKResult {
//     CGA elbowPoint;  // conformal representation of the elbow
//     float angleRad;       // angle at the motor (in radians)
// };

// // This struct holds the final output of the single-orientation IK.
// struct SphericalRobotIKResult {
//     float r_b, r_e;
//     CGA rb_s0, rb_s1, rb_s2;           // motor (pivot) positions
//     CGA rotation_centre;      // rotation centre of the robot
//     CGA y0, y1, y2;           // end-plate corners
//     CGA elb0, elb1, elb2;     // elbow positions
//     CGA endpoint;             // final “end point”
//     float angle0, angle1, angle2;  // motor angles (in radians)
// };

// // A function that takes in the rotor angles (theta0, theta1, theta2) 
// // and returns the full IK results of the 3-DoF spherical robot.
// inline SphericalRobotIKResult computeSphericalRobotIK(
//     float theta0, float theta1, float theta2,
//     // geometry parameters:
//     float r_b, float r_e
// ) 
// {
//     // Each motor axis is set in the e1-e3 plane, separated by 120 degrees.
//     // (We used to do e1-e2, but now e2 <-> e3 are swapped.)
//     CGA s0 = 1.0f * e1; 
//     CGA s1 = (-0.5f * e1) + ((std::sqrt(3.0f) / 2.0f) * e3);
//     CGA s2 = (-0.5f * e1) - ((std::sqrt(3.0f) / 2.0f) * e3);

//     // Rotation centre: -(1/3)*r_b * e2
//     CGA rotation_centre = -(1.0f / 3.0f) * r_b * e2;
//     CGA up_rotation_centre = up(rotation_centre);

//     // Motor sphere radius
//     CGA temp = (r_b * s0) - rotation_centre;
//     float dotVal = (temp | temp)[0];
//     float l = std::sqrt(std::fabs(dotVal));
//     float d = std::sqrt(l*l - r_e*r_e);

//     // Create the “fixed” sphere S about rotation_centre
//     CGA S = !( up_rotation_centre - 0.5f * (l*l) * ni );

//     // Generate the “fixed” planes (a0, a1, a2)
//     CGA Tr0 = CGA(1.0f,0) - 0.5f * ( ((-r_b * s0) + rotation_centre) ^ ni );
//     CGA a0_full = Tr0 * ( ( up(r_b*s0) | S ) ^ ni ) * ~Tr0;
//     CGA a0 = Grade(a0_full, 4);

//     CGA Tr1 = CGA(1.0f,0) - 0.5f * ( ((-r_b * s1) + rotation_centre) ^ ni );
//     CGA a1_full = Tr1 * ( ( up(r_b*s1) | S ) ^ ni ) * ~Tr1;
//     CGA a1 = Grade(a1_full, 4);

//     CGA Tr2 = CGA(1.0f,0) - 0.5f * ( ((-r_b * s2) + rotation_centre) ^ ni );
//     CGA a2_full = Tr2 * ( ( up(r_b*s2) | S ) ^ ni ) * ~Tr2;
//     CGA a2 = Grade(a2_full, 4);

//     // Build the rotor R_ee
//     CGA R2 = cga_utils::rot(e1 * e2, theta2);
//     CGA R1 = cga_utils::rot(e3 * e2, theta1);
//     CGA R0 = cga_utils::rot(e1 * e3, theta0);

//     // Then multiply:
//     CGA R_ee = (R2 * R1 * R0).normalized();

//     // End-plate corners in R^3
//     CGA x0 = r_e * s0 - d * e2;
//     CGA x1 = r_e * s1 - d * e2;
//     CGA x2 = r_e * s2 - d * e2;

//     CGA y0 = up(R_ee * x0 * ~R_ee + rotation_centre);
//     CGA y1 = up(R_ee * x1 * ~R_ee + rotation_centre);
//     CGA y2 = up(R_ee * x2 * ~R_ee + rotation_centre);

//     // Build translator for each corner
//     CGA Tx0 = CGA(1.0f,0) + 0.5f * ((R_ee*x0*~R_ee) ^ ni);
//     CGA planeFactor0 = ((S|y0) ^ ni).normalized();
//     CGA p0_full = Tx0 * planeFactor0 * ~Tx0;
//     CGA p0 = Grade(p0_full, 4);

//     CGA Tx1 = CGA(1.0f,0) + 0.5f * ((R_ee*x1*~R_ee) ^ ni);
//     CGA planeFactor1 = ((S|y1) ^ ni).normalized();
//     CGA p1_full = Tx1 * planeFactor1 * ~Tx1;
//     CGA p1 = Grade(p1_full, 4);

//     CGA Tx2 = CGA(1.0f,0) + 0.5f * ((R_ee*x2*~R_ee) ^ ni);
//     CGA planeFactor2 = ((S|y2) ^ ni).normalized();
//     CGA p2_full = Tx2 * planeFactor2 * ~Tx2;
//     CGA p2 = Grade(p2_full, 4);

//     // Intersect the circles for the elbows
//     CGA T0_full = (a0 & p0) & S;
//     CGA T0_graded = Grade(T0_full, 2);
//     CGA T0 = T0_graded.normalized();
//     float T0sqVal = (T0*T0)[0];
//     float invLenT0 = 1.0f / std::sqrt(std::fabs(T0sqVal));
//     CGA elb0 = (CGA(1.0f,0) + T0 * invLenT0) * (T0 | ni);

//     CGA T1_full = (a1 & p1) & S;
//     CGA T1_graded = Grade(T1_full, 2);
//     CGA T1 = T1_graded.normalized();
//     float T1sqVal = (T1*T1)[0];
//     float invLenT1 = 1.0f / std::sqrt(std::fabs(T1sqVal));
//     CGA elb1 = (CGA(1.0f,0) + T1 * invLenT1) * (T1 | ni);

//     CGA T2_full = (a2 & p2) & S;
//     CGA T2_graded = Grade(T2_full, 2);
//     CGA T2 = T2_graded.normalized();
//     float T2sqVal = (T2*T2)[0];
//     float invLenT2 = 1.0f / std::sqrt(std::fabs(T2sqVal));
//     CGA elb2 = (CGA(1.0f,0) + T2 * invLenT2) * (T2 | ni);

//     // End point
//     CGA triProduct = (y0 ^ y1 ^ y2 ^ ni);
//     CGA endpoint = (triProduct * up_rotation_centre) * triProduct;

//     // Now compute the angles at each motor
//     // We'll define a little helper to get the 3D vector from elbow -> rotation_centre
//     // and from elbow -> motor pivot, then find angleBetween.
//     Eigen::Vector3f rc3 = cga_utils::G2R( down(up(rotation_centre)) );

//     auto getL1 = [&](const CGA &elb) -> Eigen::Vector3f {
//         Eigen::Vector3f e = cga_utils::G2R( down(elb) );
//         return rc3 - e; // elbow -> rotation_centre
//     };
//     auto getL2 = [&](const CGA &elb, const CGA &s_i) -> Eigen::Vector3f {
//         CGA pivotCga = up(r_b*s_i + rotation_centre);
//         Eigen::Vector3f pivot3 = cga_utils::G2R( down(pivotCga) );
//         Eigen::Vector3f e = cga_utils::G2R( down(elb) );
        
//         return pivot3 - e; // elbow -> motor pivot
//     };

//     float angle0 = angleBetween(getL1(elb0), getL2(elb0, s0));
//     float angle1 = angleBetween(getL1(elb1), getL2(elb1, s1));
//     float angle2 = angleBetween(getL1(elb2), getL2(elb2, s2));


//     // Assemble results:
//     SphericalRobotIKResult result;
//     result.r_b = r_b;
//     result.r_e = r_e;
//     result.rb_s0 = r_b * s0;
//     result.rb_s1 = r_b * s1;
//     result.rb_s2 = r_b * s2;
//     result.rotation_centre = rotation_centre;
//     result.y0 = y0;
//     result.y1 = y1;
//     result.y2 = y2;
//     result.elb0 = elb0;
//     result.elb1 = elb1;
//     result.elb2 = elb2;
//     result.endpoint = endpoint;
//     result.angle0 = angle0;
//     result.angle1 = angle1;
//     result.angle2 = angle2;

//     return result;
// }

// } // namespace cga_ik_spherical_robot

// #endif // CGA_IK_SPHERICAL_ROBOT_HPP






// // Old old code 
// #ifndef CGA_IK_SPHERICAL_ROBOT_HPP
// #define CGA_IK_SPHERICAL_ROBOT_HPP

// #include <cmath>        // for std::acos, std::sqrt
// #include <Eigen/Dense>
// #include "cga_ik/cga_utils.hpp"
// #include "robot_math_utils/robot_math_utils_v1_9.hpp"

// using cga_utils::up;
// using cga_utils::down;
// using cga_utils::Grade;
// using RM = RMUtils;

// namespace cga_ik_spherical_robot {

// // A small helper to compute the Euclidean angle between two 3D vectors.
// inline float angleBetween(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
// {
//     float dotVal = v1.dot(v2);
//     float norms  = v1.norm() * v2.norm();
//     if (norms < 1e-9f) {
//         return 0.0f; // handle degenerate case
//     }
//     float cosVal = dotVal / norms;
//     // Clamp numerical error for safe acos:
//     if (cosVal > 1.0f)  cosVal = 1.0f;
//     if (cosVal < -1.0f) cosVal = -1.0f;
//     return std::acos(cosVal); // returns angle in [0, π]
// }

// // This struct holds elbow data and the “motor angle” for a single motor’s solution.
// struct MotorIKResult {
//     CGA elbowPoint;  // conformal representation of the elbow
//     float angleRad;       // angle at the motor (in radians)
// };

// // This struct holds the final output of the single-orientation IK.
// struct SphericalRobotIKResult {
//     float r_b, r_e;
//     CGA rb_s0, rb_s1, rb_s2;           // motor (pivot) positions
//     CGA rotation_centre;      // rotation centre of the robot
//     CGA y0, y1, y2;           // end-plate corners
//     CGA elb0, elb1, elb2;     // elbow positions
//     CGA endpoint;             // final “end point”
//     float angle0, angle1, angle2;  // motor angles (in radians)
// };

// // A function that takes in the rotor angles (theta0, theta1, theta2) 
// // and returns the full IK results of the 3-DoF spherical robot.
// inline SphericalRobotIKResult computeSphericalRobotIK(
//     float theta0, float theta1, float theta2,
//     // geometry parameters:
//     float r_b, float r_e
// ) 
// {
//     // Each motor axis is set in the e1-e2 plane, separated by 120 degrees.
//     CGA s0 = 1.0f * e1; 
//     CGA s1 = (-0.5f * e1) + ((std::sqrt(3.0f) / 2.0f) * e2);
//     CGA s2 = (-0.5f * e1) - ((std::sqrt(3.0f) / 2.0f) * e2);

//     // Rotation centre: -(1/3)*r_b * e3
//     CGA rotation_centre = -(1.0f / 3.0f) * r_b * e3;
//     CGA up_rotation_centre = up(rotation_centre);

//     // Motor sphere radius
//     CGA temp = (r_b * s0) - rotation_centre;
//     float dotVal = (temp | temp)[0];
//     float l = std::sqrt(std::fabs(dotVal));
//     float d = std::sqrt(l*l - r_e*r_e);

//     // Create the “fixed” sphere S about rotation_centre
//     CGA S = !( up_rotation_centre - 0.5f * (l*l) * ni );

//     // Generate the “fixed” planes (a0, a1, a2)
//     CGA Tr0 = CGA(1.0f,0) - 0.5f * (((-r_b * s0) + rotation_centre) ^ ni);
//     CGA a0_full = Tr0 * ( ( up(r_b*s0) | S ) ^ ni ) * ~Tr0;
//     CGA a0 = Grade(a0_full, 4);

//     CGA Tr1 = CGA(1.0f,0) - 0.5f * (((-r_b * s1) + rotation_centre) ^ ni);
//     CGA a1_full = Tr1 * ( ( up(r_b*s1) | S ) ^ ni ) * ~Tr1;
//     CGA a1 = Grade(a1_full, 4);

//     CGA Tr2 = CGA(1.0f,0) - 0.5f * (((-r_b * s2) + rotation_centre) ^ ni);
//     CGA a2_full = Tr2 * ( ( up(r_b*s2) | S ) ^ ni ) * ~Tr2;
//     CGA a2 = Grade(a2_full, 4);

//     // Build the rotor R_ee = e^(theta2/2 * e13) * e^(theta1/2 * e23) * e^(theta0/2 * e12)
//     CGA R2 = cga_utils::rot(e13, theta2);
//     CGA R1 = cga_utils::rot(e23, theta1);
//     CGA R0 = cga_utils::rot(e12, theta0);
//     CGA R_ee = (R2 * R1 * R0).normalized();

//     // End-plate corners in R^3
//     CGA x0 = r_e * s0 - d * e3;
//     CGA x1 = r_e * s1 - d * e3;
//     CGA x2 = r_e * s2 - d * e3;

//     CGA y0 = up(R_ee * x0 * ~R_ee + rotation_centre);
//     CGA y1 = up(R_ee * x1 * ~R_ee + rotation_centre);
//     CGA y2 = up(R_ee * x2 * ~R_ee + rotation_centre);

//     // Build translator for each corner
//     CGA Tx0 = CGA(1.0f,0) + 0.5f * ((R_ee*x0*~R_ee) ^ ni);
//     CGA planeFactor0 = ((S|y0) ^ ni).normalized();
//     CGA p0_full = Tx0 * planeFactor0 * ~Tx0;
//     CGA p0 = Grade(p0_full, 4);

//     CGA Tx1 = CGA(1.0f,0) + 0.5f * ((R_ee*x1*~R_ee) ^ ni);
//     CGA planeFactor1 = ((S|y1) ^ ni).normalized();
//     CGA p1_full = Tx1 * planeFactor1 * ~Tx1;
//     CGA p1 = Grade(p1_full, 4);

//     CGA Tx2 = CGA(1.0f,0) + 0.5f * ((R_ee*x2*~R_ee) ^ ni);
//     CGA planeFactor2 = ((S|y2) ^ ni).normalized();
//     CGA p2_full = Tx2 * planeFactor2 * ~Tx2;
//     CGA p2 = Grade(p2_full, 4);

//     // Intersect the circles for the elbows
//     CGA T0_full = (a0 & p0) & S;
//     CGA T0_graded = Grade(T0_full, 2);
//     CGA T0 = T0_graded.normalized();
//     float T0sqVal = (T0*T0)[0];
//     float invLenT0 = 1.0f / std::sqrt(std::fabs(T0sqVal));
//     CGA elb0 = (CGA(1.0f,0) + T0 * invLenT0) * (T0 | ni);

//     CGA T1_full = (a1 & p1) & S;
//     CGA T1_graded = Grade(T1_full, 2);
//     CGA T1 = T1_graded.normalized();
//     float T1sqVal = (T1*T1)[0];
//     float invLenT1 = 1.0f / std::sqrt(std::fabs(T1sqVal));
//     CGA elb1 = (CGA(1.0f,0) + T1 * invLenT1) * (T1 | ni);

//     CGA T2_full = (a2 & p2) & S;
//     CGA T2_graded = Grade(T2_full, 2);
//     CGA T2 = T2_graded.normalized();
//     float T2sqVal = (T2*T2)[0];
//     float invLenT2 = 1.0f / std::sqrt(std::fabs(T2sqVal));
//     CGA elb2 = (CGA(1.0f,0) + T2 * invLenT2) * (T2 | ni);

//     // End point
//     CGA triProduct = (y0 ^ y1 ^ y2 ^ ni);
//     CGA endpoint = (triProduct * up_rotation_centre) * triProduct;

//     // Now compute the angles at each motor
//     // We'll define a little helper to get the 3D vector from elbow -> rotation_centre
//     // and from elbow -> motor pivot, then find angleBetween.
//     Eigen::Vector3f rc3 = cga_utils::G2R( down(up(rotation_centre)) );

//     auto getL1 = [&](const CGA &elb) -> Eigen::Vector3f {
//         Eigen::Vector3f e = cga_utils::G2R( down(elb) );
//         return rc3 - e; // elbow -> rotation_centre
//     };
//     auto getL2 = [&](const CGA &elb, const CGA &s_i) -> Eigen::Vector3f {
//         CGA pivotCga = up(r_b*s_i + rotation_centre);
//         Eigen::Vector3f pivot3 = cga_utils::G2R( down(pivotCga) );
//         Eigen::Vector3f e = cga_utils::G2R( down(elb) );
        
//         return pivot3 - e; // elbow -> motor pivot
//     };

//     float angle0 = angleBetween(getL1(elb0), getL2(elb0, s0));
//     float angle1 = angleBetween(getL1(elb1), getL2(elb1, s1));
//     float angle2 = angleBetween(getL1(elb2), getL2(elb2, s2));

//     // Assemble results:
//     SphericalRobotIKResult result;
//     result.r_b = r_b;
//     result.r_e = r_e;
//     result.rb_s0 = r_b * s0;
//     result.rb_s1 = r_b * s1;
//     result.rb_s2 = r_b * s2;
//     result.rotation_centre = rotation_centre;
//     result.y0 = y0;
//     result.y1 = y1;
//     result.y2 = y2;
//     result.elb0 = elb0;
//     result.elb1 = elb1;
//     result.elb2 = elb2;
//     result.endpoint = endpoint;
//     result.angle0 = angle0;
//     result.angle1 = angle1;
//     result.angle2 = angle2;

//     return result;
// }

// } // namespace cga_ik_spherical_robot

// #endif // CGA_IK_SPHERICAL_ROBOT_HPP
