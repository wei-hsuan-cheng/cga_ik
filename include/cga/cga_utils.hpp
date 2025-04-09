#ifndef CGA_UTILS_HPP
#define CGA_UTILS_HPP

#include "cga/cga.hpp"
#include <Eigen/Dense>
#include <iostream>

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

using cga::CGA;

namespace cga_utils {
    //-----------------------------------------------------------------
    // R2G: Convert a 3D vector (an R3 vector) into a G3 vector.
    // In our conformal model, a G3 vector is represented as a CGA with 
    // its scalar part zero and its grade‑1 components in indices 1–5.
    // Now the input is an Eigen::Vector3f.
    //-----------------------------------------------------------------
    inline CGA R2G(const Eigen::Vector3f &v) {
        CGA res;
        res[0] = 0.0f;   // scalar part
        res[1] = v(0);   // e1
        res[2] = v(1);   // e2
        res[3] = v(2);   // e3
        // In a pure G3 vector, e4 and e5 are not used:
        res[4] = 0.0f;
        res[5] = 0.0f;
        return res;
    }

    // G2R
    inline Eigen::Vector3f G2R(const CGA &vec_G3) {
        return Eigen::Vector3f(vec_G3[1], vec_G3[2], vec_G3[3]);
    }

    //-----------------------------------------------------------------
    // Grade projection function.
    // Extracts only the components of a multivector that are of grade k.
    // (For demonstration, we hard-code the indices corresponding to each grade.)
    //-----------------------------------------------------------------
    inline CGA Grade(const CGA &m, int k) {
        CGA res;
        if (k == 0) {
            res = CGA(m[0], 0);
        } else if (k == 1) {
            res = CGA(m[1], 1) + CGA(m[2], 2) + CGA(m[3], 3)
                + CGA(m[4], 4) + CGA(m[5], 5);
        } else if (k == 2) {
            for (int i = 6; i <= 15; i++) {
                res = res + CGA(m[i], i);
            }
        } else if (k == 3) {
            for (int i = 16; i <= 25; i++) {
                res = res + CGA(m[i], i);
            }
        } else if (k == 4) {
            for (int i = 26; i <= 30; i++) {
                res = res + CGA(m[i], i);
            }
        } else if (k == 5) {
            res = CGA(m[31], 31);
        } else {
            std::cout << "Grade error (k must be between 0 and 5)" << std::endl;
        }
        return res;
    }

    //-----------------------------------------------------------------
    // Rotor function.
    // Given a bivector and an angle (in radians), returns the rotor for the rotation.
    // The rotor is given by:
    //    rotor = cos(angle/2) - sin(angle/2) * (bivector.normalized())
    //-----------------------------------------------------------------
    inline CGA rot(const CGA &bivector, float angle) {
        return CGA(cos(angle / 2), 0) + (- sin(angle / 2)) * bivector.normalized();
    }

    //-----------------------------------------------------------------
    // Translator function.
    // Given a translation vector (a grade‑1 element), returns the translator.
    // In the conformal model, the translator is typically given by:
    //    translator = 1 - 0.5 * (translation_vector * ni)
    // where ni is the "infinity" vector.
    //-----------------------------------------------------------------
    inline CGA trans(const CGA &trans_vec) {
        return CGA(1.0f, 0) - 0.5f * (trans_vec * ni);
    }

    //-----------------------------------------------------------------
    // Up function.
    // Converts a point from Euclidean coordinates (R3) into a conformal point in G41.
    // Two overloads are provided:
    //   (a) one that accepts three floats,
    //   (b) one that accepts a G3 vector (with only grade‑1 components)
    //-----------------------------------------------------------------
    inline CGA up(float x, float y, float z) {
        float d = x*x + y*y + z*z;
        return x * e1 + y * e2 + z * e3 + 0.5f * d * ni + no;
    }

    inline CGA up(const CGA &vec_G3) {
        // Assumes vec_G3 has scalar part zero and only grade‑1 components (indices 1–3)
        return vec_G3 + 0.5f * (vec_G3 * vec_G3) * ni + no;
    }

    //-----------------------------------------------------------------
    // Down function.
    // Projects a conformal point in G41 back to a Euclidean vector (G3 vector).
    // The formula used is:
    //   down(vec_G41) = (vec_G41 ^ (no ^ ni)) * (no ^ ni) * ((-1)*vec_G41 | ni).inverse()
    //-----------------------------------------------------------------
    inline CGA down(const CGA &vec_G41) {
        return (vec_G41 ^ (no ^ ni)) * (no ^ ni) * (((-1.0f) * vec_G41 | ni).inverse());
    }

    //-----------------------------------------------------------------
    // Cross product in GA
    //-----------------------------------------------------------------
    // inline CGA cross(const CGA &a, const CGA &b) {
    //     return (-1) * e123 * (a ^ b); // Take the dual of the wedge product
    // }

    inline CGA cross(const CGA &a, const CGA &b) {
        CGA wedge = a ^ b;
        return (-1) * e123 * wedge; // Take the dual of the wedge product
    }

    //-----------------------------------------------------------------
    // Conversions between different SO(3) representations.
    // zyx_euler <-> CGA rotor
    //-----------------------------------------------------------------
    inline CGA zyxEuler2Rotor(const Vector3f &zyx_euler) {
        CGA R_z = cga_utils::rot(e1 * e2, zyx_euler(0));
        CGA R_y = cga_utils::rot(e3 * e1, zyx_euler(1));
        CGA R_x = cga_utils::rot(e2 * e3, zyx_euler(2));
        return (R_z * R_y * R_x).normalized();
    }




} // namespace cga_utils

#endif // CGA_UTILS_HPP




