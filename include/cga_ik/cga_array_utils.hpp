

#ifndef CGA_UTILS_HPP
#define CGA_UTILS_HPP

#include "cga_ik/cga_array.hpp"
#include <iostream>
#include <cmath>
#include <array>

using cga::CGA;

namespace cga_utils {

//-----------------------------------------------------------------
// R2G: Convert a 3-element array (an R3 vector) into a G3 vector.
// In our conformal model, a G3 vector is represented as a CGA with 
// the scalar part zero and its grade‑1 components in indices 1–5.
//-----------------------------------------------------------------
inline CGA R2G(const std::array<float, 3>& arr) {
    CGA res;
    res[0] = 0.0f;   // scalar part
    res[1] = arr[0]; // e1
    res[2] = arr[1]; // e2
    res[3] = arr[2]; // e3
    res[4] = 0.0f;   // e4 (unused in G3)
    res[5] = 0.0f;   // e5 (unused in G3)
    return res;
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
    return CGA(1.0f, 0) - 0.5f * (trans_vec * cga::ni);
}


//-----------------------------------------------------------------
// Up function.
// Converts a point from Euclidean coordinates (R3) into a conformal point in G41.
// (There are two possible versions: one that takes three floats,
//  and one that takes a G3 vector already represented as a CGA.)
//-----------------------------------------------------------------
inline CGA up(float x, float y, float z) {
    float d = x*x + y*y + z*z;
    return x * cga::e1 + y * cga::e2 + z * cga::e3 + 0.5f * d * cga::ni + cga::no;
}

inline CGA up(const CGA &vec_G3) {
    // Assumes vec_G3 has scalar part zero and only grade-1 components (indices 1–3)
    return vec_G3 + 0.5f * (vec_G3 * vec_G3) * cga::ni + cga::no;
}

//-----------------------------------------------------------------
// Down function.
// Projects a conformal point in G41 back to a Euclidean vector (G3 vector).
// The formula used is:
//    down(vec_G41) = (vec_G41 ^ (no ^ ni)) * (no ^ ni) * ((-1)*vec_G41 | ni).inverse()
//-----------------------------------------------------------------
inline CGA down(const CGA &vec_G41) {
    return (vec_G41 ^ (cga::no ^ cga::ni)) * (cga::no ^ cga::ni) * (((-1.0f) * vec_G41 | cga::ni).inverse());
}

} // namespace cga_utils

#endif // CGA_UTILS_HPP

