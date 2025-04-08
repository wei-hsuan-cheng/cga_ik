#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "cga/cga_utils.hpp"
#include "robot_math_utils/robot_math_utils_v1_9.hpp"

using RM = RMUtils;

int main(int argc, char ** argv) {
    // Initialize ROS 2.
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cga_test");
    std::cout << "\n----- Starting CGA tests -----\n" << std::endl;

    // Form the sum of all basis elements (for demonstration).
    CGA G41 = CGA(1.0f, 0) + e1 + e2 + e3 + e4 + e5 +
                    e12 + e13 + e14 + e15 +
                    e23 + e24 + e25 +
                    e34 + e35 + e45 +
                    e123 + e124 + e125 + e134 + e135 + e145 +
                    e234 + e235 + e245 + e345 +
                    e1234 + e1235 + e1245 + e1345 + e2345 +
                    e12345;
    std::cout << "G41 bases = ";
    G41.log();
    std::cout << "G41 dimension = " << 32 << std::endl;

    // -------------------------------
    // Basic tests.
    // -------------------------------
    // a = 2*e1 + 1*e2
    CGA a = 2 * e1 + 1 * e2;
    std::cout << "\na = ";
    a.log();
    std::cout << "a[1] = " << a[1] << std::endl;

    // Reverse of G41
    CGA G41_rev = ~G41;
    std::cout << "\nReverse of G41 = ";
    G41_rev.log();

    // Dual of G41 (using operator !)
    CGA G41_dual = !G41;
    std::cout << "\nDual of G41 = ";
    G41_dual.log();

    // Inner products.
    CGA inner_e1e1 = e1 | e1;
    CGA inner_e1e2 = e1 | e2;
    CGA inner_ni_no = ni | no;
    std::cout << "\nInner product e1 | e1 = ";
    inner_e1e1.log();
    std::cout << "Inner product e1 | e2 = ";
    inner_e1e2.log();
    std::cout << "Inner product ni | no = ";
    inner_ni_no.log();

    // Outer products.
    CGA outer_e1e1 = e1 ^ e1;
    CGA outer_e1e2 = e1 ^ e2;
    CGA outer_ni_no = ni ^ no;
    std::cout << "\nOuter product e1 ^ e1 = ";
    outer_e1e1.log();
    std::cout << "Outer product e1 ^ e2 = ";
    outer_e1e2.log();
    std::cout << "Outer product ni ^ no = ";
    outer_ni_no.log();

    // Geometric products.
    CGA geo_e1e1 = e1 * e1;
    CGA geo_e1e2 = e1 * e2;
    CGA geo_ni_no = ni * no;
    std::cout << "\nGeometric product e1 * e1 = ";
    geo_e1e1.log();
    std::cout << "Geometric product e1 * e2 = ";
    geo_e1e2.log();
    std::cout << "Geometric product ni * no = ";
    geo_ni_no.log();

    // Regressive product.
    // For demonstration, let S1 = !(no - ni) and S2 = !(cga::up(1,0,0) - no).
    CGA S1 = !(no - ni);
    CGA S2 = !(cga_utils::up(1.0f, 0.0f, 0.0f) - no);
    CGA regressive_S1S2 = S1 & S2;
    std::cout << "\nRegressive product S1 & S2 = ";
    regressive_S1S2.log();

    // Norm of a.
    float norm_a = a.norm();
    std::cout << "\nNorm of a = " << norm_a << std::endl;

    // Normalized a.
    CGA normalized_a = a.normalized();
    std::cout << "\nNormalized a = ";
    normalized_a.log();

    // Inverse.
    CGA inverse_e12 = e12.inverse();
    std::cout << "\nInverse of e12 = ";
    inverse_e12.log();
    CGA prod_e12_inv = e12 * inverse_e12;
    std::cout << "e12 * inverse_e12 = ";
    prod_e12_inv.log();

    // Unit conversion
    float angle_deg = 45.0f;
    float angle_rad = angle_deg * RM::d2r;
    std::cout << "\n45° in [rad] = " << angle_rad << std::endl;

    // Construct a multivector d.
    // Example: d = e1 + 2*e2 + 3*e3 + 4*e4 + 5*e5 + 12*e12 + 1235*e1235 + 12345*e12345
    CGA d = (0 * CGA(1.0f, 0)) + e1 + (2 * e2) + (3 * e3) +
            (4 * e4) + (5 * e5) + (12 * e12) + (1235 * e1235) + (12345 * e12345);
    std::cout << "Multivector d = ";
    d.log();

    // Test Grade projection for each grade.
    std::cout << "\nGrade-0 component of d:" << std::endl;
    cga_utils::Grade(d, 0).log();

    std::cout << "\nGrade-1 component of d:" << std::endl;
    cga_utils::Grade(d, 1).log();

    std::cout << "\nGrade-2 component of d:" << std::endl;
    cga_utils::Grade(d, 2).log();

    std::cout << "\nGrade-3 component of d:" << std::endl;
    cga_utils::Grade(d, 3).log();

    std::cout << "\nGrade-4 component of d:" << std::endl;
    cga_utils::Grade(d, 4).log();

    std::cout << "\nGrade-5 component of d:" << std::endl;
    cga_utils::Grade(d, 5).log();

    // Test rotor function.
    // For example, a rotor that rotates by 45° in the e2^e3 plane.
    float angle = M_PI / 4; // 45 degrees
    CGA bivector = e2 ^ e3;
    CGA rot = cga_utils::rot(bivector, angle);
    std::cout << "\nRotor for 45° rotation in the e2^e3 plane:" << std::endl;
    rot.log();

    // Test translator function.
    // For example, translate by 3 units along e1.
    CGA trans = cga_utils::trans(3 * e1);
    std::cout << "\nTranslator for a translation of 3 units along e1:" << std::endl;
    trans.log();
    
    // ------------------------------------------------------------------
    // 1. R3 vector -> G3 vector -> Conformal (G41) point -> down-projection back to G3.
    // ------------------------------------------------------------------
    // R3 vector
    Eigen::Vector3f PX_R3(1.0f, 0.0f, 0.0f);
    // Convert R3 vector to a G3 vector.
    CGA PX_G3 = cga_utils::R2G(PX_R3);
    // Lift the G3 vector to a conformal point using the up-projection.
    CGA PX = cga_utils::up(PX_G3);
    // Down-project the conformal point back to a Euclidean (G3) vector.
    CGA PX_down = cga_utils::down(PX);

    // ------------------------------------------------------------------
    // 2. Basics: point, line, and sphere.
    // ------------------------------------------------------------------
    // Create a line: in our example we mimic the Python "LINE = PX ^ no ^ ni".
    // (Using the wedge operator '^'; note that the wedge is associative.)
    CGA LINE = PX ^ (cga::no ^ cga::ni);
    // Create a sphere: using the dual of (no - ni).
    CGA SPHERE = !(cga::no - cga::ni);

    // Output the results:
    std::cout << "\nGrade-1 point (conformal point, PX): ";
    PX.log();

    std::cout << "\nG3 point by down-projection (PX_down): ";
    PX_down.log();

    std::cout << "\nGrade-3 line (PX ^ (no ^ ni)): ";
    LINE.log();

    std::cout << "\nGrade-4 sphere (dual of (no - ni)): ";
    SPHERE.log();


    rclcpp::shutdown();
    return 0;
}
