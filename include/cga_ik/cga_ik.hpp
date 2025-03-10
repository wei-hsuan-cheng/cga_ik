
/*
cga_ik, an inverse kinematics (IK) solver based on conformal geometric algebra (CGA)
Written by wei-hsuan-cheng

G41 structure: e1 * e1 = e2 * e2 = e3 * e3 = e4 * e4 = +1, e5 * e5 = -1
Definition of DUAL here: M.Dual() = e12345 * M = M * e12345
Null bases: ni = e5 + e4, no = 0.5 * (e5 - e4)

Two singularities of the robot arm
1) Shoulder singularity
2) Wrist singularity

References for this code:
https://slides.com/hugohadfield/game2020
https://www.mic-journal.no/ABS/MIC-2016-1-6.asp/
https://www.sciencedirect.com/science/article/pii/S0094114X22001045
*/

/*
Potential issues:
1) Numerical robustness issue close to singularities
2) Second possibility of shoulder singularity is not yet considered
3) Wrist singularity is not yet considered
4) Some unsolvable problem by closed-form solution may be further passed to numerical solution pipeline
5) Elbow singularity is not yet considered
*/


#ifndef CGA_IK_HPP
#define CGA_IK_HPP

#include <vector>
#include <iostream>
#include <tuple>
#include "cga_ik/cga_utils.hpp"
#include "robot_math_utils/robot_math_utils_v1_8.hpp"  // This header defines DHParams and DHTable

using cga_utils::up;
using cga_utils::down;
using RM = RMUtils;

namespace cga_ik {

    // Data structure for the SE(3) pose in CGA.
    struct CGAKinematicsPose {
        CGA org;  // Null point (frame origin)
        CGA x_hat;  // x-axis vector (frame orientation)
        CGA y_hat;  // y-axis vector (frame orientation)
        CGA z_hat;  // z-axis vector (frame orientation)

        // Default constructor.
        CGAKinematicsPose() {}

        // Parameterized constructor.
        CGAKinematicsPose(const CGA& org, const CGA& x_hat, 
                        const CGA& y_hat, const CGA& z_hat)
            : org(org), x_hat(x_hat), y_hat(y_hat), z_hat(z_hat) {}
    };

    // Data structure for CGA rotor and translator.
    struct RotorTranslator {
        CGA R;  // Rotor
        CGA T;  // Translator

        // Default constructor.
        RotorTranslator() {}

        // Parameterized constructor.
        RotorTranslator(const CGA& R, const CGA& T)
            : R(R), T(T) {}
    };

    // -------------------------------------------------------------------
    // Data type for the solutions of "n" null points for the robot's joint frames in IK problem.
    // This structure is designed to hold a collection of CGA null points.
    // -------------------------------------------------------------------
    struct CGAIKNullPoints {
        std::vector<CGA> points;  // Container for null points

        // Default constructor.
        CGAIKNullPoints() = default;

        // Constructor to initialize with a vector of CGA points.
        explicit CGAIKNullPoints(const std::vector<CGA>& pts)
            : points(pts) {}

        // Helper method: add a new null point.
        void add(const CGA& point) {
            points.push_back(point);
        }

        // Helper method: print all null points.
        void print() const {
            for (const auto& p : points) {
                p.log();
            }
        }
    };

    // Data type for robot configuration.
    struct CGAIKRobotConfig {
        int kud;  // elbow: 1 for up, -1 for down
        int klr;  // shoulder: 1 for right, -1 for left
        int kfn;  // wrist: 1 for not flipped, -1 for flipped

        // Default constructor initializes to 1.
        CGAIKRobotConfig() {}

        // Parameterized constructor.
        CGAIKRobotConfig(int kud, int klr, int kfn)
            : kud(kud), klr(klr), kfn(kfn) {}
    };

    // PosRot (position and rotation matrix) <-> CGA kinematics SE(3) pose.
    static inline CGAKinematicsPose PosRot2CGAKinematicsPose(const PosRot& pos_rot_1_2) {
        // Cast the Eigen::Vector3d to Eigen::Vector3f (since your cga functions use float)
        Eigen::Vector3f R1 = pos_rot_1_2.rot.col(0).cast<float>();
        Eigen::Vector3f R2 = pos_rot_1_2.rot.col(1).cast<float>();
        Eigen::Vector3f R3 = pos_rot_1_2.rot.col(2).cast<float>();
        Eigen::Vector3f p  = pos_rot_1_2.pos.cast<float>();

        // Convert the position and the rotation axes into CGA objects.
        CGA org = up(cga_utils::R2G(p));
        CGA x_hat = cga_utils::R2G(R1);
        CGA y_hat = cga_utils::R2G(R2);
        CGA z_hat = cga_utils::R2G(R3);

        return CGAKinematicsPose(org, x_hat, y_hat, z_hat);
    }

    static inline CGAKinematicsPose R6Pose2CGAKinematicsPose(const Vector6d& pose) {
        PosRot pos_rot = RM::R6Pose2PosRot(pose);
        return PosRot2CGAKinematicsPose(pos_rot);
    }

    static inline Vector6d CGAKinematicsPose2R6Pose(const CGAKinematicsPose& pose_cga) {
        CGA org = pose_cga.org;
        CGA x_hat = pose_cga.x_hat;
        CGA y_hat = pose_cga.y_hat;
        CGA z_hat = pose_cga.z_hat;

        Matrix3d RMat;
        RMat << (x_hat | e1)[0], (y_hat | e1)[0], (z_hat | e1)[0],
                (x_hat | e2)[0], (y_hat | e2)[0], (z_hat | e2)[0],
                (x_hat | e3)[0], (y_hat | e3)[0], (z_hat | e3)[0];

        double thy = atan2(-RMat(2,0), sqrt(RMat(0,0) * RMat(0,0) + RMat(1,0) * RMat(1,0)));
        double thz = atan2( RMat(1,0) / cos(thy),  RMat(0,0) / cos(thy));
        double thx = atan2( RMat(2,1) / cos(thy),  RMat(2,2) / cos(thy));
        
        return Vector6d(down(org)[1], down(org)[2], down(org)[3], thx, thy, thz);
    }
        

    // // Convert CGA motor to CGA kinematics SE(3) pose (still not sure)
    // static inline CGAKinematicsPose Motor2CGAKinematicsPose(const CGA M) {
    //     CGA org = M * no * ~M;
    //     CGA x_hat = M * e1 * ~M;
    //     CGA y_hat = M * e2 * ~M;
    //     CGA z_hat = M * e3 * ~M;

    //     return CGAKinematicsPose(org, x_hat, y_hat, z_hat);
    // }

    // CGA rotor and translator <-> CGA kinematics SE(3) pose
    static inline CGAKinematicsPose RotorTranslator2CGAKinematicsPose(const RotorTranslator& RT) {
        CGA R = RT.R;
        CGA T = RT.T;
        CGA org = T * no * ~T;
        CGA x_hat = R * e1 * ~R;
        CGA y_hat = R * e2 * ~R;
        CGA z_hat = R * e3 * ~R;

        return CGAKinematicsPose(org, x_hat, y_hat, z_hat);
    }

    // //  (still not sure)
    // static inline RotorTranslator CGAKinematicsPose2RotorTranslator(const CGAKinematicsPose& pose_cga) {
    //     CGA R = up( pose_cga.x_hat ^ pose_cga.y_hat ^ pose_cga.z_hat );
    //     CGA T = up( pose_cga.org );
    //     return RotorTranslator(R, T);
    // }




    // Existing functions for loading the DH table and robot configuration.
    static inline DHTable loadDHTable() {
        double mm2m = 1e-3;
        DHParams dh_j1(Eigen::Vector4d(0, 0, 145.2 * mm2m, 0));
        DHParams dh_j2(Eigen::Vector4d(-M_PI / 2, 0, 0, -M_PI / 2));
        DHParams dh_j3(Eigen::Vector4d(0, 329.0 * mm2m, 0, 0));
        DHParams dh_j4(Eigen::Vector4d(0, 311.5 * mm2m, -122.3 * mm2m, -M_PI / 2));
        DHParams dh_j5(Eigen::Vector4d(-M_PI / 2, 0, 106 * mm2m, 0));
        DHParams dh_j6(Eigen::Vector4d(-M_PI / 2, 0, 113.15 * mm2m, M_PI));

        return DHTable({dh_j1, dh_j2, dh_j3, dh_j4, dh_j5, dh_j6});
    }

    // Helper function to return the default robot configuration.
    static inline CGAIKRobotConfig setRobotConfig(const int& kud = 1, const int& klr = 1, const int& kfn = 1) {
        return CGAIKRobotConfig(kud, klr, kfn);
    }

    // Global variables for CGA IK
    // Extract the robot configuration.
    inline int kud;
    inline int klr;
    inline int kfn;

    double d1;
    double a2;
    double a3;
    double d4;
    double d5;
    double d6;

    inline CGA X0 = no;  // Base frame origin.
    inline CGA X6;
    inline CGA x6;
    inline CGA y6;
    inline CGA z6;
    inline CGA X5;
    inline CGA X1;
    inline CGA Sc;
    inline CGA K0;
    inline CGA C5k;
    inline CGA PPc;
    inline double PPcd;
    inline CGA Xc;

    inline bool shoulder_singularity = false;

    inline CGA Pc;
    inline CGA Pc_ver;
    inline CGA P56;
    inline CGA n56;
    inline CGA Pc_hor;
    inline CGA L54;
    inline CGA S5;
    inline CGA PP4;
    inline float PP4d;
    inline CGA X4;
    inline CGA S4;
    inline CGA L34;
    inline CGA PP3;
    inline float PP3d;
    inline CGA X3;

    inline CGA S3;
    inline CGA S1;
    inline CGA C2;
    inline CGA PP2;
    inline float PP2d;
    inline CGA X2;
    inline bool reachable = true;

    inline CGAIKNullPoints null_points_g41;
    inline CGAIKNullPoints null_points_g3;
    float cga_offset1 = 0.0f, cga_offset2 = 0.0f, cga_offset3 = 0.0f, cga_offset4 = 0.0f, cga_offset5 = 0.0f, cga_offset6 = M_PI;
    inline Vector6d joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // FK; frame {0}, {1}, ..., {6}
    inline std::vector<CGA> M0(7);
    inline std::vector<CGA> orgi(7); // org0, org1, ..., org6
    inline std::vector<CGA> R0(7);
    inline std::vector<CGA> x_hati(7), y_hati(7), z_hati(7);
    inline std::vector<CGAKinematicsPose> pose_cgai(7);

    static void CheckShoulderSingularity() {
        // Intersect 2 spheres gives 1 circle
        Sc = !(X5 - 0.5f * static_cast<float>(d4 * d4) * ni);
        K0 = ( !(no - (!(Sc) | no) * ni) ).normalized();
        C5k = (Sc & K0).normalized();
        // Intersect the circle and a plane gives a point pair
        PPc = (X5 ^ e1 ^ e2 ^ ni) & C5k;
        PPc = (-1) * PPc;
        PPcd = ((PPc | PPc) * (((PPc ^ ni) | (PPc ^ ni)).inverse()))[0];

        // If PPcd > 0 then the spheres intersect.
        // The square of the point pair describes if the spheres intersect
        if (PPcd > 0) {
            // If the spheres intersect then we can just choose one solution. Extract each point in the point pair by method of projectors
            Xc = (1 + klr * PPc * (1.0f / sqrt((PPc * PPc)[0]))) * (PPc | ni);
            Xc = up(down(Xc));
            shoulder_singularity = false;
        } else {
            // Check if the target X6's fourth component is (approximately) equal to (d1 + a2 + a3 + d5)
            if (fabs(X6[3] - static_cast<float>(d1 + a2 + a3 + d5)) < 1e-6f) {
                // Shoulder singularity (Xc = alpha * e3 for all alpha is real number. In this case, Xc lies in the rotational axis of joint 1)
                // Still another possibility for shoulder singularity (wrist flipped), while it's rarely used so skipped for now!
                Xc = up(down(X6) - static_cast<float>(d6 - d4) * z6);
                shoulder_singularity = true;
            } else {
                // If the sphere at origin vanishes (radius = 0), the point pair becomes a single point
                Xc = no;
                shoulder_singularity = false;
            }
        }
    }

    static inline void SolveX4() {
        // --- Solve for X4 ---

        if (!shoulder_singularity) {
            // Compute the vertical plane through Xc:
            Pc = no ^ e3 ^ Xc ^ ni;
            // Shift the plane so it passes through X5:
            Pc_ver = !( !(Pc) + ( X5 | !(Pc) ) * ni );
            // Compute a plane perpendicular to Pc_ver through X5:
            P56 = !(X5 ^ X6) ^ ni;
            n56 = (-1) * (((P56 | no) | ni)).normalized();
            Pc_hor = X5 ^ n56 ^ ni;
            // Line through X4 and X5:
            L54 = Pc_ver & Pc_hor;
            // Sphere centered at X5 with radius d5:
            S5 = ( !(X5 - 0.5f * static_cast<float>(d5 * d5) * ni) ).normalized();
            // Intersect the sphere and the line gives a point pair PP4:
            PP4 = !(L54) | S5;
            PP4d = ((PP4 | PP4) * (((PP4 ^ ni) | (PP4 ^ ni)).inverse()))[0];

            if (PP4d > 0) {
                CGA X4_temp = (1 + kfn * PP4 * (1.0f / sqrt((PP4 * PP4)[0]))) * (PP4 | ni);
                X4 = up(down(X4_temp));
            } else {
                std::cerr << "unsolvable! (during solving X4)" << std::endl;
            }
        } else if (shoulder_singularity) {
            Pc = no ^ y6 ^ x6 ^ ni;
            X4 = up(down(X6) - d6 * z6 - d5 * e3);
        }

    }

    static inline void SolveX3() {
        // --- Solve for X3 ---

        if (!shoulder_singularity) {
            S4 = ( !(X4 - 0.5f * static_cast<float>(d4 * d4) * ni) ).normalized();
            L34 = X4 ^ !(Pc) ^ ni;
            PP3 = !(L34) | S4;
            PP3d = ((PP3 | PP3) * (((PP3 ^ ni) | (PP3 ^ ni)).inverse()))[0];

            if (PP3d > 0) {
                CGA X3_temp = (1 + klr * PP3 * (1.0f / sqrt((PP3 * PP3)[0]))) * (PP3 | ni);
                X3 = up(down(X3_temp));
            } else {
                std::cerr << "unsolvable! (during solving X3)" << std::endl;
            }
        } else if (shoulder_singularity) {
            X3 = up(down(X0) + (d1 + a2 + a3) * e3);
        }
        
    }

    static inline void SolveX2() {
        // --- Solve for X2 ---

        if (!shoulder_singularity) {
            S3 = ( !(X3 - 0.5f * static_cast<float>(a3 * a3) * ni) ).normalized();
            S1 = ( !(X1 - 0.5f * static_cast<float>(a2 * a2) * ni) ).normalized();
            C2 = (S1 & S3).normalized();
            PP2 = (-1) * (Pc & C2);
            PP2d = ((PP2 | PP2) * (((PP2 ^ ni) | (PP2 ^ ni)).inverse()))[0];

            if (PP2d > 0) {
                CGA X2_temp = (1 + kud * PP2 * (1.0f / sqrt((PP2 * PP2)[0]))) * (PP2 | ni);
                X2 = up(down(X2_temp));
                reachable = true;
            } else {
                // If the spheres do not intersect, use an alternative computation.
                X2 = up(down(X1) + static_cast<float>(a2 + a3) * (down(X3) - down(X1)).normalized());
                reachable = false;
            }
        } else if (shoulder_singularity) {
            reachable = true;
            X2 = up(down(X0) + (d1 + a2) * e3);
        }

    }


    // Solve for the null points (frame origins) used in the IK solution.
    static inline void SolveNullPoints(const Vector6d& pose, const DHTable& dh_table, const CGAIKRobotConfig& robot_config) {
        // Extract the target pose.
        CGAKinematicsPose pose_cga = R6Pose2CGAKinematicsPose(pose);

        // Extract the robot configuration.
        kud = robot_config.kud;
        klr = robot_config.klr;
        kfn = robot_config.kfn;

        // Extract the DH parameters from the DH table.
        // Here we assume that dh_table.dh_table is a matrix with each row [alpha, a, d, theta]
        d1 = dh_table.dh_table(0, 2);
        a2 = dh_table.dh_table(2, 1);
        a3 = dh_table.dh_table(3, 1);
        d4 = dh_table.dh_table(3, 2);
        d5 = dh_table.dh_table(4, 2);
        d6 = dh_table.dh_table(5, 2);

        // X6 (target pose; input CGAKinematicsPose)
        X6 = pose_cga.org;
        x6 = pose_cga.x_hat;
        y6 = pose_cga.y_hat;
        z6 = pose_cga.z_hat;

        // X5 and X1 (trivial)
        X5 = up( down(X6) - static_cast<float>(d6) * z6 );
        X1 = up( down(X0) + static_cast<float>(d1) * e3 );

        // Check shoulder singularity
        CheckShoulderSingularity();

        // Solve for X4, X3, and X2
        SolveX4();
        SolveX3();
        SolveX2();

        // Create the container for null points and add the computed null points.
        null_points_g41.add(X0);
        null_points_g41.add(X1);
        null_points_g41.add(X2);
        null_points_g41.add(X3);
        null_points_g41.add(X4);
        null_points_g41.add(X5);

        null_points_g3.add(down(X0));
        null_points_g3.add(down(X1));
        null_points_g3.add(down(X2));
        null_points_g3.add(down(X3));
        null_points_g3.add(down(X4));
        null_points_g3.add(down(X5));
    }


    // Solve for the joint angles (frame origins) used in the IK solution.
    static inline void SolveJointAngles() {
        // Inverse kinematics of the robot:
        // Forming geometric relations among the null points
        
        if (!shoulder_singularity) {            
            // --- Form lines ---
            CGA L01 = (no ^ e3 ^ ni).normalized();
            CGA L12 = (X1 ^ X2 ^ ni).normalized();
            CGA L23 = (X2 ^ X3 ^ ni).normalized();

            // --- Form plane through Xc ---
            CGA Pc = no ^ e3 ^ Xc ^ ni;

            // --- Compute intermediate vectors for joints 1-6 ---
            // Joint 1:
            CGA ath1 = e2; 
            ath1 = ath1.normalized();
            CGA bth1 = static_cast<float>(klr) * !(Pc);
            bth1 = bth1.normalized();
            CGA Nth1 = (e1 ^ e2).normalized();

            // Joint 2:
            CGA ath2 = (L01 | (ni ^ no)).normalized();
            CGA bth2 = (L12 | (ni ^ no)).normalized();
            CGA Nth2 = ((Pc | no) | ni).normalized();
            Nth2 = (static_cast<float>(klr) * Nth2).normalized();

            // Joint 3:
            CGA ath3 = (L12 | (ni ^ no)).normalized();
            CGA bth3 = (L23 | (ni ^ no)).normalized();
            CGA Nth3 = ((Pc | no) | ni).normalized();
            Nth3 = (static_cast<float>(klr) * Nth3).normalized();

            // To compute joints 4-6, we need to use L54 and n56.
            CGA Pc_ver = !( !(Pc) + (X5 | !(Pc) ) * ni);
            CGA P56 = !(X5 ^ X6) ^ ni;
            CGA n56 = (-1) * ((P56 | no) | ni);
            n56 = n56.normalized();
            CGA Pc_hor = X5 ^ n56 ^ ni;
            CGA L54 = Pc_ver & Pc_hor;

            // Joint 4:
            CGA ath4 = (L23 | (ni ^ no)).normalized();
            CGA bth4 = static_cast<float>(kfn) * ((-1) * (L54 | (ni ^ no)));
            bth4 = bth4.normalized();
            CGA Nth4 = ((Pc | no) | ni).normalized();
            Nth4 = (static_cast<float>(klr) * Nth4).normalized();

            // Joint 5:
            CGA ath5 = ( static_cast<float>(klr) * !(Pc) ).normalized();
            CGA bth5 = (-1) * z6; 
            bth5 = bth5.normalized();
            CGA Nth5 = (( !(L54) ^ no ) | ni).normalized();
            Nth5 = (static_cast<float>(kfn) * Nth5).normalized();

            // Joint 6 (still problematic):
            CGA ath6 = static_cast<float>(kfn) * ((-1) * (L54 | (ni ^ no)));
            ath6 = ath6.normalized();
            CGA bth6 = (-1) * y6; 
            bth6 = bth6.normalized();
            CGA Nth6 = (1 * n56).normalized();

            // --- Solve joint angles using atan2.
            float theta1 = std::atan2(((ath1 ^ bth1) * Nth1.inverse())[0], (ath1 | bth1)[0]) + cga_offset1;
            theta1 = RM::ConstrainedAngle(theta1, true);
            
            float theta2 = std::atan2(((ath2 ^ bth2) * Nth2.inverse())[0], (ath2 | bth2)[0]) + cga_offset2;
            theta2 = RM::ConstrainedAngle(theta2, true);
            
            float theta3 = std::atan2(((ath3 ^ bth3) * Nth3.inverse())[0], (ath3 | bth3)[0]) + cga_offset3;
            theta3 = RM::ConstrainedAngle(theta3, true);
            
            float theta4 = std::atan2(((ath4 ^ bth4) * Nth4.inverse())[0], (ath4 | bth4)[0]) + cga_offset4;
            theta4 = RM::ConstrainedAngle(theta4, true);
            
            float theta5 = std::atan2(((ath5 ^ bth5) * Nth5.inverse())[0], (ath5 | bth5)[0]) + cga_offset5;
            theta5 = RM::ConstrainedAngle(theta5, true);
            
            float theta6 = std::atan2(((ath6 ^ bth6) * Nth6.inverse())[0], (ath6 | bth6)[0]) + cga_offset6;
            theta6 = RM::ConstrainedAngle(theta6, true);

            joints = Vector6d(theta1, theta2, theta3, theta4, theta5, theta6);
        
        } else if (shoulder_singularity) {
            // --- Shoulder singularity branch ---
            // For joint 1, compute the angle using the vertical plane.
            // Use the same procedure as in the non-singular case for joint 1.
            float cga_offset1 = 0.0f;
            CGA ath1 = e2;
            ath1 = ath1.normalized();
            CGA bth1 = static_cast<float>(klr) * (!Pc);
            bth1 = bth1.normalized();
            CGA Nth1 = (e1 ^ e2).normalized();
            float theta1 = std::atan2(((ath1 ^ bth1) * Nth1.inverse())[0], (ath1 | bth1)[0]) + cga_offset1;
            theta1 = RM::ConstrainedAngle(theta1, true);

            // Set joint angles 2 to 6 to zero.
            float theta2 = 0.0f;
            float theta3 = 0.0f;
            float theta4 = 0.0f;
            float theta5 = 0.0f;
            float theta6 = 0.0f;
            joints = Vector6d(theta1, theta2, theta3, theta4, theta5, theta6);

        }

    }


    // D-H parameters transformation
    static inline CGAKinematicsPose DHTransformCGA(const double alpha, const double a, const double d, const double theta) {
        CGA Ralpha = cga_utils::rot(e23, static_cast<float>(alpha));
        CGA Ta     = cga_utils::trans(static_cast<float>(a) * e1);
        CGA Td     = cga_utils::trans(static_cast<float>(d) * e3);
        CGA Rtheta = cga_utils::rot(e12, static_cast<float>(theta));

        CGA M = Ralpha * Ta * Td * Rtheta;
        CGA R = Ralpha * Rtheta;
        CGA org = M * no * ~M;
        CGA x_hat = R * e1 * ~R;
        CGA y_hat = R * e2 * ~R;
        CGA z_hat = R * e3 * ~R;

        return CGAKinematicsPose(org, x_hat, y_hat, z_hat);
    }

    static inline Vector6d DHTransform(const double alpha, const double a, const double d, const double theta) {
        return CGAKinematicsPose2R6Pose( DHTransformCGA(alpha, a, d, theta) );
    }

    // static inline Vector6d DHTransform(const DHTable& dh_table, const VectorXd& joints, const int& joint_index) {
    //     CGAKinematicsPose pose_cga = DHTransformCGA(dh_table, joints, joint_index);
    //     return CGAKinematicsPose2R6Pose(pose_cga);
    // }

    // Forward Kinematics: Compute the motor (end‐effector) CGA SE(3) pose from a 6‑D joint vector.
    static inline Vector6d CGAFK(const Vector6d& joints, const DHTable& dh_table) {
        // --- Compute the joint transformations.
        // We build the cumulative motor transformation as a product of individual joint transformations.
        // We denote the transformation for joint i as:
        //     M_i = Ralpha_{i-1} * Ta_{i-1} * Td_{i} * Rtheta_{i}
        // where (using your cga_utils functions):
        //   - Ralpha_{i-1} = rot(e23, alpha_{i-1})
        //   - Ta_{i-1} = trans( a_{i-1} * e1 )
        //   - Td_{i} = trans( d_{i} * e3 )
        //   - Rtheta_{i} = rot(e12, theta_i + dh_offset_i)
        // The cumulative transformation from base to frame i is:
        //     M_0_i = M_0_{i-1} * M_i
        
        // Initialization
        M0[0] = CGA(1.0f, 0);
        orgi[0] = no;
        R0[0] = CGA(1.0f, 0);
        x_hati[0] = e1;
        y_hati[0] = e2;
        z_hati[0] = e3;
        pose_cgai[0] = CGAKinematicsPose(orgi[0], x_hati[0], y_hati[0], z_hati[0]);
        
        // Loop over each joint to build the overall transformation.
        for (int i = 0; i < 6; i++) {
            CGA Ralpha = cga_utils::rot(e23, static_cast<float>(dh_table.dh_table(i, 0)));
            CGA Ta     = cga_utils::trans(static_cast<float>(dh_table.dh_table(i, 1)) * e1);
            CGA Td     = cga_utils::trans(static_cast<float>(dh_table.dh_table(i, 2)) * e3);
            CGA Rtheta = cga_utils::rot(e12, static_cast<float>(dh_table.dh_table(i, 3) + joints(i)));
            
            CGA Mi = Ralpha * Ta * Td * Rtheta;
            M0[i+1] = M0[i] * Mi;

            // Compute the frame origin as a null point.
            orgi[i+1] = M0[i+1] * no * ~(M0[i+1]);
            // For orientation, assume that Ta and Td do not affect the rotational part.
            R0[i+1] = R0[i] * (Ralpha * Rtheta);
            
            // Compute the axes by “sandwiching” the basis vectors.
            x_hati[i+1] = R0[i+1] * e1 * ~(R0[i+1]);
            y_hati[i+1] = R0[i+1] * e2 * ~(R0[i+1]);
            z_hati[i+1] = R0[i+1] * e3 * ~(R0[i+1]);

            pose_cgai[i+1] = CGAKinematicsPose(orgi[i+1], x_hati[i+1], y_hati[i+1], z_hati[i+1]);

        }
        
        // The final motor frame is given by the 6th frame (index 6).
        CGA org_6 = orgi[6];
        CGA x_hat6   = x_hati[6];
        CGA y_hat6   = y_hati[6];
        CGA z_hat6   = z_hati[6];
        
        // Return a R6Pose representing the final motor (end-effector) pose.
        return CGAKinematicsPose2R6Pose(CGAKinematicsPose(org_6, x_hat6, y_hat6, z_hat6));
    }





} // namespace cga_ik

#endif // CGA_IK_HPP