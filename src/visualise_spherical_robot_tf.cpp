#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <memory>

#include "cga_ik/cga_utils.hpp"
#include "cga_ik_spherical_robot/cga_ik_spherical_robot.hpp"
#include "robot_math_utils/robot_math_utils_v1_9.hpp"

using RM = RMUtils;

class VisualiseSphericalRobotTF : public rclcpp::Node
{
public:
  VisualiseSphericalRobotTF()
  : Node("visualise_spherical_robot_tf")
  {
    // Initialise time spec
    initTimeSpec();
    // Initialise CGA IK for spherical robot
    initSRBIK();
    // Initialise robot visuals
    initRobotVisual();


    // Set up a TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(Ts_ * 1000)),
      std::bind(&VisualiseSphericalRobotTF::visualise_spherical_robot_tf_callback_, this)
    );

    // Publishers
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/spherical_robot/joint_states", 10);

    srb_outer_sphere_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_outer_sphere", 10);

    srb_ltri_0_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_ltri_0", 10);
    srb_ltri_1_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_ltri_1", 10);
    srb_ltri_2_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_ltri_2", 10);
    srb_utri_0_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_utri_0", 10);
    srb_utri_1_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_utri_1", 10);
    srb_utri_2_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_utri_2", 10);

    RCLCPP_INFO(this->get_logger(), "visualise_spherical_robot_tf node started.");

  }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // We'll also publish 3 joint angles
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angles_pub_;

    // Publisher for the sphere marker
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr srb_outer_sphere_pub_;

    // Publisher for the triangle marker
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr srb_ltri_0_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr srb_ltri_1_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr srb_ltri_2_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr srb_utri_0_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr srb_utri_1_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr srb_utri_2_pub_;

    double fs_, Ts_;
    int k_;
    double t_;

    Quaternionf quat_cmd_;

    float r_b_, ratio_c_b_, r_c_, ratio_e_b_, r_e_;
    CGA e_principal_, rot_cen_, s_0_, s_1_;
    cga_ik_spherical_robot::SphericalRobotIKResult ik_result_;

    Vector4f outer_sphere_color_;
    std::vector<Vector3f> linkArcPts_;
    Vector4f utri_color_, ltri_color_, link_color_, plate_color_;
    float link_thickness_, plate_thickness_;
    

    void initTimeSpec()
    {
        fs_ = 30.0;
        Ts_ = 1.0 / fs_;
        k_ = 0;
        t_ = 0.0;
    }

    void initSRBIK()
    {
        // IK target pose (quaternion orientation)
        quat_cmd_ = Quaternionf::Identity();

        // Spherical robot geometries
        // Load geometric params from yaml file
        // Declare default parameters (in case the YAML file doesn't provide them)
        this->declare_parameter<float>("r_b", 1.0);
        this->declare_parameter<float>("ratio_c_b", 1.0);
        this->declare_parameter<float>("ratio_e_b", 1.0);

        // Retrieve the parameters
        // Base radius, rotation centre height, and end-plate radius
        this->get_parameter("r_b", r_b_);
        this->get_parameter("ratio_c_b", ratio_c_b_);
        this->get_parameter("ratio_e_b", ratio_e_b_);

        r_c_ = ratio_c_b_ * r_b_;
        r_e_ = ratio_e_b_ * r_b_;

        // Spherical robot coordinate system
        // Principal axis of the 3-DoF spherical robot
        e_principal_ = e3;

        // Rotation centre
        rot_cen_ = r_c_ * e_principal_;

        // Each motor position is set in the base plane, separated by 120 [deg].
        s_0_ = e1;
        CGA rot_e12_120 = cga_utils::rot(e1 * e2, (float) (2.0 * M_PI / 3.0)); // Rotate s_0_ by 120 degrees to get s_1_
        s_1_ = rot_e12_120 * s_0_ * ~rot_e12_120;

        // IK solver for the spherical robot
        ik_result_ = cga_ik_spherical_robot::computeSphericalRobotIK(r_b_, r_e_,
                                                                     e_principal_, 
                                                                     rot_cen_, 
                                                                     s_0_, s_1_, 
                                                                     Quaternionf::Identity());
        
        // Print initial poses of each component
        // Motor positions and orientations
        std::cout << "Initial position and orientation of the spherical robot:" << std::endl;
        std::cout << "motor_0 pos [m] = " << ik_result_.pos_rot_cen_m_0.transpose() << std::endl;
        std::cout << "motor_0 rpy [rad] = " << RM::Quat2zyxEuler( ik_result_.quat_rot_cen_m_0.cast<double>() ).reverse().transpose() << std::endl;
        std::cout << "motor_1 pos [m] = " << ik_result_.pos_rot_cen_m_1.transpose() << std::endl;
        std::cout << "motor_1 rpy [rad] = " << RM::Quat2zyxEuler( ik_result_.quat_rot_cen_m_1.cast<double>() ).reverse().transpose() << std::endl;
        std::cout << "motor_2 pos [m] = " << ik_result_.pos_rot_cen_m_2.transpose() << std::endl;
        std::cout << "motor_2 rpy [rad] = " << RM::Quat2zyxEuler( ik_result_.quat_rot_cen_m_2.cast<double>() ).reverse().transpose() << std::endl;

    }

    void initRobotVisual()
    {
        // Outer sphere
        outer_sphere_color_ = Vector4f(0.0f, 1.0f, 0.0f, 0.15f);

        // Triangles
        utri_color_ = Vector4f(0.0f, 0.0f, 1.0f, 0.8f);
        ltri_color_ = Vector4f(0.0f, 1.0f, 0.0f, 0.8f);

    }

    void publishTF(const Vector3f &p, const Quaternionf &q, const std::string &child, const std::string &parent)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = parent;
        tf.child_frame_id = child;
        tf.transform.translation.x = p.x();
        tf.transform.translation.y = p.y();
        tf.transform.translation.z = p.z();

        // No orientation, so we set identity
        tf.transform.rotation.w = q.w();
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(tf);
    };

    void publishSphereMarker(
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
        int marker_id,
        const std::string &ns,
        const std::string &frame_id,
        float radius,
        const Vector3f &centre,
        Vector4f rgba_color)
    {
        visualization_msgs::msg::Marker sphere_marker;
        sphere_marker.header.stamp = this->now();
        sphere_marker.header.frame_id = frame_id;
        sphere_marker.ns = ns;
        sphere_marker.id = marker_id;
        sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::msg::Marker::ADD;

        sphere_marker.scale.x = radius * 2.0f; // Diameter
        sphere_marker.scale.y = radius * 2.0f; // Diameter
        sphere_marker.scale.z = radius * 2.0f; // Diameter

        sphere_marker.pose.position.x = centre.x();
        sphere_marker.pose.position.y = centre.y();
        sphere_marker.pose.position.z = centre.z();

        sphere_marker.color.r = rgba_color(0);
        sphere_marker.color.g = rgba_color(1);
        sphere_marker.color.b = rgba_color(2);
        sphere_marker.color.a = rgba_color(3);

        pub->publish(sphere_marker);
    }

    void publishTriMarker(
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
        int marker_id,
        const std::string &ns,
        const std::string &frame_id,
        const Vector3f &p0,
        const Vector3f &p1,
        const Vector3f &p2,
        float thickness,
        Vector4f rgba_color)
    {
        // Build a TRIANGLE_LIST marker from the 3 points
        visualization_msgs::msg::Marker tri_marker;
        tri_marker.header.stamp = this->now();
        tri_marker.header.frame_id = frame_id;
        tri_marker.ns = ns;
        tri_marker.id = marker_id;
        tri_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        tri_marker.action = visualization_msgs::msg::Marker::ADD;

        // We'll set scale = 1,1,1 since we're specifying actual points
        tri_marker.scale.x = 1.0f;
        tri_marker.scale.y = 1.0f;
        tri_marker.scale.z = 1.0f;

        // Use the color passed in
        tri_marker.color.r = rgba_color(0);
        tri_marker.color.g = rgba_color(1);
        tri_marker.color.b = rgba_color(2);
        tri_marker.color.a = rgba_color(3);

        // No rotation offset in the marker's own pose
        tri_marker.pose.orientation.w = 1.0f;

        // Convert the three Eigen points to geometry_msgs::Point
        auto eigenToPoint = [&](const Vector3f &v) {
            geometry_msgs::msg::Point pt;
            pt.x = v.x();
            pt.y = v.y();
            pt.z = v.z();
            return pt;
        };

        tri_marker.points.push_back(eigenToPoint(p0));
        tri_marker.points.push_back(eigenToPoint(p1));
        tri_marker.points.push_back(eigenToPoint(p2));

        // Publish the marker
        pub->publish(tri_marker);
    }

    std::vector<Vector3f> generateArcPoints(
        const Vector3f &centre,
        const Vector3f &planeN,
        const Vector3f &startDir,
        float radius,
        float startDeg,
        float endDeg,
        int steps = 10)
    {
        std::vector<Vector3f> arc_pts;
        arc_pts.reserve(steps + 1);

        float startRad = startDeg * M_PI / 180.0f;
        float endRad   = endDeg   * M_PI / 180.0f;

        // We'll assume planeN and startDir are unit vectors, orthonormal in-plane.
        // If needed, we can orthonormalize them here, but let's assume the user does that.

        float delta = (endRad - startRad) / float(steps);
        
        for (int i = 0; i <= steps; i++)
        {
            float angle = startRad + i * delta;
            // param eq: P(angle) = centre + radius*( cos(angle)*startDir + sin(angle)*(planeN x startDir) )
            // But we need a second in-plane direction orthonormal to startDir in planeN:
            // We'll define crossDir = planeN.cross(startDir).normalized().
            Vector3f crossDir = planeN.cross(startDir).normalized();

            float c = std::cos(angle);
            float s = std::sin(angle);
            Vector3f arc_pt = centre + radius * (c * startDir + s * crossDir);

            arc_pts.push_back(arc_pt);
        }
        return arc_pts;
    }

    void publishArcLineStrip(
        const std::vector<Vector3f> &arc_pts,
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
        int marker_id,
        const std::string &ns,
        const std::string &frame_id,
        float thickness,
        Vector4f rgba_color)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = frame_id;
        marker.ns = ns;
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // thickness in meters (scale.x is line width for LINE_STRIP)
        marker.scale.x = thickness;

        marker.color.r = rgba_color(0);
        marker.color.g = rgba_color(1);
        marker.color.b = rgba_color(2);
        marker.color.a = rgba_color(3);

        marker.pose.orientation.w = 1.0f; // no rotation offset

        for (auto &pt : arc_pts)
        {
            geometry_msgs::msg::Point geom_pt;
            geom_pt.x = pt.x();
            geom_pt.y = pt.y();
            geom_pt.z = pt.z();
            marker.points.push_back(geom_pt);
        }

        pub->publish(marker);
    }


  
    void getTargetPose()
    {
        // Update time
        k_ += 1;
        t_ = k_ * Ts_;

        // Target orientation for the IK problem
        float freq = 0.1;  // [cycles/callback]
        float th = (0.25 * M_PI) * std::sin(2.0 * M_PI * freq * t_);
        // float th = 0.0;

        Vector4d axis_ang(0.0, 0.0, 1.0, (double) th);
        // Vector4d axis_ang(0.0, 1.0, 0.0, (double) th);
        // Vector4d axis_ang(1.0, 0.0, 0.0, (double) th);

        Vector3d so3 = (axis_ang.head(3)).normalized() * axis_ang(3);
        quat_cmd_ = RM::so32Quat(so3).cast<float>();
    }

    void solveIK() 
    {
        // Call our CGA-based IK solver for the spherical robot
        ik_result_ = cga_ik_spherical_robot::computeSphericalRobotIK(r_b_, r_e_,
                                                                     e_principal_, 
                                                                     rot_cen_, 
                                                                     s_0_, s_1_, 
                                                                     quat_cmd_);
        
    }

    void publishIKSolution()
    {
        // Publish the joint states for these angles
        sensor_msgs::msg::JointState js_msg;
        js_msg.header.stamp = this->now();
        js_msg.name = {"joint_0", "joint_1", "joint_2"};
        js_msg.position = {ik_result_.th_0, ik_result_.th_1, ik_result_.th_2};
        joint_pub_->publish(js_msg);

        // Optionally publish them as an array
        std_msgs::msg::Float64MultiArray angles_msg;
        angles_msg.data = {ik_result_.th_0 * RM::r2d, ik_result_.th_1 * RM::r2d, ik_result_.th_2 * RM::r2d};
        angles_pub_->publish(angles_msg);
    }

    void publishAllTransforms()
    {
        
        // Poses w.r.t. the rotation centre
        // Motors of the robot
        publishTF(ik_result_.pos_rot_cen_m_0, ik_result_.quat_rot_cen_m_0, "srb_motor_0", "srb_rot_cen");
        publishTF(ik_result_.pos_rot_cen_m_1, ik_result_.quat_rot_cen_m_1, "srb_motor_1", "srb_rot_cen");
        publishTF(ik_result_.pos_rot_cen_m_2, ik_result_.quat_rot_cen_m_2, "srb_motor_2", "srb_rot_cen");

        // End-plate corners and centre 
        publishTF(ik_result_.pos_rot_cen_epl_0, ik_result_.quat_rot_cen_epl_0, "srb_epl_0", "srb_rot_cen");
        publishTF(ik_result_.pos_rot_cen_epl_1, ik_result_.quat_rot_cen_epl_1, "srb_epl_1", "srb_rot_cen");
        publishTF(ik_result_.pos_rot_cen_epl_2, ik_result_.quat_rot_cen_epl_2, "srb_epl_2", "srb_rot_cen");
        publishTF(ik_result_.pos_rot_cen_epl_c, ik_result_.quat_rot_cen_epl_c, "srb_epl_c", "srb_rot_cen");

        // End-point on the outer sphere
        publishTF(ik_result_.pos_rot_cen_ept, ik_result_.quat_rot_cen_ept, "srb_ept", "srb_rot_cen");

        // Elbows
        publishTF(ik_result_.pos_rot_cen_elb_0, ik_result_.quat_rot_cen_elb_0, "srb_elbow_0", "srb_rot_cen");
        publishTF(ik_result_.pos_rot_cen_elb_1, ik_result_.quat_rot_cen_elb_1, "srb_elbow_1", "srb_rot_cen");
        publishTF(ik_result_.pos_rot_cen_elb_2, ik_result_.quat_rot_cen_elb_2, "srb_elbow_2", "srb_rot_cen");

        // // (Optional) end-effector
        // Vector3f pos_yc_ee = (ik_result_.r_s - ik_result_.d) * cga_utils::G2R(e_principal_);
        // publishTF(pos_yc_ee, Quaternionf::Identity(), "srb_ee", "srb_epl_c");

    }
    

    void publisherMarkers()
    {
        // Publish outer sphere marker
        publishSphereMarker(
            srb_outer_sphere_pub_,
            0,                // marker_id
            "srb_outer_sphere",         // ns
            "srb_rot_cen",       // frame_id
            ik_result_.r_s,
            Vector3f::Zero(), // at rot_cen
            outer_sphere_color_ // RGBA
        );

        // Publish triangle markers
        // Lower triangles for rot_cen-elb-m
        publishTriMarker(
                         srb_ltri_0_pub_,
                         0,                // marker_id
                         "srb_ltri_0",         // ns
                         "srb_rot_cen",       // frame_id
                         Vector3f::Zero(), // at rot_cen
                         ik_result_.pos_rot_cen_elb_0,
                         ik_result_.pos_rot_cen_m_0,
                         plate_thickness_,
                         ltri_color_
                        );
        
        publishTriMarker(
                         srb_ltri_1_pub_,
                         0,                // marker_id
                         "srb_ltri_1",         // ns
                         "srb_rot_cen",       // frame_id
                         Vector3f::Zero(), // at rot_cen
                         ik_result_.pos_rot_cen_elb_1,
                         ik_result_.pos_rot_cen_m_1,
                         plate_thickness_,
                         ltri_color_
                        );
        
        publishTriMarker(
                         srb_ltri_2_pub_,
                         0,                // marker_id
                         "srb_ltri_2",         // ns
                         "srb_rot_cen",       // frame_id
                         Vector3f::Zero(), // at rot_cen
                         ik_result_.pos_rot_cen_elb_2,
                         ik_result_.pos_rot_cen_m_2,
                         plate_thickness_,
                         ltri_color_
                        );

        // Upper triangles for rot_cen-elb-epl
        publishTriMarker(
                         srb_utri_0_pub_,
                         0,                // marker_id
                         "srb_utri_0",         // ns
                         "srb_rot_cen",       // frame_id
                         Vector3f::Zero(), // at rot_cen
                         ik_result_.pos_rot_cen_elb_0,
                         ik_result_.pos_rot_cen_epl_0,
                         plate_thickness_,
                         utri_color_
                        );
        
        publishTriMarker(
                         srb_utri_1_pub_,
                         0,                // marker_id
                         "srb_utri_1",         // ns
                         "srb_rot_cen",       // frame_id
                         Vector3f::Zero(), // at rot_cen
                         ik_result_.pos_rot_cen_elb_1,
                         ik_result_.pos_rot_cen_epl_1,
                         plate_thickness_,
                         utri_color_
                        );
        
        publishTriMarker(
                         srb_utri_2_pub_,
                         0,                // marker_id
                         "srb_utri_2",         // ns
                         "srb_rot_cen",       // frame_id
                         Vector3f::Zero(), // at rot_cen
                         ik_result_.pos_rot_cen_elb_2,
                         ik_result_.pos_rot_cen_epl_2,
                         plate_thickness_,
                         utri_color_
                        );



    }

    void debuggingInfo()
    {
        // // Compute the are of the end-plate triangle

        // // Compare the lengths
        // std::cout << "(Outer sphere radius) r_s = d_rc_elp_i = d_rc_elb_i = " << ik_result_.r_s << std::endl;
        // std::cout << "d = d_rc_yc = " << ik_result_.d << std::endl;


    }

    void visualise_spherical_robot_tf_callback_()
    {
        
        auto start = std::chrono::steady_clock::now();

        // Get IK target pose (orientation)
        getTargetPose();
        // Solve IK (joint angles) for the spherical robot
        solveIK();

        auto end = std::chrono::steady_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
        double frequency = (elapsed_ms > 0.0) ? 1000.0 / elapsed_ms : 0.0;
        RCLCPP_INFO(this->get_logger(), "Ts = %.2f [ms], fs = %.2f [Hz]", elapsed_ms, frequency);


        // Publish IK solution:
        publishIKSolution();
        // Publish all TF frames:
        publishAllTransforms();
        // Publish markers
        publisherMarkers();
        // Debugging
        debuggingInfo();
    }


};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualiseSphericalRobotTF>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
