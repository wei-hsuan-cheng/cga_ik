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


    // Set up a TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(Ts_ * 1000)),
      std::bind(&VisualiseSphericalRobotTF::visualise_spherical_robot_tf_callback_, this)
    );

    // Publishers
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/spherical_robot/joint_states", 10);
    angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/spherical_robot/angles_array", 10);

    srb_ee_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_ee", 10);
    srb_base_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_base", 10);
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

    // Publisher for the triangle marker
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr srb_ee_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr srb_base_marker_pub_;
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

    float r_b_, ratio_e_b_, r_e_;
    CGA e_principal_, rot_cen_, s_0_, s_1_;
    cga_ik_spherical_robot::SphericalRobotIKResult ik_result_;
    

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
        // Base radius and end-plate radius
        r_b_ = 0.5f;
        ratio_e_b_ = 0.6f;
        r_e_ = r_b_ * ratio_e_b_;

        // Spherical robot coordinate system
        // Principal axis of the 3-DoF spherical robot
        e_principal_ = e3;

        // Rotation centre
        rot_cen_ = (1.0f / 3.0f) * r_b_ * e_principal_;

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

    void publishTriMarker(
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
        int marker_id,
        const std::string &ns,
        const std::string &frame_id,
        const Vector3f &p0,
        const Vector3f &p1,
        const Vector3f &p2,
        float r_color,  // Red   [0..1]
        float g_color,  // Green [0..1]
        float b_color,  // Blue  [0..1]
        float a_color   // Alpha [0..1]
        )
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
        tri_marker.color.r = r_color;
        tri_marker.color.g = g_color;
        tri_marker.color.b = b_color;
        tri_marker.color.a = a_color;

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

  
    void getTargetPose()
    {
        // Update time
        k_ += 1;
        t_ = k_ * Ts_;

        // Target orientation for the IK problem
        float freq = 0.1;  // [cycles/callback]
        float th = (0.25 * M_PI) * std::sin(2.0 * M_PI * freq * t_);
        // float th = 0.0;

        // Vector4d axis_ang(0.0, 0.0, 1.0, (double) th);
        // Vector4d axis_ang(0.0, 1.0, 0.0, (double) th);
        Vector4d axis_ang(1.0, 0.0, 0.0, (double) th);

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
        {
            geometry_msgs::msg::TransformStamped base_tf;
            base_tf.header.stamp = this->now();
            base_tf.header.frame_id = "world";
            base_tf.child_frame_id = "srb_base";

            base_tf.transform.rotation.w = 1.0; // identity rotation
            base_tf.transform.translation.x = 0.0;
            base_tf.transform.translation.y = 0.0;
            base_tf.transform.translation.z = 0.0;
            
            tf_broadcaster_->sendTransform(base_tf);
        }

        // Motors of the robot
        publishTF(ik_result_.m_0, ik_result_.quat_m_0_i, "srb_motor_0_i", "srb_base");
        publishTF(ik_result_.m_0, ik_result_.quat_m_0, "srb_motor_0", "srb_base");
        
        publishTF(ik_result_.m_1, ik_result_.quat_m_1_i, "srb_motor_1_i", "srb_base");
        publishTF(ik_result_.m_1, ik_result_.quat_m_1, "srb_motor_1", "srb_base");

        publishTF(ik_result_.m_2, ik_result_.quat_m_2_i, "srb_motor_2_i", "srb_base");
        publishTF(ik_result_.m_2, ik_result_.quat_m_2, "srb_motor_2", "srb_base");


        // Rotation centre
        publishTF(ik_result_.rot_cen, Quaternionf::Identity(), "srb_rot_cen", "srb_base");


        // End-plate corners and centre 
        publishTF(ik_result_.epl_0, ik_result_.quat_epl_0, "srb_epl_0", "srb_base");
        publishTF(ik_result_.epl_1, ik_result_.quat_epl_1, "srb_epl_1", "srb_base");
        publishTF(ik_result_.epl_2, ik_result_.quat_epl_2, "srb_epl_2", "srb_base");
        publishTF(ik_result_.epl_c, ik_result_.quat_epl_c, "srb_epl_c", "srb_base");


        // Elbows
        publishTF(ik_result_.elb_0, Quaternionf::Identity(), "srb_elbow_0", "srb_base");
        publishTF(ik_result_.elb_1, Quaternionf::Identity(), "srb_elbow_1", "srb_base");
        publishTF(ik_result_.elb_2, Quaternionf::Identity(), "srb_elbow_2", "srb_base");


        // End-effector
        Vector3f pos_yc_ee = (ik_result_.r_s - ik_result_.d) * cga_utils::G2R(e_principal_);
        publishTF(pos_yc_ee, Quaternionf::Identity(), "srb_ee", "srb_epl_c");

    }

    void publisherMarkers()
    {
        // End-plate
        publishTriMarker(
                         srb_ee_marker_pub_,
                         0,                // marker_id
                         "srb_ee",         // ns
                         "srb_base",       // frame_id
                         ik_result_.epl_0,
                         ik_result_.epl_1,
                         ik_result_.epl_2,
                         1.0f, 0.0f, 0.0f, 1.0f // RGBA
                        );
        
        // Base
        publishTriMarker(
                         srb_base_marker_pub_,
                         0,                // marker_id
                         "srb_base",         // ns
                         "srb_base",       // frame_id
                         ik_result_.m_0,
                         ik_result_.m_1,
                         ik_result_.m_2,
                         1.0f, 0.0f, 0.0f, 1.0f // RGBA
                        );
        
        // Lower triangles for rot_cen-elb-m
        publishTriMarker(
                         srb_ltri_0_pub_,
                         0,                // marker_id
                         "srb_ltri_0",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rot_cen,
                         ik_result_.elb_0,
                         ik_result_.m_0,
                         0.0f, 1.0f, 0.0f, 1.0f // RGBA
                        );
        
        publishTriMarker(
                         srb_ltri_1_pub_,
                         0,                // marker_id
                         "srb_ltri_1",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rot_cen,
                         ik_result_.elb_1,
                         ik_result_.m_1,
                         0.0f, 1.0f, 0.0f, 1.0f // RGBA
                        );
        
        publishTriMarker(
                         srb_ltri_2_pub_,
                         0,                // marker_id
                         "srb_ltri_2",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rot_cen,
                         ik_result_.elb_2,
                         ik_result_.m_2,
                         0.0f, 1.0f, 0.0f, 1.0f // RGBA
                        );

        // Upper triangles for rot_cen-elb-epl
        publishTriMarker(
                         srb_utri_0_pub_,
                         0,                // marker_id
                         "srb_utri_0",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rot_cen,
                         ik_result_.elb_0,
                         ik_result_.epl_0,
                         0.0f, 0.0f, 1.0f, 1.0f // RGBA
                        );
        
        publishTriMarker(
                         srb_utri_1_pub_,
                         0,                // marker_id
                         "srb_utri_1",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rot_cen,
                         ik_result_.elb_1,
                         ik_result_.epl_1,
                         0.0f, 0.0f, 1.0f, 1.0f // RGBA
                        );
        
        publishTriMarker(
                         srb_utri_2_pub_,
                         0,                // marker_id
                         "srb_utri_2",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rot_cen,
                         ik_result_.elb_2,
                         ik_result_.epl_2,
                         0.0f, 0.0f, 1.0f, 1.0f // RGBA
                        );

    }

    void debuggingInfo()
    {
        // // Compute the are of the end-plate triangle
        // Vector3f v1 = ik_result_.epl_1 - ik_result_.epl_0;
        // Vector3f v2 = ik_result_.epl_2 - ik_result_.epl_0;
        // float A_tri_ep = (0.5 * v1.cross(v2)).norm();
        // // std::cout << "A_tri_ep = " << A_tri_ep << std::endl;

        // // Compare the lengths
        // std::cout << "(Outer sphere radius) r_s = d_rc_elp_i = d_rc_elb_i = " << ik_result_.r_s << std::endl;
        // std::cout << "d = d_rc_yc = " << ik_result_.d << std::endl;


    }

    void visualise_spherical_robot_tf_callback_()
    {
        
        // Get IK target pose (orientation)
        getTargetPose();

        // Solve IK (joint angles) for the spherical robot
        solveIK();

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
