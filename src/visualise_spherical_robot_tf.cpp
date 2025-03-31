#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <cmath>

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
    // Set up a TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Frequency ~ 30 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),  // ~30 fps
      std::bind(&VisualiseSphericalRobotTF::timerCallback, this));

    // We’ll publish the spherical robot’s joint angles (3 DoF)
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/spherical_robot/joint_states", 10);

    // We can also publish a Float64MultiArray if you want to replicate that
    angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/spherical_robot/angles_array", 10);

    // Publisher for a Marker that draws the triangle end-plate
    srb_ee_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_ee", 10);
    srb_base_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_base", 10);
    srb_ltri_0_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_ltri_0", 10);
    srb_ltri_1_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_ltri_1", 10);
    srb_ltri_2_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_ltri_2", 10);
    srb_utri_0_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_utri_0", 10);
    srb_utri_1_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_utri_1", 10);
    srb_utri_2_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spherical_robot/markers/srb_utri_2", 10);

    // Initialise time spec
    initTimeSpec();
    // Initialise CGA IK for spherical robot
    initSRBIK();

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

    float r_b_, r_e_;
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
        r_b_ = 0.5f;
        r_e_ = 0.4f;
        ik_result_ = cga_ik_spherical_robot::computeSphericalRobotIK(Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f), r_b_, r_e_);
    }

    void publishPointAsTF(const Eigen::Vector3f &p, const std::string &child, const std::string &parent)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = parent;
        tf.child_frame_id = child;
        tf.transform.translation.x = p.x();
        tf.transform.translation.y = p.y();
        tf.transform.translation.z = p.z();

        // No orientation, so we set identity
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
        tf.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(tf);
    };

    void publishTriMarker(
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
        int marker_id,
        const std::string &ns,
        const std::string &frame_id,
        const Eigen::Vector3f &p0,
        const Eigen::Vector3f &p1,
        const Eigen::Vector3f &p2,
        float r_color,  // Red   [0..1]
        float g_color,  // Green [0..1]
        float b_color,  // Blue  [0..1]
        float a_color   // Alpha [0..1]
    )
    {
        // Build a TRIANGLE_LIST marker from the 3 points p0, p1, p2
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
        auto eigenToPoint = [&](const Eigen::Vector3f &v) {
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

  
    void timerCallback()
    {
        // 1) Update time
        k_ += 1;
        t_ = k_ * Ts_;

        // 2) Target orientation for the IK problem
        float freq = 0.1;  // [cycles/callback]
        float th = (0.25 * M_PI) * std::sin(2.0 * M_PI * freq * t_);

        // Vector4d axis_ang(0.0, 0.0, 1.0, (double) th);
        // Vector4d axis_ang(0.0, 1.0, 0.0, (double) th);
        Vector4d axis_ang(1.0, 0.0, 0.0, (double) th);

        Vector3d so3 = (axis_ang.head(3)).normalized() * axis_ang(3); // u_hat * theta
        Eigen::Quaternionf quat = RM::so32Quat(so3).cast<float>();

        // 3) Call our CGA-based IK solver for the spherical robot
        ik_result_ = cga_ik_spherical_robot::computeSphericalRobotIK(quat, r_b_, r_e_);

        // 4) Publish the joint states for these angles
        sensor_msgs::msg::JointState js_msg;
        js_msg.header.stamp = this->now();
        js_msg.name = {"joint_0", "joint_1", "joint_2"};
        js_msg.position = {ik_result_.angle0, ik_result_.angle1, ik_result_.angle2};
        joint_pub_->publish(js_msg);

        // Optionally publish them as an array
        std_msgs::msg::Float64MultiArray angles_msg;
        angles_msg.data = {ik_result_.angle0 * RM::r2d, ik_result_.angle1 * RM::r2d, ik_result_.angle2 * RM::r2d};
        angles_pub_->publish(angles_msg);

        // 5) Publish TF frames:
        {
            geometry_msgs::msg::TransformStamped base_tf;
            base_tf.header.stamp = this->now();
            base_tf.header.frame_id = "world";
            base_tf.child_frame_id = "srb_base";

            base_tf.transform.translation.x = 0.0;
            base_tf.transform.translation.y = 0.0;
            base_tf.transform.translation.z = 0.0;
            base_tf.transform.rotation.w = 1.0; // identity rotation

            tf_broadcaster_->sendTransform(base_tf);
        }

        // Extract each motor in 3D:
        publishPointAsTF(ik_result_.m0, "srb_motor_0", "srb_base");
        publishPointAsTF(ik_result_.m1, "srb_motor_1", "srb_base");
        publishPointAsTF(ik_result_.m2, "srb_motor_2", "srb_base");

        // Rotation centre of the robot
        publishPointAsTF(ik_result_.rotation_centre, "srb_rot_cen", "srb_base");

        // For publishing y0, y1, and y2
        publishPointAsTF(ik_result_.y0, "srb_y_0", "srb_base");
        publishPointAsTF(ik_result_.y1, "srb_y_1", "srb_base");
        publishPointAsTF(ik_result_.y2, "srb_y_2", "srb_base");

        // Elbow frames: use (ik_result_.elb0, elb1, elb2)
        publishPointAsTF(ik_result_.elb0, "srb_elbow_0", "srb_base");
        publishPointAsTF(ik_result_.elb1, "srb_elbow_1", "srb_base");
        publishPointAsTF(ik_result_.elb2, "srb_elbow_2", "srb_base");

        // End-effector: let's treat the “endpoint” from the IK result as a single final frame
        publishPointAsTF(ik_result_.endpoint, "srb_ee", "srb_base");

        // Publish markers
        // End-plate (end-effector)
        publishTriMarker(
                         srb_ee_marker_pub_,
                         0,                // marker_id
                         "srb_ee",         // ns
                         "srb_base",       // frame_id
                         ik_result_.y0,
                         ik_result_.y1,
                         ik_result_.y2,
                         1.0f, 0.0f, 0.0f, 1.0f // RGBA for red
                        );
        
        // Base
        publishTriMarker(
                         srb_base_marker_pub_,
                         0,                // marker_id
                         "srb_base",         // ns
                         "srb_base",       // frame_id
                         ik_result_.m0,
                         ik_result_.m1,
                         ik_result_.m2,
                         0.0f, 0.0f, 0.0f, 1.0f // RGBA for red
                        );
        
        // Lower triangles for centre-elb-motor
        publishTriMarker(
                         srb_ltri_0_pub_,
                         0,                // marker_id
                         "srb_ltri_0",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rotation_centre,
                         ik_result_.elb0,
                         ik_result_.m0,
                         0.0f, 1.0f, 0.0f, 1.0f // RGBA for red
                        );
        
        publishTriMarker(
                         srb_ltri_1_pub_,
                         0,                // marker_id
                         "srb_ltri_1",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rotation_centre,
                         ik_result_.elb1,
                         ik_result_.m1,
                         0.0f, 1.0f, 0.0f, 1.0f // RGBA for red
                        );
        
        publishTriMarker(
                         srb_ltri_2_pub_,
                         0,                // marker_id
                         "srb_ltri_2",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rotation_centre,
                         ik_result_.elb2,
                         ik_result_.m2,
                         0.0f, 1.0f, 0.0f, 1.0f // RGBA for red
                        );

        // Upper triangles for centre-elb-plate
        publishTriMarker(
                         srb_utri_0_pub_,
                         0,                // marker_id
                         "srb_utri_0",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rotation_centre,
                         ik_result_.elb0,
                         ik_result_.y0,
                         0.0f, 0.0f, 1.0f, 1.0f // RGBA for red
                        );
        
        publishTriMarker(
                         srb_utri_1_pub_,
                         0,                // marker_id
                         "srb_utri_1",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rotation_centre,
                         ik_result_.elb1,
                         ik_result_.y1,
                         0.0f, 0.0f, 1.0f, 1.0f // RGBA for red
                        );
        
        publishTriMarker(
                         srb_utri_2_pub_,
                         0,                // marker_id
                         "srb_utri_2",         // ns
                         "srb_base",       // frame_id
                         ik_result_.rotation_centre,
                         ik_result_.elb2,
                         ik_result_.y2,
                         0.0f, 0.0f, 1.0f, 1.0f // RGBA for red
                        );



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
