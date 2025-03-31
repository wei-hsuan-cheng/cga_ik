#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <cmath>

#include "cga_ik/cga_utils.hpp"
#include "cga_ik_spherical_robot/cga_ik_spherical_robot.hpp"
#include "robot_math_utils/robot_math_utils_v1_9.hpp"

// We'll use an inline namespace for convenience
namespace
{
// A simple helper to convert radians to degrees
inline double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}

} // anonymous namespace


class VisualiseSphericalRobotTF : public rclcpp::Node
{
public:
  VisualiseSphericalRobotTF()
  : Node("visualise_spherical_robot_tf"), t_(0.0)
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

    RCLCPP_INFO(this->get_logger(), "visualise_spherical_robot_tf node started.");
  }

private:
  void timerCallback()
  {
    // 1) Update time
    t_ += 1.0;  // increment once per callback

    // 2) Compute sinusoidal angles in radians
    //    For example, each motor angle changes with different frequencies
    double freq0 = 0.001;  // [cycles/callback]
    double freq1 = 0.0007;
    double freq2 = 0.0004;

    double theta0 = 0.2 * std::sin(2.0 * M_PI * freq0 * t_);
    double theta1 = 0.4 * std::cos(2.0 * M_PI * freq1 * t_);
    double theta2 = 0.3 * std::sin(2.0 * M_PI * freq2 * t_);

    // double theta0 = 0.0;
    // double theta1 = 0.0;
    // double theta2 = 0.0;

    // 3) Call our CGA-based IK solver for the spherical robot
    //    geometry: outer sphere radius = 0.5, end plate radius = 0.4
    auto ik_result = cga_ik_spherical_robot::computeSphericalRobotIK(
                      float(theta0), float(theta1), float(theta2),
                      /*r_b*/ 0.5f, /*r_e*/ 0.4f);

    // This returns the corners (y0,y1,y2), elbows (elb0,elb1,elb2), 
    // final endpoint, and motor angles in radians (angle0,angle1,angle2).

    // 4) Publish the joint states for these angles
    sensor_msgs::msg::JointState js_msg;
    js_msg.header.stamp = this->now();
    js_msg.name = {"joint_0", "joint_1", "joint_2"};
    js_msg.position = {ik_result.angle0, ik_result.angle1, ik_result.angle2};
    joint_pub_->publish(js_msg);

    // Optionally publish them as an array
    std_msgs::msg::Float64MultiArray angles_msg;
    angles_msg.data = {ik_result.angle0, ik_result.angle1, ik_result.angle2};
    angles_pub_->publish(angles_msg);

    // 5) Publish TF frames:
    //    We define a base frame "srb_base", pivot frames "srb_pivot_i",
    //    elbow frames "srb_elbow_i", and an end-effector frame "srb_ee".
    //    The base will be placed in the “world” frame with an offset if desired.

    // (A) The base transform (world -> srb_base) as identity
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

    // (B) We want to visualize each pivot i and each elbow i
    //     The pivot for motor i is at r_b * s_i + rotation_centre in 3D
    //     The elbow is from ik_result.elb_i (a conformal point).
    //     We'll place them as frames w.r.t. the base.
    //     We define a small helper to build a geometry_msgs transform from a 3D position only.
    auto publishPointAsTF = [&](const Eigen::Vector3f &p, const std::string &child, const std::string &parent)
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


    // Extract each pivot in 3D:
    Eigen::Vector3f pivot0_3d = cga_utils::G2R( ik_result.s0 );
    Eigen::Vector3f pivot1_3d = cga_utils::G2R( ik_result.s1 );
    Eigen::Vector3f pivot2_3d = cga_utils::G2R( ik_result.s2 );

    // std::cout << "pivot0_3d = " << pivot0_3d.transpose() << std::endl;
    // std::cout << "pivot1_3d = " << pivot1_3d.transpose() << std::endl;
    // std::cout << "pivot2_3d = " << pivot2_3d.transpose() << std::endl;

    // Publish them w.r.t. base:
    publishPointAsTF(pivot0_3d, "srb_pivot_0", "srb_base");
    publishPointAsTF(pivot1_3d, "srb_pivot_1", "srb_base");
    publishPointAsTF(pivot2_3d, "srb_pivot_2", "srb_base");

    // Rotation centre of the robot

    Eigen::Vector3f rotation_centre_3d = cga_utils::G2R( ik_result.rotation_centre );
    publishPointAsTF(rotation_centre_3d, "rotation_centre", "srb_base");

    // (C) Elbow frames: use (ik_result.elb0, elb1, elb2)
    // down(...) each to get a 3D location

    Eigen::Vector3f elb0_3d = cga_utils::G2R( down(ik_result.elb0) );
    Eigen::Vector3f elb1_3d = cga_utils::G2R( down(ik_result.elb1) );
    Eigen::Vector3f elb2_3d = cga_utils::G2R( down(ik_result.elb2) );
    
    
    std::cout << "elb0 = " << elb0_3d.transpose() << std::endl;
    std::cout << "elb1 = " << elb1_3d.transpose() << std::endl;
    std::cout << "elb2 = " << elb2_3d.transpose() << std::endl;
    std::cout << "--------------------------------" << std::endl;


    publishPointAsTF(elb0_3d, "srb_elbow_0", "srb_base");
    publishPointAsTF(elb1_3d, "srb_elbow_1", "srb_base");
    publishPointAsTF(elb2_3d, "srb_elbow_2", "srb_base");

    // (D) End-effector: let's treat the “endpoint” from the IK result as a single final frame
    Eigen::Vector3f ee_3d = cga_utils::G2R( down(ik_result.endpoint) );
    publishPointAsTF(ee_3d, "srb_end_effector", "srb_base");

    // for publishing y0, y1, and y2
    Eigen::Vector3f y0_3d = cga_utils::G2R( down(ik_result.y0) );
    Eigen::Vector3f y1_3d = cga_utils::G2R( down(ik_result.y1) );
    Eigen::Vector3f y2_3d = cga_utils::G2R( down(ik_result.y2) );

    // std::cout << "y0_3d = " << y0_3d.transpose() << std::endl;
    // std::cout << "y1_3d = " << y1_3d.transpose() << std::endl;
    // std::cout << "y2_3d = " << y2_3d.transpose() << std::endl;
    // std::cout << "--------------------------------" << std::endl;


    publishPointAsTF(y0_3d, "srb_y_0", "srb_base");
    publishPointAsTF(y1_3d, "srb_y_1", "srb_base");
    publishPointAsTF(y2_3d, "srb_y_2", "srb_base");




  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // We'll also publish 3 joint angles
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angles_pub_;

  double t_; // internal time-step counter
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualiseSphericalRobotTF>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
