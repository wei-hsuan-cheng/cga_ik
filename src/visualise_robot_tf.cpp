#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <memory>

#include "robot_math_utils/robot_math_utils_v1_8.hpp"
#include "cga_ik/cga_ik.hpp"

using namespace std::chrono_literals;
using RM = RMUtils;

class VisualiseRobotTF : public rclcpp::Node
{
public:
  VisualiseRobotTF()
  : Node("visualise_robot_tf"), t_(0.0), dh_table_(cga_ik::loadDHTable())
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(16ms, std::bind(&VisualiseRobotTF::visualise_robot_tf_callback_, this));

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_angles", 10);
    
    // Initialization
    target_pose_ = Vector6d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    robot_config_ = cga_ik::setRobotConfig(1, 1, 1);
    joint_angles_ = Vector6d(0.0, 0.0, 90.0, 0.0, 90.0, -90.0) * RM::d2r;
    resulting_pose_ = Vector6d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

private:
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;

  // DH table and joint angles.
  DHTable dh_table_;
  Vector6d target_pose_;
  cga_ik::CGAIKRobotConfig robot_config_;
  Vector6d joint_angles_;
  Vector6d resulting_pose_;
  double t_;

  // Define conversion factors.
  const double mm2m = 1e-3;

  // Helper function to compute the D-H transformation
  PosQuat dh_transform(double alpha, double a, double d, double theta)
  {
    // Rotation about x-axis by alpha.
    PosQuat pos_quat_alpha;
    pos_quat_alpha.pos = Vector3d::Zero();
    pos_quat_alpha.quat = RM::Quatx(alpha);

    // Translation along x-axis by a.
    PosQuat pos_quat_a;
    pos_quat_a.pos = Vector3d(a, 0, 0);
    pos_quat_a.quat = Quaterniond::Identity();
    
    // Translation along z-axis by d.
    PosQuat pos_quat_d;
    pos_quat_d.pos = Vector3d(0, 0, d);
    pos_quat_d.quat = Quaterniond::Identity();

    // Rotation about z-axis by theta.
    PosQuat pos_quat_theta;
    pos_quat_theta.pos = Vector3d::Zero();
    pos_quat_theta.quat = RM::Quatz(theta);
    
    return RM::TransformPosQuats({pos_quat_alpha, pos_quat_a, pos_quat_d, pos_quat_theta});
  }

  
  // Helper function to publish a transform.
  void publish_tf(const PosQuat& pos_quat, const std::string& child_frame_id, const std::string& parent_frame_id)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = parent_frame_id;
    tf_msg.child_frame_id = child_frame_id;
    tf_msg.transform.translation.x = pos_quat.pos(0);
    tf_msg.transform.translation.y = pos_quat.pos(1);
    tf_msg.transform.translation.z = pos_quat.pos(2);
    tf_msg.transform.rotation.w = pos_quat.quat.w();
    tf_msg.transform.rotation.x = pos_quat.quat.x();
    tf_msg.transform.rotation.y = pos_quat.quat.y();
    tf_msg.transform.rotation.z = pos_quat.quat.z();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  // Update the target pose in a time-varying way.
  void get_target_pose()
  {
    t_ += 1.0;  // Increase by one per callback

    double d1 = dh_table_.dh_table(0, 2);
    double a2 = dh_table_.dh_table(2, 1);
    double a3 = dh_table_.dh_table(3, 1);
    double d4 = dh_table_.dh_table(3, 2);
    double d5 = dh_table_.dh_table(4, 2);
    double d6 = dh_table_.dh_table(5, 2);

    target_pose_ = Vector6d(a3 + d5, d4, d1 + a2 - d6, M_PI, M_PI / 4.0, M_PI);

    // double offset_x = 0.1 * sin(2.0 * M_PI * 0.001 * t_); // [m]
    // double offset_y = 0.1 * cos(2.0 * M_PI * 0.002 * t_); // [m]
    // double offset_z = 0.1 * sin(2.0 * M_PI * 0.003 * t_); // [m]
    // double offset_thx = 0.1 * sin(2.0 * M_PI * 0.004 * t_); // [rad]
    // double offset_thy = 0.1 * cos(2.0 * M_PI * 0.005 * t_); // [rad]
    // double offset_thz = 0.1 * sin(2.0 * M_PI * 0.006 * t_); // [rad]

    double offset_x = 0.1 * cos(2.0 * M_PI * 0.003 * t_); // [m]
    double offset_y = 0.0; // [m]
    double offset_z = 0.1 * sin(2.0 * M_PI * 0.003 * t_); // [m]
    double offset_thx = 0.0; // [rad]
    double offset_thy = 0.0; // [rad]
    double offset_thz = 0.0; // [rad]

    Vector6d offset(offset_x, offset_y, offset_z, offset_thx, offset_thy, offset_thz);
    target_pose_ += offset;

    // RM::PrintVec(target_pose_, "\ntarget_pose [m, rad]");

  }

  void solve_ik() {
    // Different robot configurations
    robot_config_ = cga_ik::setRobotConfig(1, 1, 1);
    // robot_config_ = cga_ik::setRobotConfig(1, -1, -1);
    // robot_config_ = cga_ik::setRobotConfig(1, 1, -1);
    // robot_config_ = cga_ik::setRobotConfig(-1, 1, 1);
    // robot_config_ = cga_ik::setRobotConfig(1, -1, 1);

    cga_ik::SolveNullPoints(target_pose_, dh_table_, robot_config_);
    
    if (cga_ik::reachable) {
        cga_ik::SolveJointAngles();
        joint_angles_ = cga_ik::joints;
        // RM::PrintVec(joint_angles_ * RM::r2d, "\njoint_angles [deg]");
    } else {
        std::cerr << "\nTarget target_pose is not reachable!" << std::endl;
    }
  }

  // Update the joint angles in a time-varying way.
  void time_varying_joint_angles()
  {
    t_ += 1.0;  // Increase by one per callback (like the Python counter)
    double theta1 = 10.0 * sin(2.0 * M_PI * 0.001 * t_);
    double theta2 = 10.0 * cos(2.0 * M_PI * 0.002 * t_);
    double theta3 = 10.0 * sin(2.0 * M_PI * 0.003 * t_);
    double theta4 = 10.0 * cos(2.0 * M_PI * 0.004 * t_);
    double theta5 = 10.0 * sin(2.0 * M_PI * 0.005 * t_);
    double theta6 = 10.0 * cos(2.0 * M_PI * 0.006 * t_);
    
    Vector6d new_angles(theta1, theta2, theta3, theta4, theta5, theta6);
    new_angles = new_angles * RM::d2r;  // convert to radians
    Vector6d offset(0.0, 0.0, 90.0, 0.0, 90.0, -90.0);
    offset = offset * RM::d2r;
    joint_angles_ = new_angles + offset;
  }
  
  // Publish transforms for base, each joint, and the end-effector.
  void publish_transforms()
  {
    // Publish joint angles
    auto message = std_msgs::msg::Float64MultiArray();
    std::vector<double> angles(joint_angles_.data(), joint_angles_.data() + joint_angles_.size());
    message.data = angles;
    publisher_->publish(message);

    // Publish the base transform (world -> base) as identity.
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "base";
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_broadcaster_->sendTransform(tf_msg);
    
    // For each joint (6 joints) compute and publish its transform.
    for (int i = 0; i < 6; i++) {
      double alpha = dh_table_.dh_table(i, 0);
      double a = dh_table_.dh_table(i, 1);
      double d = dh_table_.dh_table(i, 2);
      double dh_offset = dh_table_.dh_table(i, 3);
      double theta = joint_angles_(i) + dh_offset;
      
      PosQuat transform = dh_transform(alpha, a, d, theta);
      
      std::string child_frame = "j" + std::to_string(i + 1);
      std::string parent_frame = (i == 0) ? "base" : "j" + std::to_string(i);
      publish_tf(transform, child_frame, parent_frame);
    }

    // CGA FK
    resulting_pose_ = cga_ik::CGAFK(joint_angles_, dh_table_);

    // Compare FK and IK
    Vector6d pose_res = RM::R6Poses2RelativeR6Pose(target_pose_, resulting_pose_);
    RCLCPP_INFO(this->get_logger(), 
      "Residual pose [m, rad] = %.4f, %.4f, %.4f, %.4f, %.4f, %.4f", 
      pose_res(0), pose_res(1), pose_res(2), pose_res(3), pose_res(4), pose_res(5)
      );

    publish_tf(RM::R6Pose2PosQuat(target_pose_), "j6_ik", "base");
    publish_tf(RM::R6Pose2PosQuat(resulting_pose_), "j6_fk", "base");

    // Publish the end-effector (E) transform.
    // For example, we set a fixed transform for E relative to joint 6.
    PosQuat pos_quat_6_e = RM::R6Pose2PosQuat(Vector6d(0, 0, 100 * mm2m, 0, 0, 0));
    publish_tf(pos_quat_6_e, "E", "j6"); 

  }
  
  // Timer callback runs at ~30 Hz.
  void visualise_robot_tf_callback_()
  {
    auto start = std::chrono::steady_clock::now();
    get_target_pose();
    solve_ik();
    auto end = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
    double frequency = (elapsed_ms > 0.0) ? 1000.0 / elapsed_ms : 0.0;
    RCLCPP_INFO(this->get_logger(), "dt = %.2f [ms], f = %.2f [Hz]", elapsed_ms, frequency);


    publish_transforms();
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualiseRobotTF>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
