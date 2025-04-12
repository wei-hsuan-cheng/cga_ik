#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "geometry_msgs/msg/quaternion.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <memory>

#include "robot_math_utils/robot_math_utils_v1_10.hpp"
#include "cga_ik_cobot_6dof/cga_ik_cobot_6dof.hpp"

using namespace std::chrono_literals;
using RM = RMUtils;

class VisualiseCobot6DoF : public rclcpp::Node
{
public:
  VisualiseCobot6DoF()
  : Node("visualise_cobot_6dof")
  {
    // Initialisation
    initTimeSpec();
    initDHTable();
    initTCPParams();
    initControlParams();
    initCobotIK();

    // ROS 2 components
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(Ts_ * 1000)),
      std::bind(&VisualiseCobot6DoF::visualise_cobot_6dof_callback_, this)
    );

    quat_sub_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
      "/mpu6050_imu/quat", 10,
      std::bind(&VisualiseCobot6DoF::quat_callback_, this, std::placeholders::_1));

    cga_ik_joint_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/visualise_cobot_6dof/joint_states", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    RCLCPP_INFO(this->get_logger(), "visualise_cobot_6dof node started.");
    
  }

private:
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr quat_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cga_ik_joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // Sampling rate and period
  double t_;
  float fs_, Ts_;

  // Controller params
  Quaterniond quat_cmd_;

  // DH table and joint angles.
  DHTable dh_table_;
  Vector6d target_pose_;
  cga_ik_cobot_6dof::RobotConfig robot_config_;
  Vector6d joint_angles_;
  Vector6d resulting_pose_;
  

  // TCP parameters
  PosQuat pos_quat_f_tcp_;

  // Define conversion factors.
  const double mm2m = 1e-3;
  const double m2mm = 1e3;

  // Subscriber callbacks
  void quat_callback_(const geometry_msgs::msg::Quaternion::SharedPtr msg)
  {
    quat_cmd_ = Quaterniond(msg->w, msg->x, msg->y, msg->z);
    // std::cout << "quat_cmd_ = " << quat_cmd_.w() << ", " << quat_cmd_.x() << ", " << quat_cmd_.y() << ", " << quat_cmd_.z() << std::endl;
  }

  DHParams convertDHParams(const std::vector<double> & dh_j_yaml)
  {
    // Convert to dh_params format with unit conversion: [mm -> m], [deg -> rad]
    return DHParams(dh_j_yaml[0] * RM::d2r, dh_j_yaml[1] * mm2m, dh_j_yaml[2] * mm2m, dh_j_yaml[3] * RM::d2r);
  }
  
  void initTimeSpec() 
  {
    t_ = 0.0;
    fs_ = 60.0;
    Ts_ = 1.0 / fs_;
  }

  void initDHTable()
  {
    // Load D-H table from yaml file
    this->declare_parameter<std::vector<double>>("dh_j1", {0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("dh_j2", {0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("dh_j3", {0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("dh_j4", {0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("dh_j5", {0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("dh_j6", {0.0, 0.0, 0.0, 0.0});

    std::vector<double> dh_j1_yaml;
    std::vector<double> dh_j2_yaml;
    std::vector<double> dh_j3_yaml;
    std::vector<double> dh_j4_yaml;
    std::vector<double> dh_j5_yaml;
    std::vector<double> dh_j6_yaml;

    this->get_parameter("dh_j1", dh_j1_yaml);
    this->get_parameter("dh_j2", dh_j2_yaml);
    this->get_parameter("dh_j3", dh_j3_yaml);
    this->get_parameter("dh_j4", dh_j4_yaml);
    this->get_parameter("dh_j5", dh_j5_yaml);
    this->get_parameter("dh_j6", dh_j6_yaml);

    // dh_table_ = cga_ik_cobot_6dof::loadTM5700DHTable();
    dh_table_ = DHTable({convertDHParams(dh_j1_yaml),
                         convertDHParams(dh_j2_yaml),
                         convertDHParams(dh_j3_yaml),
                         convertDHParams(dh_j4_yaml),
                         convertDHParams(dh_j5_yaml),
                         convertDHParams(dh_j6_yaml)});

    std::cout << "\n----- D-H table (alpha_{i-1}, a_{i-1}, d_{i}, theta_{i}) -----" << std::endl;
    std::cout << dh_table_ << std::endl;
  }

  void initTCPParams()
  {
    // Load pos_quat_f_tcp_ from yaml file
    // Declare default parameters (in case the YAML file doesn't provide them)
    this->declare_parameter<std::vector<double>>("pose_f_tcp_translation", {0.0, 0.0, 0.0});
    this->declare_parameter<double>("pose_f_tcp_rotation_z", 0.0);

    // Retrieve the parameters
    std::vector<double> translation;
    double rotation_z;
    this->get_parameter("pose_f_tcp_translation", translation);
    this->get_parameter("pose_f_tcp_rotation_z", rotation_z);
    pos_quat_f_tcp_ = PosQuat(Vector3d(translation[0], translation[1], translation[2]), 
                              RM::Quatz(rotation_z));

    std::cout << "[VisualiseCobot6DoF] Loaded params from yaml file" << std::endl;
    std::cout << "pos_quat_f_tcp_: " << pos_quat_f_tcp_.pos.transpose() << ", " << pos_quat_f_tcp_.quat.w() << ", " << pos_quat_f_tcp_.quat.x() << ", " << pos_quat_f_tcp_.quat.y() << ", " << pos_quat_f_tcp_.quat.z() << std::endl;
  }

  void initControlParams()
  {
    quat_cmd_ = Quaterniond::Identity();
  }

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
  void publishTF(const PosQuat& pos_quat, const std::string& child_frame_id, const std::string& parent_frame_id)
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

  void initCobotIK()
  {
    target_pose_ = Vector6d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    robot_config_ = cga_ik_cobot_6dof::setRobotConfig(1, 1, 1);
    joint_angles_ = Vector6d(0.0, 0.0, 90.0, 0.0, 90.0, -90.0) * RM::d2r;
    resulting_pose_ = Vector6d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  void getTargetPose()
  {
    t_ += 1.0;  // Increase by one per callback

    double d1 = dh_table_.dh_table(0, 2);
    double a2 = dh_table_.dh_table(2, 1);
    double a3 = dh_table_.dh_table(3, 1);
    double d4 = dh_table_.dh_table(3, 2);
    double d5 = dh_table_.dh_table(4, 2);
    double d6 = dh_table_.dh_table(5, 2);

    target_pose_ = Vector6d(a3 + d5, d4, d1 + a2 - d6, M_PI, M_PI / 4.0, M_PI);

    double f_motion = 0.001;
    double offset_x = 0.1 * cos(2.0 * M_PI * f_motion * t_); // [m]
    double offset_y = 0.0; // [m]
    double offset_z = 0.1 * sin(2.0 * M_PI * f_motion * t_); // [m]
    double offset_thx = 0.0; // [rad]
    double offset_thy = 0.0; // [rad]
    double offset_thz = 0.0; // [rad]

    Vector6d pose_offset(offset_x, offset_y, offset_z, offset_thx, offset_thy, offset_thz);
    target_pose_ += pose_offset;

    // RM::PrintVec(target_pose_, "\ntarget_pose [m, rad]");

  }

  void getPoseCommand()
  {
    double d1 = dh_table_.dh_table(0, 2);
    double a2 = dh_table_.dh_table(2, 1);
    double a3 = dh_table_.dh_table(3, 1);
    double d4 = dh_table_.dh_table(3, 2);
    double d5 = dh_table_.dh_table(4, 2);
    double d6 = dh_table_.dh_table(5, 2);


    Vector3d pos_offset(0.0, 0.0, 0.0);
    PosQuat pos_quat_offset = PosQuat(pos_offset, quat_cmd_);

    PosQuat target_pos_quat_centre = RM::R6Pose2PosQuat( Vector6d(a3 + d5, d4, d1 + a2 - d6, M_PI, M_PI / 4.0, M_PI) );
    PosQuat target_pos_quat = RM::TransformPosQuats({target_pos_quat_centre, pos_quat_offset});
    target_pose_ = RM::PosQuat2R6Pose(target_pos_quat);

    // RM::PrintVec(target_pose_, "\ntarget_pose [m, rad]");

  }

  void solveIK() {
    // Different robot configurations (elbow up/down, shoulder left/right, wrist not flipped/flipped)
    robot_config_ = cga_ik_cobot_6dof::setRobotConfig(1, 1, 1);
    // robot_config_ = cga_ik_cobot_6dof::setRobotConfig(1, -1, -1);
    // robot_config_ = cga_ik_cobot_6dof::setRobotConfig(1, 1, -1);
    // robot_config_ = cga_ik_cobot_6dof::setRobotConfig(-1, 1, 1);
    // robot_config_ = cga_ik_cobot_6dof::setRobotConfig(1, -1, 1);

    cga_ik_cobot_6dof::SolveNullPoints(target_pose_, dh_table_, robot_config_);
    
    if (cga_ik_cobot_6dof::reachable) {
        cga_ik_cobot_6dof::SolveJointAngles();
        joint_angles_ = cga_ik_cobot_6dof::joints;
    } else {
        std::cerr << "\nTarget target_pose is not reachable!" << std::endl;
    }
  }

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
    Vector6d pose_offset(0.0, 0.0, 90.0, 0.0, 90.0, -90.0);
    pose_offset = pose_offset * RM::d2r;
    joint_angles_ = new_angles + pose_offset;
  }
  
  // Publish transforms for base, each joint, and the tool.
  void publishAllTransforms()
  {
    // Publish joint angles
    auto message = std_msgs::msg::Float64MultiArray();
    std::vector<double> angles(joint_angles_.data(), joint_angles_.data() + joint_angles_.size());
    message.data = angles;
    cga_ik_joint_pub_->publish(message);

    // Joint state publisher
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->get_clock()->now();
    joint_state.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    joint_state.position = {joint_angles_(0), joint_angles_(1), joint_angles_(2),
                            joint_angles_(3), joint_angles_(4), joint_angles_(5)};
    joint_state_pub_->publish(joint_state);

    // Publish the base transform (world -> base) as identity.
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "cga_ik_base";
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = -0.5;
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
      
      std::string child_frame = "cga_ik_j" + std::to_string(i + 1);
      std::string parent_frame = (i == 0) ? "cga_ik_base" : "cga_ik_j" + std::to_string(i);
      publishTF(transform, child_frame, parent_frame);
    }

    // CGA FK
    resulting_pose_ = cga_ik_cobot_6dof::CGAFK(joint_angles_, dh_table_);

    // Compare FK and IK
    Vector6d pose_res = RM::R6Poses2RelativeR6Pose(target_pose_, resulting_pose_);

    // // If residual pose > certian threshold, print the residual pose.
    // if (pose_res.head(3).norm() * m2mm > 1e-2 || pose_res.tail(3).norm() * RM::r2d > 1e-2)
    // {
    //   RCLCPP_INFO(this->get_logger(), 
    //   "Residual pose [m, rad] = %.4f, %.4f, %.4f, %.4f, %.4f, %.4f", 
    //   pose_res(0), pose_res(1), pose_res(2), pose_res(3), pose_res(4), pose_res(5));
    // }

    publishTF(RM::R6Pose2PosQuat(target_pose_), "target_pose", "cga_ik_base");
    publishTF(RM::R6Pose2PosQuat(resulting_pose_), "cga_fk_j6", "cga_ik_base");

    // Publish the flange and tool (tcp) transform.
    PosQuat pos_quat_6_f = RM::R6Pose2PosQuat(Vector6d::Zero());
    publishTF(pos_quat_6_f, "cga_ik_flange", "cga_ik_j6"); 

    publishTF(pos_quat_f_tcp_, "cga_ik_tcp", "cga_ik_flange"); 

  }
  
  // Timer callback runs at ~30 Hz.
  void visualise_cobot_6dof_callback_()
  {
    auto start = std::chrono::steady_clock::now();

    // Get IK target pose
    getTargetPose();
    // getPoseCommand();

    // Solve IK (joint angles)
    solveIK();

    auto end = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
    double frequency = (elapsed_ms > 0.0) ? 1000.0 / elapsed_ms : 0.0;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Ts = %.2f [ms], fs = %.2f [Hz]", elapsed_ms, frequency);

    // Publish TFs
    publishAllTransforms();

  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualiseCobot6DoF>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
