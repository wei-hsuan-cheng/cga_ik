#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// The generated header from "SPM.action"
#include "cga_ik_action_interfaces/action/spm.hpp"

// Standard includes 
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

// Eigen, etc.
#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>

// Math utils
#include "cga/cga_utils.hpp"
#include "cga_ik_spm_3dof/cga_ik_spm_3dof.hpp"
#include "robot_math_utils/robot_math_utils_v1_11.hpp"


using SPM = cga_ik_action_interfaces::action::SPM;
using GoalHandleSPM  = rclcpp_action::ServerGoalHandle<SPM>;

using namespace cga_ik_spm_3dof;
using RM = RMUtils;
std::string node_name = "spm_action_server";

struct TimeStampedPoint {
  rclcpp::Time stamp;
  geometry_msgs::msg::Point point;
};


class SPMActionServer : public rclcpp::Node
{
public:
  SPMActionServer()
  : Node(node_name)
  {
    // Load ROS 2 parameters from yaml file
    loadYAMLParams();

    // Initialisation
    initTimeSpec();
    resetTime();
    initFSM();
    initMotionParams();
    initControlParams();
    initSPMIK();
    initRobotControl();
    initRobotVisual();

    // ROS 2 components
    // Action server
    action_server__ = rclcpp_action::create_server<SPM>(
      this,
      "spm", // the action name
      std::bind(&SPMActionServer::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SPMActionServer::handle_cancel,   this, std::placeholders::_1),
      std::bind(&SPMActionServer::handle_accepted, this, std::placeholders::_1)
    );

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Pubs and subs
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/spm_3dof/joint_states", 10);
    joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/spm_3dof/joint_cmd", 10);
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    spm_pivot_sphere_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/spm_pivot_sphere", qos);
    spm_motor_sphere_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/spm_motor_sphere", qos);
    spm_elbow_sphere_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/spm_elbow_sphere", qos);
    spm_epl_sphere_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/spm_epl_sphere", qos);
    spm_ltri_0_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/spm_ltri_0", qos);
    spm_ltri_1_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/spm_ltri_1", qos);
    spm_ltri_2_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/spm_ltri_2", qos);
    spm_utri_0_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/spm_utri_0", qos);
    spm_utri_1_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/spm_utri_1", qos);
    spm_utri_2_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/spm_utri_2", qos);
    light_ray_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/spm_light_ray", qos);

    ee_traj_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/spm_3dof/markers/ee_traj", 10);

    RCLCPP_INFO(this->get_logger(), "SPM Action Server started.");
  }

private:
  // Service/Action server and client
  rclcpp_action::Server<SPM>::SharedPtr action_server__;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // We'll also publish 3 joint angles
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angles_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;

  // Publisher for the sphere marker
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spm_pivot_sphere_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spm_motor_sphere_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spm_elbow_sphere_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spm_epl_sphere_pub_;

  // Publisher for the triangle marker
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spm_ltri_0_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spm_ltri_1_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spm_ltri_2_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spm_utri_0_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spm_utri_1_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spm_utri_2_pub_;

  // Publisher for the light ray marker
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr light_ray_marker_pub_;

  // Publisher for the trajectory marker
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ee_traj_pub_;

  double fs_, Ts_;
  int k_;
  double t_;
  rclcpp::Time current_time_;

  float r_c_, ang_b_m_, r_b_, d_, r_e_;
  float r_s_piv_, r_s_m_, r_s_elb_, r_s_epl_;
  float z_rot_cen_ee_;
  int krl_;
  double th_z_ee_reset_;
  std::string spm_mode_;

  CGA e_principal_, s_0_, s_1_;
  Vector4f pivot_sphere_color_, motor_sphere_color_, elbow_sphere_color_, epl_sphere_color_;
  Vector4f utri_color_, ltri_color_, light_ray_color_, ee_traj_color_;

  // For the trajectory
  std::deque<TimeStampedPoint> trajectory_buffer_;
  // const double TRAIL_DURATION_SEC = 2.0; // keep last few seconds of end-effector path
  const double TRAIL_DURATION_SEC = 30.0; // keep last few seconds of end-effector path

  // Motion params
  double freq_cmd_;
  double th_cmd_mag_; // [rad]
  double th_cmd_; // [rad]
  Vector3d axis_cmd_;

  // C-axis (z-rotation) and A-axis (x-rotation) of milling machines
  Vector3d axis_c_;
  double ang_c_;
  Vector3d axis_a_;
  double ang_a_;

  // Piecewise motions fragment
  int motion_fragment_;
  double t_end_fragmet_;

  // IK solver object and result
  std::shared_ptr<CGAIKSPM3DoF> ik_solver_;
  Vector9f ik_joint_angles_;
  
  SPM3DoFIKResetOrigin ik_reset_origin_result_;
  SPM3DoFIKResult ik_result_;

  // IK pose command
  Quaternionf quat_ubase_epl_c_cmd_;
  // End pose
  Quaternionf quat_ubase_epl_c_end_;
  // Pose error threshold
  double quat_err_thresh_;
  // IK motor joints command
  Vector3f joint_angles_cmd_;

  // S-curve
  struct SCurveParams
  {
    SCurveParams(double T_in = 1.0)
    : T(T_in),
      lambda((4.0 / T_in) * 5.0)
    {}
    double T;      
    double lambda; 
  };
  SCurveParams scur_;

  // FSM
  bool stop_action_;
  std::mutex data_mutex_;
  bool task_finished_;

  enum class SPMFSMState {
    IDLE,
    INITIATE,
    REORIGINING,
    REORIGINED,
    CONTROL,
    HOMING,
    HOMED,
    END,
  };
  SPMFSMState fsm_state_;

  void initTimeSpec()
  {
      fs_ = 60.0;
      Ts_ = 1.0 / fs_;
      current_time_ = this->now();
  }

  void resetTime()
  {
    k_ = 0;
    t_ = 0.0;
  }
  
  void initFSM()
  {
    fsm_state_   = SPMFSMState::IDLE; // Initial state duing the execution
    stop_action_ = false;
    task_finished_ = false;
  }

  void initMotionParams()
  {
    // freq_cmd_ = 0.1;
    // th_cmd_mag_ = 60.0 * RM::d2r; // [rad]
    // th_cmd_ = 0.0;

    // double ang = 45.0 * RM::d2r; // [rad]
    // axis_cmd_ = Vector3d(std::cos(ang), std::sin(ang), -1.0);

    // C-axis (z-rotation) and A-axis (x-rotation) of milling machines    
    // C-axis
    this->get_parameter("ang_c", ang_c_); // [deg]
    axis_c_ = Vector3d(std::cos(ang_c_ * RM::d2r + M_PI / 2.0), std::sin(ang_c_ * RM::d2r + M_PI / 2.0), 0.0);
    // A-axis
    this->get_parameter("ang_a", ang_a_); // [deg]
    freq_cmd_ = 0.1;

    quat_ubase_epl_c_end_ = RM::so32Quat( axis_c_.normalized() * ang_a_ * RM::d2r ).cast<float>();

    // Piecewise motions fragment
    motion_fragment_ = 0;
    t_end_fragmet_ = 0.0;

    quat_err_thresh_ = 0.01 * RM::d2r; // [rad]
  }

  void initControlParams()
  {
    // S-curve params
    scur_ = SCurveParams{5.0};
  }

  void loadYAMLParams()
  {
    // Geometric params
    // Lower parts
    this->declare_parameter<float>("r_c", 1.0);
    this->declare_parameter<float>("ang_b_m", 0.0);
    this->declare_parameter<float>("r_b", 1.0);
    // Upper parts
    this->declare_parameter<float>("d", 1.0);
    this->declare_parameter<float>("r_e", 1.0);
    // Sphere radii
    this->declare_parameter<float>("r_s_piv", 1.0);
    this->declare_parameter<float>("r_s_m", 1.0);
    this->declare_parameter<float>("r_s_elb", 1.0);
    this->declare_parameter<float>("r_s_epl", 1.0);
    // Elbow configuration
    this->declare_parameter<int>("krl", 1);
    // Re-origining the coordinate system (mechanical origin -> control origin); set initial z-rotation
    this->declare_parameter<double>("th_z_ee_reset", 0.0);
    // SPM mode
    this->declare_parameter<std::string>("spm_mode", "HOME");


    // Motion params
    this->declare_parameter<float>("ang_c", 0.0);
    this->declare_parameter<float>("ang_a", 0.0);
  }

  void initSPMIK()
  {   
    // Retrieve parameters
    this->get_parameter("r_c", r_c_);
    this->get_parameter("ang_b_m", ang_b_m_);
    this->get_parameter("r_b", r_b_);
    this->get_parameter("d", d_);
    this->get_parameter("r_e", r_e_);
    this->get_parameter("r_s_piv", r_s_piv_);
    this->get_parameter("r_s_m", r_s_m_);
    this->get_parameter("r_s_elb", r_s_elb_);
    this->get_parameter("r_s_epl", r_s_epl_);
    this->get_parameter("krl", krl_);
    this->get_parameter("th_z_ee_reset", th_z_ee_reset_);
    spm_mode_ = "HOME";

    // Initial IK target pose (quaternion orientation)
    quat_ubase_epl_c_cmd_ = Quaternionf::Identity();

    // SPM coordinate system
    // Principal axis of the 3-DoF spm
    e_principal_ = e3;

    // Each motor position is set in the base plane, separated by 120 [deg].
    s_0_ = e1;
    float ang_apart = 2.0 * M_PI / 3.0; // +120 [deg]
    // float ang_apart = -2.0 * M_PI / 3.0; // -120 [deg]
    CGA rot_e12 = cga_utils::rot(e1 * e2, ang_apart); // Rotate s_0_ by +-120 [deg] to get s_1_
    s_1_ = rot_e12 * s_0_ * ~rot_e12;

    // End-effector z-offset
    z_rot_cen_ee_ = 2.0f * r_s_m_;
    
    // Construct IK solver
    ik_solver_ = std::make_shared<CGAIKSPM3DoF>(
    r_c_, ang_b_m_, r_b_, d_, r_e_,
    r_s_piv_, r_s_m_, r_s_elb_, r_s_epl_,
    z_rot_cen_ee_, krl_,
    e_principal_, s_0_, s_1_,
    th_z_ee_reset_
    );

    // Re-origining
    ik_reset_origin_result_ = ik_solver_->resetOrigin(spm_mode_);
    // Solve initial pose
    ik_result_ = ik_solver_->solveIK(quat_ubase_epl_c_cmd_);

    // IK joint angles [rad]
    ik_joint_angles_ << ik_result_.th_0, /* Actual motor angles */ 
                        ik_result_.th_1, 
                        ik_result_.th_2,
                        ik_reset_origin_result_.th_0_nom, /* Nominal motor angles */ 
                        ik_reset_origin_result_.th_1_nom, 
                        ik_reset_origin_result_.th_2_nom,
                        ik_result_.th_0 - ik_reset_origin_result_.th_0_nom, /* Deviated motor angles */
                        ik_result_.th_1 - ik_reset_origin_result_.th_1_nom,
                        ik_result_.th_2 - ik_reset_origin_result_.th_2_nom;



    // Print SPM geometric parameters
    std::cout << "--------------------------------------------------------------" << std::endl;
    std::cout << "---------- Geometric parameters of the spm [m, deg] ----------" << std::endl;
    std::cout << "--------------------------------------------------------------" << std::endl;

    std::cout << "[Lower parts]" << std::endl;
    std::cout << "r_c = " << r_c_ << std::endl;
    std::cout << "ang_b_m = " << ang_b_m_ << std::endl;
    std::cout << "r_b = " << r_b_ << std::endl;
    
    std::cout << "[Upper parts]" << std::endl;
    std::cout << "d = " << d_ << std::endl;
    std::cout << "r_e = " << r_e_ << std::endl;

    std::cout << "[Sphere radii]" << std::endl;
    std::cout << "r_s_piv = " << r_s_piv_ << std::endl;
    std::cout << "r_s_m = " << r_s_m_ << std::endl;
    std::cout << "r_s_elb = " << r_s_elb_ << std::endl;
    std::cout << "r_s_epl = " << r_s_epl_ << std::endl;

    std::cout << "[Initial IK angles] th_0_init, th_1_init, th_2_init [deg] = " << ik_result_.th_0 * RM::r2d << ", " << ik_result_.th_1 * RM::r2d << ", " << ik_result_.th_2 * RM::r2d << std::endl;
    
    std::cout << "--------------------------------------------------------------" << std::endl;


  }

  void initRobotControl()
  {
    joint_angles_cmd_ << ik_reset_origin_result_.th_0_nom, /* Nominal motor angles */ 
                         ik_reset_origin_result_.th_1_nom, 
                         ik_reset_origin_result_.th_2_nom;
  }

  void initRobotVisual()
  {
      // Pivot sphere
      pivot_sphere_color_ = Vector4f(1.0f, 0.0f, 1.0f, 0.15f);

      // Motor sphere
      motor_sphere_color_ = Vector4f(0.0f, 0.0f, 1.0f, 0.15f);

      // Elbow sphere
      elbow_sphere_color_ = Vector4f(0.0f, 1.0f, 0.0f, 0.15f);

      // End-plate sphere
      epl_sphere_color_ = Vector4f(1.0f, 0.0f, 0.0f, 0.15f);

      // Triangles
      utri_color_ = Vector4f(0.0f, 0.0f, 1.0f, 0.8f);
      ltri_color_ = Vector4f(0.0f, 1.0f, 0.0f, 0.8f);

      // Light ray marker
      light_ray_color_ = Vector4f(0.0f, 0.0f, 1.0f, 0.7f);

      // Traj color
      ee_traj_color_ = Vector4f(1.0f, 0.0f, 1.0f, 0.7f);
  }

  void publishTF(
        const Vector3f &p, 
        const Quaternionf &q, 
        const std::string &child, 
        const std::string &parent)
  {
      geometry_msgs::msg::TransformStamped tf;
      // tf.header.stamp = this->now();
      tf.header.stamp = current_time_;

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
  }

  void publishSphereMarker(
        const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub,
        const int &marker_id,
        const std::string &ns,
        const std::string &frame_id,
        const float &radius,
        const Vector3f &centre,
        const Vector4f &rgba_color)
  {
      visualization_msgs::msg::Marker sphere_marker;
      // sphere_marker.header.stamp = this->now();
      // sphere_marker.header.stamp = current_time_;

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
        const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub,
        const int &marker_id,
        const std::string &ns,
        const std::string &frame_id,
        const Vector3f &p0,
        const Vector3f &p1,
        const Vector3f &p2,
        const Vector4f &rgba_color)
  {
      // Build a TRIANGLE_LIST marker from the 3 points
      visualization_msgs::msg::Marker tri_marker;
      // tri_marker.header.stamp = this->now();
      // tri_marker.header.stamp = current_time_;

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

  void publishLightRayMarker(
        const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub,
        const int &marker_id,
        const std::string &ns,
        const std::string &frame_id,
        const Vector3f &starting_point,
        const Vector3f &ending_point,
        const Vector4f &rgba_color)
  {
      visualization_msgs::msg::Marker light_ray_marker;
      // light_ray_marker.header.stamp = this->now();
      // light_ray_marker.header.stamp = current_time_;

      light_ray_marker.header.frame_id = frame_id;
      light_ray_marker.ns = ns;
      light_ray_marker.id = marker_id;
      light_ray_marker.type = visualization_msgs::msg::Marker::ARROW;
      light_ray_marker.action = visualization_msgs::msg::Marker::ADD;

      light_ray_marker.scale.x = 0.001;  // Shaft diameter
      light_ray_marker.scale.y = 0.002;  // Head diameter
      light_ray_marker.scale.z = 0.02;  // Head length

      // Set the marker color to blue for z-axis
      light_ray_marker.color.r = rgba_color(0);
      light_ray_marker.color.g = rgba_color(1);
      light_ray_marker.color.b = rgba_color(2);
      light_ray_marker.color.a = rgba_color(3);

      // Set the arrow origin point and direction
      geometry_msgs::msg::Point pt_s;
      pt_s.x = starting_point(0);
      pt_s.y = starting_point(1);
      pt_s.z = starting_point(2);

      geometry_msgs::msg::Point pt_e;
      pt_e.x = ending_point(0);
      pt_e.y = ending_point(1);
      pt_e.z = ending_point(2); // Length of the arrow

      light_ray_marker.points.push_back(pt_s);
      light_ray_marker.points.push_back(pt_e);

      pub->publish(light_ray_marker);
  }

  void publishTrajMarker(
        const Vector4f rgba_color)
  {
      // Publish traj marker using line strip
      // EE point to trajectory buffer
      {
          TimeStampedPoint new_pt;
          new_pt.stamp = current_time_;
          new_pt.point.x = ik_solver_->pos_rot_cen_ee().x();
          new_pt.point.y = ik_solver_->pos_rot_cen_ee().y();
          new_pt.point.z = ik_solver_->pos_rot_cen_ee().z();
          trajectory_buffer_.push_back(new_pt);

          // Remove old points beyond TRAIL_DURATION_SEC
          while (!trajectory_buffer_.empty())
          {
          double dt = (current_time_ - trajectory_buffer_.front().stamp).seconds();
          if (dt > TRAIL_DURATION_SEC) {
              trajectory_buffer_.pop_front();
          } else {
              break;
          }
          }
      }

      visualization_msgs::msg::Marker traj_marker;
      traj_marker.header.stamp = current_time_;
      traj_marker.header.frame_id = "spm_rot_cen";   // or "world" if you store ept in world coords
      traj_marker.ns = "ee_traj";
      traj_marker.id = 0;
      traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      traj_marker.action = visualization_msgs::msg::Marker::ADD;

      // thickness in meters
      traj_marker.scale.x = 0.003; // e.g. 3 [mm] wide

      // color
      traj_marker.color.r = rgba_color(0);
      traj_marker.color.g = rgba_color(1);
      traj_marker.color.b = rgba_color(2);
      traj_marker.color.a = rgba_color(3);

      // no lifetime needed, we manually remove old points from buffer
      // for a fade effect, you can also do .lifetime = rclcpp::Duration(2,0)

      // push the points
      for (auto & tsp : trajectory_buffer_) {
          traj_marker.points.push_back(tsp.point);
      }

      ee_traj_pub_->publish(traj_marker);
  }

  bool waitForTime(const double &t_sec = 1.0)
  {
      // Waiting for t_sec [s]
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), t_sec * 1000, "[waitForTime] t_sec = %f", t_sec);

      // Update time
      t_ = k_ * Ts_;
      k_ += 1;

      return (t_ > t_sec) ? true : false;
  }

  void getReoriginingPose()
  {
      // Update time
      t_ = k_ * Ts_;
      k_ += 1;

      // Target orientation for the IK problem
      double th = RM::SCurve(th_z_ee_reset_ * RM::d2r, 0.0, scur_.lambda, t_, scur_.T);
      quat_ubase_epl_c_cmd_ = RM::so32Quat( Vector3d(0.0, 0.0, 1.0) * th ).cast<float>();
  }

  void getReoriginedPose()
  {
    // Update time
    t_ = k_ * Ts_;
    k_ += 1;

    // Target orientation for the IK problem
    // Go from quat_ubase_epl_c_end_ to identity
    Vector3d so3 = RM::Quat2so3( quat_ubase_epl_c_end_.inverse().cast<double>() );
    double th = RM::SCurve(so3.norm(), 0.0, scur_.lambda, t_, scur_.T);
    quat_ubase_epl_c_cmd_ = RM::so32Quat( so3.normalized() * th ).cast<float>();
  }

  void getHomingPose()
  {
    // Update time
    t_ = k_ * Ts_;
    k_ += 1;

    // // // Target orientation for the IK problem
    // // Go from quat_ubase_epl_c_end_ to identity
    // Vector3d d_so3 = RM::Quat2so3( quat_ubase_epl_c_end_.inverse().cast<double>() * Quaterniond::Identity() );
    // double th = RM::SCurve(d_so3.norm(), 0.0, scur_.lambda, t_, scur_.T);
    // Quaternionf d_quat = RM::so32Quat( d_so3.normalized() * th ).cast<float>();
    // quat_ubase_epl_c_cmd_ = quat_ubase_epl_c_end_* d_quat;

    // // Target orientation for the IK problem
    // Go from quat_ubase_epl_c_end_ to Quatz(-th_z_ee_reset_ * RM::d2r)
    Vector3d d_so3 = RM::Quat2so3( quat_ubase_epl_c_end_.inverse().cast<double>() * RM::Quatz(-th_z_ee_reset_ * RM::d2r) );
    double th = RM::SCurve(d_so3.norm(), 0.0, scur_.lambda, t_, scur_.T);
    Quaternionf d_quat = RM::so32Quat( d_so3.normalized() * th ).cast<float>();
    quat_ubase_epl_c_cmd_ = quat_ubase_epl_c_end_* d_quat;
  }

  Vector2f xy2rtheta(const Vector2f &xy)
  {
      // Cartisian to polar coordinates
      Vector2f rtheta;
      rtheta(0) = xy.norm();
      rtheta(1) = std::atan2(xy(1), xy(0));
      return rtheta;
  }

  Vector2f getXYSixPointStarMotion()
  {
      // Star of David pattern parameters
      const double inner_radius = 0.5; // Inner points
      const double outer_radius = inner_radius * std::sqrt(3);  // [m] size of the star
      const double vel = 2.0 * outer_radius / 5.0; // [m/s] same velocity as before

      // 0 [deg]
      Vector2f p_0_deg = Vector2f(inner_radius * std::cos(0.0), 
                inner_radius * std::sin(0.0));              
      // 30 [deg]
      Vector2f p_30_deg = Vector2f(outer_radius * std::cos(M_PI/6), 
                outer_radius * std::sin(M_PI/6));  
      // 60 [deg]
      Vector2f p_60_deg = Vector2f(inner_radius * std::cos(M_PI/3), 
                inner_radius * std::sin(M_PI/3)); 
      // 90 [deg]
      Vector2f p_90_deg = Vector2f(outer_radius * std::cos(M_PI/2), 
                outer_radius * std::sin(M_PI/2));  
      // 120 [deg]
      Vector2f p_120_deg = Vector2f(inner_radius * std::cos(2*M_PI/3), 
                inner_radius * std::sin(2*M_PI/3));
      // 150 [deg]
      Vector2f p_150_deg = Vector2f(outer_radius * std::cos(5*M_PI/6), 
                outer_radius * std::sin(5*M_PI/6));
      // 180 [deg]
      Vector2f p_180_deg = Vector2f(inner_radius * std::cos(M_PI), 
                inner_radius * std::sin(M_PI));
      // 210 [deg]
      Vector2f p_210_deg = Vector2f(outer_radius * std::cos(7*M_PI/6), 
                outer_radius * std::sin(7*M_PI/6));
      // 240 [deg]
      Vector2f p_240_deg = Vector2f(inner_radius * std::cos(4*M_PI/3), 
                inner_radius * std::sin(4*M_PI/3));
      // 270 [deg]
      Vector2f p_270_deg = Vector2f(outer_radius * std::cos(3*M_PI/2), 
                outer_radius * std::sin(3*M_PI/2));
      // 300 [deg]
      Vector2f p_300_deg = Vector2f(inner_radius * std::cos(5*M_PI/3), 
                inner_radius * std::sin(5*M_PI/3));
      // 330 [deg]
      Vector2f p_330_deg = Vector2f(outer_radius * std::cos(11*M_PI/6), 
                outer_radius * std::sin(11*M_PI/6));


      Vector2f motion_xy;

      // The Star of David (alternating outer and inner points)
      const std::array<Vector2f, 17> star_points = {
          p_0_deg,
          p_30_deg,
          p_150_deg,
          p_270_deg,

          p_0_deg,
          p_90_deg,
          p_210_deg,
          p_330_deg,

          p_0_deg,
          p_30_deg,
          p_150_deg,
          p_270_deg,

          p_0_deg,
          p_90_deg,
          p_210_deg,
          p_330_deg,

          p_0_deg
      };
      
      // Calculate total path length for velocity control
      double total_length = 0.0;
      for (size_t i = 0; i < star_points.size(); ++i) {
          size_t next_i = (i + 1) % star_points.size();
          total_length += (star_points[next_i] - star_points[i]).norm();
      }
      
      // Calculate current position along path
      double path_position = std::fmod(vel * (t_ - t_end_fragmet_), total_length);
      
      // Find current segment
      double accumulated_length = 0.0;
      for (size_t i = 0; i < star_points.size(); ++i) {
          size_t next_i = (i + 1) % star_points.size();
          double segment_length = (star_points[next_i] - star_points[i]).norm();
          
          if (path_position <= accumulated_length + segment_length) {
              // We're in this segment
              double alpha = (path_position - accumulated_length) / segment_length;
              motion_xy = star_points[i] + alpha * (star_points[next_i] - star_points[i]);
              
              // Check if we've completed a full cycle
              if (path_position + vel*Ts_ >= total_length) {
                  t_end_fragmet_ = t_; // Reset timing for next cycle
              }
              
              return motion_xy;
          }
          accumulated_length += segment_length;
      }
      
      // Default return (shouldn't reach here)
      return Vector2f::Zero();
  }


  Vector2f getXYFivePointStarMotion()
  {
      // Five-point star pattern parameters
      const double outer_radius = 0.5;  // [m] radius to star points
      const double inner_radius = outer_radius * 0.382; // Golden ratio for perfect star
      const double vel = 2.0 * outer_radius / 5.0; // [m/s] same velocity as before

      // Calculate the 10 points of the star (5 outer, 5 inner)
      std::array<Vector2f, 10> star_points;
      for (int i = 0; i < 5; ++i) {
          double outer_angle = 2.0 * M_PI * i / 5.0 - M_PI/2.0; // Offset by -90° to start at top
          double inner_angle = outer_angle + M_PI / 5.0; // Inner points offset by 36°
          
          // Outer points (0, 2, 4, 6, 8)
          star_points[2*i] = Vector2f(
              outer_radius * std::cos(outer_angle),
              outer_radius * std::sin(outer_angle)
          );
          
          // Inner points (1, 3, 5, 7, 9)
          star_points[2*i + 1] = Vector2f(
              inner_radius * std::cos(inner_angle),
              inner_radius * std::sin(inner_angle)
          );
      }

      Vector2f motion_xy;

      // The star path (connect all points in order)
      const std::array<Vector2f, 11> star_path = {
          star_points[0], star_points[1], star_points[2], star_points[3], star_points[4],
          star_points[5], star_points[6], star_points[7], star_points[8], star_points[9],
          star_points[0]  // Close the star
      };
      
      // Calculate total path length for velocity control
      double total_length = 0.0;
      for (size_t i = 0; i < star_path.size() - 1; ++i) {
          total_length += (star_path[i+1] - star_path[i]).norm();
      }
      
      // Calculate current position along path
      double path_position = std::fmod(vel * (t_ - t_end_fragmet_), total_length);
      
      // Find current segment
      double accumulated_length = 0.0;
      for (size_t i = 0; i < star_path.size() - 1; ++i) {
          double segment_length = (star_path[i+1] - star_path[i]).norm();
          
          if (path_position <= accumulated_length + segment_length) {
              // We're in this segment
              double alpha = (path_position - accumulated_length) / segment_length;
              motion_xy = star_path[i] + alpha * (star_path[i+1] - star_path[i]);
              
              // Check if we've completed a full cycle
              if (path_position + vel*Ts_ >= total_length) {
                  t_end_fragmet_ = t_; // Reset timing for next cycle
              }
              
              return motion_xy;
          }
          accumulated_length += segment_length;
      }
      
      // Default return (shouldn't reach here)
      return Vector2f::Zero();
  }

  Vector2f getXYMotion()
  {
    
    // Linear motion in x-direction
    double x_max = 0.5; // [m]
    double y_max = 0.5; // [m]
    double vel = 2.0 * x_max / 5.0; // [m/s]
    Vector2f motion_xy;

    // Piecewise linear motions
    switch (motion_fragment_) {
      case 0:
        {
          // (x_max, y_max) -> (-x_max, y_max)
          double end = -x_max;
          double actual = x_max - vel * (t_ - t_end_fragmet_);
          if (actual > end) {
            motion_xy(0) = actual;
            motion_xy(1) = y_max;
            motion_fragment_ = 0;
          } else {
            // Reset time
            t_end_fragmet_ = t_;
            motion_xy(0) = end;
            motion_xy(1) = y_max;
            motion_fragment_ = 1;
            
          }

          break;
        }

      case 1:
        {
          // (-x_max, y_max) -> (-x_max, -y_max)
          double end = -y_max;
          double actual = y_max - vel * (t_ - t_end_fragmet_);
          if (actual > end) {
            motion_xy(0) = -x_max;
            motion_xy(1) = actual;
            motion_fragment_ = 1;
          } else {
            motion_xy(0) = -x_max;
            motion_xy(1) = end;
            motion_fragment_ = 2;
            // Reset time
            t_end_fragmet_ = t_;
          }

          break;
        }
      
      case 2:
        {
          // (-x_max, -y_max) -> (x_max, -y_max)
          double end = x_max;
          double actual = -x_max + vel * (t_ - t_end_fragmet_);
          if (actual < end) {
            motion_xy(0) = actual;
            motion_xy(1) = -y_max;
            motion_fragment_ = 2;
          } else {
            motion_xy(0) = end;
            motion_xy(1) = -y_max;
            motion_fragment_ = 3;
            // Reset time
            t_end_fragmet_ = t_;
          }

          break;
        }
      
      case 3:
        {
          // (x_max, -y_max) -> (x_max, y_max)
          double end = y_max;
          double actual = -y_max + vel * (t_ - t_end_fragmet_);
          if (actual < end) {
            motion_xy(0) = x_max;
            motion_xy(1) = actual;
            motion_fragment_ = 3;
          } else {
            motion_xy(0) = x_max;
            motion_xy(1) = end;
            motion_fragment_ = 0;
            // Reset time
            t_end_fragmet_ = t_;
          }

          break;
        }

      default:
        {
          break;
        }
    }

    
    return motion_xy;
  }

  void getTargetPose()
  {
      // Update time
      t_ = k_ * Ts_;
      k_ += 1;

      // // Target orientation for the IK problem
      // // th_cmd_ = (th_cmd_mag_) * std::sin(2.0 * M_PI * freq_cmd_ * t_); // [rad]
      // th_cmd_ = RM::SCurve(th_cmd_mag_, 0.0, scur_.lambda, t_, scur_.T); // [rad]
      // quat_ubase_epl_c_cmd_ = RM::so32Quat( axis_cmd_.normalized() * th_cmd_ ).cast<float>();


      // // C-axis (z-rotation) and A-axis (x-rotation) of milling machines
      // th_cmd_ = RM::SCurve(ang_a_ * RM::d2r, 0.0, scur_.lambda, t_, scur_.T); // [rad]
      // quat_ubase_epl_c_cmd_ = RM::so32Quat( axis_c_.normalized() * th_cmd_ ).cast<float>();


      // // C-axis (z-rotation) and A-axis (x-rotation) of milling machines
      // // Circular motion
      // double freq_c = 0.5;
      // double ang_c = 2.0 * M_PI * freq_c * t_;
      // ang_c_ = RM::SCurve(ang_c, 0.0, scur_.lambda, t_, scur_.T); // [rad]
      // axis_c_ = Vector3d(std::cos(ang_c_ + M_PI / 2.0), std::sin(ang_c_ + M_PI / 2.0), 0.0);
      
      // double freq_a = freq_c * 0.1;
      // double ang_a = 45.0 * RM::d2r * std::sin(2.0 * M_PI * freq_a * t_); // [rad]
      // ang_a_ = RM::SCurve(ang_a, 0.0, scur_.lambda, t_, scur_.T); // [rad]
      
      // quat_ubase_epl_c_cmd_ = RM::so32Quat( axis_c_.normalized() * ang_a_ ).cast<float>();


      // C-axis (z-rotation) and A-axis (x-rotation) of milling machines
      // Vector2f motion_xy = getXYMotion();
      // Vector2f motion_xy = getXYSixPointStarMotion();
      Vector2f motion_xy = getXYFivePointStarMotion();
      Vector2f rtheta = xy2rtheta(motion_xy);

      ang_c_ = RM::SCurve(rtheta(1), 0.0, scur_.lambda, t_, scur_.T); // [rad]
      ang_a_ = RM::SCurve(rtheta(0), 0.0, scur_.lambda, t_, scur_.T); // [rad]

      axis_c_ = Vector3d(std::cos(ang_c_ + M_PI / 2.0), std::sin(ang_c_ + M_PI / 2.0), 0.0);
      quat_ubase_epl_c_cmd_ = RM::so32Quat( axis_c_.normalized() * ang_a_ ).cast<float>();

  }

  void solveSPMIK() 
  {
      ik_result_ = ik_solver_->solveIK(quat_ubase_epl_c_cmd_);

      // IK joint angles [rad]
      ik_joint_angles_ << ik_result_.th_0, /* Actual motor angles */ 
                          ik_result_.th_1, 
                          ik_result_.th_2,
                          ik_reset_origin_result_.th_0_nom, /* Nominal motor angles */ 
                          ik_reset_origin_result_.th_1_nom, 
                          ik_reset_origin_result_.th_2_nom,
                          ik_result_.th_0 - ik_reset_origin_result_.th_0_nom, /* Deviated motor angles */
                          ik_result_.th_1 - ik_reset_origin_result_.th_1_nom,
                          ik_result_.th_2 - ik_reset_origin_result_.th_2_nom;
      
      // Joint angles command
      joint_angles_cmd_ << ik_result_.th_0, /* Actual motor angles */ 
                           ik_result_.th_1, 
                           ik_result_.th_2;

  }

  void sendJointCmd()
  {
      // Publish the joint states for these angles
      std_msgs::msg::Float64MultiArray angles_msg;
      sensor_msgs::msg::JointState js_msg;
      std_msgs::msg::Float64MultiArray joint_cmd_msg;

      // js_msg.header.stamp = this->now();
      js_msg.header.stamp = current_time_;

      for (int i = 0; i < ik_joint_angles_.size(); ++i) {
          angles_msg.data.push_back(ik_joint_angles_(i) * RM::r2d);
          js_msg.position.push_back(ik_joint_angles_(i));
      }
      js_msg.name = {"joint_0", "joint_1", "joint_2", 
                      "joint_0_nom", "joint_1_nom", "joint_2_nom",
                      "d_joint_0", "d_joint_1", "d_joint_2"
                    };
      
      for (int i = 0; i < joint_angles_cmd_.size(); ++i) {
          joint_cmd_msg.data.push_back(joint_angles_cmd_(i));
      }

      angles_pub_->publish(angles_msg);
      joint_pub_->publish(js_msg);
      joint_cmd_pub_->publish(joint_cmd_msg);
      
  }

  void publishAllTransforms()
  {
      // Poses w.r.t. the base
      // Rotation centre
      publishTF(ik_solver_->pos_base_rot_cen(), ik_solver_->quat_base_rot_cen(), "spm_rot_cen", "spm_base");

      // Poses w.r.t. the rotation centre
      // Pivots of the robot
      publishTF(ik_solver_->pos_rot_cen_piv_0(), ik_solver_->quat_rot_cen_piv_0(), "spm_pivot_0", "spm_rot_cen");
      publishTF(ik_solver_->pos_rot_cen_piv_1(), ik_solver_->quat_rot_cen_piv_1(), "spm_pivot_1", "spm_rot_cen");
      publishTF(ik_solver_->pos_rot_cen_piv_2(), ik_solver_->quat_rot_cen_piv_2(), "spm_pivot_2", "spm_rot_cen");

      // Motors of the robot
      publishTF(ik_solver_->pos_rot_cen_m_0(), ik_solver_->quat_rot_cen_m_0(), "spm_motor_0", "spm_rot_cen");
      publishTF(ik_solver_->pos_rot_cen_m_1(), ik_solver_->quat_rot_cen_m_1(), "spm_motor_1", "spm_rot_cen");
      publishTF(ik_solver_->pos_rot_cen_m_2(), ik_solver_->quat_rot_cen_m_2(), "spm_motor_2", "spm_rot_cen");
      
      // End-plate corners and centre 
      publishTF(ik_solver_->pos_rot_cen_epl_0(), ik_solver_->quat_rot_cen_epl_0(), "spm_epl_0", "spm_rot_cen");
      publishTF(ik_solver_->pos_rot_cen_epl_1(), ik_solver_->quat_rot_cen_epl_1(), "spm_epl_1", "spm_rot_cen");
      publishTF(ik_solver_->pos_rot_cen_epl_2(), ik_solver_->quat_rot_cen_epl_2(), "spm_epl_2", "spm_rot_cen");
      
      
      // End-plate centre, sphere end-point, and end-effector
      publishTF(ik_solver_->pos_rot_cen_epl_c(), ik_solver_->quat_rot_cen_epl_c(), "spm_epl_c", "spm_rot_cen");
      publishTF(ik_solver_->pos_rot_cen_epl_c(), ik_solver_->quat_rot_cen_epl_c_remapped(), "spm_epl_c_remapped", "spm_rot_cen"); // Remapped end-plate centre
      publishTF(ik_solver_->pos_rot_cen_ept(), ik_solver_->quat_rot_cen_ept_remapped(), "spm_ept", "spm_rot_cen");
      publishTF(ik_solver_->pos_rot_cen_ee(), ik_solver_->quat_rot_cen_ee_remapped(), "spm_ee", "spm_rot_cen");

      // Elbows
      publishTF(ik_solver_->pos_rot_cen_elb_0(), ik_solver_->quat_rot_cen_elb_0(), "spm_elbow_0", "spm_rot_cen");
      publishTF(ik_solver_->pos_rot_cen_elb_1(), ik_solver_->quat_rot_cen_elb_1(), "spm_elbow_1", "spm_rot_cen");
      publishTF(ik_solver_->pos_rot_cen_elb_2(), ik_solver_->quat_rot_cen_elb_2(), "spm_elbow_2", "spm_rot_cen");

  }

  void publisherAllMarkers()
  {
      // Publish pivot sphere marker
      publishSphereMarker(
          spm_pivot_sphere_pub_,
          0,                // marker_id
          "spm_pivot_sphere",         // ns
          "spm_rot_cen",       // frame_id
          ik_solver_->r_s_piv(),
          Vector3f::Zero(), // at rot_cen
          pivot_sphere_color_ // RGBA
      );

      // Publish motor sphere marker
      publishSphereMarker(
          spm_motor_sphere_pub_,
          0,                // marker_id
          "spm_motor_sphere",         // ns
          "spm_rot_cen",       // frame_id
          ik_solver_->r_s_m(),
          Vector3f::Zero(), // at rot_cen
          motor_sphere_color_ // RGBA
      );

      // Publish elbow sphere marker
      publishSphereMarker(
          spm_elbow_sphere_pub_,
          0,                // marker_id
          "spm_elbow_sphere",         // ns
          "spm_rot_cen",       // frame_id
          ik_solver_->r_s_elb(),
          Vector3f::Zero(), // at rot_cen
          elbow_sphere_color_ // RGBA
      );

      // Publish end-plate sphere marker
      publishSphereMarker(
          spm_epl_sphere_pub_,
          0,                // marker_id
          "spm_epl_sphere",         // ns
          "spm_rot_cen",       // frame_id
          ik_solver_->r_s_epl(),
          Vector3f::Zero(), // at rot_cen
          epl_sphere_color_ // RGBA
      );

      // Publish triangle markers
      // Lower triangles for rot_cen-elb-m
      publishTriMarker(
                        spm_ltri_0_pub_,
                        0,                // marker_id
                        "spm_ltri_0",         // ns
                        "spm_rot_cen",       // frame_id
                        Vector3f::Zero(), // at rot_cen
                        ik_solver_->pos_rot_cen_elb_0(),
                        ik_solver_->pos_rot_cen_m_0(),
                        ltri_color_
                      );
      
      publishTriMarker(
                        spm_ltri_1_pub_,
                        0,                // marker_id
                        "spm_ltri_1",         // ns
                        "spm_rot_cen",       // frame_id
                        Vector3f::Zero(), // at rot_cen
                        ik_solver_->pos_rot_cen_elb_1(),
                        ik_solver_->pos_rot_cen_m_1(),
                        ltri_color_
                      );
      
      publishTriMarker(
                        spm_ltri_2_pub_,
                        0,                // marker_id
                        "spm_ltri_2",         // ns
                        "spm_rot_cen",       // frame_id
                        Vector3f::Zero(), // at rot_cen
                        ik_solver_->pos_rot_cen_elb_2(),
                        ik_solver_->pos_rot_cen_m_2(),
                        ltri_color_
                      );

      // Upper triangles for rot_cen-elb-epl
      publishTriMarker(
                        spm_utri_0_pub_,
                        0,                // marker_id
                        "spm_utri_0",         // ns
                        "spm_rot_cen",       // frame_id
                        Vector3f::Zero(), // at rot_cen
                        ik_solver_->pos_rot_cen_elb_0(),
                        ik_solver_->pos_rot_cen_epl_0(),
                        utri_color_
                      );
      
      publishTriMarker(
                        spm_utri_1_pub_,
                        0,                // marker_id
                        "spm_utri_1",         // ns
                        "spm_rot_cen",       // frame_id
                        Vector3f::Zero(), // at rot_cen
                        ik_solver_->pos_rot_cen_elb_1(),
                        ik_solver_->pos_rot_cen_epl_1(),
                        utri_color_
                      );
      
      publishTriMarker(
                        spm_utri_2_pub_,
                        0,                // marker_id
                        "spm_utri_2",         // ns
                        "spm_rot_cen",       // frame_id
                        Vector3f::Zero(), // at rot_cen
                        ik_solver_->pos_rot_cen_elb_2(),
                        ik_solver_->pos_rot_cen_epl_2(),
                        utri_color_
                      );

      // Publish light ray marker
      publishLightRayMarker(
                            light_ray_marker_pub_,
                            0,                // marker_id
                            "spm_light_ray",         // ns
                            "spm_ept",       // frame_id
                            Vector3f::Zero(), // at rot_cen
                            Vector3f(0.0, 0.0, ik_solver_->r_s_elb() * 2.0f), // at spm_ept
                            light_ray_color_
                          );
      
      // Publish the trajectory line strip
      publishTrajMarker(ee_traj_color_);

  }

  bool checkAchieveNewOrigin()
  {
    // Check if the re-origining is achieved
    Quaternionf relative_quat = quat_ubase_epl_c_cmd_.inverse() * RM::Quatz(th_z_ee_reset_ * RM::d2r).cast<float>();
    Vector3d relative_so3 = RM::Quat2so3(relative_quat.cast<double>());
    return ( relative_so3.norm() < quat_err_thresh_ ) ? true : false;
  }

  bool checkAchieveHome()
  {
    // Check if the homing is achieved
    Quaternionf relative_quat = quat_ubase_epl_c_cmd_.inverse() * RM::Quatz(-th_z_ee_reset_ * RM::d2r).cast<float>();
    Vector3d relative_so3 = RM::Quat2so3(relative_quat.cast<double>());
    return ( relative_so3.norm() < quat_err_thresh_ ) ? true : false;
  }

  bool checkFinishTask()
  {
      // Check if quat_ubase_epl_c_cmd_ achieved quat_ubase_epl_c_end_
      Quaternionf relative_quat = quat_ubase_epl_c_cmd_.inverse() * quat_ubase_epl_c_end_;
      Vector3d relative_so3 = RM::Quat2so3(relative_quat.cast<double>());
      return ( relative_so3.norm() < quat_err_thresh_ ) ? true : false;
  }

  // -------------
  // SUB + UTILS
  // -------------


  // -------------
  // GOAL callbacks
  // -------------
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const SPM::Goal> goal_msg)
  {
    RCLCPP_INFO(this->get_logger(),
      "Received SPM goal: start=%d", (int)goal_msg->start);
    if (!goal_msg->start) {
      RCLCPP_WARN(this->get_logger(), "Goal->start == false => reject!");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Re-initialise each time goal is accepted
    initTimeSpec();
    resetTime();
    initFSM();
    initMotionParams();
    initControlParams();
    initSPMIK();
    initRobotControl();
    initRobotVisual();

    RCLCPP_INFO(this->get_logger(), "Goal accepted => start SPM!");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSPM> /*gh*/)
  {
    RCLCPP_INFO(this->get_logger(), "[SPM] Cancel request => accept!");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSPM> gh)
  {
    // Spin up a new thread
    std::thread([this, gh]() {
      this->execute(gh);
    }).detach();
  }



  // -------------
  // EXECUTION
  // -------------
  void execute(const std::shared_ptr<GoalHandleSPM> gh)
  {
    RCLCPP_INFO(this->get_logger(), "[SPM] Start execution (SPM).");

    auto result   = std::make_shared<SPM::Result>();
    auto feedback = std::make_shared<SPM::Feedback>();

    /* Main loop */
    rclcpp::Rate loop_rate(fs_);
    while (rclcpp::ok()) {
      if (gh->is_canceling()) {
        RCLCPP_WARN(this->get_logger(), "[SPM] Canceled by client!");
        
        // Reset time
        resetTime();

        result->success = false;
        result->message = "Canceled by client";
        gh->canceled(result);
        return;
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (stop_action_) {
          RCLCPP_WARN(this->get_logger(), "[SPM] Stopped externally => abort!");
          result->success = false;
          result->message = "Stopped externally";
          gh->abort(result);
          return;
        }
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!true) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM] Waiting for data subscription. . .");
          loop_rate.sleep();
          continue;
        }
      }


      // Update current time
      current_time_ = this->now();
  

      // Switch FSM states
      switch (fsm_state_)
      {
        case SPMFSMState::IDLE:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM FSM] IDLE");
          
          if (task_finished_) {
            // Switch FSM state after waiting for a while
            fsm_state_ = (waitForTime(TRAIL_DURATION_SEC)) ? SPMFSMState::END : fsm_state_;
          } else {
            // Switch FSM state
            fsm_state_ = SPMFSMState::INITIATE;
          }
          break;
        }

        case SPMFSMState::INITIATE:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM FSM] INITIATE");

          if (true) {
            // Reset time
            resetTime();
            // Switch FSM state
            fsm_state_ = SPMFSMState::REORIGINING;
          }
          break;
        }

        case SPMFSMState::REORIGINING:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM FSM] REORIGINING");
          
          // Re-origining pose
          getReoriginingPose();
          // Solve IK (joint angles) for the spm
          solveSPMIK();
          // Send joint angles command (moving to the new origin)
          sendJointCmd();

          // Check if new origin is achieved
          if (checkAchieveNewOrigin()) {   
            // Reset time
            resetTime();         
            // Switch FSM state
            fsm_state_ = SPMFSMState::REORIGINED;
          }
          break;
        }

        case SPMFSMState::REORIGINED:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM FSM] REORIGINED");

          // Change SPM mode to control
          spm_mode_ = "CONTROL";
          std::cout << "[SPM mode]: " << spm_mode_ << std::endl;
          
          // Reset origin after finishing re-origining
          ik_reset_origin_result_ = ik_solver_->resetOrigin(spm_mode_);
          
          // Update the joint angles command for motors
          joint_angles_cmd_ << ik_reset_origin_result_.th_0_nom, /* Nominal motor angles */ 
                               ik_reset_origin_result_.th_1_nom, 
                               ik_reset_origin_result_.th_2_nom;
          
          std::cout << "[Re-origining angle] th_z_ee_reset [deg] = " << th_z_ee_reset_ << std::endl;
          std::cout << "[Re-origining orientation] quat_ubase_epl_c_reset = " << ik_reset_origin_result_.quat_ubase_epl_c_reset.w() << ", " << ik_reset_origin_result_.quat_ubase_epl_c_reset.x() << ", " << ik_reset_origin_result_.quat_ubase_epl_c_reset.y() << ", " << ik_reset_origin_result_.quat_ubase_epl_c_reset.z() << std::endl;
          std::cout << "[Re-origined IK nominal angles] th_0_nom, th_1_nom, th_2_nom [deg] = " << ik_reset_origin_result_.th_0_nom * RM::r2d << ", " << ik_reset_origin_result_.th_1_nom * RM::r2d << ", " << ik_reset_origin_result_.th_2_nom * RM::r2d << std::endl;
          
          
          // Print re-originined joint angles
          /* 
           * Print joint angles feedback from the motor 
           */

          

          if (true) {
            // Reset time
            resetTime();    
            // Reset IK target pose to identity
            quat_ubase_epl_c_cmd_ = Quaternionf::Identity();     
            // Switch FSM state
            fsm_state_ = SPMFSMState::CONTROL;
          }
          break;
        }

        case SPMFSMState::CONTROL:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM FSM] CONTROL");
          
          // Get IK target pose (orientation)
          getTargetPose();
          // Solve IK (joint angles) for the spm
          solveSPMIK();
          // Send joint angles command
          sendJointCmd();

          // // Check finishing the task
          // task_finished_ = checkFinishTask();

          if (task_finished_) {
            // Reset time
            resetTime();
            // Save end pose after finishing the task
            quat_ubase_epl_c_end_ = quat_ubase_epl_c_cmd_;
            // Switch FSM state
            fsm_state_ = SPMFSMState::HOMING;
          }
          break;
        }

        case SPMFSMState::HOMING:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM FSM] HOMING");
          
          // Homing pose
          getHomingPose();
          // Solve IK (joint angles) for the spm
          solveSPMIK();
          // Send joint angles command (moving to the home)
          sendJointCmd();

          // Check if home is achieved
          if (checkAchieveHome()) {        
            // Reset time
            resetTime();    
            // Switch FSM state
            fsm_state_ = SPMFSMState::HOMED;
          }
          break;
        }

        case SPMFSMState::HOMED:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM FSM] HOMED");

          // Change SPM mode to control
          spm_mode_ = "HOME";
          std::cout << "[SPM mode]: " << spm_mode_ << std::endl;
          
          // Reset origin after finishing homing
          ik_reset_origin_result_ = ik_solver_->resetOrigin(spm_mode_);
          
          // Update the joint angles command for motors
          joint_angles_cmd_ << ik_reset_origin_result_.th_0_nom, /* Nominal motor angles */ 
                               ik_reset_origin_result_.th_1_nom, 
                               ik_reset_origin_result_.th_2_nom;
          
          std::cout << "[Re-origining orientation] quat_ubase_epl_c_reset = " << ik_reset_origin_result_.quat_ubase_epl_c_reset.w() << ", " << ik_reset_origin_result_.quat_ubase_epl_c_reset.x() << ", " << ik_reset_origin_result_.quat_ubase_epl_c_reset.y() << ", " << ik_reset_origin_result_.quat_ubase_epl_c_reset.z() << std::endl;
          std::cout << "[Re-origined IK nominal angles] th_0_nom, th_1_nom, th_2_nom [deg] = " << ik_reset_origin_result_.th_0_nom * RM::r2d << ", " << ik_reset_origin_result_.th_1_nom * RM::r2d << ", " << ik_reset_origin_result_.th_2_nom * RM::r2d << std::endl;
          
          
          // Print re-originined joint angles
          /* 
           * Print joint angles feedback from the motor 
           */

          

          if (true) {
            // Reset time
            resetTime();     
            // Reset IK target pose to identity
            quat_ubase_epl_c_cmd_ = Quaternionf::Identity();
            // Switch FSM state        
            fsm_state_ = SPMFSMState::IDLE;
          }
          break;
        }
          
        case SPMFSMState::END:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM FSM] END");
            
          result->success = true;
          result->message = "Task finished => SPM done!";
          gh->succeed(result);

          // Switch FSM state
          fsm_state_ = SPMFSMState::IDLE;
          return;
        }

      }


      // Publishing data
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Build feedback => current_angles
        feedback->current_angles.clear();
        for (int i = 0; i < ik_joint_angles_.size(); ++i) {
          feedback->current_angles.push_back(ik_joint_angles_(i));
        }

        // Publish feedback
        gh->publish_feedback(feedback);

        // Publish all TF frames:
        publishAllTransforms();
        // Publish markers
        publisherAllMarkers();

      }


      loop_rate.sleep();
    } // while ok

    // Shut down
    RCLCPP_WARN(this->get_logger(), "[SPM] ROS shutting down => abort!");
    result->success = false;
    result->message = "ROS shutdown";
    gh->abort(result);
  }

}; // end class SPMActionServer


// Main
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SPMActionServer>());
  rclcpp::shutdown();
  return 0;
}
