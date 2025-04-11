#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <memory>

#include "cga/cga_utils.hpp"
#include "cga_ik_spm_3dof/cga_ik_spm_3dof.hpp"
#include "robot_math_utils/robot_math_utils_v1_10.hpp"

using namespace cga_ik_spm_3dof;
using RM = RMUtils;

struct TimeStampedPoint {
  rclcpp::Time stamp;
  geometry_msgs::msg::Point point;
};


class VisualiseSPM3DoF : public rclcpp::Node
{
public:
  VisualiseSPM3DoF()
  : Node("visualise_spm_3dof")
  {
    // Initialisation
    initTimeSpec();
    initSPMIK();
    initRobotVisual();

    // ROS 2 components
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(Ts_ * 1000)),
      std::bind(&VisualiseSPM3DoF::visualise_spm_3dof_callback_, this)
    );

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/spm_3dof/joint_states", 10);
    
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

    RCLCPP_INFO(this->get_logger(), "visualise_spm_3dof node started.");

  }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // We'll also publish 3 joint angles
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr angles_pub_;

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

    double th_z_ee_init_;
    Vector4d axis_ang_cmd_;
    Quaternionf quat_cmd_;

    float r_c_, ang_b_m_, r_b_, d_, r_e_;
    float r_s_piv_, r_s_m_, r_s_elb_, r_s_epl_;
    float z_rot_cen_ee_;
    int krl_;

    CGA e_principal_, s_0_, s_1_;
    Vector4f pivot_sphere_color_, motor_sphere_color_, elbow_sphere_color_, epl_sphere_color_;
    Vector4f utri_color_, ltri_color_, light_ray_color_, ee_traj_color_;

    // For the trajectory
    std::deque<TimeStampedPoint> trajectory_buffer_;
    const double TRAIL_DURATION_SEC = 2.0; // keep last 2 seconds of end-effector path

    // IK solver object and result
    std::shared_ptr<CGAIKSPM3DoF> ik_solver_;
    SPM3DoFIKResetOrigin ik_reset_origin_result_;
    SPM3DoFIKResult ik_result_;


    void initTimeSpec()
    {
        fs_ = 60.0;
        Ts_ = 1.0 / fs_;
        k_ = 0;
        t_ = 0.0;
        current_time_ = this->now();
    }

    void initSPMIK()
    {   
        // SPM geometries
        // Load geometric params from yaml file
        // Declare default parameters (in case the YAML file doesn't provide them)
        this->declare_parameter<float>("r_c", 1.0);
        this->declare_parameter<float>("ang_b_m", 0.0);
        this->declare_parameter<float>("r_b", 1.0);
        this->declare_parameter<float>("d", 1.0);
        this->declare_parameter<float>("r_e", 1.0);

        this->declare_parameter<float>("r_s_piv", 1.0);
        this->declare_parameter<float>("r_s_m", 1.0);
        this->declare_parameter<float>("r_s_elb", 1.0);
        this->declare_parameter<float>("r_s_epl", 1.0);

        this->declare_parameter<int>("krl", 1);

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

        // Initial z-rotation for re-origining the coordinate system (mechanical origin -> geometric origin)
        this->declare_parameter<double>("th_z_ee_init", 0.0);
        this->get_parameter("th_z_ee_init", th_z_ee_init_);

        // Initial IK target pose (quaternion orientation)
        // axis_ang_cmd_ = Vector4d(0.0, 0.0, 1.0, th_z_ee_init_ * RM::d2r);
        // Vector3d so3 = (axis_ang_cmd_.head(3)).normalized() * axis_ang_cmd_(3);
        // quat_cmd_ = RM::so32Quat(so3).cast<float>();
        quat_cmd_ = RM::Quatz(th_z_ee_init_ * RM::d2r).cast<float>();
        

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
        e_principal_, s_0_, s_1_
        );

        // Solve initial pose
        ik_reset_origin_result_ = ik_solver_->resetEEOrigin(th_z_ee_init_);
        ik_result_ = ik_solver_->computeIK(quat_cmd_);

        // Print robot geometric parameters
        std::cout << "--------------------------------------------------------------" << std::endl;
        std::cout << "---------- Geometric parameters of the spm [m, deg] ----------" << std::endl;
        std::cout << "--------------------------------------------------------------" << std::endl;

        std::cout << "Lower parts" << std::endl;
        std::cout << "r_c = " << r_c_ << std::endl;
        std::cout << "ang_b_m = " << ang_b_m_ << std::endl;
        std::cout << "r_b = " << r_b_ << std::endl;
        
        std::cout << "Upper parts" << std::endl;
        std::cout << "d = " << d_ << std::endl;
        std::cout << "r_e = " << r_e_ << std::endl;

        std::cout << "Sphere radii" << std::endl;
        std::cout << "r_s_piv = " << r_s_piv_ << std::endl;
        std::cout << "r_s_m = " << r_s_m_ << std::endl;
        std::cout << "r_s_elb = " << r_s_elb_ << std::endl;
        std::cout << "r_s_epl = " << r_s_epl_ << std::endl;

        std::cout << "th_z_ee_init = " << th_z_ee_init_ << std::endl;
        std::cout << "[nom] th_0_init, th_1_init, th_2_init = " << ik_reset_origin_result_.th_0_nom * RM::r2d << ", " << ik_reset_origin_result_.th_1_nom * RM::r2d << ", " << ik_reset_origin_result_.th_2_nom * RM::r2d << std::endl;
        std::cout << "[nom] quat_ee_nom = " << ik_reset_origin_result_.quat_ee_nom.w() << ", " << ik_reset_origin_result_.quat_ee_nom.x() << ", " << ik_reset_origin_result_.quat_ee_nom.y() << ", " << ik_reset_origin_result_.quat_ee_nom.z() << std::endl;

        // std::cout << "ik_solver.s_0 = " << std::endl; ik_solver_->s_0_.log();

        std::cout << "--------------------------------------------------------------" << std::endl;


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

    void publishTrajMarkerLineStrip(
        const Vector4f rgba_color)
    {
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
        traj_marker.scale.x = 0.003; // e.g. 3mm wide

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


    void getTargetPose()
    {
        // Update time
        k_ += 1;
        t_ = k_ * Ts_;

        // Target orientation for the IK problem
        // double th = th_z_ee_init_;

        double freq = 0.1;
        double th = th_z_ee_init_ * std::sin(2.0 * M_PI * freq * t_);

        // double th = 0.0;

        axis_ang_cmd_ = Vector4d(0.0, 0.0, 1.0, th);
        // axis_ang_cmd_ = Vector4d(0.0, 1.0, 0.0, th);
        // axis_ang_cmd_ = Vector4d(1.0, 0.0, 0.0, th);

        Vector3d so3 = (axis_ang_cmd_.head(3)).normalized() * axis_ang_cmd_(3) * RM::d2r;
        quat_cmd_ = RM::so32Quat(so3).cast<float>();
    }

    void solveSPMIK() 
    {
        ik_result_ = ik_solver_->computeIK(quat_cmd_);
    }

    void publishIKSolution()
    {
        // Publish the joint states for these angles
        sensor_msgs::msg::JointState js_msg;
        // js_msg.header.stamp = this->now();
        js_msg.header.stamp = current_time_;

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
        publishTF(ik_solver_->pos_rot_cen_epl_c(), ik_solver_->quat_rot_cen_epl_c(), "spm_epl_c", "spm_rot_cen");

        // End-point on the end-plate sphere
        publishTF(ik_solver_->pos_rot_cen_ept(), ik_solver_->quat_rot_cen_ept(), "spm_ept", "spm_rot_cen");

        // End-effector
        publishTF(ik_solver_->pos_rot_cen_ee(), ik_solver_->quat_rot_cen_ee(), "spm_ee", "spm_rot_cen");

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
        publishTrajMarkerLineStrip(ee_traj_color_);


    }

    void debuggingInfo()
    {
        // // Compute the are of the end-plate triangle

        // // Compare the lengths
        // std::cout << "(Pivot sphere radius) r_s_piv = d_rc_elb_i = " << ik_result_.r_s_piv << std::endl;
        // std::cout << "(Elbow sphere radius) r_s_elb = d_rc_elp_i = " << ik_result_.r_s_elb << std::endl;
        // std::cout << "d = d_rc_yc = " << ik_result_.d << std::endl;


    }

    void visualise_spm_3dof_callback_()
    {
        auto start = std::chrono::steady_clock::now();

        // Get IK target pose (orientation)
        getTargetPose();
        // Solve IK (joint angles) for the spm
        solveSPMIK();

        auto end = std::chrono::steady_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
        double frequency = (elapsed_ms > 0.0) ? 1000.0 / elapsed_ms : 0.0;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Ts = %.2f [ms], fs = %.2f [Hz]", elapsed_ms, frequency);


        current_time_ = this->now();
        // Publish IK solution:
        publishIKSolution();
        // Publish all TF frames:
        publishAllTransforms();
        // Publish markers
        publisherAllMarkers();
        // Debugging
        debuggingInfo();
    }


};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisualiseSPM3DoF>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
