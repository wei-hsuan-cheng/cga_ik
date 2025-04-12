#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// The generated header from "SPM.action"
#include "cga_ik_action_interfaces/action/spm.hpp"

// Standard includes 
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// Robot math utils
#include "robot_math_utils/robot_math_utils_v1_10.hpp" 

// Eigen, etc.
#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>


using SPM = cga_ik_action_interfaces::action::SPM;
using GoalHandleSPM  = rclcpp_action::ServerGoalHandle<SPM>;

using RM = RMUtils;
std::string node_name = "spm_action_server";

class SPMActionServer : public rclcpp::Node
{
public:
  SPMActionServer()
  : Node(node_name),
    fs_(30.0),            
    Ts_(1.0 / fs_)
  {
    // Initialise params
    fsm_state_   = SPMFSMState::IDLE; // Initial state duing the execution
    stop_action_ = false;
    k_ = 0;
    t_ = 0;

    std::cout << "[SPM] Loaded params" << std::endl;


    // ROS 2 components

    // Action server
    action_server__ = rclcpp_action::create_server<SPM>(
      this,
      "spm", // the action name
      std::bind(&SPMActionServer::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SPMActionServer::handle_cancel,   this, std::placeholders::_1),
      std::bind(&SPMActionServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "SPM Action Server started.");
  }

private:
  // Service/Action server and client
  rclcpp_action::Server<SPM>::SharedPtr action_server__;

  // Subs and pubs

  /* Control sampling rate */
  double fs_, Ts_;
  int k_;
  double t_;


  // FSM
  bool stop_action_;
  std::mutex data_mutex_;

  enum class SPMFSMState {
    IDLE,
    RESET_ORIGIN,
    CONTROL
  };
  SPMFSMState fsm_state_;


  // -------------
  // SUB + UTILS
  // -------------


  Vector6d mrad2mmdeg(const Vector6d & vec)
  {
    Vector6d vec_mm_deg;
    vec_mm_deg << vec.head(3) * 1000.0, vec.tail(3) * (180.0 / M_PI);
    return vec_mm_deg;
  }

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
      RCLCPP_WARN(this->get_logger(), "Goal->start == false => reject");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Update parameters each time goal is accepted
    fsm_state_ = SPMFSMState::IDLE;

    RCLCPP_INFO(this->get_logger(), "Goal accepted => start self-alignment!");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSPM> /*gh*/)
  {
    RCLCPP_INFO(this->get_logger(), "[SPM] Cancel request => accept");
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
        result->success = false;
        result->message = "Canceled by client";
        gh->canceled(result);
        return;
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (stop_action_) {
          RCLCPP_WARN(this->get_logger(), "[SPM] Stopped externally => abort");
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

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!true) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM] Waiting for target ID selection. . .");
          loop_rate.sleep();
          continue;
        }
      }

      // Evaluate the current pos_so3_error
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Build feedback => current_angles
        feedback->current_angles.clear();
        feedback->current_angles.push_back(0.0);
        feedback->current_angles.push_back(1.0);


        // Publish feedback
        gh->publish_feedback(feedback);

      }

  
      // Switch FSM states
      switch (fsm_state_)
      {
        case SPMFSMState::IDLE:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM FSM] IDLE");
          if (true) {
            fsm_state_ = SPMFSMState::RESET_ORIGIN; // Start vision alignment
          }
          break;
        }
          
        case SPMFSMState::RESET_ORIGIN:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM FSM] RESET_ORIGIN");
          if (true) {
            fsm_state_ = SPMFSMState::CONTROL;
          }
          break;
        }
          
        case SPMFSMState::CONTROL:
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[SPM FSM] CONTROL");
          result->success = true;
          result->message = "Finished loopping => SPM done.";
          gh->succeed(result);

          fsm_state_ = SPMFSMState::IDLE;
          return;
        }

      }

      loop_rate.sleep();
    } // while ok

    // Shut down
    RCLCPP_WARN(this->get_logger(), "[SPM] ROS shutting down => abort");
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
