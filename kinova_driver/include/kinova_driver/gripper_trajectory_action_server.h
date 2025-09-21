#ifndef GRIPPER_TRAJECTORY_ACTION_SERVER_H
#define GRIPPER_TRAJECTORY_ACTION_SERVER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "kinova_msgs/action/set_fingers_position.hpp"
#include "kinova_msgs/msg/finger_position.hpp"
#include <atomic>

namespace kinova
{
using FJTAS = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJTAS = rclcpp_action::ServerGoalHandle<FJTAS>;

using SFPAC = kinova_msgs::action::SetFingersPosition;
using GoalHandleSFPAC = rclcpp_action::ClientGoalHandle<SFPAC>;

class GripperTrajectoryActionController
{
public:
  GripperTrajectoryActionController(std::shared_ptr<rclcpp::Node> n, std::string &robot_name);
  ~GripperTrajectoryActionController();

  void handle_accepted(const std::shared_ptr<GoalHandleFJTAS> gh);
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FJTAS::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJTAS> gh);

private:
  std::shared_ptr<rclcpp::Node> nh_;

  rclcpp_action::Server<FJTAS>::SharedPtr action_server_follow_;
  rclcpp_action::Client<SFPAC>::SharedPtr action_client_set_finger_;
  rclcpp::Subscription<kinova_msgs::msg::FingerPosition>::SharedPtr sub_fingers_state_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  bool has_active_goal_ {false};
  bool first_fb_ {true};
  rclcpp::Time start_time_;
  std::shared_ptr<GoalHandleFJTAS> active_goal_;
  std::shared_ptr<FJTAS::Result> active_result_;
  trajectory_msgs::msg::JointTrajectory current_traj_;
  kinova_msgs::msg::FingerPosition last_finger_state_, empty_finger_state_;
  std::atomic<uint64_t> goal_serial_{0};

  std::vector<std::string> joint_names_;
  double goal_time_constraint_ {1.0};
  double final_settle_time_ {0.8};
  double tick_tolerance_ {80.0};
  double finger_max_turn_ {6800.0};
  double finger_conv_ratio_ {80.0 / 6800.0};

  // Conversion for joint positions -> finger ticks
  std::string position_units_ {"radians"}; // radians|meters|ticks
  double finger_max_angle_rad_ {1.3962634015954636}; // ~80 deg
  bool send_only_last_point_ {true};

  void goalCBFollow(std::shared_ptr<GoalHandleFJTAS> gh, uint64_t serial);
  void controllerStateCB(const kinova_msgs::msg::FingerPosition::SharedPtr msg);
  void watchdog();
  bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b) const;
  std::array<double,3> positionsToTicks(const std::vector<double> &positions, const std::vector<std::string> &names) const;
  std::array<double,3> ticksToPositions(const kinova_msgs::msg::FingerPosition &ticks,
                                        const std::vector<std::string> &names) const;
};

} // namespace kinova

#endif // GRIPPER_TRAJECTORY_ACTION_SERVER_H
