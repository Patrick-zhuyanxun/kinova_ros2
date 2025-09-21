#include <kinova_driver/gripper_trajectory_action_server.h>
#include <sstream>

using namespace kinova;

namespace kinova {

static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }

GripperTrajectoryActionController::GripperTrajectoryActionController(std::shared_ptr<rclcpp::Node> n, std::string &robot_name)
    : nh_(n)
{
  std::string address;

  // Action server namespace mirrors arm: /<robot>_gripper/follow_joint_trajectory
  address = "/" + robot_name + std::string("_gripper/follow_joint_trajectory");
  action_server_follow_ = rclcpp_action::create_server<FJTAS>(
      nh_, address,
      std::bind(&GripperTrajectoryActionController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GripperTrajectoryActionController::handle_cancel, this, std::placeholders::_1),
      std::bind(&GripperTrajectoryActionController::handle_accepted, this, std::placeholders::_1));

  // Client to finger position action provided by driver
  address = "/" + robot_name + std::string("_driver/finger_positions");
  action_client_set_finger_ = rclcpp_action::create_client<SFPAC>(nh_, address);

  // Finger state subscription for goal completion
  address = "/" + robot_name + std::string("_driver/out/finger_position");
  sub_fingers_state_ = nh_->create_subscription<kinova_msgs::msg::FingerPosition>(
      address, 1, std::bind(&GripperTrajectoryActionController::controllerStateCB, this, std::placeholders::_1));

  // Params
  if (!nh_->has_parameter("constraints/goal_time")) nh_->declare_parameter("constraints/goal_time", goal_time_constraint_);
  nh_->get_parameter("constraints/goal_time", goal_time_constraint_);
  if (!nh_->has_parameter("constraints/final_settle_time")) nh_->declare_parameter("constraints/final_settle_time", final_settle_time_);
  nh_->get_parameter("constraints/final_settle_time", final_settle_time_);
  if (!nh_->has_parameter("finger_max_turn")) nh_->declare_parameter("finger_max_turn", finger_max_turn_);
  nh_->get_parameter("finger_max_turn", finger_max_turn_);
  if (!nh_->has_parameter("finger_conv_ratio")) nh_->declare_parameter("finger_conv_ratio", finger_conv_ratio_);
  nh_->get_parameter("finger_conv_ratio", finger_conv_ratio_);
  if (!nh_->has_parameter("tick_tolerance")) nh_->declare_parameter("tick_tolerance", tick_tolerance_);
  nh_->get_parameter("tick_tolerance", tick_tolerance_);
  if (!nh_->has_parameter("position_units")) nh_->declare_parameter("position_units", position_units_);
  nh_->get_parameter("position_units", position_units_);
  if (!nh_->has_parameter("finger_max_angle_rad")) nh_->declare_parameter("finger_max_angle_rad", finger_max_angle_rad_);
  nh_->get_parameter("finger_max_angle_rad", finger_max_angle_rad_);
  if (!nh_->has_parameter("send_only_last_point")) nh_->declare_parameter("send_only_last_point", send_only_last_point_);
  nh_->get_parameter("send_only_last_point", send_only_last_point_);

  // Finger joint names (robot-prefixed, to match MoveIt trajectories)
  joint_names_.resize(3);
  joint_names_[0] = robot_name + "_joint_finger_1";
  joint_names_[1] = robot_name + "_joint_finger_2";
  joint_names_[2] = robot_name + "_joint_finger_3";

  // Optional: allow system to start before first goals (no spinning here to avoid executor conflicts)
  (void)last_finger_state_;

  // Watchdog similar to arm
  watchdog_timer_ = nh_->create_wall_timer(std::chrono::seconds(1), std::bind(&GripperTrajectoryActionController::watchdog, this));
}

GripperTrajectoryActionController::~GripperTrajectoryActionController()
{
  sub_fingers_state_.reset();
  watchdog_timer_.reset();
}

bool GripperTrajectoryActionController::setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b) const {
  if (a.size() != b.size()) return false;
  for (auto &n : a) if (std::count(b.begin(), b.end(), n) != 1) return false;
  for (auto &n : b) if (std::count(a.begin(), a.end(), n) != 1) return false;
  return true;
}

std::array<double,3> GripperTrajectoryActionController::positionsToTicks(const std::vector<double> &positions, const std::vector<std::string> &names) const {
  // Map positions based on configured unit type
  // radians: 0..finger_max_angle_rad -> 0..finger_max_turn
  // meters:  gap meters -> ticks via finger_conv_ratio
  // ticks:   pass-through
  std::array<double,3> ticks{0,0,0};
  auto idxOfSuffix = [&](const std::string &suffix){
    for (size_t i = 0; i < names.size(); ++i) {
      const auto &nm = names[i];
      if (nm.size() >= suffix.size() && nm.compare(nm.size()-suffix.size(), suffix.size(), suffix) == 0) {
        return static_cast<int>(i);
      }
    }
    return -1;
  };
  int i1 = idxOfSuffix("_joint_finger_1");
  int i2 = idxOfSuffix("_joint_finger_2");
  int i3 = idxOfSuffix("_joint_finger_3");
  auto conv_one = [&](int i)->double{
    if (i < 0 || i >= static_cast<int>(positions.size())) return 0.0;
    double p = positions[i];
    if (position_units_ == "radians") {
      double s = clamp(p, 0.0, finger_max_angle_rad_);
      return (s / finger_max_angle_rad_) * finger_max_turn_;
    } else if (position_units_ == "meters") {
      double s = clamp(p, 0.0, finger_max_turn_ * finger_conv_ratio_);
      return s / finger_conv_ratio_;
    } else { // ticks
      return clamp(p, 0.0, finger_max_turn_);
    }
  };
  ticks[0] = conv_one(i1);
  ticks[1] = conv_one(i2);
  ticks[2] = conv_one(i3);
  return ticks;
}

std::array<double,3> GripperTrajectoryActionController::ticksToPositions(const kinova_msgs::msg::FingerPosition &ticks,
                                        const std::vector<std::string> &names) const {
  std::array<double,3> pos{0,0,0};
  auto set_if_suffix = [&](const std::string &suffix, double value){
    for (size_t i = 0; i < names.size(); ++i) {
      const auto &nm = names[i];
      if (nm.size() >= suffix.size() && nm.compare(nm.size()-suffix.size(), suffix.size(), suffix) == 0) {
        pos[i] = value;
        break;
      }
    }
  };
  auto clamp_ticks = [&](double t){ return clamp(t, 0.0, finger_max_turn_); };
  if (position_units_ == "radians") {
    double r1 = (clamp_ticks(ticks.finger1) / finger_max_turn_) * finger_max_angle_rad_;
    double r2 = (clamp_ticks(ticks.finger2) / finger_max_turn_) * finger_max_angle_rad_;
    double r3 = (clamp_ticks(ticks.finger3) / finger_max_turn_) * finger_max_angle_rad_;
    set_if_suffix("_joint_finger_1", r1);
    set_if_suffix("_joint_finger_2", r2);
    set_if_suffix("_joint_finger_3", r3);
  } else if (position_units_ == "meters") {
    double m1 = clamp_ticks(ticks.finger1) * finger_conv_ratio_;
    double m2 = clamp_ticks(ticks.finger2) * finger_conv_ratio_;
    double m3 = clamp_ticks(ticks.finger3) * finger_conv_ratio_;
    set_if_suffix("_joint_finger_1", m1);
    set_if_suffix("_joint_finger_2", m2);
    set_if_suffix("_joint_finger_3", m3);
  } else { // ticks
    set_if_suffix("_joint_finger_1", clamp_ticks(ticks.finger1));
    set_if_suffix("_joint_finger_2", clamp_ticks(ticks.finger2));
    set_if_suffix("_joint_finger_3", clamp_ticks(ticks.finger3));
  }
  return pos;
}

rclcpp_action::GoalResponse GripperTrajectoryActionController::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FJTAS::Goal> goal)
{
  RCLCPP_INFO(nh_->get_logger(), "Gripper FollowJointTrajectory: received goal");
  (void) uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperTrajectoryActionController::handle_cancel(const std::shared_ptr<GoalHandleFJTAS> gh)
{
  RCLCPP_INFO(nh_->get_logger(), "Gripper FollowJointTrajectory: cancel request");
  // Cancel the active goal, if any
  (void) gh;
  if (has_active_goal_ && active_goal_) {
    has_active_goal_ = false;
    auto res = std::make_shared<FJTAS::Result>();
    try { active_goal_->canceled(res); } catch (...) {}
    active_goal_.reset();
    current_traj_.points.clear();
    ++goal_serial_; // preempt any running goal thread
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperTrajectoryActionController::handle_accepted(const std::shared_ptr<GoalHandleFJTAS> goal_handle)
{
  RCLCPP_INFO(nh_->get_logger(), "Gripper FollowJointTrajectory: accepted goal");
  // If a goal is active, cancel it now to avoid RViz/MoveIt holding UI
  if (has_active_goal_ && active_goal_) {
    try {
      active_goal_->canceled(active_result_ ? active_result_ : std::make_shared<FJTAS::Result>());
    } catch (...) {}
    has_active_goal_ = false;
    current_traj_.points.clear();
  }
  // Preempt previous goal by incrementing serial
  const auto serial = ++goal_serial_;
  std::thread{std::bind(&GripperTrajectoryActionController::goalCBFollow, this, std::placeholders::_1, std::placeholders::_2), goal_handle, serial}.detach();
}

void GripperTrajectoryActionController::goalCBFollow(std::shared_ptr<GoalHandleFJTAS> gh, uint64_t serial)
{
  const auto goal = gh->get_goal();
  active_goal_ = gh;
  active_result_.reset(new FJTAS::Result());
  // Accept trajectories that at least contain the expected finger joints
  size_t present = 0;
  for (const auto &req : joint_names_) {
    present += (std::count(goal->trajectory.joint_names.begin(), goal->trajectory.joint_names.end(), req) == 1);
  }
  if (present != joint_names_.size()) {
    RCLCPP_ERROR(nh_->get_logger(), "Gripper joint names mismatch: expected 3 finger joints with robot prefix");
    auto res = std::make_shared<FJTAS::Result>();
    gh->abort(res);
    return;
  }

  // Prepare trajectory and append settle point
  current_traj_ = goal->trajectory;
  start_time_ = nh_->get_clock()->now();
  if (current_traj_.header.stamp.sec == 0 && current_traj_.header.stamp.nanosec == 0)
    current_traj_.header.stamp = start_time_;
  if (current_traj_.points.empty()) {
    RCLCPP_WARN(nh_->get_logger(), "Gripper trajectory empty; abort");
    auto res = std::make_shared<FJTAS::Result>();
    gh->abort(res);
    return;
  }
  // Force last point velocities to zero and append settle hold (unless we only send last point)
  auto &last_pt = current_traj_.points.back();
  last_pt.velocities.assign(joint_names_.size(), 0.0);
  if (!send_only_last_point_ && final_settle_time_ > 1e-6) {
    auto settle_pt = last_pt;
    double total_sec = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9 + final_settle_time_;
    settle_pt.time_from_start.sec = static_cast<int32_t>(floor(total_sec));
    settle_pt.time_from_start.nanosec = static_cast<uint32_t>((total_sec - settle_pt.time_from_start.sec) * 1e9);
    current_traj_.points.push_back(settle_pt);
  }

  // Send either last point (default) or each point in sequence as SetFingersPosition goal(s)
  has_active_goal_ = true;
  if (send_only_last_point_) {
    const auto &pt = current_traj_.points.back();
    auto ticks = positionsToTicks(pt.positions, current_traj_.joint_names);
    kinova_msgs::action::SetFingersPosition::Goal g;
    g.fingers.finger1 = clamp(ticks[0], 0.0, finger_max_turn_);
    g.fingers.finger2 = clamp(ticks[1], 0.0, finger_max_turn_);
    g.fingers.finger3 = clamp(ticks[2], 0.0, finger_max_turn_);

  // When coalescing to last point, send immediately (do not delay RViz’s next goal)

    if (!action_client_set_finger_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(nh_->get_logger(), "Finger action server unavailable");
      auto res = std::make_shared<FJTAS::Result>();
      gh->abort(res);
      has_active_goal_ = false;
      return;
    }
    if (serial != goal_serial_) {
      RCLCPP_INFO(nh_->get_logger(), "Newer gripper goal arrived before send, skipping send");
      has_active_goal_ = false;
      if (active_goal_) active_goal_->canceled(active_result_);
      return;
    }
    (void)action_client_set_finger_->async_send_goal(g);
    RCLCPP_INFO(nh_->get_logger(), "Gripper trajectory sent last waypoint only");
  } else {
    for (size_t i = 0; i < current_traj_.points.size() && rclcpp::ok(); ++i) {
      const auto &pt = current_traj_.points[i];
      auto ticks = positionsToTicks(pt.positions, current_traj_.joint_names);
      kinova_msgs::action::SetFingersPosition::Goal g;
      g.fingers.finger1 = clamp(ticks[0], 0.0, finger_max_turn_);
      g.fingers.finger2 = clamp(ticks[1], 0.0, finger_max_turn_);
      g.fingers.finger3 = clamp(ticks[2], 0.0, finger_max_turn_);

      auto due = start_time_ + pt.time_from_start;
      while (rclcpp::ok() && nh_->get_clock()->now() < due) {
        if (serial != goal_serial_) {
          RCLCPP_INFO(nh_->get_logger(), "Newer gripper goal arrived, aborting previous sequence");
          has_active_goal_ = false;
          if (active_goal_) active_goal_->canceled(active_result_);
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }

      if (!action_client_set_finger_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(nh_->get_logger(), "Finger action server unavailable");
        auto res = std::make_shared<FJTAS::Result>();
        gh->abort(res);
        has_active_goal_ = false;
        return;
      }
      if (serial != goal_serial_) {
        RCLCPP_INFO(nh_->get_logger(), "Newer gripper goal arrived before send, skipping remaining points");
        has_active_goal_ = false;
        if (active_goal_) active_goal_->canceled(active_result_);
        return;
      }
      (void)action_client_set_finger_->async_send_goal(g);
    }
    RCLCPP_INFO(nh_->get_logger(), "Gripper trajectory published %zu waypoints", current_traj_.points.size());
  }
}

void GripperTrajectoryActionController::controllerStateCB(const kinova_msgs::msg::FingerPosition::SharedPtr msg)
{
  last_finger_state_ = *msg;
  if (!has_active_goal_) return;
  if (current_traj_.points.empty()) return;
  // If another goal superseded this, do nothing
  if (!active_goal_) return;

  // Completion criteria
  const int last = static_cast<int>(current_traj_.points.size()) - 1;
  const auto ticks = positionsToTicks(current_traj_.points[last].positions, current_traj_.joint_names);
  const double e1 = std::fabs(msg->finger1 - ticks[0]);
  const double e2 = std::fabs(msg->finger2 - ticks[1]);
  const double e3 = std::fabs(msg->finger3 - ticks[2]);
  const bool within_tol = (e1 <= tick_tolerance_ && e2 <= tick_tolerance_ && e3 <= tick_tolerance_);
  // Publish feedback for MoveIt/RViz
  try {
    if (active_goal_) {
      auto fb = std::make_shared<FJTAS::Feedback>();
      fb->joint_names = current_traj_.joint_names;
      fb->desired.positions = current_traj_.points[last].positions;
      auto actual_pos = ticksToPositions(*msg, current_traj_.joint_names);
      fb->actual.positions.assign(actual_pos.begin(), actual_pos.end());
      fb->error.positions.resize(fb->desired.positions.size());
      for (size_t i = 0; i < fb->desired.positions.size(); ++i) {
        fb->error.positions[i] = fb->desired.positions[i] - fb->actual.positions[i];
      }
      active_goal_->publish_feedback(fb);
    }
  } catch (...) {}
  if (send_only_last_point_) {
    if (within_tol) {
      has_active_goal_ = false;
      auto res = std::make_shared<FJTAS::Result>();
      if (active_goal_) {
        active_goal_->succeed(res);
        active_goal_.reset();
      }
      RCLCPP_INFO(nh_->get_logger(), "Gripper trajectory goal succeeded");
    }
  } else {
    rclcpp::Time end_time = start_time_ + current_traj_.points[last].time_from_start;
    rclcpp::Time now = nh_->get_clock()->now();
    if (now >= (end_time - rclcpp::Duration(goal_time_constraint_, 0)) && within_tol) {
      has_active_goal_ = false;
      auto res = std::make_shared<FJTAS::Result>();
      if (active_goal_) {
        active_goal_->succeed(res);
        active_goal_.reset();
      }
      RCLCPP_INFO(nh_->get_logger(), "Gripper trajectory goal succeeded");
    }
  }
}

void GripperTrajectoryActionController::watchdog()
{
  if (!has_active_goal_) return;
  if (current_traj_.points.empty()) return;
  // If we're far past the final due time and haven't succeeded, abort to avoid hanging
  const auto last = static_cast<int>(current_traj_.points.size()) - 1;
  rclcpp::Time end_time;
  if (send_only_last_point_) {
    // Use a generous fixed timeout window when not timing against trajectory times
    end_time = start_time_ + rclcpp::Duration(10, 0);
  } else {
    end_time = start_time_ + current_traj_.points[last].time_from_start + rclcpp::Duration(goal_time_constraint_ * 2.0, 0);
  }
  const rclcpp::Time now = nh_->get_clock()->now();
  if (now > end_time) {
    has_active_goal_ = false;
    auto res = std::make_shared<FJTAS::Result>();
  if (active_goal_) { active_goal_->abort(res); active_goal_.reset(); }
    RCLCPP_WARN(nh_->get_logger(), "Gripper trajectory watchdog aborted overdue goal");
  }
}

} // namespace kinova

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("gripper_trajectory_action_server");
  std::string robot_name;
  if ((argc <= 1) || (argv[argc-1] == NULL)) {
    std::cerr << "No kinova_robot_name provided in the argument!" << std::endl;
    return -1;
  } else {
    robot_name = argv[argc-1];
  }
  kinova::GripperTrajectoryActionController server(node, robot_name);
  (void)server;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
