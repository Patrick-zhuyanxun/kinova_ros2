#include <kinova_driver/joint_trajectory_action_server.h>
#include <angles/angles.h>
#include <sstream>

using namespace kinova;

JointTrajectoryActionController::JointTrajectoryActionController(std::shared_ptr<rclcpp::Node> n, std::string &robot_name):
nh_(n),
has_active_goal_(false),
first_fb_(true)
{
std::string robot_type = robot_name;
std::string address;

// address = "/" + robot_name + "_driver/robot_type";
// nh_.getParam(address,robot_type);
// if (robot_type == "")
// {
// ROS_ERROR_STREAM("Parameter "<<address<<" not found, make sure robot driver node is running");
// }

address = "/" + robot_name + "/follow_joint_trajectory";

action_server_follow_ = rclcpp_action::create_server<FJTAS>(
nh_,
address,
std::bind(&JointTrajectoryActionController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
std::bind(&JointTrajectoryActionController::handle_cancel, this, std::placeholders::_1),
std::bind(&JointTrajectoryActionController::handle_accepted, this, std::placeholders::_1));

int arm_joint_num = robot_type[3]-'0';
joint_names_.resize(arm_joint_num);

for (uint i = 0; i<joint_names_.size(); i++)
{
joint_names_[i] = robot_name + "_joint_" + std::to_string(i+1);
}

// Allowed time after nominal trajectory end for goal constraints to be met (seconds)
goal_time_constraint_ = 3.0;
if (!nh_->has_parameter("constraints/goal_time"))
nh_->declare_parameter("constraints/goal_time", goal_time_constraint_);
nh_->get_parameter("constraints/goal_time", goal_time_constraint_);

// Gets the constraints for each joint.
for (size_t i = 0; i < joint_names_.size(); ++i)
{
std::string ns = std::string("constraints/") + joint_names_[i];
double g = 0.01;
double t = 0.01;

if (!nh_->has_parameter(ns + "/goal"))
nh_->declare_parameter(ns + "/goal", g);
nh_->get_parameter(ns + "/goal", g);
if (!nh_->has_parameter(ns + "/trajectory"))
nh_->declare_parameter(ns + "/trajectory", t);
nh_->get_parameter(ns + "/trajectory", t);

goal_constraints_[joint_names_[i]] = g;
trajectory_constraints_[joint_names_[i]] = t;
}

stopped_velocity_tolerance_ = 0.5;
if (!nh_->has_parameter("constraints/stopped_velocity_tolerance"))
nh_->declare_parameter("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_);
nh_->get_parameter("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_);

// Additional settling (hold) time appended to the received trajectory to allow joints to converge
final_settle_time_ = 0.2; // seconds
if (!nh_->has_parameter("constraints/final_settle_time"))
nh_->declare_parameter("constraints/final_settle_time", final_settle_time_);
nh_->get_parameter("constraints/final_settle_time", final_settle_time_);

pub_controller_command_ = nh_->create_publisher<trajectory_msgs::msg::JointTrajectory>
("/"+ robot_name + "_driver/trajectory_controller/command", 1);
sub_controller_state_ = nh_->create_subscription<control_msgs::action::FollowJointTrajectory_Feedback>("/" + robot_name + "_driver/trajectory_controller/state",
1, std::bind(&JointTrajectoryActionController::controllerStateCB, this, std::placeholders::_1));
watchdog_timer_ = nh_->create_wall_timer(std::chrono::seconds(1), std::bind(&JointTrajectoryActionController::watchdog, this));

long unsigned int started_waiting_for_controller = nh_->get_clock()->now().seconds();
while (rclcpp::ok() && last_controller_state_ == empty_controller_state_)
{
if (started_waiting_for_controller != nh_->get_clock()->now().seconds() &&
nh_->get_clock()->now().seconds() > started_waiting_for_controller + 30)
{
RCLCPP_WARN(nh_->get_logger(), "Waited for the controller for 30 seconds, but it never showed up. Continue waiting the feedback of trajectory state on topic /trajectory_controller/state ...");
started_waiting_for_controller = nh_->get_clock()->now().seconds();
}
rclcpp::spin_some(nh_);
rclcpp::Rate(1).sleep();
}

RCLCPP_INFO(nh_->get_logger(), "Start Follow_Joint_Trajectory_Action server!");
RCLCPP_INFO(nh_->get_logger(), "Waiting for an plan execution (goal) from Moveit");
}

JointTrajectoryActionController::~JointTrajectoryActionController()
{
pub_controller_command_.reset();
sub_controller_state_.reset();
watchdog_timer_.reset();
}

static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
{
if (a.size() != b.size())
return false;

for (size_t i = 0; i < a.size(); ++i)
{
if (count(b.begin(), b.end(), a[i]) != 1)
return false;
}
for (size_t i = 0; i < b.size(); ++i)
{
if (count(a.begin(), a.end(), b[i]) != 1)
return false;
}

return true;
}

void JointTrajectoryActionController::watchdog()
{
if (!has_active_goal_)
return;

rclcpp::Time now = nh_->get_clock()->now();
bool should_abort = false;

if (!last_controller_state_.header.stamp.sec)
{
should_abort = true;
RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000, "Aborting goal because no controller state messages were received.");
}
else if ((now - last_controller_state_.header.stamp) > rclcpp::Duration(5,0))
{
should_abort = true;
RCLCPP_WARN(nh_->get_logger(), "Aborting goal because we haven't heard from the controller in %.3lf seconds",
(now.seconds() - last_controller_state_.header.stamp.sec));
}

if (!current_traj_.points.empty())
{
rclcpp::Duration traj_duration = current_traj_.points.back().time_from_start;
if (now - start_time_ > traj_duration + rclcpp::Duration(goal_time_constraint_, 0))
{
should_abort = true;
RCLCPP_WARN(nh_->get_logger(), "Aborting goal because trajectory time exceeded (%.3f s + %.3f s tolerance)",
traj_duration.seconds(), goal_time_constraint_);
}
}

if (should_abort && has_active_goal_)
{
trajectory_msgs::msg::JointTrajectory empty;
empty.joint_names = joint_names_;
// pub_controller_command_->publish(empty);
active_result_->error_code = control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL;
active_result_->error_string = "Aborted by watchdog";
active_goal_->abort(active_result_);
has_active_goal_ = false;
}
}

rclcpp_action::GoalResponse JointTrajectoryActionController::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FJTAS::Goal>goal)
{
RCLCPP_INFO(nh_->get_logger(), "Received goal request");
(void) uuid;
return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryActionController::handle_cancel(const std::shared_ptr<GoalHandleFJTAS> gh)
{
if (active_goal_ == gh)
{
// Marks the current goal as canceled.
active_goal_->canceled(active_result_);
has_active_goal_ = false;
}
RCLCPP_INFO(nh_->get_logger(), "Received request to cancel goal");
(void) gh;
return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryActionController::handle_accepted(const std::shared_ptr<GoalHandleFJTAS> goal_handle)
{
RCLCPP_INFO(nh_->get_logger(), "Joint_trajectory_action_server accepted goal!");
std::thread{std::bind(&JointTrajectoryActionController::goalCBFollow, this, std::placeholders::_1), goal_handle}.detach();
}

void JointTrajectoryActionController::goalCBFollow(std::shared_ptr<GoalHandleFJTAS> gh)
{
const auto goal = gh->get_goal();
RCLCPP_INFO(nh_->get_logger(), "Joint_trajectory_action_server received goal!");

active_result_ = std::make_shared<FJTAS::Result>();
active_result_->error_code = FJTAS::Result::SUCCESSFUL;
active_result_->error_string.clear();

// Ensures that the joints in the goal match the joints we are commanding.
if (!setsEqual(joint_names_, goal->trajectory.joint_names))
{
RCLCPP_ERROR(nh_->get_logger(), "Joints on incoming goal don't match our joints");
gh->abort(active_result_);
return;
}

// Cancel currently active goal if different
if (has_active_goal_ && active_goal_ != gh)
{
trajectory_msgs::msg::JointTrajectory empty;
empty.joint_names = joint_names_;
// pub_controller_command_->publish(empty);
active_goal_->canceled(active_result_);
has_active_goal_ = false;
}

// Activate new goal
active_goal_ = gh;
has_active_goal_ = true;
first_fb_ = true;
start_time_ = nh_->get_clock()->now();

// Sends the trajectory along to the controller
current_traj_ = goal->trajectory;
if (current_traj_.header.stamp.sec == 0 && current_traj_.header.stamp.nanosec == 0)
current_traj_.header.stamp = start_time_;

// Ensure trajectory has at least one point
if (current_traj_.points.empty())
{
RCLCPP_WARN(nh_->get_logger(), "Received empty trajectory - aborting goal");
gh->abort(active_result_);
has_active_goal_ = false;
return;
}

// Guarantee last point velocities vector size equals joints (fill zeros if missing)
trajectory_msgs::msg::JointTrajectoryPoint &last_pt = current_traj_.points.back();
if (last_pt.velocities.size() != joint_names_.size())
{
last_pt.velocities.resize(joint_names_.size(), 0.0);
}
// Force last point velocities to zero to command a stop exactly at final pose
bool any_last_nonzero = false;
for (double &v : last_pt.velocities)
{
if (fabs(v) > 1e-6) any_last_nonzero = true;
v = 0.0;
}

// Append an explicit settling hold point if settle time > 0
if (final_settle_time_ > 1e-6)
{
trajectory_msgs::msg::JointTrajectoryPoint settle_pt = last_pt;
// Keep positions same, zero velocities/accelerations/effort
settle_pt.velocities.assign(joint_names_.size(), 0.0);
settle_pt.accelerations.clear();
settle_pt.effort.clear();
// Add settle time to builtin_interfaces::msg::Duration manually
double total_sec = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9 + final_settle_time_;
settle_pt.time_from_start.sec = static_cast<int32_t>(floor(total_sec));
settle_pt.time_from_start.nanosec = static_cast<uint32_t>((total_sec - settle_pt.time_from_start.sec) * 1e9);
current_traj_.points.push_back(settle_pt);
RCLCPP_DEBUG(nh_->get_logger(), "Appended settling point (%.2fs after last) any_last_nonzero=%s", final_settle_time_, any_last_nonzero?"true":"false");
}

pub_controller_command_->publish(current_traj_);
RCLCPP_INFO(nh_->get_logger(), "Joint_trajectory_action_server published goal with %zu points (start stamp %.2f)!", current_traj_.points.size(), current_traj_.header.stamp.sec + current_traj_.header.stamp.nanosec * 1e-9);
}

void JointTrajectoryActionController::controllerStateCB(const control_msgs::action::FollowJointTrajectory_Feedback::SharedPtr msg)
{
RCLCPP_INFO_ONCE(nh_->get_logger(), "Joint_trajectory_action_server received feedback of trajectory state from topic: /trajectory_controller/state");

last_controller_state_ = *msg;

rclcpp::Time now = nh_->get_clock()->now();

if (!has_active_goal_)
return;
if (current_traj_.points.empty())
return;

//joint trajectory message seems to have header.stamp = 0
// using first feedback msg as guide for starting timestamp
if (first_fb_)
{
start_time_ = msg->header.stamp;
first_fb_ = false;
}

if (now - start_time_ < current_traj_.points[0].time_from_start)
{
return;
}

if (!setsEqual(joint_names_, msg->joint_names))
{
RCLCPP_ERROR_ONCE(nh_->get_logger(), "Joint names from the controller don't match our joint names.");
return;
}

int last = current_traj_.points.size() - 1;
rclcpp::Time end_time = start_time_ + current_traj_.points[last].time_from_start;

// Only decide success/failure at or after the nominal end time (no early success window)
if (now >= end_time)
{
// Checks that we have ended inside the goal constraints
bool inside_goal_constraints = true;
for (size_t i = 0; i < msg->joint_names.size() && inside_goal_constraints; ++i)
{
// computing error from goal pose
double err = angles::shortest_angular_distance(current_traj_.points[last].positions[i], msg->actual.positions[i]);
double abs_error = fabs(err);
double goal_constraint = goal_constraints_[msg->joint_names[i]];
if (goal_constraint >= 0.0 && abs_error > goal_constraint){
inside_goal_constraints = false;
RCLCPP_DEBUG(nh_->get_logger(), "Joint %s outside goal tolerance: error=%.6f tol=%.6f", msg->joint_names[i].c_str(), abs_error, goal_constraint);
}
// It's important to be stopped if that's desired.
if (!(msg->desired.velocities.empty()) && (fabs(msg->desired.velocities[i]) < 1e-6))
{
if (fabs(msg->actual.velocities[i]) > stopped_velocity_tolerance_)
inside_goal_constraints = false;
}
}

if (inside_goal_constraints)
{
if (has_active_goal_)
{
active_goal_->succeed(active_result_);
has_active_goal_ = false;
first_fb_ = true;
RCLCPP_INFO(nh_->get_logger(), "Joint trajectory goal succeeded.");
}
}
else if (now - end_time < rclcpp::Duration(goal_time_constraint_, 0))
{
// Within tolerance window after end_time: allow more time to settle before failing.
}
else
{
// Build detailed per-joint diagnostics prior to aborting
std::ostringstream oss;
oss.setf(std::ios::fixed); oss.precision(6);
oss << "Goal constraint violation details:";
for (size_t i = 0; i < msg->joint_names.size(); ++i)
{
double goal_pos = current_traj_.points[last].positions[i];
double actual_pos = msg->actual.positions[i];
double abs_error = fabs(actual_pos - goal_pos);
double tol = goal_constraints_[msg->joint_names[i]];
oss << "\n " << msg->joint_names[i]
<< ": goal=" << goal_pos
<< " actual=" << actual_pos
<< " error=" << abs_error
<< " tol=" << tol;
}
RCLCPP_WARN(nh_->get_logger(), "Aborting because we wound up outside the goal constraints. %s", oss.str().c_str());
active_goal_->abort(active_result_);
has_active_goal_ = false;
}
}
}

int main(int argc, char** argv)
{
rclcpp::init(argc, argv);
std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("follow_joint_trajectory_action_server");

// Retrieve the (non-option) argument:
std::string robot_name = "";
if ( (argc <= 1) || (argv[argc-1] == NULL) ) // there is NO input...
{
std::cerr << "No kinova_robot_name provided in the argument!" << std::endl;
return -1;
}
else
{
robot_name = argv[argc-1];
}
kinova::JointTrajectoryActionController jtac(node, robot_name);

rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}