#include <kinova_driver/kinova_joint_trajectory_controller.h>
#include <angles/angles.h>
// #include <ros/console.h>

using namespace kinova;

JointTrajectoryController::JointTrajectoryController(kinova::KinovaComm &kinova_comm, std::shared_ptr<rclcpp::Node> n):
kinova_comm_(kinova_comm),
nh_(n)
{
//RCLCPP_DEBUG_STREAM_ONCE(nh_->get_logger(), "Get in: " << __PRETTY_FUNCTION__);

// Default values (will be overridden by parameters if provided)
prefix_ = "j2n6s300";
robot_type = "j2n6s300";

// Prefer the launch-provided kinova_robotType to avoid hard-coded defaults
std::string kinova_robot_type_param;
if (!nh_->has_parameter("kinova_robotType"))
nh_->declare_parameter("kinova_robotType", kinova_robot_type_param);
nh_->get_parameter("kinova_robotType", kinova_robot_type_param);
if (!kinova_robot_type_param.empty()) {
robot_type = kinova_robot_type_param;
prefix_ = kinova_robot_type_param; // keep joint name prefix consistent with URDF (e.g., j2s6s300_joint_1)
}
if (!nh_->has_parameter("robot_name"))
nh_->declare_parameter("robot_name", prefix_);
if (!nh_->has_parameter("robot_type"))
nh_->declare_parameter("robot_type", robot_type);
nh_->get_parameter("robot_name", prefix_);
nh_->get_parameter("robot_type", robot_type);
std::string pubsub_prefix = robot_type + "_driver/";

number_joint_ =robot_type[3] - '0';

// // Display debug information in teminal
// if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
// ros::console::notifyLoggerLevelsChanged();
// }

sub_command_ = nh_->create_subscription<trajectory_msgs::msg::JointTrajectory>(pubsub_prefix+"trajectory_controller/command", 1,
std::bind(&JointTrajectoryController::commandCB, this, std::placeholders::_1));

pub_joint_feedback_ = nh_->create_publisher<control_msgs::action::FollowJointTrajectory_Feedback>(pubsub_prefix+"trajectory_controller/state", 1);
pub_joint_velocity_ = nh_->create_publisher<kinova_msgs::msg::JointVelocity>(pubsub_prefix+"in/joint_velocity", 1);

traj_frame_id_ = "root"; 
joint_names_.resize(number_joint_);
//std::cout << "joint names in feedback of trajectory state are: " << std::endl;
for (uint i = 0; i<joint_names_.size(); i++)
{
joint_names_[i] = prefix_ + "_joint_" + std::to_string(i+1);
std::cout << joint_names_[i] << " ";
}
std::cout << std::endl;

timer_pub_joint_vel_ = nh_->create_wall_timer(std::chrono::milliseconds(10), std::bind(&JointTrajectoryController::pub_joint_vel, this));
flag_timer_pub_joint_vel_ = false;
terminate_thread_ = false;

thread_update_state_ = std::make_shared<std::thread>(std::bind(&JointTrajectoryController::update_state, this));
thread_update_state_->detach();

traj_feedback_msg_.joint_names.resize(joint_names_.size());
traj_feedback_msg_.desired.positions.resize(joint_names_.size());
traj_feedback_msg_.desired.velocities.resize(joint_names_.size());
traj_feedback_msg_.actual.positions.resize(joint_names_.size());
traj_feedback_msg_.actual.velocities.resize(joint_names_.size());
traj_feedback_msg_.error.positions.resize(joint_names_.size());
traj_feedback_msg_.error.velocities.resize(joint_names_.size());
traj_feedback_msg_.joint_names = joint_names_;

// counter in the timer to publish joint velocity command: pub_joint_vel()
traj_command_points_index_ = 0;

for(int i=0; i<num_possible_joints; i++) current_velocity_command[i] = 0;
//RCLCPP_DEBUG_STREAM_ONCE(nh_->get_logger(), "Get out: " << __PRETTY_FUNCTION__);

// Initialize P control parameters
last_phase_kp_ = 3.0; // Higher gain for faster convergence
last_phase_tolerance_rad_ = 0.01; // ~0.57 degrees
last_phase_min_speed_deg_s_ = 0.1; // Higher min speed to avoid stalling

last_motion_target_positions_.resize(number_joint_);
settle_end_time_sec_ = 0.0;
last_reported_index_ = -1;
}

JointTrajectoryController::~JointTrajectoryController()
{
//RCLCPP_DEBUG_STREAM_ONCE(nh_->get_logger(), "Get in: " << __PRETTY_FUNCTION__);
RCLCPP_WARN(nh_->get_logger(), "destruction entered!");
{
boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
terminate_thread_ = true;
}

sub_command_.reset();
pub_joint_feedback_.reset();
pub_joint_velocity_.reset();

timer_pub_joint_vel_->cancel();
timer_pub_joint_vel_.reset();
thread_update_state_->join();

//RCLCPP_DEBUG_STREAM_ONCE(nh_->get_logger(), "Get out: " << __PRETTY_FUNCTION__);
}


void JointTrajectoryController::commandCB(const trajectory_msgs::msg::JointTrajectory::SharedPtr traj_msg)
{
//RCLCPP_DEBUG_STREAM_ONCE(nh_->get_logger(), "Get in: " << __PRETTY_FUNCTION__);

bool command_abort = false;

// // if receive new command, clear all trajectory and stop api
// kinova_comm_.stopAPI();
// if(!kinova_comm_.isStopped())
// {
// ros::Duration(0.01).sleep();
// }
// kinova_comm_.eraseAllTrajectories();

// kinova_comm_.startAPI();
// if(kinova_comm_.isStopped())
// {
// ros::Duration(0.01).sleep();
// }

// Guard against null or empty trajectories (can happen on abort/flush)
if (!traj_msg)
{
RCLCPP_WARN(nh_->get_logger(), "Received null JointTrajectory message");
return;
}
traj_command_points_ = traj_msg->points;
RCLCPP_INFO_STREAM(nh_->get_logger(), "Trajectory controller Receive trajectory with points number: " << traj_command_points_.size());

if (traj_command_points_.empty())
{
// Stop any ongoing motion
for (int i = 0; i < num_possible_joints; ++i) current_velocity_command[i] = 0.0f;
kinova_msgs::msg::JointVelocity stop_msg; // all zeros by default
pub_joint_velocity_->publish(stop_msg);
flag_timer_pub_joint_vel_ = false;
traj_command_points_index_ = 0;
last_motion_target_positions_.clear();
settle_end_time_sec_ = 0.0;
last_reported_index_ = -1;
RCLCPP_WARN(nh_->get_logger(), "Received empty trajectory; stopping and clearing state.");
return;
}

// Map the index in joint_names and the msg
std::vector<int> lookup(number_joint_, -1);

for (size_t j = 0; j<number_joint_; j++)
{
for (size_t k = 0; k<traj_msg->joint_names.size(); k++)
if(traj_msg->joint_names[k] == joint_names_[j]) // find joint_j in msg;
{
lookup[j] = k;
break;
}

if (lookup[j] == -1) // if joint_j not found in msg;
{
std::string error_msg = "Joint name : " + joint_names_[j] + " not found in the msg.";
RCLCPP_ERROR(nh_->get_logger(), "%s", error_msg.c_str());
command_abort = true;
return;
}
}

// check msg validation
for (size_t j = 0; j<traj_command_points_.size(); j++)
{
// position should not be empty
if (traj_command_points_[j].positions.empty()) // find joint_j in msg;
{
RCLCPP_ERROR_STREAM(nh_->get_logger(), "Positions in trajectory command cannot be empty at point: " << j);
command_abort = true;
break;
}
// position size match
if (traj_command_points_[j].positions.size() != number_joint_)
{
RCLCPP_ERROR_STREAM(nh_->get_logger(), "Positions at point " << j << " has size " << traj_command_points_[j].positions.size() << " in trajectory command, which does not match joint number! ");
command_abort = true;
break;
}

// if velocity provided, size match
if (!traj_command_points_[j].velocities.empty() && traj_command_points_[j].velocities.size() != number_joint_)
{
RCLCPP_ERROR_STREAM(nh_->get_logger(), "Velocities at point " << j << " has size " << traj_command_points_[j].velocities.size() << " in trajectory command, which does not match joint number! ");
command_abort = true;
break;
}
// All points must have monotonically increasing time_from_start
if (j > 0)
{
const double t_prev = traj_command_points_[j-1].time_from_start.sec + traj_command_points_[j-1].time_from_start.nanosec * 1e-9;
const double t_cur = traj_command_points_[j].time_from_start.sec + traj_command_points_[j].time_from_start.nanosec * 1e-9;
if (t_cur < t_prev - 1e-12)
{
RCLCPP_ERROR_STREAM(nh_->get_logger(), "time_from_start must be non-decreasing (point " << j-1 << " -> " << j << ")");
command_abort = true;
break;
}
}
}

if(command_abort)
return;

// Ensure velocities exist for each point; if missing, compute via finite diff from positions/time_from_start
for (size_t i = 0; i < traj_command_points_.size(); ++i)
{
auto &pt = traj_command_points_[i];
if (pt.velocities.size() != number_joint_)
{
pt.velocities.assign(number_joint_, 0.0);
if (i > 0)
{
const auto &prev = traj_command_points_[i-1];
const double t_prev = prev.time_from_start.sec + prev.time_from_start.nanosec * 1e-9;
const double t_cur = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
const double dt = std::max(1e-6, t_cur - t_prev);
for (size_t j = 0; j < number_joint_; ++j)
{
const double dq = pt.positions[j] - prev.positions[j];
pt.velocities[j] = dq / dt; // rad/s
}
}
}
}

// store angle velocity command sent to robot (deg/s)
kinova_angle_command_.resize(traj_command_points_.size());
for (size_t i = 0; i<traj_command_points_.size(); i++)
{
kinova_angle_command_[i].InitStruct(); // initial joint velocity to zeros.

kinova_angle_command_[i].Actuator1 = traj_command_points_[i].velocities[0] * 180.0 / M_PI;
kinova_angle_command_[i].Actuator2 = traj_command_points_[i].velocities[1] * 180.0 / M_PI;
kinova_angle_command_[i].Actuator3 = traj_command_points_[i].velocities[2] * 180.0 / M_PI;
kinova_angle_command_[i].Actuator4 = traj_command_points_[i].velocities[3] * 180.0 / M_PI;
if (number_joint_ >= 5) kinova_angle_command_[i].Actuator5 = traj_command_points_[i].velocities[4] * 180.0 / M_PI;
if (number_joint_ >= 6) kinova_angle_command_[i].Actuator6 = traj_command_points_[i].velocities[5] * 180.0 / M_PI;
if (number_joint_ >= 7) kinova_angle_command_[i].Actuator7 = traj_command_points_[i].velocities[6] * 180.0 / M_PI;
}

std::vector<double> durations(traj_command_points_.size(), 0.0); // computed by time_from_start
double trajectory_duration = traj_command_points_[0].time_from_start.sec + traj_command_points_[0].time_from_start.nanosec / 1000000000;

durations[0] = trajectory_duration;
// RCLCPP_DEBUG_STREAM(nh_->get_logger(), "durationsn 0 is: " << durations[0]);

for (int i = 1; i<traj_command_points_.size(); i++)
{
durations[i] = (traj_command_points_[i].time_from_start.sec + traj_command_points_[i].time_from_start.nanosec / 1000000000) - (traj_command_points_[i-1].time_from_start.sec + traj_command_points_[i-1].time_from_start.nanosec / 1000000000);
trajectory_duration += durations[i];
// RCLCPP_DEBUG_STREAM(nh_->get_logger(), "durations " << i << " is: " << durations[i]);
}

// reset indices/flags and start timer to publish joint velocity command
traj_command_points_index_ = 0;
for (int i = 0; i < num_possible_joints; ++i) { current_velocity_command[i] = 0.0f; }
last_motion_target_positions_.assign(number_joint_, 0.0);
if (!traj_command_points_.empty())
{
// Detect settle point (last point velocities zero and possibly duplicate positions)
size_t last_idx = traj_command_points_.size() - 1;
size_t motion_last_idx = last_idx;
if (last_idx > 0)
{
bool same_positions = true;
for (size_t j = 0; j < number_joint_; ++j)
{
if (fabs(traj_command_points_[last_idx].positions[j] - traj_command_points_[last_idx - 1].positions[j]) > 1e-9)
{ same_positions = false; break; }
}
if (same_positions)
motion_last_idx = last_idx - 1;
}
for (size_t j = 0; j < number_joint_; ++j)
last_motion_target_positions_[j] = traj_command_points_[motion_last_idx].positions[j];
settle_end_time_sec_ = traj_command_points_.back().time_from_start.sec + traj_command_points_.back().time_from_start.nanosec * 1e-9;
}
time_pub_joint_vel_ = nh_->get_clock()->now();
flag_timer_pub_joint_vel_ = true;
last_reported_index_ = -1;

//RCLCPP_DEBUG_STREAM_ONCE(nh_->get_logger(), "Get out: " << __PRETTY_FUNCTION__);
}

void JointTrajectoryController::pub_joint_vel()
{
if (!flag_timer_pub_joint_vel_) return;

// send out each velocity command with corresponding duration delay.

kinova_msgs::msg::JointVelocity joint_velocity_msg;

if (traj_command_points_index_ < kinova_angle_command_.size() && rclcpp::ok())
{
const rclcpp::Duration current_time_from_start = nh_->get_clock()->now() - time_pub_joint_vel_;
// Detect if the trajectory has an appended settle point (same positions as previous)
const size_t last_idx = (kinova_angle_command_.empty() ? 0 : kinova_angle_command_.size() - 1);
size_t motion_last_idx = last_idx;
if (traj_command_points_.size() >= 2)
{
bool same_positions = true;
for (size_t j = 0; j < number_joint_; ++j)
{
if (fabs(traj_command_points_[last_idx].positions[j] - traj_command_points_[last_idx - 1].positions[j]) > 1e-9)
{
same_positions = false; break;
}
}
if (same_positions && last_idx > 0)
motion_last_idx = last_idx - 1; // treat previous point as last motion segment
}
// No per-joint remaining time: keep commanding motion_last_idx velocities until settle_end

// Select the velocity to send right now
if (traj_command_points_index_ < static_cast<int>(kinova_angle_command_.size()))
{
// Start from the nominal command of this segment;
// if we are on/after the final motion segment, keep using its velocity
const size_t base_idx = static_cast<size_t>(traj_command_points_index_);

current_velocity_command[0] = kinova_angle_command_[base_idx].Actuator1;
current_velocity_command[1] = kinova_angle_command_[base_idx].Actuator2;
current_velocity_command[2] = kinova_angle_command_[base_idx].Actuator3;
current_velocity_command[3] = kinova_angle_command_[base_idx].Actuator4;
if (number_joint_ >=5) current_velocity_command[4] = kinova_angle_command_[base_idx].Actuator5;
if (number_joint_ >=6) current_velocity_command[5] = kinova_angle_command_[base_idx].Actuator6;
if (number_joint_ >=7) current_velocity_command[6] = kinova_angle_command_[base_idx].Actuator7;
}

// After the nominal settle time, ensure we converge to the last motion target by applying a small proportional velocity until within tolerance
const double t_now_sec = (nh_->get_clock()->now() - time_pub_joint_vel_).nanoseconds() * 1e-9;
if (t_now_sec >= settle_end_time_sec_ - 1e-6 && !last_motion_target_positions_.empty())
{
// Build P controller on joint error
for (size_t i = 0; i < number_joint_; ++i)
{
// Use shortest angular distance for rotary joints
double err = angles::shortest_angular_distance(traj_feedback_msg_.actual.positions[i], last_motion_target_positions_[i]);
if (fabs(err) <= last_phase_tolerance_rad_)
{
current_velocity_command[i] = 0.0f;
}
else
{
double v_rad_s = last_phase_kp_ * err; // rad/s
double v_deg_s = v_rad_s * 180.0 / M_PI;
// enforce a minimum non-zero speed so we don't stall
if (fabs(v_deg_s) < last_phase_min_speed_deg_s_)
v_deg_s = (v_deg_s >= 0 ? last_phase_min_speed_deg_s_ : -last_phase_min_speed_deg_s_);
current_velocity_command[i] = static_cast<float>(v_deg_s);
}
}
}

// Fill outgoing message from current_velocity_command
joint_velocity_msg.joint1 = current_velocity_command[0];
joint_velocity_msg.joint2 = (number_joint_ > 1 ? current_velocity_command[1] : 0.0f);
joint_velocity_msg.joint3 = (number_joint_ > 2 ? current_velocity_command[2] : 0.0f);
joint_velocity_msg.joint4 = (number_joint_ > 3 ? current_velocity_command[3] : 0.0f);
joint_velocity_msg.joint5 = (number_joint_ > 4 ? current_velocity_command[4] : 0.0f);
joint_velocity_msg.joint6 = (number_joint_ > 5 ? current_velocity_command[5] : 0.0f);
joint_velocity_msg.joint7 = (number_joint_ > 6 ? current_velocity_command[6] : 0.0f);

// In debug: compare values with topic: follow_joint_trajectory/goal, command
// RCLCPP_DEBUG_STREAM_ONCE(nh_->get_logger(), std::endl <<" joint_velocity_msg.joint1: " << joint_velocity_msg.joint1 * M_PI/180 <<
// std::endl <<" joint_velocity_msg.joint2: " << joint_velocity_msg.joint2 * M_PI/180 <<
// std::endl <<" joint_velocity_msg.joint3: " << joint_velocity_msg.joint3 * M_PI/180 <<
// std::endl <<" joint_velocity_msg.joint4: " << joint_velocity_msg.joint4 * M_PI/180 <<
// std::endl <<" joint_velocity_msg.joint5: " << joint_velocity_msg.joint5 * M_PI/180 <<
// std::endl <<" joint_velocity_msg.joint6: " << joint_velocity_msg.joint6 * M_PI/180 );

pub_joint_velocity_->publish(joint_velocity_msg);

if(current_time_from_start >= traj_command_points_[traj_command_points_index_].time_from_start)
{
if (traj_command_points_index_ != last_reported_index_)
{
RCLCPP_INFO_STREAM(nh_->get_logger(), "Moved to point " << traj_command_points_index_);
last_reported_index_ = traj_command_points_index_;
}
// Advance index but keep within bounds
if (traj_command_points_index_ + 1 < static_cast<int>(kinova_angle_command_.size()))
{
++traj_command_points_index_;
}
else
{
// We are on the last point already; keep index but rely on remaining_motion_time to taper velocities
}
}
}
else // if come accross all the points, then stop timer.
{
joint_velocity_msg.joint1 = 0;
joint_velocity_msg.joint2 = 0;
joint_velocity_msg.joint3 = 0;
joint_velocity_msg.joint4 = 0;
joint_velocity_msg.joint5 = 0;
joint_velocity_msg.joint6 = 0;
joint_velocity_msg.joint7 = 0;

traj_command_points_.clear();
for(int i=0; i<num_possible_joints; i++) current_velocity_command[i] = 0;

traj_command_points_index_ = 0;
flag_timer_pub_joint_vel_ = false;
last_reported_index_ = -1;
}
}

void JointTrajectoryController::update_state()
{
// RCLCPP_DEBUG_STREAM_ONCE(nh_->get_logger(), "Get in: " << __PRETTY_FUNCTION__);

previous_pub_ = nh_->get_clock()->now();
while (rclcpp::ok())
{
// check if terminate command is sent from main thread
{
boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
if (terminate_thread_)
{
break;
}
}

traj_feedback_msg_.header.frame_id = traj_frame_id_;
traj_feedback_msg_.header.stamp = nh_->get_clock()->now();
KinovaAngles current_joint_angles;
KinovaAngles current_joint_velocity;
AngularPosition current_joint_command;

kinova_comm_.getAngularCommand(current_joint_command);
kinova_comm_.getJointAngles(current_joint_angles);
kinova_comm_.getJointVelocities(current_joint_velocity);

traj_feedback_msg_.desired.positions[0] = current_joint_command.Actuators.Actuator1 *M_PI/180;
traj_feedback_msg_.desired.positions[1] = current_joint_command.Actuators.Actuator2 *M_PI/180;
traj_feedback_msg_.desired.positions[2] = current_joint_command.Actuators.Actuator3 *M_PI/180;
traj_feedback_msg_.desired.positions[3] = current_joint_command.Actuators.Actuator4 *M_PI/180;
if (number_joint_>=6)
{
traj_feedback_msg_.desired.positions[4] = current_joint_command.Actuators.Actuator5 *M_PI/180;
traj_feedback_msg_.desired.positions[5] = current_joint_command.Actuators.Actuator6 *M_PI/180;
if (number_joint_==7)
traj_feedback_msg_.desired.positions[6] = current_joint_command.Actuators.Actuator7 *M_PI/180;
}

traj_feedback_msg_.actual.positions[0] = current_joint_angles.Actuator1 *M_PI/180;
traj_feedback_msg_.actual.positions[1] = current_joint_angles.Actuator2 *M_PI/180;
traj_feedback_msg_.actual.positions[2] = current_joint_angles.Actuator3 *M_PI/180;
traj_feedback_msg_.actual.positions[3] = current_joint_angles.Actuator4 *M_PI/180;
if (number_joint_>=6)
{
traj_feedback_msg_.actual.positions[4] = current_joint_angles.Actuator5 *M_PI/180;
traj_feedback_msg_.actual.positions[5] = current_joint_angles.Actuator6 *M_PI/180;
if (number_joint_==7)
traj_feedback_msg_.actual.positions[6] = current_joint_angles.Actuator7 *M_PI/180;
}

traj_feedback_msg_.actual.velocities[0] = current_joint_velocity.Actuator1 *M_PI/180;
traj_feedback_msg_.actual.velocities[1] = current_joint_velocity.Actuator2 *M_PI/180;
traj_feedback_msg_.actual.velocities[2] = current_joint_velocity.Actuator3 *M_PI/180;
traj_feedback_msg_.actual.velocities[3] = current_joint_velocity.Actuator4 *M_PI/180;
if (number_joint_>=6)
{
traj_feedback_msg_.actual.velocities[4] = current_joint_velocity.Actuator5 *M_PI/180;
traj_feedback_msg_.actual.velocities[5] = current_joint_velocity.Actuator6 *M_PI/180;
if (number_joint_==7)
traj_feedback_msg_.actual.velocities[6] = current_joint_velocity.Actuator7 *M_PI/180;
}

for (size_t j = 0; j<joint_names_.size(); j++)
{
traj_feedback_msg_.error.positions[j] = angles::shortest_angular_distance(traj_feedback_msg_.desired.positions[j], traj_feedback_msg_.actual.positions[j]);
}

// RCLCPP_WARN_STREAM(nh_->get_logger(), "I'm publishing after second: " << (nh_->get_clock()->now().seconds() - previous_pub_.seconds()));
pub_joint_feedback_->publish(traj_feedback_msg_);
previous_pub_ = nh_->get_clock()->now();
rclcpp::Rate(10).sleep();
}
//RCLCPP_DEBUG_STREAM_ONCE(nh_->get_logger(), "Get out: " << __PRETTY_FUNCTION__);
}
