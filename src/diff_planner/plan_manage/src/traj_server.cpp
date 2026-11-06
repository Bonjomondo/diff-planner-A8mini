#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/RCIn.h>
#include <std_srvs/Trigger.h>
#include <algorithm>
#include <cmath>

using namespace Eigen;

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

bool receive_traj_ = false;
bool landing_requested_ = false;
bool awaiting_takeoff_ = false;
bool awaiting_goal_ = false;
ros::Time flight_epoch_(0), goal_epoch_(0);
mavros_msgs::State fcu_state_;
uint8_t landed_state_ = mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED;
ros::WallTime fcu_received_, extended_received_, rc_received_;
bool rc_down_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;
double yaw_rate_max_, yaw_acceleration_max_;
double odom_yaw_ = 0.0, yaw_target_ = 0.0;
bool yaw_initialized_ = false, yaw_output_active_ = false;
ros::Time odom_yaw_received_(0), yaw_update_time_(0);
double yaw_custom_;
bool receive_yaw_ = false;
ros::Time receive_yaw_time_(0);

double wrapYaw(double yaw)
{
  return std::remainder(yaw, 2.0 * M_PI);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  const auto &q = msg->pose.pose.orientation;
  Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
  if (!orientation.coeffs().allFinite() || !std::isfinite(orientation.norm()) ||
      orientation.norm() < 1e-6)
    return;
  orientation.normalize();
  const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
  odom_yaw_ = atan2(rotation(1, 0), rotation(0, 0));
  odom_yaw_received_ = ros::Time::now();
}

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void stopCommandOutput(const char *reason)
{
  if (!landing_requested_)
  {
    ROS_WARN("[traj_server] %s received: stop publishing PositionCommand and clear "
             "the active trajectory.", reason);
  }
  landing_requested_ = true;
  receive_traj_ = false;
  receive_yaw_ = false;
  traj_.reset();
  yaw_output_active_ = false;
  last_yawdot_ = 0.0;
}

bool receiptFresh(const ros::WallTime &stamp, double timeout)
{
  const double age = (ros::WallTime::now() - stamp).toSec();
  return !stamp.isZero() && age >= 0.0 && age <= timeout;
}

void fcuCallback(const mavros_msgs::StateConstPtr &msg)
{
  fcu_state_ = *msg;
  fcu_received_ = ros::WallTime::now();
}

void extendedCallback(const mavros_msgs::ExtendedStateConstPtr &msg)
{
  landed_state_ = msg->landed_state;
  extended_received_ = ros::WallTime::now();
}

void rcCallback(const mavros_msgs::RCInConstPtr &msg)
{
  rc_received_ = ros::WallTime::now();
  rc_down_ = msg->channels.size() > 7 &&
             std::all_of(msg->channels.begin(), msg->channels.begin() + 8,
                 [](uint16_t pwm) { return pwm >= 900 && pwm <= 2100; }) &&
             msg->channels[7] > 1899 && msg->channels[7] < 2099;
}

bool resetTrajectory(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response)
{
  if (!receiptFresh(fcu_received_, 2.5) || !receiptFresh(extended_received_, 2.5) ||
      !fcu_state_.connected || fcu_state_.armed ||
      landed_state_ != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND ||
      !receiptFresh(rc_received_, 0.5) || !rc_down_)
  {
    response.message = "Trajectory reset requires fresh connected FCU, ON_GROUND, disarmed and fresh RC8 DOWN";
    return true;
  }
  receive_traj_ = receive_yaw_ = false;
  traj_.reset();
  heartbeat_time_ = receive_yaw_time_ = start_time_ = ros::Time(0);
  traj_duration_ = 0.0;
  traj_id_ = 0;
  last_yaw_ = last_yawdot_ = yaw_custom_ = yaw_target_ = 0.0;
  yaw_initialized_ = yaw_output_active_ = false;
  yaw_update_time_ = ros::Time(0);
  last_pos_.setZero();
  flight_epoch_ = ros::Time::now();
  goal_epoch_ = ros::Time(0);
  awaiting_takeoff_ = awaiting_goal_ = true;
  landing_requested_ = false;
  response.success = true;
  response.message = "Trajectory cleared; waiting for separate TAKEOFF and a new world goal";
  return true;
}

void newGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (landing_requested_ || awaiting_takeoff_ || !awaiting_goal_ ||
      msg->header.frame_id != "world" || msg->header.stamp <= flight_epoch_ ||
      msg->header.stamp > ros::Time::now() ||
      !std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y) ||
      !std::isfinite(msg->pose.position.z))
    return;
  goal_epoch_ = msg->header.stamp;
  awaiting_goal_ = false;
}

void planningStopCallback(const std_msgs::EmptyConstPtr &)
{
  stopCommandOutput("planning stop");
}

void takeoffLandCallback(const quadrotor_msgs::TakeoffLandConstPtr &msg)
{
  if (msg->takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
  {
    stopCommandOutput("LAND");
    return;
  }

  if (msg->takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF)
  {
    if (landing_requested_)
    {
      ROS_WARN("[traj_server] TAKEOFF cannot clear the stop/landing latch. "
               "After ground + disarmed + RC8 DOWN, call /mission/reset for a new flight.");
      return;
    }
    receive_traj_ = false;
    yaw_initialized_ = yaw_output_active_ = false;
    last_yawdot_ = 0.0;
    if (awaiting_takeoff_)
    {
      flight_epoch_ = ros::Time::now();
      awaiting_takeoff_ = false;
    }
  }
}

void yawCallback(const quadrotor_msgs::PositionCommandPtr msg)
{
  if (landing_requested_ || awaiting_takeoff_ || awaiting_goal_)
    return;

  if (!std::isfinite(msg->yaw) || msg->yaw <= -100.0)
  {
    receive_yaw_ = false;
    return;
  }
  receive_yaw_ = true;
  receive_yaw_time_ = ros::Time::now();
  yaw_custom_ = wrapYaw(msg->yaw);
  // std::cout << "Received yaw:  " << yaw_custom_ << std::endl;
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (landing_requested_ || awaiting_takeoff_ || awaiting_goal_)
  {
    ROS_WARN_THROTTLE(2.0, "[traj_server] Ignore trajectory while LAND is active.");
    return;
  }
  // Exclude queued trajectories from the previous flight, even after reset/TAKEOFF.
  if (!goal_epoch_.isZero() && msg->start_time < goal_epoch_)
    return;

  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  const size_t coefficient_count = msg->duration.size() * 6;
  const auto finite = [](double value) { return std::isfinite(value); };
  if (msg->duration.empty() || coefficient_count != msg->coef_x.size() ||
      coefficient_count != msg->coef_y.size() || coefficient_count != msg->coef_z.size() ||
      !std::all_of(msg->duration.begin(), msg->duration.end(),
                   [](double value) { return std::isfinite(value) && value > 0.0; }) ||
      !std::all_of(msg->coef_x.begin(), msg->coef_x.end(), finite) ||
      !std::all_of(msg->coef_y.begin(), msg->coef_y.end(), finite) ||
      !std::all_of(msg->coef_z.begin(), msg->coef_z.end(), finite))
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, const Eigen::Vector3d &pos,
                                      const Eigen::Vector3d &vel, double dt)
{
  // At low speed the short preview can point sideways because of hover drift.
  // Use the remaining route direction until there is a reliable moving preview.
  const Eigen::Vector3d endpoint = traj_->getPos(traj_duration_);
  Eigen::Vector3d dir = endpoint - pos;
  const bool moving = vel.head<2>().norm() >= 0.2;
  if (moving)
    dir = traj_->getPos(std::min(t_cur + time_forward_, traj_duration_)) - pos;

  // Ignore vertical displacement and small end-of-segment corrections.
  yaw_target_ = dir.head<2>().norm() > (moving ? 0.1 : 0.3)
                    ? atan2(dir.y(), dir.x())
                    : last_yaw_;

  const double override_age = (ros::Time::now() - receive_yaw_time_).toSec();
  if (receive_yaw_ && override_age >= 0.0 && override_age < 0.5)
    yaw_target_ = yaw_custom_;
  else
    receive_yaw_ = false;

  double error = wrapYaw(yaw_target_ - last_yaw_);
  // Keep the selected turn direction when a near-180-degree target jitters
  // across the wrap boundary during a reversal.
  if (fabs(error) > M_PI - 0.1 && fabs(last_yawdot_) > 0.01)
    error = std::copysign(fabs(error), last_yawdot_);

  // A critically damped heading response with bounded angular acceleration.
  // Integrate the same rate that is reported in PositionCommand.yaw_dot.
  constexpr double response = 3.0;
  const double acceleration = std::max(-yaw_acceleration_max_,
      std::min(response * response * error - 2.0 * response * last_yawdot_,
               yaw_acceleration_max_));
  const double rate = std::max(-yaw_rate_max_,
      std::min(last_yawdot_ + acceleration * dt, yaw_rate_max_));
  last_yaw_ = wrapYaw(last_yaw_ + 0.5 * (last_yawdot_ + rate) * dt);
  last_yawdot_ = rate;
  return {last_yaw_, last_yawdot_};
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

void cmdCallback(const ros::TimerEvent &e)
{
  const ros::Time time_now = ros::Time::now();
  const double elapsed = (time_now - yaw_update_time_).toSec();
  yaw_update_time_ = time_now;
  const double dt = std::max(0.0, std::min(elapsed, 0.05));
  const bool resume_yaw = !yaw_output_active_ || elapsed > 0.1;
  yaw_output_active_ = false;
  if (elapsed <= 0.0)
  {
    last_yawdot_ = 0.0;
    return;
  }

  // LAND has priority over every queued or newly generated trajectory. px4ctrl only
  // leaves CMD_CTRL after PositionCommand has been silent for msg_timeout/cmd.
  if (landing_requested_)
    return;

  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_)
    return;

  if ((time_now - heartbeat_time_).toSec() > 0.5)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");

    receive_traj_ = false;
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
    return;
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
  const ros::Time time_last = time_now - ros::Duration(dt);
  static bool finished = false;
#endif
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);

    if (resume_yaw)
    {
      const double odom_age = (time_now - odom_yaw_received_).toSec();
      if (!odom_yaw_received_.isZero() && odom_age >= 0.0 && odom_age < 0.5)
      {
        last_yaw_ = odom_yaw_;
        yaw_initialized_ = true;
      }
      if (!yaw_initialized_)
      {
        ROS_WARN_THROTTLE(1.0, "[traj_server] Waiting for odometry to initialize yaw.");
        return;
      }
      last_yawdot_ = 0.0;
      yaw_target_ = last_yaw_;
    }
    yaw_yawdot = calculate_yaw(t_cur, pos, vel, resume_yaw ? 0.01 : dt);
    yaw_output_active_ = true;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
    if (slowly_flip_yaw_target_ > M_PI)
      slowly_flip_yaw_target_ -= 2 * M_PI;
    if (slowly_flip_yaw_target_ < -M_PI)
      slowly_flip_yaw_target_ += 2 * M_PI;
    constexpr double CENTER[2] = {0.0, 0.0};
    slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

    // publish
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
#if FLIP_YAW_AT_END or TURN_YAW_TO_CENTER_AT_END
    finished = false;
#endif
  }

#if FLIP_YAW_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    if (slowly_flip_yaw_target_ > 0)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }
    else
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_flip_yaw_target_)
      {
        finished = true;
      }
    }

    yaw_yawdot.first = last_yaw_;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif

#if TURN_YAW_TO_CENTER_AT_END
  else if (t_cur >= traj_duration_)
  {
    if (finished)
      return;

    /* hover when finished traj_ */
    pos = traj_->getPos(traj_duration_);
    vel.setZero();
    acc.setZero();
    jer.setZero();

    double d_yaw = last_yaw_ - slowly_turn_to_center_target_;
    if (d_yaw >= M_PI)
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ > M_PI)
        last_yaw_ -= 2 * M_PI;
    }
    else if (d_yaw <= -M_PI)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ < -M_PI)
        last_yaw_ += 2 * M_PI;
    }
    else if (d_yaw >= 0)
    {
      last_yaw_ -= (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = -M_PI / 2;
      if (last_yaw_ <= slowly_turn_to_center_target_)
        finished = true;
    }
    else
    {
      last_yaw_ += (time_now - time_last).toSec() * M_PI / 2;
      yaw_yawdot.second = M_PI / 2;
      if (last_yaw_ >= slowly_turn_to_center_target_)
        finished = true;
    }

    yaw_yawdot.first = last_yaw_;

    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber odom_sub = nh.subscribe("odometry", 1, odomCallback);
  ros::Subscriber yaw_sub = nh.subscribe("/planning/yaw", 10, yawCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  ros::Subscriber planning_stop_sub =
      nh.subscribe("/planning/stop", 10, planningStopCallback);
  ros::Subscriber takeoff_land_sub =
      nh.subscribe("/px4ctrl/takeoff_land", 10, takeoffLandCallback);
  ros::Subscriber goal_sub = nh.subscribe("/goal", 1, newGoalCallback);
  ros::Subscriber fcu_sub = nh.subscribe("/mavros/state", 1, fcuCallback);
  ros::Subscriber extended_sub = nh.subscribe("/mavros/extended_state", 1, extendedCallback);
  ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 1, rcCallback);
  ros::ServiceServer reset_srv = nh.advertiseService("/planning/reset_trajectory", resetTrajectory);
  
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, 1.0);
  nh.param("traj_server/yaw_rate_max", yaw_rate_max_, M_PI / 4.0);
  nh.param("traj_server/yaw_acceleration_max", yaw_acceleration_max_, M_PI / 2.0);
  if (!std::isfinite(time_forward_) || time_forward_ <= 0.0 ||
      !std::isfinite(yaw_rate_max_) || yaw_rate_max_ <= 0.0 ||
      !std::isfinite(yaw_acceleration_max_) || yaw_acceleration_max_ <= 0.0)
  {
    ROS_FATAL("[traj_server] Yaw preview, rate and acceleration limits must be finite and positive.");
    return 1;
  }
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}
