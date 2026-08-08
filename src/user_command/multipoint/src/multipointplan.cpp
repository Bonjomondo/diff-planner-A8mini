#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/RCIn.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt32.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

enum MissionState
{
    FLYING,
    HOVERING,
    WAITING_GIMBAL,
    FINISHED
};

enum GimbalMode
{
    GIMBAL_ANGLE = 0,
    GIMBAL_RANGE = 1
};

enum RC_EIGHT_STATE
{
    RC_EIGHT_UP = 999,
    RC_EIGHT_MIDDLE = 1499,
    RC_EIGHT_DOWN = 1999
};

struct Waypoint
{
    uint32_t id;
    double x;
    double y;
    double z;
    double hover_sec;
    double gimbal_yaw_deg;
    double gimbal_yaw_min_deg;
    double gimbal_yaw_max_deg;
    double gimbal_pitch_deg;
    double gimbal_settle_sec;
    GimbalMode gimbal_mode;
    bool run_gimbal;
};

class MultipointPlanner
{
public:
    MultipointPlanner()
        : nh_(),
          pnh_("~"),
          state_(FINISHED),
          current_index_(0),
          odom_received_(false),
          mission_uses_gimbal_(true),
          rc_eight_pre_(RC_EIGHT_DOWN),
          rc_initialized_(false),
          landing_latched_(false)
    {
        pnh_.param<std::string>("yaml_path", yaml_path_, std::string());
        pnh_.param<std::string>("goal_frame_id", goal_frame_id_, "world");
        pnh_.param<std::string>("mission_csv_path", mission_csv_path_,
                                "/tmp/a8mini_mission_timestamps.csv");
        pnh_.param("position_tolerance", position_tolerance_, 0.7);
        pnh_.param("velocity_tolerance", velocity_tolerance_, 0.2);
        pnh_.param("arrival_stable_sec", arrival_stable_sec_, 0.5);
        pnh_.param("gimbal_retry_sec", gimbal_retry_sec_, 2.0);
        pnh_.param("land_command_retry_sec", land_command_retry_sec_, 1.0);
        pnh_.param("start_plan", enable_start_trigger_, 1);
        pnh_.param("back_plan", enable_back_trigger_, 1);
        pnh_.param("enable_rc", enable_rc_, true);

        if (position_tolerance_ <= 0.0 || velocity_tolerance_ < 0.0 ||
            arrival_stable_sec_ < 0.0 || gimbal_retry_sec_ <= 0.0 ||
            land_command_retry_sec_ <= 0.0)
        {
            throw std::runtime_error("Invalid mission tolerance or timing parameter");
        }

        loadWaypoints();

        point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal", 10);
        gimbal_task_pub_ =
            nh_.advertise<std_msgs::Float64MultiArray>("/mission/gimbal_task", 10);
        takeoff_land_pub_ =
            nh_.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
        startcommand_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        backcommand_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>("/back_trigger", 10);

        odom_sub_ = nh_.subscribe("odom_topic", 10, &MultipointPlanner::odomCallback, this);
        gimbal_done_sub_ = nh_.subscribe("/mission/gimbal_done", 10,
                                         &MultipointPlanner::gimbalDoneCallback, this);
        if (enable_start_trigger_)
        {
            startcommand_sub_ = nh_.subscribe("/move_base_simple/goal", 10,
                                              &MultipointPlanner::startCallback, this);
        }
        if (enable_back_trigger_)
        {
            backcommand_sub_ = nh_.subscribe("/back_trigger", 10,
                                             &MultipointPlanner::backCallback, this);
        }
        if (enable_rc_)
        {
            rc_sub_ = nh_.subscribe("/mavros/rc/in", 10, &MultipointPlanner::rcCallback, this);
        }

        timer_ = nh_.createTimer(ros::Duration(0.05), &MultipointPlanner::timerCallback, this);

        ROS_INFO("A8 mini mission ready: %zu waypoint(s), position tolerance %.2f m, "
                 "velocity tolerance %.2f m/s",
                 mission_waypoints_.size(), position_tolerance_, velocity_tolerance_);
    }

private:
    static bool finite(double value)
    {
        return std::isfinite(value);
    }

    static const char *gimbalModeName(GimbalMode mode)
    {
        return mode == GIMBAL_RANGE ? "range" : "angle";
    }

    void validateWaypoint(const Waypoint &waypoint, std::set<uint32_t> *ids) const
    {
        if (waypoint.id == 0 || !ids->insert(waypoint.id).second)
        {
            throw std::runtime_error("Waypoint IDs must be unique positive integers");
        }
        if (!finite(waypoint.x) || !finite(waypoint.y) || !finite(waypoint.z) ||
            !finite(waypoint.hover_sec) || !finite(waypoint.gimbal_yaw_deg) ||
            !finite(waypoint.gimbal_yaw_min_deg) || !finite(waypoint.gimbal_yaw_max_deg) ||
            !finite(waypoint.gimbal_pitch_deg) || !finite(waypoint.gimbal_settle_sec))
        {
            throw std::runtime_error("Waypoint fields must be finite numbers");
        }
        if (waypoint.hover_sec < 0.0 || waypoint.gimbal_settle_sec < 0.0 ||
            waypoint.gimbal_settle_sec > 60.0)
        {
            throw std::runtime_error(
                "hover_sec cannot be negative and gimbal_settle_sec must be in [0, 60]");
        }
        // A8 mini limits from the supplied user manual.
        if (waypoint.gimbal_yaw_deg < -135.0 || waypoint.gimbal_yaw_deg > 135.0 ||
            waypoint.gimbal_yaw_min_deg < -135.0 ||
            waypoint.gimbal_yaw_min_deg > 135.0 ||
            waypoint.gimbal_yaw_max_deg < -135.0 ||
            waypoint.gimbal_yaw_max_deg > 135.0 ||
            waypoint.gimbal_pitch_deg < -90.0 || waypoint.gimbal_pitch_deg > 25.0)
        {
            throw std::runtime_error("A8 mini gimbal angle is outside its controllable range");
        }
        if (waypoint.gimbal_yaw_min_deg >= waypoint.gimbal_yaw_max_deg)
        {
            throw std::runtime_error(
                "gimbal_yaw_min_deg must be less than gimbal_yaw_max_deg");
        }
    }

    void loadWaypoints()
    {
        if (yaml_path_.empty())
        {
            throw std::runtime_error("The yaml_path parameter is empty");
        }

        YAML::Node root = YAML::LoadFile(yaml_path_);
        YAML::Node waypoints = root["waypoints"];
        if (!waypoints || !waypoints.IsSequence() || waypoints.size() == 0)
        {
            throw std::runtime_error("points.yaml must contain a non-empty 'waypoints' sequence");
        }

        std::set<uint32_t> ids;
        for (std::size_t i = 0; i < waypoints.size(); ++i)
        {
            const YAML::Node &node = waypoints[i];
            if (!node.IsMap() || !node["id"] || !node["x"] || !node["y"] || !node["z"] ||
                !node["hover_sec"] || !node["gimbal_pitch_deg"] ||
                !node["gimbal_settle_sec"])
            {
                std::ostringstream error;
                error << "Waypoint " << i << " is missing one or more required fields";
                throw std::runtime_error(error.str());
            }

            const long long raw_id = node["id"].as<long long>();
            if (raw_id <= 0 ||
                static_cast<unsigned long long>(raw_id) > std::numeric_limits<uint32_t>::max())
            {
                throw std::runtime_error("Waypoint ID is outside the UInt32 range");
            }

            Waypoint waypoint;
            waypoint.id = static_cast<uint32_t>(raw_id);
            waypoint.x = node["x"].as<double>();
            waypoint.y = node["y"].as<double>();
            waypoint.z = node["z"].as<double>();
            waypoint.hover_sec = node["hover_sec"].as<double>();
            waypoint.gimbal_mode = GIMBAL_ANGLE;
            if (node["gimbal_mode"])
            {
                const std::string mode = node["gimbal_mode"].as<std::string>();
                if (mode == "angle")
                {
                    waypoint.gimbal_mode = GIMBAL_ANGLE;
                }
                else if (mode == "range" || mode == "sweep")
                {
                    waypoint.gimbal_mode = GIMBAL_RANGE;
                }
                else
                {
                    throw std::runtime_error(
                        "gimbal_mode must be 'angle' or 'range' ('sweep' is also accepted)");
                }
            }
            if (waypoint.gimbal_mode == GIMBAL_ANGLE && !node["gimbal_yaw_deg"])
            {
                throw std::runtime_error(
                    "gimbal_yaw_deg is required when gimbal_mode is 'angle'");
            }
            waypoint.gimbal_yaw_deg =
                node["gimbal_yaw_deg"] ? node["gimbal_yaw_deg"].as<double>() : 0.0;
            waypoint.gimbal_yaw_min_deg = node["gimbal_yaw_min_deg"]
                                               ? node["gimbal_yaw_min_deg"].as<double>()
                                               : -135.0;
            waypoint.gimbal_yaw_max_deg = node["gimbal_yaw_max_deg"]
                                               ? node["gimbal_yaw_max_deg"].as<double>()
                                               : 135.0;
            waypoint.gimbal_pitch_deg = node["gimbal_pitch_deg"].as<double>();
            waypoint.gimbal_settle_sec = node["gimbal_settle_sec"].as<double>();
            waypoint.run_gimbal = true;
            validateWaypoint(waypoint, &ids);
            mission_waypoints_.push_back(waypoint);

            ROS_INFO("Loaded waypoint %u: position [%.2f, %.2f, %.2f], hover %.2f s, "
                     "gimbal [mode %s, yaw %.1f, range %.1f..%.1f, pitch %.1f], "
                     "settle %.2f s",
                     waypoint.id, waypoint.x, waypoint.y, waypoint.z, waypoint.hover_sec,
                     gimbalModeName(waypoint.gimbal_mode),
                     waypoint.gimbal_yaw_deg, waypoint.gimbal_yaw_min_deg,
                     waypoint.gimbal_yaw_max_deg, waypoint.gimbal_pitch_deg,
                     waypoint.gimbal_settle_sec);
        }

        // Keep compatibility with the original package's optional return route.
        YAML::Node return_points = root["test_back"];
        if (return_points && return_points.IsSequence())
        {
            for (std::size_t i = 0; i < return_points.size(); ++i)
            {
                if (!return_points[i].IsSequence() || return_points[i].size() != 3)
                {
                    throw std::runtime_error("Each test_back point must be [x, y, z]");
                }
                Waypoint waypoint;
                waypoint.id = static_cast<uint32_t>(i + 1);
                waypoint.x = return_points[i][0].as<double>();
                waypoint.y = return_points[i][1].as<double>();
                waypoint.z = return_points[i][2].as<double>();
                waypoint.hover_sec = 0.0;
                waypoint.gimbal_yaw_deg = 0.0;
                waypoint.gimbal_yaw_min_deg = -135.0;
                waypoint.gimbal_yaw_max_deg = 135.0;
                waypoint.gimbal_pitch_deg = 0.0;
                waypoint.gimbal_settle_sec = 0.0;
                waypoint.gimbal_mode = GIMBAL_ANGLE;
                waypoint.run_gimbal = false;
                return_waypoints_.push_back(waypoint);
            }
        }
    }

    void odomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        odom_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->pose.pose.position.z;
        odom_velocity_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
            msg->twist.twist.linear.z;
        odom_received_ = true;
    }

    void startCallback(const geometry_msgs::PoseStamped::ConstPtr &)
    {
        if (landing_latched_)
        {
            ROS_WARN("Ignoring mission trigger because landing is latched");
            return;
        }
        if (state_ != FINISHED)
        {
            ROS_WARN("Ignoring start trigger because a mission is already active");
            return;
        }
        startMission(mission_waypoints_, true);
    }

    void backCallback(const geometry_msgs::PoseStamped::ConstPtr &)
    {
        if (landing_latched_)
        {
            ROS_WARN("Ignoring return trigger because landing is latched");
            return;
        }
        if (return_waypoints_.empty())
        {
            ROS_ERROR("No test_back route is configured");
            return;
        }
        if (state_ != FINISHED)
        {
            ROS_WARN("Return trigger cancels the active waypoint mission");
        }
        startMission(return_waypoints_, false);
    }

    void startMission(const std::vector<Waypoint> &waypoints, bool use_gimbal)
    {
        active_waypoints_ = waypoints;
        current_index_ = 0;
        mission_uses_gimbal_ = use_gimbal;
        arrival_candidate_time_ = ros::Time(0);
        arrived_time_ = ros::Time(0);

        if (use_gimbal)
        {
            openMissionCsv();
            ROS_INFO("Received mission trigger");
        }
        else
        {
            closeMissionCsv();
            ROS_INFO("Received return trigger");
        }
        publishCurrentGoal();
    }

    void publishCurrentGoal()
    {
        const Waypoint &waypoint = active_waypoints_.at(current_index_);
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = goal_frame_id_;
        goal.pose.position.x = waypoint.x;
        goal.pose.position.y = waypoint.y;
        goal.pose.position.z = waypoint.z;
        goal.pose.orientation.w = 1.0;
        point_pub_.publish(goal);

        state_ = FLYING;
        arrival_candidate_time_ = ros::Time(0);
        ROS_INFO("Published waypoint %u goal: [x %.2f, y %.2f, z %.2f]",
                 waypoint.id, waypoint.x, waypoint.y, waypoint.z);
    }

    void timerCallback(const ros::TimerEvent &)
    {
        const ros::Time now = ros::Time::now();
        if (landing_latched_ &&
            (now - last_land_publish_time_).toSec() >= land_command_retry_sec_)
        {
            publishLandingCommand(true);
        }

        if (state_ == FINISHED)
        {
            return;
        }
        if (!odom_received_)
        {
            ROS_WARN_THROTTLE(5.0, "Waiting for odometry before evaluating waypoint arrival");
            return;
        }

        const Waypoint &waypoint = active_waypoints_.at(current_index_);

        if (state_ == FLYING)
        {
            const Eigen::Vector3d target(waypoint.x, waypoint.y, waypoint.z);
            const double distance = (target - odom_position_).norm();
            const double speed = odom_velocity_.norm();

            if (distance <= position_tolerance_ && speed <= velocity_tolerance_)
            {
                if (arrival_candidate_time_.isZero())
                {
                    arrival_candidate_time_ = now;
                }
                if ((now - arrival_candidate_time_).toSec() >= arrival_stable_sec_)
                {
                    arrived_time_ = now;
                    hover_start_time_ = now;
                    state_ = HOVERING;
                    ROS_INFO("Waypoint %u arrived, ros_time=%.3f, distance=%.3f, speed=%.3f",
                             waypoint.id, arrived_time_.toSec(), distance, speed);
                }
            }
            else
            {
                arrival_candidate_time_ = ros::Time(0);
            }
            return;
        }

        if (state_ == HOVERING)
        {
            if ((now - hover_start_time_).toSec() < waypoint.hover_sec)
            {
                return;
            }

            if (mission_uses_gimbal_ && waypoint.run_gimbal)
            {
                publishGimbalTask(false);
                state_ = WAITING_GIMBAL;
            }
            else
            {
                advanceWaypoint();
            }
            return;
        }

        if (state_ == WAITING_GIMBAL &&
            (now - last_gimbal_publish_time_).toSec() >= gimbal_retry_sec_)
        {
            publishGimbalTask(true);
        }
    }

    void publishGimbalTask(bool retry)
    {
        const Waypoint &waypoint = active_waypoints_.at(current_index_);
        std_msgs::Float64MultiArray task;
        task.data.reserve(7);
        task.data.push_back(static_cast<double>(waypoint.id));
        task.data.push_back(waypoint.gimbal_yaw_deg);
        task.data.push_back(waypoint.gimbal_pitch_deg);
        task.data.push_back(waypoint.gimbal_settle_sec);
        task.data.push_back(static_cast<double>(waypoint.gimbal_mode));
        task.data.push_back(waypoint.gimbal_yaw_min_deg);
        task.data.push_back(waypoint.gimbal_yaw_max_deg);
        gimbal_task_pub_.publish(task);
        last_gimbal_publish_time_ = ros::Time::now();

        if (retry)
        {
            ROS_WARN("No gimbal_done for waypoint %u yet; republished gimbal task", waypoint.id);
        }
        else
        {
            ROS_INFO("Published gimbal task for waypoint %u: mode %s, yaw %.1f, "
                     "range %.1f..%.1f, pitch %.1f, settle %.2f s",
                     waypoint.id, gimbalModeName(waypoint.gimbal_mode),
                     waypoint.gimbal_yaw_deg, waypoint.gimbal_yaw_min_deg,
                     waypoint.gimbal_yaw_max_deg, waypoint.gimbal_pitch_deg,
                     waypoint.gimbal_settle_sec);
        }
    }

    void gimbalDoneCallback(const std_msgs::UInt32ConstPtr &msg)
    {
        if (state_ != WAITING_GIMBAL)
        {
            ROS_WARN("Ignoring gimbal_done %u because no gimbal task is pending", msg->data);
            return;
        }

        const Waypoint &waypoint = active_waypoints_.at(current_index_);
        if (msg->data != waypoint.id)
        {
            ROS_WARN("Ignoring gimbal_done %u; currently waiting for waypoint %u",
                     msg->data, waypoint.id);
            return;
        }

        const ros::Time done_time = ros::Time::now();
        ROS_INFO("Waypoint %u gimbal done, ros_time=%.3f", waypoint.id, done_time.toSec());
        writeMissionCsv(waypoint, arrived_time_, done_time);
        advanceWaypoint();
    }

    void advanceWaypoint()
    {
        ++current_index_;
        if (current_index_ >= active_waypoints_.size())
        {
            state_ = FINISHED;
            closeMissionCsv();
            ROS_INFO("Waypoint mission finished, ros_time=%.3f", ros::Time::now().toSec());
            return;
        }
        publishCurrentGoal();
    }

    void openMissionCsv()
    {
        closeMissionCsv();
        if (mission_csv_path_.empty())
        {
            return;
        }

        mission_csv_.open(mission_csv_path_.c_str(), std::ios::out | std::ios::trunc);
        if (!mission_csv_.is_open())
        {
            ROS_ERROR("Cannot open mission CSV: %s", mission_csv_path_.c_str());
            return;
        }
        mission_csv_ << "waypoint_id,arrived_time,gimbal_done_time,yaw,pitch,gimbal_mode,"
                        "yaw_min,yaw_max\n";
        mission_csv_.flush();
        ROS_INFO("Mission timestamps will be written to %s", mission_csv_path_.c_str());
    }

    void writeMissionCsv(const Waypoint &waypoint, const ros::Time &arrived,
                         const ros::Time &done)
    {
        if (!mission_csv_.is_open())
        {
            return;
        }
        mission_csv_ << waypoint.id << ',' << std::fixed << std::setprecision(3)
                     << arrived.toSec() << ',' << done.toSec() << ','
                     << waypoint.gimbal_yaw_deg << ',' << waypoint.gimbal_pitch_deg << ','
                     << gimbalModeName(waypoint.gimbal_mode) << ','
                     << waypoint.gimbal_yaw_min_deg << ',' << waypoint.gimbal_yaw_max_deg
                     << '\n';
        mission_csv_.flush();
    }

    void closeMissionCsv()
    {
        if (mission_csv_.is_open())
        {
            mission_csv_.close();
        }
    }

    static bool isInRcState(uint16_t state, uint16_t input)
    {
        return input > state - 100 && input < state + 100;
    }

    static const char *rcStateName(RC_EIGHT_STATE state)
    {
        if (state == RC_EIGHT_UP)
        {
            return "UP";
        }
        if (state == RC_EIGHT_MIDDLE)
        {
            return "MIDDLE";
        }
        return "DOWN";
    }

    static bool classifyRcState(uint16_t input, RC_EIGHT_STATE *state)
    {
        if (isInRcState(RC_EIGHT_UP, input))
        {
            *state = RC_EIGHT_UP;
            return true;
        }
        if (isInRcState(RC_EIGHT_MIDDLE, input))
        {
            *state = RC_EIGHT_MIDDLE;
            return true;
        }
        if (isInRcState(RC_EIGHT_DOWN, input))
        {
            *state = RC_EIGHT_DOWN;
            return true;
        }
        return false;
    }

    void publishLandingCommand(bool retry)
    {
        quadrotor_msgs::TakeoffLand command;
        command.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::LAND;
        takeoff_land_pub_.publish(command);
        last_land_publish_time_ = ros::Time::now();

        if (retry)
        {
            ROS_WARN("Landing remains latched; republished LAND command so px4ctrl can "
                     "accept it after entering AUTO_HOVER");
        }
        else
        {
            ROS_WARN("RC channel 8: LAND requested; active waypoint mission cancelled");
        }
    }

    void requestLanding()
    {
        state_ = FINISHED;
        closeMissionCsv();
        landing_latched_ = true;
        last_land_publish_time_ = ros::Time(0);
        publishLandingCommand(false);
    }

    void rcCallback(const mavros_msgs::RCInConstPtr &msg)
    {
        if (msg->channels.size() <= 7)
        {
            ROS_WARN_THROTTLE(5.0, "RC message has fewer than eight channels");
            return;
        }

        const uint16_t raw_input = msg->channels[7];
        const unsigned int raw_log = static_cast<unsigned int>(raw_input);
        RC_EIGHT_STATE current;
        if (!classifyRcState(raw_input, &current))
        {
            ROS_WARN_THROTTLE(2.0,
                              "RC channel 8 raw=%u is outside UP/MIDDLE/DOWN windows",
                              raw_log);
            return;
        }

        ROS_INFO_THROTTLE(2.0,
                          "RC channel 8 raw=%u state=%s previous=%s initialized=%s "
                          "landing_latched=%s",
                          raw_log, rcStateName(current), rcStateName(rc_eight_pre_),
                          rc_initialized_ ? "true" : "false",
                          landing_latched_ ? "true" : "false");

        if (!rc_initialized_)
        {
            if (current != RC_EIGHT_DOWN)
            {
                ROS_WARN_THROTTLE(5.0,
                                  "Waiting for RC channel 8 to start in DOWN; raw=%u state=%s",
                                  raw_log, rcStateName(current));
                return;
            }
            rc_initialized_ = true;
            rc_eight_pre_ = RC_EIGHT_DOWN;
            ROS_INFO("RC channel 8 initialized in DOWN (raw=%u)", raw_log);
            return;
        }

        if (landing_latched_)
        {
            return;
        }
        if (current == rc_eight_pre_)
        {
            return;
        }

        const RC_EIGHT_STATE previous = rc_eight_pre_;
        rc_eight_pre_ = current;
        ROS_INFO("RC channel 8 transition %s -> %s (raw=%u)",
                 rcStateName(previous), rcStateName(current), raw_log);

        // Entering DOWN always requests landing. This also covers a direct UP -> DOWN
        // transition that could be missed by the previous sequence-only implementation.
        if (current == RC_EIGHT_DOWN)
        {
            requestLanding();
        }
        else if (current == RC_EIGHT_MIDDLE && previous == RC_EIGHT_DOWN)
        {
            quadrotor_msgs::TakeoffLand command;
            command.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
            takeoff_land_pub_.publish(command);
            ROS_INFO("RC channel 8: takeoff");
        }
        else if (current == RC_EIGHT_UP && previous == RC_EIGHT_MIDDLE)
        {
            geometry_msgs::PoseStamped trigger;
            trigger.header.stamp = ros::Time::now();
            startcommand_pub_.publish(trigger);
            ROS_INFO("RC channel 8: start waypoint mission");
        }
        else if (current == RC_EIGHT_MIDDLE && previous == RC_EIGHT_UP)
        {
            geometry_msgs::PoseStamped trigger;
            trigger.header.stamp = ros::Time::now();
            backcommand_pub_.publish(trigger);
            ROS_INFO("RC channel 8: return route");
        }
        else
        {
            ROS_WARN("RC channel 8 transition %s -> %s has no assigned action",
                     rcStateName(previous), rcStateName(current));
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher point_pub_;
    ros::Publisher gimbal_task_pub_;
    ros::Publisher takeoff_land_pub_;
    ros::Publisher startcommand_pub_;
    ros::Publisher backcommand_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gimbal_done_sub_;
    ros::Subscriber startcommand_sub_;
    ros::Subscriber backcommand_sub_;
    ros::Subscriber rc_sub_;
    ros::Timer timer_;

    std::vector<Waypoint> mission_waypoints_;
    std::vector<Waypoint> return_waypoints_;
    std::vector<Waypoint> active_waypoints_;
    MissionState state_;
    std::size_t current_index_;

    Eigen::Vector3d odom_position_;
    Eigen::Vector3d odom_velocity_;
    bool odom_received_;
    bool mission_uses_gimbal_;

    ros::Time arrival_candidate_time_;
    ros::Time arrived_time_;
    ros::Time hover_start_time_;
    ros::Time last_gimbal_publish_time_;
    ros::Time last_land_publish_time_;

    std::string yaml_path_;
    std::string goal_frame_id_;
    std::string mission_csv_path_;
    double position_tolerance_;
    double velocity_tolerance_;
    double arrival_stable_sec_;
    double gimbal_retry_sec_;
    double land_command_retry_sec_;
    int enable_start_trigger_;
    int enable_back_trigger_;
    bool enable_rc_;

    RC_EIGHT_STATE rc_eight_pre_;
    bool rc_initialized_;
    bool landing_latched_;
    std::ofstream mission_csv_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multipointplan_node");
    try
    {
        MultipointPlanner planner;
        ros::spin();
    }
    catch (const std::exception &error)
    {
        ROS_FATAL("Failed to start multipoint mission node: %s", error.what());
        return 1;
    }
    return 0;
}
