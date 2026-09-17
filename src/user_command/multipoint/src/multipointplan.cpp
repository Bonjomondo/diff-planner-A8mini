#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt32.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
#include "mission_guard.h"

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
          clicked_route_started_(false),
          rc_eight_pre_(RC_EIGHT_DOWN),
          rc_initialized_(false),
          landing_latched_(false),
          automatic_landing_(false)
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
        pnh_.param("enable_rviz_click_route", enable_rviz_click_route_, true);
        pnh_.param<std::string>("clicked_point_topic", clicked_point_topic_,
                                "/clicked_point");
        pnh_.param("clicked_point_height", clicked_point_height_, 1.0);
        pnh_.param("clicked_point_use_z", clicked_point_use_z_, false);
        pnh_.param("clicked_max_points", clicked_max_points_, 50);
        pnh_.param<std::string>("mission_source", mission_source_, "clicked");
        pnh_.param("enable_gimbal_actions", enable_gimbal_actions_, true);
        pnh_.param("require_flight_ready", require_flight_ready_, true);
        pnh_.param("odom_timeout_sec", odom_timeout_sec_, 0.5);
        pnh_.param("flight_state_timeout_sec", flight_state_timeout_sec_, 2.5);
        pnh_.param("waypoint_timeout_sec", waypoint_timeout_sec_, 120.0);
        pnh_.param("gimbal_timeout_sec", gimbal_timeout_sec_, 90.0);
        if ((mission_source_ != "clicked" && mission_source_ != "preset") ||
            goal_frame_id_ != "world" ||
            (mission_source_ == "clicked" && !enable_rviz_click_route_) ||
            !finite(odom_timeout_sec_) || odom_timeout_sec_ <= 0.0 ||
            !finite(flight_state_timeout_sec_) || flight_state_timeout_sec_ <= 0.0 ||
            !finite(waypoint_timeout_sec_) || waypoint_timeout_sec_ <= 0.0 ||
            !finite(gimbal_timeout_sec_) || gimbal_timeout_sec_ <= 0.0)
        {
            throw std::runtime_error("Invalid mission source/frame/timeout: use clicked|preset and world");
        }

        if (position_tolerance_ <= 0.0 || velocity_tolerance_ < 0.0 ||
            arrival_stable_sec_ < 0.0 || gimbal_retry_sec_ <= 0.0 ||
            land_command_retry_sec_ <= 0.0 || clicked_max_points_ <= 0 ||
            !finite(clicked_point_height_))
        {
            throw std::runtime_error("Invalid mission tolerance or timing parameter");
        }

        loadWaypoints();

        point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal", 10);
        gimbal_task_pub_ =
            nh_.advertise<std_msgs::Float64MultiArray>("/mission/gimbal_task", 10);
        takeoff_land_pub_ =
            nh_.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
        planning_stop_pub_ = nh_.advertise<std_msgs::Empty>("/planning/stop", 1, true);
        startcommand_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        backcommand_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>("/back_trigger", 10);
        clicked_markers_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("/mission/clicked_waypoints", 1,
                                                            true);

        odom_sub_ = nh_.subscribe("odom_topic", 10, &MultipointPlanner::odomCallback, this);
        fcu_sub_ = nh_.subscribe("/mavros/state", 1, &MultipointPlanner::fcuCallback, this);
        extended_sub_ = nh_.subscribe("/mavros/extended_state", 1,
                                     &MultipointPlanner::extendedCallback, this);
        controller_sub_ = nh_.subscribe("/px4ctrl/state", 1,
                                       &MultipointPlanner::controllerCallback, this);
        ready_sub_ = nh_.subscribe("/px4ctrl/mission_ready", 1,
                                  &MultipointPlanner::readyCallback, this);
        heartbeat_sub_ = nh_.subscribe("/drone_0_traj_server/heartbeat", 1,
                                      &MultipointPlanner::heartbeatCallback, this);
        land_sub_ = nh_.subscribe("/mission/land", 1,
                                 &MultipointPlanner::landCallback, this);
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
        if (enable_rviz_click_route_)
        {
            clicked_point_sub_ = nh_.subscribe(clicked_point_topic_, 50,
                                                &MultipointPlanner::clickedPointCallback, this);
            start_clicked_route_sub_ = nh_.subscribe(
                "/mission/start_clicked_route", 1,
                &MultipointPlanner::startClickedRouteCallback, this);
            clear_clicked_route_sub_ = nh_.subscribe(
                "/mission/clear_clicked_route", 1,
                &MultipointPlanner::clearClickedRouteCallback, this);
            publishClickedMarkers();
        }

        timer_ = nh_.createTimer(ros::Duration(0.05), &MultipointPlanner::timerCallback, this);

        ROS_INFO("Mission ready: %zu YAML waypoint(s), position tolerance %.2f m, "
                 "velocity tolerance %.2f m/s, RViz click route %s",
                 mission_waypoints_.size(), position_tolerance_, velocity_tolerance_,
                 enable_rviz_click_route_ ? "enabled" : "disabled");
        ROS_INFO("Mission configuration: source=%s gimbal_actions=%s flight_gate=%s",
                 mission_source_.c_str(), enable_gimbal_actions_ ? "true" : "false",
                 require_flight_ready_ ? "true" : "false");
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
            waypoint.run_gimbal = enable_gimbal_actions_;
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
        odom_received_ = odom_position_.allFinite() && odom_velocity_.allFinite() &&
                         (msg->header.frame_id == goal_frame_id_ ||
                          msg->header.frame_id == "/" + goal_frame_id_);
        odom_receive_time_ = ros::WallTime::now();
        odom_stamp_ = msg->header.stamp;
    }

    void fcuCallback(const mavros_msgs::StateConstPtr &msg)
    {
        fcu_state_ = *msg;
        fcu_receive_time_ = ros::WallTime::now();
    }

    void extendedCallback(const mavros_msgs::ExtendedStateConstPtr &msg)
    {
        landed_state_ = msg->landed_state;
        extended_receive_time_ = ros::WallTime::now();
    }

    void controllerCallback(const std_msgs::StringConstPtr &msg)
    {
        controller_state_ = msg->data;
        controller_receive_time_ = ros::WallTime::now();
    }

    void readyCallback(const std_msgs::BoolConstPtr &msg)
    {
        controller_ready_ = msg->data;
        ready_receive_time_ = ros::WallTime::now();
    }

    void heartbeatCallback(const std_msgs::EmptyConstPtr &)
    {
        heartbeat_receive_time_ = ros::WallTime::now();
    }

    bool fresh(const ros::WallTime &stamp, double timeout) const
    {
        return !stamp.isZero() && mission_guard::fresh(
            (ros::WallTime::now() - stamp).toSec(), timeout);
    }

    bool odomFresh() const
    {
        return odom_received_ && fresh(odom_receive_time_, odom_timeout_sec_) &&
            !odom_stamp_.isZero() && mission_guard::fresh(
                (ros::Time::now() - odom_stamp_).toSec(), odom_timeout_sec_);
    }

    bool flightReady(bool starting) const
    {
        if (!require_flight_ready_)
            return true;  // Only the simulator launch disables this gate.
        mission_guard::FlightStatus status;
        status.fresh_state = fresh(fcu_receive_time_, flight_state_timeout_sec_) &&
                             fresh(extended_receive_time_, flight_state_timeout_sec_);
        status.connected = fcu_state_.connected;
        status.armed = fcu_state_.armed;
        status.offboard = fcu_state_.mode == "OFFBOARD";
        status.in_air = landed_state_ == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR;
        status.controller_fresh = fresh(controller_receive_time_, 0.5) &&
                                  fresh(ready_receive_time_, 0.5);
        status.controller_ready = controller_ready_;
        status.hover_or_command = controller_state_ == "AUTO_HOVER" || controller_state_ == "CMD_CTRL";
        return mission_guard::flightReady(status, starting);
    }

    void abortMission(const char *reason)
    {
        ROS_ERROR("Mission stopped: %s. Check aircraft/RC; land and restart the stack before a new mission.", reason);
        state_ = FINISHED;
        mission_fault_ = true;
        automatic_landing_ = false;
        arrival_candidate_time_ = ros::Time(0);
        planning_stop_pub_.publish(std_msgs::Empty());
        closeMissionCsv();
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
        if (mission_source_ == "clicked")
        {
            startClickedRoute();
            return;
        }
        startMission(mission_waypoints_, enable_gimbal_actions_);
    }

    Waypoint makeClickedWaypoint(const geometry_msgs::Point &point, uint32_t id) const
    {
        Waypoint waypoint;
        waypoint.id = id;
        waypoint.x = point.x;
        waypoint.y = point.y;
        waypoint.z = clicked_point_use_z_ ? point.z : clicked_point_height_;
        waypoint.hover_sec = 0.0;
        waypoint.gimbal_yaw_deg = 0.0;
        waypoint.gimbal_yaw_min_deg = -135.0;
        waypoint.gimbal_yaw_max_deg = 135.0;
        waypoint.gimbal_pitch_deg = 0.0;
        waypoint.gimbal_settle_sec = 0.0;
        waypoint.gimbal_mode = GIMBAL_ANGLE;
        waypoint.run_gimbal = false;
        return waypoint;
    }

    void clickedPointCallback(const geometry_msgs::PointStampedConstPtr &msg)
    {
        if (!enable_rviz_click_route_)
        {
            return;
        }
        if (state_ != FINISHED || clicked_route_started_)
        {
            ROS_WARN_THROTTLE(3.0,
                              "Ignoring RViz point: clear the finished route before marking "
                              "a new one");
            return;
        }
        if (static_cast<int>(clicked_waypoints_.size()) >= clicked_max_points_)
        {
            ROS_WARN_THROTTLE(3.0, "RViz click route reached its %d-point limit",
                              clicked_max_points_);
            return;
        }
        if (msg->header.frame_id.empty())
        {
            ROS_WARN("Ignoring RViz point without a frame_id");
            return;
        }

        geometry_msgs::PointStamped point_in_goal_frame = *msg;
        if (msg->header.frame_id != goal_frame_id_)
        {
            try
            {
                tf_listener_.transformPoint(goal_frame_id_, *msg, point_in_goal_frame);
            }
            catch (const tf::TransformException &error)
            {
                ROS_WARN("Cannot transform clicked point from '%s' to '%s': %s",
                         msg->header.frame_id.c_str(), goal_frame_id_.c_str(), error.what());
                return;
            }
        }

        const geometry_msgs::Point &point = point_in_goal_frame.point;
        const double z = clicked_point_use_z_ ? point.z : clicked_point_height_;
        if (!finite(point.x) || !finite(point.y) || !finite(z))
        {
            ROS_WARN("Ignoring non-finite RViz clicked point");
            return;
        }

        const uint32_t id = static_cast<uint32_t>(clicked_waypoints_.size() + 1);
        clicked_waypoints_.push_back(makeClickedWaypoint(point, id));
        publishClickedMarkers();
        ROS_INFO("RViz route point %u added: [%.2f, %.2f, %.2f] in frame '%s'",
                 id, point.x, point.y, z, goal_frame_id_.c_str());
    }

    void startClickedRouteCallback(const std_msgs::EmptyConstPtr &)
    {
        startClickedRoute();
    }

    void startClickedRoute()
    {
        if (mission_source_ != "clicked")
        {
            ROS_WARN("RViz route execution disabled in preset mode; restart with mission_source:=clicked");
            return;
        }
        if (!enable_rviz_click_route_ || clicked_waypoints_.empty())
        {
            ROS_WARN("No RViz clicked points are waiting to be flown");
            return;
        }
        if (landing_latched_)
        {
            ROS_WARN("Ignoring RViz route start because landing is latched");
            return;
        }
        if (state_ != FINISHED || clicked_route_started_)
        {
            ROS_WARN("Ignoring RViz route start because a route is already active or consumed");
            return;
        }

        if (!startMission(clicked_waypoints_, false))
            return;
        clicked_route_started_ = true;
        ROS_INFO("Started RViz clicked route with %zu point(s); A8 mini actions disabled",
                 active_waypoints_.size());
    }

    void clearClickedRouteCallback(const std_msgs::EmptyConstPtr &)
    {
        if (state_ != FINISHED)
        {
            ROS_WARN("Cannot clear RViz clicked route while it is active; land first");
            return;
        }
        clicked_waypoints_.clear();
        clicked_route_started_ = false;
        publishClickedMarkers();
        ROS_INFO("Cleared RViz clicked route");
    }

    void publishClickedMarkers()
    {
        if (!enable_rviz_click_route_)
        {
            return;
        }

        const ros::Time stamp = ros::Time::now();
        visualization_msgs::MarkerArray marker_array;

        visualization_msgs::Marker points;
        points.header.frame_id = goal_frame_id_;
        points.header.stamp = stamp;
        points.ns = "rviz_clicked_route";
        points.id = 0;
        points.type = visualization_msgs::Marker::SPHERE_LIST;
        points.action = clicked_waypoints_.empty() ? visualization_msgs::Marker::DELETE
                                                    : visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.scale.x = 0.22;
        points.scale.y = 0.22;
        points.scale.z = 0.22;
        points.color.r = 1.0;
        points.color.g = 0.65;
        points.color.b = 0.05;
        points.color.a = 1.0;

        visualization_msgs::Marker line;
        line.header = points.header;
        line.ns = points.ns;
        line.id = 1;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = clicked_waypoints_.size() < 2 ? visualization_msgs::Marker::DELETE
                                                    : visualization_msgs::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.05;
        line.color.r = 1.0;
        line.color.g = 0.35;
        line.color.b = 0.05;
        line.color.a = 0.9;

        for (std::size_t i = 0; i < clicked_waypoints_.size(); ++i)
        {
            geometry_msgs::Point marker_point;
            marker_point.x = clicked_waypoints_[i].x;
            marker_point.y = clicked_waypoints_[i].y;
            marker_point.z = clicked_waypoints_[i].z;
            points.points.push_back(marker_point);
            line.points.push_back(marker_point);

            visualization_msgs::Marker label;
            label.header = points.header;
            label.ns = points.ns;
            label.id = static_cast<int32_t>(1000 + i);
            label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label.action = visualization_msgs::Marker::ADD;
            label.pose.orientation.w = 1.0;
            label.pose.position = marker_point;
            label.pose.position.z += 0.18;
            label.scale.z = 0.24;
            label.color.r = 1.0;
            label.color.g = 0.9;
            label.color.b = 0.1;
            label.color.a = 1.0;
            label.text = std::to_string(i + 1);
            marker_array.markers.push_back(label);
        }

        marker_array.markers.push_back(points);
        marker_array.markers.push_back(line);
        for (std::size_t i = clicked_waypoints_.size(); i < last_clicked_marker_count_; ++i)
        {
            visualization_msgs::Marker label;
            label.header = points.header;
            label.ns = points.ns;
            label.id = static_cast<int32_t>(1000 + i);
            label.action = visualization_msgs::Marker::DELETE;
            marker_array.markers.push_back(label);
        }

        clicked_markers_pub_.publish(marker_array);
        last_clicked_marker_count_ = clicked_waypoints_.size();
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
        if (startMission(return_waypoints_, false, true))
            ROS_INFO("Return route accepted; any previous waypoint mission was replaced");
    }

    bool startMission(const std::vector<Waypoint> &waypoints, bool use_gimbal,
                      bool returning = false)
    {
        // The legacy gimbal protocol deduplicates by waypoint ID only. Until
        // session IDs are available, do not silently skip actions on a rerun.
        if (use_gimbal && gimbal_mission_consumed_)
        {
            ROS_WARN("Gimbal route already used in this run; land and restart the complete stack before repeating it");
            return false;
        }
        if (landing_latched_ || mission_fault_ || waypoints.empty() || !odomFresh() ||
            !flightReady(!returning) ||
            !fresh(heartbeat_receive_time_, 0.5) || point_pub_.getNumSubscribers() == 0 ||
            (use_gimbal && gimbal_task_pub_.getNumSubscribers() == 0))
        {
            ROS_WARN("Mission start rejected: need fresh world odometry, planner heartbeat/goal subscriber, "
                     "OFFBOARD + armed + IN_AIR + controller mission_ready; payload missions also need gimbal. "
                     "Route is preserved; confirm hover and trigger again.");
            return false;
        }
        active_waypoints_ = waypoints;
        current_index_ = 0;
        mission_uses_gimbal_ = use_gimbal;
        if (use_gimbal)
            gimbal_mission_consumed_ = true;
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
            ROS_INFO("Starting route without gimbal actions");
        }
        publishCurrentGoal();
        return true;
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
        goal_publish_time_ = ros::WallTime::now();

        state_ = FLYING;
        arrival_candidate_time_ = ros::Time(0);
        ROS_INFO("Published waypoint %u goal: [x %.2f, y %.2f, z %.2f]",
                 waypoint.id, waypoint.x, waypoint.y, waypoint.z);
    }

    void timerCallback(const ros::TimerEvent &)
    {
        const ros::Time now = ros::Time::now();
        if (landing_latched_ && automatic_landing_ &&
            fresh(fcu_receive_time_, flight_state_timeout_sec_) &&
            fresh(extended_receive_time_, flight_state_timeout_sec_) &&
            !fcu_state_.armed &&
            landed_state_ == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
        {
            automatic_landing_ = false;
            ROS_INFO("Landing complete: ground + disarmed; LAND retries stopped. Restart stack for next flight.");
        }
        if (landing_latched_ && automatic_landing_ &&
            (now - last_land_publish_time_).toSec() >= land_command_retry_sec_)
        {
            publishLandingCommand(true);
        }

        if (state_ == FINISHED)
        {
            return;
        }
        if (!odomFresh() || !flightReady(false) || !fresh(heartbeat_receive_time_, 0.5))
        {
            abortMission("odometry, controller, FCU or planner heartbeat is no longer valid");
            return;
        }

        const Waypoint &waypoint = active_waypoints_.at(current_index_);

        if (state_ == FLYING)
        {
            if ((ros::WallTime::now() - goal_publish_time_).toSec() > waypoint_timeout_sec_)
            {
                abortMission("waypoint timeout (goal may have been rejected, changed or unreachable)");
                return;
            }
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
            const Eigen::Vector3d target(waypoint.x, waypoint.y, waypoint.z);
            if ((target - odom_position_).norm() > position_tolerance_ ||
                odom_velocity_.norm() > velocity_tolerance_)
            {
                state_ = FLYING;
                arrival_candidate_time_ = ros::Time(0);
                return;
            }
            if ((now - hover_start_time_).toSec() < waypoint.hover_sec)
            {
                return;
            }

            if (mission_uses_gimbal_ && waypoint.run_gimbal)
            {
                publishGimbalTask(false);
                state_ = WAITING_GIMBAL;
                gimbal_wait_time_ = ros::WallTime::now();
            }
            else
            {
                advanceWaypoint();
            }
            return;
        }

        if (state_ == WAITING_GIMBAL &&
            (ros::WallTime::now() - gimbal_wait_time_).toSec() > gimbal_timeout_sec_)
        {
            abortMission("gimbal completion timeout");
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
        if (!odomFresh() || !flightReady(false))
        {
            abortMission("invalid flight state at gimbal completion");
            return;
        }
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
            ROS_WARN("LAND requested; active waypoint mission cancelled");
        }
    }

    void requestLanding(bool automatic)
    {
        state_ = FINISHED;
        closeMissionCsv();
        landing_latched_ = true;
        automatic_landing_ = automatic;
        last_land_publish_time_ = ros::Time(0);

        // Stop the trajectory command source for both automatic and manual landing.
        // This is separate from LAND because manual RC takeover must not keep receiving
        // automatic LAND retries.
        std_msgs::Empty stop;
        planning_stop_pub_.publish(stop);

        if (automatic_landing_)
        {
            publishLandingCommand(false);
        }
        else
        {
            ROS_WARN("RC channel 8: manual LAND selected because channel 6 is out of "
                     "command mode; planner stopped and automatic LAND retry disabled");
        }
    }

    void landCallback(const std_msgs::EmptyConstPtr &)
    {
        requestLanding(true);
    }

    void rcCallback(const mavros_msgs::RCInConstPtr &msg)
    {
        if (msg->channels.size() <= 7)
        {
            ROS_WARN_THROTTLE(5.0, "RC message has fewer than eight channels");
            return;
        }

        const uint16_t raw_input = msg->channels[7];
        // Keep this threshold identical to RC_Data_t::GEAR_SHIFT_VALUE in px4ctrl:
        // gear=(PWM-1000)/1000 > 0.75 means command mode.
        const bool px4ctrl_command_mode = msg->channels[5] > 1750;
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
            if (automatic_landing_ && !px4ctrl_command_mode)
            {
                automatic_landing_ = false;
                ROS_WARN("RC channel 6 left command mode during landing: automatic LAND "
                         "retry stopped; RC hover/manual landing control is active");
            }
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
            requestLanding(px4ctrl_command_mode);
        }
        else if (current == RC_EIGHT_MIDDLE && previous == RC_EIGHT_DOWN)
        {
            quadrotor_msgs::TakeoffLand command;
            command.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
            takeoff_land_pub_.publish(command);
            ROS_INFO("RC channel 8: takeoff");
        }
        else if (current == RC_EIGHT_UP)
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
    ros::Publisher planning_stop_pub_;
    ros::Publisher startcommand_pub_;
    ros::Publisher backcommand_pub_;
    ros::Publisher clicked_markers_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gimbal_done_sub_;
    ros::Subscriber startcommand_sub_;
    ros::Subscriber backcommand_sub_;
    ros::Subscriber rc_sub_;
    ros::Subscriber clicked_point_sub_;
    ros::Subscriber start_clicked_route_sub_;
    ros::Subscriber clear_clicked_route_sub_;
    ros::Subscriber fcu_sub_, extended_sub_, controller_sub_, ready_sub_, heartbeat_sub_, land_sub_;
    ros::Timer timer_;
    tf::TransformListener tf_listener_;

    std::vector<Waypoint> mission_waypoints_;
    std::vector<Waypoint> return_waypoints_;
    std::vector<Waypoint> active_waypoints_;
    std::vector<Waypoint> clicked_waypoints_;
    MissionState state_;
    std::size_t current_index_;

    Eigen::Vector3d odom_position_;
    Eigen::Vector3d odom_velocity_;
    bool odom_received_;
    bool mission_uses_gimbal_;
    bool clicked_route_started_;
    bool enable_rviz_click_route_;
    bool clicked_point_use_z_;
    std::size_t last_clicked_marker_count_ = 0;

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
    double clicked_point_height_;
    int clicked_max_points_;
    std::string clicked_point_topic_;
    int enable_start_trigger_;
    int enable_back_trigger_;
    bool enable_rc_;
    std::string mission_source_, controller_state_;
    bool enable_gimbal_actions_, require_flight_ready_;
    bool controller_ready_ = false;
    bool mission_fault_ = false;
    bool gimbal_mission_consumed_ = false;
    uint8_t landed_state_ = mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED;
    mavros_msgs::State fcu_state_;
    ros::WallTime odom_receive_time_, fcu_receive_time_, extended_receive_time_;
    ros::WallTime controller_receive_time_, ready_receive_time_, heartbeat_receive_time_;
    ros::WallTime goal_publish_time_, gimbal_wait_time_;
    ros::Time odom_stamp_;
    double odom_timeout_sec_, flight_state_timeout_sec_, waypoint_timeout_sec_, gimbal_timeout_sec_;

    RC_EIGHT_STATE rc_eight_pre_;
    bool rc_initialized_;
    bool landing_latched_;
    bool automatic_landing_;
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
