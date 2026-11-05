// =============================================================================
//  multipointplan.cpp — 多点航线任务节点（A8 mini 云台版）
// -----------------------------------------------------------------------------
//  作用：
//    把"一组航点"变成实际飞行指令。按顺序把每个航点发布给 Diff-Planner
//    （/goal），等飞机稳定到达后按需驱动 A8 mini 云台，再推向下一个点。
//
//  三种任务来源（参数 mission_source）：
//    clicked — 在 RViz 里点选航点，攒成航线后一次性执行
//    preset  — 从 YAML（points.yaml）读取预设航点，可选 test_back 返航航线
//    remote  — 由地面站通过 ROS Service /mission/remote/command 下发航线
//
//  守卫分档（2026-11-05 起）：
//    L0 记录  —— 只记日志/遥测，不改变飞行。用于载荷类（云台动作超时）。
//    L1 宽限  —— 持续失效超过阈值才中止（mission_guard::sustained）。用于地图/围栏
//                校验、航点超时（先重发一次）。降落阶段对地图校验整体豁免。
//    L3 立即  —— 飞机已不在可控状态：飞控掉出 OFFBOARD、里程计失效、轨迹输出闩锁、
//                起飞未确认、降落被打断、走廊被占。保持一次即中止，不设宽限。
//
//  贯穿全文的安全设计：
//    1) 任何"飞机是否可飞"的判断都要求数据新鲜（fresh()），任一路超时即中止；
//    2) 异常统一走 abortMission()：锁定 mission_fault_，必须落地重启整套系统，
//       不允许带故障继续飞；
//    3) 降落是"闩锁"（landing_latched_）语义：一旦请求降落，就不再接受新的
//       起飞/任务指令，避免误触发；
//    4) 自动起降前会校验垂直走廊（地面点 → 悬停点）在栅栏内且无障碍。
//
//  主要接口：
//    发布 /goal                        几何目标点，供 Diff-Planner 规划
//    发布 /mission/gimbal_task         云台任务（Float64MultiArray，7 个字段）
//    发布 /px4ctrl/takeoff_land        起降指令（TAKEOFF / LAND）
//    发布 /planning/stop               停止规划（降落或中止时）
//    发布 /mission/clicked_waypoints   RViz 航点可视化标记
//    订阅 /mavros/state                飞控连接 / 解锁 / 模式
//    订阅 /mavros/extended_state       着陆状态
//    订阅 /px4ctrl/state               控制器状态机当前状态
//    订阅 /drone_0_traj_server/heartbeat  规划器心跳
//    订阅 odom_topic                   世界系里程计
//    服务 /mission/remote/command      地面站远程指令（仅 remote 模式）
// =============================================================================
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <multipoint/MissionCommand.h>
#include <multipoint/MissionStatus.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <std_msgs/Float64.h>
#include <ros/serialization.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt32.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <thread>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "mission_guard.h"
#include "remote_mission.h"

// 航线执行状态机（描述"航线推进到哪一步"，与飞机是否在空中无关）：
//   FLYING         —— 已发布目标点，等待到达（位置与速度都在容差内，
//                     并持续 arrival_stable_sec_ 才算真的到）
//   HOVERING       —— 已确认到点，正在消耗该航点的 hover_sec 悬停时间
//   WAITING_GIMBAL —— 云台动作已下发，等待 /mission/gimbal_done 回报
//   FINISHED       —— 本航线结束（飞机可能仍在空中悬停，也可能已触发降落）
enum MissionState
{
    FLYING,
    HOVERING,
    WAITING_GIMBAL,
    FINISHED
};

// 云台动作模式：
//   GIMBAL_ANGLE —— 指向固定角度（gimbal_yaw_deg / gimbal_pitch_deg）
//   GIMBAL_RANGE —— 在 [gimbal_yaw_min_deg, gimbal_yaw_max_deg] 区间内扫描
//                   （YAML 里写 "range" 或 "sweep" 都会映射到这里）
enum GimbalMode
{
    GIMBAL_ANGLE = 0,
    GIMBAL_RANGE = 1
};

// 遥控器第 8 通道（拨杆）三档的标称 PWM 值。
// 实际判定使用 ±100 的窗口（见 isInRcState），因此 899~1099 都会被认作 UP。
// 动作约定：UP=开始航线；MIDDLE 且从 DOWN 拨上来=起飞；
//           MIDDLE 且从 UP 拨下来=返航；DOWN=降落。
enum RC_EIGHT_STATE
{
    RC_EIGHT_UP = 999,
    RC_EIGHT_MIDDLE = 1499,
    RC_EIGHT_DOWN = 1999
};

// 单个航点：位置 + 悬停时长 + 云台动作参数。
// 坐标系固定为 goal_frame_id_（本项目强制为 "world"）；单位：米 / 秒 / 度。
// run_gimbal 为 false 时，该航点只飞过去、不动云台。
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

// 多点航线任务执行器（进程内唯一实例，由 main 里的 ros::spin 驱动）。
//
// 数据流：
//   外部触发（RViz / 遥控器 / 地面站服务）
//     → startMission() 建立活动航线
//     → timerCallback() 每 50 ms 推进状态机
//     → publishCurrentGoal() 把下一个航点交给 Diff-Planner
//     → 到点后按需 publishGimbalTask()，收到 gimbal_done 后继续
//     → 全部飞完：结束，或（自动往返模式）请求自动降落
class MultipointPlanner
{
public:
    // 构造流程：读参数 → 严格校验 → 加载航点 / 建立通信 → 启动定时器。
    // 参数非法时直接抛异常，由 main 打印 ROS_FATAL 后退出：
    // 宁可起不来，也不带着错误配置起飞。
    ~MultipointPlanner()
    {
        corridor_stop_ = true;
        if (corridor_thread_.joinable()) corridor_thread_.join();
    }

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
        // ---- 基础参数（私有命名空间 ~） ------------------------------------
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
        pnh_.param("automatic_roundtrip", automatic_roundtrip_, false);
        pnh_.param("takeoff_timeout_sec", takeoff_timeout_sec_, 35.0);
        pnh_.param("landing_timeout_sec", landing_timeout_sec_, 60.0);
        if (!finite(takeoff_timeout_sec_) || takeoff_timeout_sec_ <= 0 ||
            !finite(landing_timeout_sec_) || landing_timeout_sec_ <= 0)
            throw std::runtime_error("Invalid automatic flight timeouts");
        pnh_.param("require_flight_ready", require_flight_ready_, true);
        pnh_.param("require_map_coordinates", require_map_coordinates_, mission_source_ == "remote");
        // 地图/围栏校验连续失效多久才中止；0 表示一次失效即中止（旧行为）。
        pnh_.param("map_gate_debounce_sec", map_gate_debounce_sec_, 1.5);
        // 地图就绪标志：remote 模式下必须为 true 且新鲜，才允许上传/开始任务，
        // 用来确认点云坐标系与栅栏已经建立、航点坐标和世界系对得上。
        map_ready_sub_ = nh_.subscribe<std_msgs::Bool>("/indoor_mission/map_ready", 1,
                                                       [this](const std_msgs::Bool::ConstPtr& msg)
                                                       {
                                                           map_ready_ = msg->data;
                                                           map_ready_time_ = ros::WallTime::now();
                                                       });
        pnh_.param("odom_timeout_sec", odom_timeout_sec_, 0.5);
        pnh_.param("odom_future_tolerance_sec", odom_future_tolerance_sec_, 0.02);
        pnh_.param("flight_state_timeout_sec", flight_state_timeout_sec_, 2.5);
        pnh_.param("waypoint_timeout_sec", waypoint_timeout_sec_, 120.0);
        pnh_.param("gimbal_timeout_sec", gimbal_timeout_sec_, 90.0);
        pnh_.param("remote_max_abs_xy", remote_mission_.max_abs_xy, 30.0);
        pnh_.param("remote_min_z", remote_mission_.min_z, 0.3);
        pnh_.param("remote_max_z", remote_mission_.max_z, 2.5);
        pnh_.param("remote_reset_jump_m", remote_reset_jump_m_, 5.0);
        remote_mission_.validateLimits();
        if (mission_source_ == "remote") remote_mission_.session = newMapSession();
        if ((mission_source_ != "clicked" && mission_source_ != "preset" &&
                mission_source_ != "remote") ||
            goal_frame_id_ != "world" ||
            (mission_source_ == "clicked" && !enable_rviz_click_route_) ||
            !finite(odom_timeout_sec_) || odom_timeout_sec_ <= 0.0 ||
            !finite(odom_future_tolerance_sec_) || odom_future_tolerance_sec_ < 0.0 ||
            odom_future_tolerance_sec_ > odom_timeout_sec_ ||
            !finite(flight_state_timeout_sec_) || flight_state_timeout_sec_ <= 0.0 ||
            !finite(waypoint_timeout_sec_) || waypoint_timeout_sec_ <= 0.0 ||
            !finite(gimbal_timeout_sec_) || gimbal_timeout_sec_ <= 0.0 ||
            !finite(remote_reset_jump_m_) || remote_reset_jump_m_ <= 0.0 ||
            !finite(map_gate_debounce_sec_) || map_gate_debounce_sec_ < 0.0)
        {
            throw std::runtime_error("Invalid mission source/frame/timeout: use clicked|preset|remote and world");
        }

        if (position_tolerance_ <= 0.0 || velocity_tolerance_ < 0.0 ||
            arrival_stable_sec_ < 0.0 || gimbal_retry_sec_ <= 0.0 ||
            land_command_retry_sec_ <= 0.0 || clicked_max_points_ <= 0 ||
            !finite(clicked_point_height_))
        {
            throw std::runtime_error("Invalid mission tolerance or timing parameter");
        }

        // Remote routes do not depend on a preset YAML or payload configuration.
        if (mission_source_ != "remote") loadWaypoints();
        else enable_gimbal_actions_ = false;

        // ---- 发布者 --------------------------------------------------------
        point_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/goal", 10);
        gimbal_task_pub_ =
            nh_.advertise<std_msgs::Float64MultiArray>("/mission/gimbal_task", 10);
        takeoff_land_pub_ =
            nh_.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 10);
        planning_stop_pub_ = nh_.advertise<std_msgs::Empty>("/planning/stop", 1, true);
        planning_resume_pub_ = nh_.advertise<std_msgs::Empty>("/planning/resume", 1);
        startcommand_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        backcommand_pub_ =
            nh_.advertise<geometry_msgs::PoseStamped>("/back_trigger", 10);
        clicked_markers_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("/mission/clicked_waypoints", 1,
                                                           true);

        // ---- 订阅：飞控 / 控制器 / 规划器状态 -------------------------------
        // 这些数据都是安全门的输入，任一路超时都会导致任务中止。
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
        output_ready_sub_ = nh_.subscribe<std_msgs::Bool>("/drone_0_traj_server/output_ready", 1,
                                                          [this](const std_msgs::Bool::ConstPtr& m)
                                                          {
                                                              output_ready_ = m->data;
                                                              output_ready_time_ = ros::WallTime::now();
                                                          });
        land_sub_ = nh_.subscribe("/mission/land", 1,
                                  &MultipointPlanner::landCallback, this);
        gimbal_done_sub_ = nh_.subscribe("/mission/gimbal_done", 10,
                                         &MultipointPlanner::gimbalDoneCallback, this);
        // ---- 可选触发源（由参数决定是否订阅） -------------------------------
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
        if (enable_rviz_click_route_ && mission_source_ != "remote")
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

        // ---- 远程任务接口（仅 mission_source:=remote） ----------------------
        if (mission_source_ == "remote")
        {
            remote_status_pub_ = nh_.advertise<multipoint::MissionStatus>(
                "/mission/remote/status", 1, true);
            remote_command_srv_ = nh_.advertiseService("/mission/remote/command",
                                                       &MultipointPlanner::remoteCommandCallback, this);
            publishRemoteStatus();
        }

        // ---- 自动往返（自动起飞 + 自动降落）所需的状态输入 ------------------
        abort_automatic_pub_ = nh_.advertise<std_msgs::Empty>("/px4ctrl/abort_automatic", 1);
        takeoff_ready_sub_ = nh_.subscribe<std_msgs::Bool>("/px4ctrl/takeoff_ready", 1,
                                                           [this](const std_msgs::Bool::ConstPtr& m)
                                                           {
                                                               takeoff_ready_ = m->data;
                                                               takeoff_ready_time_ = ros::WallTime::now();
                                                           });
        auto_control_sub_ = nh_.subscribe<std_msgs::Bool>("/px4ctrl/automatic_control_ok", 1,
                                                          [this](const std_msgs::Bool::ConstPtr& m)
                                                          {
                                                              auto_control_ok_ = m->data;
                                                              auto_control_time_ = ros::WallTime::now();
                                                          });
        takeoff_height_sub_ = nh_.subscribe<std_msgs::Float64>("/px4ctrl/takeoff_height", 1,
                                                               [this](const std_msgs::Float64::ConstPtr& m)
                                                               {
                                                                   takeoff_height_ = m->data;
                                                                   takeoff_height_time_ = ros::WallTime::now();
                                                               });
        // 主循环：50 ms 一次，负责状态机推进、各项超时检查与降落指令重发。
        timer_ = nh_.createTimer(ros::Duration(0.05), &MultipointPlanner::timerCallback, this);

        ROS_INFO("Mission ready: %zu YAML waypoint(s), position tolerance %.2f m, "
                 "velocity tolerance %.2f m/s, RViz click route %s",
                 mission_waypoints_.size(), position_tolerance_, velocity_tolerance_,
                 enable_rviz_click_route_ ? "enabled" : "disabled");
        ROS_INFO("Mission configuration: source=%s gimbal_actions=%s flight_gate=%s",
                 mission_source_.c_str(), enable_gimbal_actions_ ? "true" : "false",
                 require_flight_ready_ ? "true" : "false");

        ROS_INFO("Map gate: require_map=%s debounce=%.2f s (landing phase exempt)",
                 require_map_coordinates_ ? "true" : "false", map_gate_debounce_sec_);
    }

private:
    // 地图坐标是否可用：preset/clicked 模式默认不强制（require_map_coordinates_=false）；
    // remote 模式要求 /indoor_mission/map_ready 为 true 且在 0.7 s 内刷新过。
    bool mapCoordinatesReady() const
    {
        return !require_map_coordinates_ || (map_ready_ && fresh(map_ready_time_, 0.7));
    }

    // 生成"地图会话 ID"：纳秒时间戳 + 随机数。
    // 地面站用它识别飞机侧地图是否被重建过；会话一变，旧航线全部作废。
    static std::string newMapSession()
    {
        std::ostringstream session;
        session << std::hex << ros::WallTime::now().toNSec() << "-" << std::random_device{}();
        return session.str();
    }

    // 远程任务能否开始：地图坐标就绪、未闩锁降落、无故障、里程计新鲜、
    // 飞行状态合格、规划器心跳新鲜，且 /goal 至少有一个订阅者。
    bool remoteStartReady() const
    {
        return mapCoordinatesReady() && !landing_latched_ && !mission_fault_ && odomFresh() && flightReady(true) &&
            fresh(heartbeat_receive_time_, 0.5) && point_pub_.getNumSubscribers() > 0;
    }

    // 组装回传给地面站的任务状态快照（服务响应与状态话题共用同一份内容）。
    multipoint::MissionStatus remoteStatus() const
    {
        multipoint::MissionStatus status;
        status.automatic_supported = automatic_roundtrip_;
        status.automatic = uploaded_automatic_;
        status.takeoff_ready = automaticStartReady();
        status.takeoff_height = fresh(takeoff_height_time_, .5) && finite(takeoff_height_) ? takeoff_height_ : 0.0;
        status.frame_id = goal_frame_id_;
        status.map_session_id = remote_mission_.session;
        status.mission_id = remote_mission_.id;
        status.revision = remote_mission_.revision;
        status.state = remote_mission_.state;
        status.completed = remote_mission_.completed;
        status.total = remote_mission_.points.size();
        status.current_waypoint_id = remote_mission_.active() && status.completed < status.total
                                         ? remote_mission_.points[status.completed].id
                                         : 0;
        status.reason = remote_mission_.reason;
        status.odom_fresh = odomFresh();
        status.flight_ready = remoteStartReady();
        status.world_x = odom_received_ ? odom_position_.x() : 0.0;
        status.world_y = odom_received_ ? odom_position_.y() : 0.0;
        status.world_z = odom_received_ ? odom_position_.z() : 0.0;
        status.max_abs_xy = remote_mission_.max_abs_xy;
        status.min_z = remote_mission_.min_z;
        status.max_z = remote_mission_.max_z;
        return status;
    }

    // 发布远程状态；非 remote 模式直接返回。调用方负责节流。
    void publishRemoteStatus()
    {
        if (mission_source_ != "remote") return;
        remote_status_pub_.publish(remoteStatus());
        remote_status_time_ = ros::WallTime::now();
    }

    // 把地面站下发的 remote 航点转换成统一的 Waypoint 结构。
    // 远程航线不驱动云台（run_gimbal=false），只做点到点飞行。
    std::vector<Waypoint> remoteWaypoints() const
    {
        std::vector<Waypoint> route;
        for (const auto& point : remote_mission_.points)
        {
            Waypoint waypoint{};
            waypoint.id = point.id;
            waypoint.x = point.x;
            waypoint.y = point.y;
            waypoint.z = point.z;
            waypoint.hover_sec = point.hover_sec;
            waypoint.gimbal_mode = GIMBAL_ANGLE;
            waypoint.run_gimbal = false;
            route.push_back(waypoint);
        }
        return route;
    }

    // 自动飞行期间所有输入是否健全（起飞与降落两个阶段共用）。
    // 地图/围栏不在这里判：它由 mapGateAllows() 统一带 L1 宽限处理，
    // 否则同一个条件会在两处以不同严格度出现（旧代码就是这样漏掉起飞宽限的）。
    bool automaticInputsReady() const
    {
        return odomFresh() && fresh(heartbeat_receive_time_, .5) &&
            point_pub_.getNumSubscribers() > 0 && fresh(fcu_receive_time_, flight_state_timeout_sec_) &&
            fresh(extended_receive_time_, flight_state_timeout_sec_) && fcu_state_.connected &&
            fresh(controller_receive_time_, .5) && fresh(auto_control_time_, .5) && auto_control_ok_;
    }

    // 规划器轨迹输出是否就绪：output_ready 为 true 且 0.3 s 内刷新过。
    // 起飞过程中它是硬性条件——输出一旦被 stop 闩锁就必须中止。
    bool trajectoryOutputReady() const
    {
        return output_ready_ && fresh(output_ready_time_, .3);
    }

    // 自动起飞的全部前置条件：地面停机、未解锁、控制器处于 MANUAL_CTRL、
    // 起飞高度已知、速度接近静止、起降指令有订阅者等。
    bool automaticStartReady() const
    {
        return automatic_roundtrip_ && trajectoryOutputReady() && mapCoordinatesReady() &&
            !landing_latched_ && !mission_fault_ && automaticInputsReady() &&
            !fcu_state_.armed && landed_state_ == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND &&
            controller_state_ == "MANUAL_CTRL" && fresh(takeoff_ready_time_, .5) && takeoff_ready_ &&
            fresh(takeoff_height_time_, .5) && finite(takeoff_height_) && takeoff_height_ > .1 &&
            odom_velocity_.norm() <= .1 && takeoff_land_pub_.getNumSubscribers() > 0;
    }

    // Endpoint-only check; this does not check the complete vertical segment.
    // 走廊检查放到后台线程：只有这个线程会阻塞在服务调用上，控制回路只读结果。
    void ensureCorridorWorker()
    {
        if (corridor_thread_.joinable()) return;
        corridor_thread_ = std::thread([this]() { corridorWorker(); });
    }

    void setCorridorTargets(const Eigen::Vector3d& ground, const Eigen::Vector3d& hover)
    {
        std::lock_guard<std::mutex> guard(corridor_target_mutex_);
        corridor_ground_ = ground;
        corridor_hover_ = hover;
        corridor_active_ = true;
    }

    void corridorWorker()
    {
        ros::ServiceClient check =
            nh_.serviceClient<nav_msgs::GetPlan>("/indoor_mission/check_endpoints");
        bool last_ok = true;
        while (ros::ok() && !corridor_stop_)
        {
            if (!corridor_active_)
            {
                ros::WallDuration(.05).sleep();
                continue;
            }
            Eigen::Vector3d ground, hover;
            {
                std::lock_guard<std::mutex> guard(corridor_target_mutex_);
                ground = corridor_ground_;
                hover = corridor_hover_;
            }
            const bool ok = queryCorridorClear(check, ground, hover, "periodic");
            if (ok && !last_ok)
                ROS_INFO("Corridor check recovered: both endpoints clear again");
            last_ok = ok;
            corridor_ok_ = ok;
            corridor_result_wall_ = ros::WallTime::now().toSec();
            ros::WallDuration(.2).sleep();
        }
    }

    // 结果超过 1.5 s 没更新，按"监控失效"处理并中止——宁可停下，也不要
    // 带着一个不响应的走廊监控继续下降。
    bool corridorHealthy()
    {
        const double stamp = corridor_result_wall_.load();
        if (stamp <= 0.0) return true;                 // 本阶段还没查过
        const double age = ros::WallTime::now().toSec() - stamp;
        if (age > 1.5)
        {
            ROS_WARN_THROTTLE(1.0, "Corridor check service has not answered for %.1f s", age);
            return false;
        }
        return corridor_ok_.load();
    }

    // 走廊检查的统一实现：/indoor_mission/check_endpoints 只返回"起点+终点"两个点，
    // 才说明两端都在已知、无占据、无膨胀且在本围栏内。
    // 失败时把判定依据全部打出来（服务是否应答、返回点数、耗时、两个端点坐标），
    // 否则线上只能看到一句 corridor is no longer clear，无法定位是哪一端出了问题。
    bool queryCorridorClear(ros::ServiceClient& client, const Eigen::Vector3d& ground,
                            const Eigen::Vector3d& hover, const char* where)
    {
        nav_msgs::GetPlan query;
        query.request.start.header.frame_id = query.request.goal.header.frame_id = "world";
        query.request.start.pose.orientation.w = query.request.goal.pose.orientation.w = 1.;
        query.request.start.pose.position.x = ground.x();
        query.request.start.pose.position.y = ground.y();
        query.request.start.pose.position.z = ground.z();
        query.request.goal.pose.position.x = hover.x();
        query.request.goal.pose.position.y = hover.y();
        query.request.goal.pose.position.z = hover.z();
        const double started = ros::WallTime::now().toSec();
        const bool called = client.call(query);
        const double elapsed_ms = (ros::WallTime::now().toSec() - started) * 1000.0;
        const auto poses = query.response.plan.poses.size();
        const bool clear = called && poses == 2;
        if (!clear)
            ROS_WARN_THROTTLE(1.0,
                              "Corridor check failed (%s): call_ok=%s poses=%zu took=%.1f ms "
                              "ground=(%.2f,%.2f,%.2f) hover=(%.2f,%.2f,%.2f)",
                              where, called ? "true" : "false", poses, elapsed_ms,
                              ground.x(), ground.y(), ground.z(),
                              hover.x(), hover.y(), hover.z());
        return clear;
    }

    bool verticalCorridorEndpointsClear(const Eigen::Vector3d& ground, const Eigen::Vector3d& hover)
    {
        auto check = nh_.serviceClient<nav_msgs::GetPlan>("/indoor_mission/check_endpoints");
        return queryCorridorClear(check, ground, hover, "upload");
    }

    // 自动起飞第一步：确认地面点未漂移、起飞高度未变、垂直走廊通畅，
    // 然后把状态切到 TAKING_OFF，并只发一次 TAKEOFF 指令（不重发）。
    // 后续由 tickAutomaticFlight() 跟踪进度，失败即中止并要求飞行员接管。
    void beginAutomaticTakeoff()
    {
        if (!automaticStartReady() || !uploaded_automatic_ ||
            (odom_position_ - ground_origin_).norm() > .10)
            throw std::runtime_error(
                "Automatic start requires unchanged ground position, fresh preflight/RC and disarmed FCU");
        const auto& home = remote_mission_.points.back();
        const Eigen::Vector3d hover(home.x, home.y, home.z);
        if (std::abs(home.z - ground_origin_.z() - takeoff_height_) > .03 ||
            !verticalCorridorEndpointsClear(ground_origin_, hover))
            throw std::runtime_error(
                "Takeoff height changed or vertical takeoff/landing endpoints are occupied, unknown or outside fence");
        remote_mission_.state = "TAKING_OFF";
        automatic_phase_time_ = ros::WallTime::now();
        last_corridor_check_ = ros::WallTime();
        corridor_result_wall_ = ros::WallTime::now().toSec();
        corridor_ok_ = true;
        auto_hover_since_ = ros::WallTime();
        takeoff_seen_ = false;
        quadrotor_msgs::TakeoffLand command;
        command.takeoff_land_cmd = quadrotor_msgs::TakeoffLand::TAKEOFF;
        takeoff_land_pub_.publish(command); // One shot only, after explicit start.
    }

    // 自动起飞 / 自动降落阶段的推进器（由 timerCallback 每 50 ms 调用）。
    // 负责：判定"已落地并上锁"→ COMPLETED；检查轨迹输出、输入新鲜度与阶段超时；
    //       周期性复检垂直走廊；起飞阶段确认控制器真的起飞了，
    //       落地阶段确认没有被人工打断；条件满足后自动启动航线。
    void tickAutomaticFlight()
    {
        if (!uploaded_automatic_ || (remote_mission_.state != "TAKING_OFF" && remote_mission_.state != "LANDING"))
            return;
        const bool taking_off = remote_mission_.state == "TAKING_OFF";

        // 已触地：px4ctrl 的正常收尾是"先切 MANUAL_CTRL、再退 OFFBOARD"，
        // 而 /mavros/state 里的 armed/mode 要等飞控心跳回传（几十 ms ~ 1 s）。
        // 这段时间只能按"正在收尾"处理，绝不能判成中断，否则正常降落会被误报。
        // 注意：ON_GROUND 与 px4ctrl 读的是同一条 /mavros/extended_state，
        // 所以本节点判定"已触地"一定早于 px4ctrl 切 MANUAL_CTRL。
        const bool on_ground = !taking_off &&
            fresh(extended_receive_time_, flight_state_timeout_sec_) &&
            landed_state_ == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
        if (on_ground)
        {
            if (landing_contact_time_.isZero())
                landing_contact_time_ = ros::WallTime::now();

            if (fresh(fcu_receive_time_, flight_state_timeout_sec_) &&
                fcu_state_.connected && !fcu_state_.armed)
            {
                automatic_landing_ = false;
                remote_mission_.state = "COMPLETED";
                remote_mission_.reason = "Returned, landed and disarmed";
                publishRemoteStatus();
                return;
            }
            if ((ros::WallTime::now() - landing_contact_time_).toSec() > 5.0)
            {
                // 不调用 abortMission()：那会把 px4ctrl 从 AUTO_LAND 拉回 AUTO_HOVER，
                // 在"已触地但仍上锁"时重新接管姿态，比继续等待更危险。
                // px4ctrl 每 1 s 会自己重试 disarm，这里只告警。
                ROS_ERROR_THROTTLE(2.0,
                    "Landed but FCU has not disarmed for %.1fs; px4ctrl keeps retrying, "
                    "disarm manually if it persists",
                    (ros::WallTime::now() - landing_contact_time_).toSec());
            }
            return;   // 触地后等待解锁，跳过后面的"中断"判定
        }


        // Landing intentionally stops trajectory output. Takeoff requires it alive
        // and unlatched, including while waiting for OFFBOARD/arming feedback.
        if (taking_off && !trajectoryOutputReady())
        {
            abortMission("Trajectory output unavailable or stop-latched during takeoff");
            return;
        }
        const double age = (ros::WallTime::now() - automatic_phase_time_).toSec();
        if (!automaticInputsReady() || age > (taking_off ? takeoff_timeout_sec_ : landing_timeout_sec_))
        {
            abortMission("Automatic phase inputs lost or timeout; pilot takeover required");
            return;
        }
        const auto& return_point = remote_mission_.points.back();
        // 走廊检查移到后台线程（见 corridorWorker）：这个服务由 diff_planner 提供，
        // 在 50 ms 控制回路里同步调用会把里程计回调一起阻塞住。
        ensureCorridorWorker();
        setCorridorTargets(ground_origin_,
                           Eigen::Vector3d(return_point.x, return_point.y, return_point.z));
        if (!corridorHealthy())
        {
            abortMission("Vertical takeoff/landing corridor is no longer clear");
            return;
        }
        // 自动降落阶段
        if (!taking_off)
        {
            if (!fcu_state_.armed || fcu_state_.mode != "OFFBOARD" ||
                (controller_state_ != "AUTO_LAND" && controller_state_ != "AUTO_HOVER" && controller_state_ !=
                    "CMD_CTRL"))
            {
                abortMission("自动降落已中断，请飞手立即接管飞机");
                return;
            }
            return;
        }
        const bool preparing = controller_state_ == "PREPARING_TAKEOFF";
        if (preparing || controller_state_ == "AUTO_TAKEOFF" || controller_state_ == "AUTO_HOVER") takeoff_seen_ = true;
        // The controller waits at most 2.5 s for mode feedback; allow one further
        // normal FCU heartbeat for arming feedback. Input/corridor guards stay active.
        if ((takeoff_seen_ && !preparing && controller_state_ != "AUTO_TAKEOFF" && controller_state_ != "AUTO_HOVER") ||
            (!takeoff_seen_ && age > 2.) || (age > 4. && (!fcu_state_.armed || fcu_state_.mode != "OFFBOARD")))
        {
            abortMission("Takeoff rejected or interrupted; pilot takeover required");
            return;
        }
        // Start the route after airborne readiness and low-speed stability.
        // Proximity to the recorded return point is not a takeoff-completion gate.
        if (flightReady(true) && odom_velocity_.norm() <= .1)
        {
            if (auto_hover_since_.isZero()) auto_hover_since_ = ros::WallTime::now();
            if ((ros::WallTime::now() - auto_hover_since_).toSec() >= 1.)
            {
                if (!startMission(remoteWaypoints(), false)) abortMission("Airborne route start rejected");
            }
        }
        else auto_hover_since_ = ros::WallTime();
    }

    // 开始远程任务：先校验 session/mission/revision 三元组是否匹配，
    // 自动往返模式走自动起飞流程，否则要求飞机已处于合格的悬停状态。
    void startRemoteMission()
    {
        remote_mission_.checkStart(remote_mission_.session, remote_mission_.id,
                                   remote_mission_.revision);
        if (automatic_roundtrip_)
        {
            beginAutomaticTakeoff();
            return;
        }
        if (!startMission(remoteWaypoints(), false))
            throw std::runtime_error(
                "Start rejected: confirm airborne hover, fresh world odometry, controller and planner");
    }

    // 远程指令的实际执行逻辑（upload / start / update / cancel / get_status）。
    // 抛出异常即代表拒绝，由 remoteCommandCallback 转成 accepted=false + reason。
    void applyRemoteCommand(const multipoint::MissionCommand::Request& request)
    {
        if (request.operation == "get_status") return;
        if (request.operation == "cancel")
        {
            remote_mission_.requireIdentity(request.map_session_id, request.mission_id);
            if (request.revision != remote_mission_.revision)
                throw std::runtime_error("Cancel requires current revision");
            if (remote_mission_.active()) abortMission("Cancelled by ground station; pilot takeover required");
            else if (remote_mission_.state == "READY") invalidateRemoteSession("Draft mission cancelled");
            return;
        }
        if (request.operation == "clear")
        {
            // 地面清理：解除上一次失败/降落闩锁并把机载任务复位到 IDLE。
            // 这是"落地重启整套系统"之外唯一的复位途径，所以条件比 cancel 更严：
            // 必须收到新鲜的 FCU 状态、已连接、未解锁、且扩展状态明确在地面。
            if (!fresh(fcu_receive_time_, flight_state_timeout_sec_) || !fcu_state_.connected ||
                fcu_state_.armed ||
                landed_state_ != mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
                throw std::runtime_error(
                    "Clear requires a connected, disarmed aircraft confirmed on the ground");
            if (state_ != FINISHED || remote_mission_.active())
                throw std::runtime_error("Clear requires the mission to be stopped");
            const bool had_fault = mission_fault_;
            remote_mission_.clear();
            mission_fault_ = false;
            landing_latched_ = false;
            automatic_landing_ = false;
            uploaded_automatic_ = false;
            state_ = FINISHED;
            // 同时解除 traj_server 的 stop 闩锁；它自己还会再查一次
            // 未解锁 + 在地面，空中收到会拒绝。
            planning_resume_pub_.publish(std_msgs::Empty());
            publishRemoteStatus();
            ROS_WARN("Remote mission cleared on the ground (previous fault=%s); trajectory output reset requested",
                     had_fault ? "true" : "false");
            return;
        }
        if (landing_latched_ || mission_fault_)
            throw std::runtime_error("Landing/failure is latched; land and restart the stack");
        if (!odomFresh()) throw std::runtime_error("Fresh world odometry is required");
        if (!mapCoordinatesReady()) throw std::runtime_error("Point-cloud coordinates or planner fence not verified");
        if (request.operation == "start")
        {
            remote_mission_.checkStart(request.map_session_id, request.mission_id, request.revision);
            startRemoteMission();
            return;
        }
        if (request.operation != "upload" && request.operation != "update")
            throw std::runtime_error("Unsupported operation; use upload, start, get_status or update");
        if (request.frame_id != goal_frame_id_)
            throw std::runtime_error("Route must use frame_id=world");
        std::vector<remote_mission::Point> route;
        for (const auto& p : request.waypoints)
            route.push_back({p.id, p.x, p.y, p.z, p.hover_sec});
        if (request.operation == "upload")
        {
            if (automatic_roundtrip_)
            {
                if (!request.automatic || route.size() != 2 || !automaticStartReady())
                    throw std::runtime_error(
                        "Automatic roundtrip requires exactly target/return points and ground preflight readiness");
                const Eigen::Vector3d ground(request.ground_origin.x, request.ground_origin.y, request.ground_origin.z);
                const auto& home = route.back();
                const Eigen::Vector3d hover(home.x, home.y, home.z);
                if (!ground.allFinite() || (ground - odom_position_).norm() > .1 ||
                    (hover.head<2>() - ground.head<2>()).norm() > .03 ||
                    std::abs(home.z - ground.z() - takeoff_height_) > .03 ||
                    std::hypot(route.front().x - home.x, route.front().y - home.y) < .3)
                    throw std::runtime_error(
                        "Invalid ground origin, return height or target too close");
                // 上传阶段不再校验走廊：规划器地图只覆盖已观测空间，未观测点会被判
                // unknown 而阻塞保存。起飞前（beginAutomaticTakeoff）和下降中仍然检查。
                ROS_INFO("Route uploaded without corridor pre-check; takeoff and descent will verify");
                remote_mission_.upload(request.map_session_id, request.mission_id, request.revision, route);
                ground_origin_ = ground;
                uploaded_automatic_ = true;
                return;
            }
            if (request.automatic) throw std::runtime_error("Automatic roundtrip profile is not enabled");
            remote_mission_.upload(request.map_session_id, request.mission_id, request.revision, route);
            return;
        }
        if (automatic_roundtrip_) throw std::runtime_error("Automatic roundtrip does not support route updates");
        if (remote_mission_.active() && (!flightReady(false) ||
            !fresh(heartbeat_receive_time_, 0.5) || point_pub_.getNumSubscribers() == 0))
            throw std::runtime_error("Cannot update while flight/controller/planner state is invalid");
        const bool active = remote_mission_.active();
        const bool retarget = remote_mission_.state == "FLYING" &&
            request.replace_from == current_index_;
        remote_mission_.update(request.map_session_id, request.mission_id, request.revision,
                               request.replace_from, route);
        if (active)
        {
            active_waypoints_ = remoteWaypoints();
            if (retarget) publishCurrentGoal(); // Diff-Planner replans from its current state.
        }
    }

    // 远程指令服务入口。
    // 关键约束是幂等：同一个 request_id 重发（TCP 重连、地面站重试）必须得到相同结果，
    // 绝不能因重发而多执行一次动作，所以这里缓存了请求的序列化字节与当时的响应。
    bool remoteCommandCallback(multipoint::MissionCommand::Request& request,
                               multipoint::MissionCommand::Response& response)
    {
        // Cache at the authoritative executor, including across TCP reconnects.
        // Reusing an ID with different content must never cause a second action.
        if (request.request_id.empty() || request.request_id.size() > 80 ||
            request.map_session_id.size() > 80 || request.mission_id.size() > 80 ||
            request.waypoints.size() > 200 ||
            ros::serialization::serializationLength(request) > 65536)
        {
            response.accepted = false;
            response.reason = "Invalid request ID or oversized command";
            response.mission = remoteStatus();
            return true;
        }
        std::vector<uint8_t> bytes(ros::serialization::serializationLength(request));
        ros::serialization::OStream stream(bytes.data(), bytes.size());
        ros::serialization::serialize(stream, request);
        // 幂等检查：命中相同 request_id 时，只有内容完全一致才复用当时的结论；
        // 同一 id 但内容不同视为客户端错误，绝不执行第二次。
        for (const auto& cached : remote_requests_)
        {
            if (cached.id != request.request_id) continue;
            response.accepted = cached.bytes == bytes && cached.accepted;
            response.reason = cached.bytes == bytes ? cached.reason : "request_id reused with different content";
            response.mission = remoteStatus(); // Never rewind the client's progress on a retry.
            return true;
        }
        try
        {
            applyRemoteCommand(request);
            response.accepted = true;
            response.reason = "Accepted by mission executor";
        }
        catch (const std::exception& error)
        {
            response.accepted = false;
            response.reason = error.what();
        }
        remote_requests_.push_back({request.request_id, bytes, response.accepted, response.reason});
        if (remote_requests_.size() > 256) remote_requests_.pop_front();
        response.mission = remoteStatus();
        ROS_INFO("Remote command %s: accepted=%s mission=%s revision=%u reason=%s",
                 request.operation.c_str(), response.accepted ? "true" : "false",
                 remote_mission_.id.c_str(), remote_mission_.revision, response.reason.c_str());
        publishRemoteStatus();
        return true;
    }

    // 作废当前远程会话（换一个新的 session id），并让进行中的任务中止。
    // 触发场景：里程计断流/跳变/时钟复位，或地面站主动取消尚未开始的草稿。
    void invalidateRemoteSession(const std::string& reason)
    {
        if (mission_source_ != "remote") return;
        // 注意：这里的中止不是重复代码——remote_mission_.invalidate() 会把状态
        // 置为 FAILED，之后就再也不会进入自动阶段的分支，别的守卫也就没机会
        // 触发。真正通知控制器停止的只有这一句。
        if (state_ != FINISHED || remote_mission_.active()) abortMission(reason.c_str());
        remote_had_fresh_odom_ = false;
        remote_mission_.invalidate(newMapSession(), reason);
        publishRemoteStatus();
    }

    // 是否为有限数（排除 NaN / inf）。所有来自参数与 YAML 的数值都要过这一关。
    static bool finite(double value)
    {
        return std::isfinite(value);
    }

    // 云台模式的可读名称，仅用于日志与 CSV 记录。
    static const char* gimbalModeName(GimbalMode mode)
    {
        return mode == GIMBAL_RANGE ? "range" : "angle";
    }

    // 航点合法性校验（YAML 与远程航线共用）：
    //   id 必须为正且唯一；所有字段必须有限；悬停/云台稳定时间非负；
    //   云台角度必须落在 A8 mini 手册给出的可控范围内。
    void validateWaypoint(const Waypoint& waypoint, std::set<uint32_t>* ids) const
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

    // 从 YAML 读取预设航线（preset 模式）。
    // 支持：enable_gimbal 总开关、waypoints 主航线、test_back 兼容旧版的返航航线。
    // 缺字段、超范围、id 重复都会在启动阶段直接报错，不会带着坏数据起飞。
    void loadWaypoints()
    {
        if (yaml_path_.empty())
        {
            throw std::runtime_error("The yaml_path parameter is empty");
        }

        YAML::Node root = YAML::LoadFile(yaml_path_);

        // If the YAML contains enable_gimbal, use it as the ground truth unless
        // the launch/rosparam already forced it off.  This lets points.yaml be
        // the single source of truth for gimbal configuration.
        if (root["enable_gimbal"])
        {
            const bool yaml_gimbal = root["enable_gimbal"].as<bool>();
            if (!yaml_gimbal)
            {
                enable_gimbal_actions_ = false;
                ROS_INFO("enable_gimbal is false in YAML; disabling gimbal actions");
            }
        }

        YAML::Node waypoints = root["waypoints"];
        if (!waypoints || !waypoints.IsSequence() || waypoints.size() == 0)
        {
            throw std::runtime_error("points.yaml must contain a non-empty 'waypoints' sequence");
        }

        std::set<uint32_t> ids;
        for (std::size_t i = 0; i < waypoints.size(); ++i)
        {
            const YAML::Node& node = waypoints[i];
            // Core navigation fields are always required.
            if (!node.IsMap() || !node["id"] || !node["x"] || !node["y"] || !node["z"])
            {
                std::ostringstream error;
                error << "Waypoint " << i << " is missing one or more required fields (id, x, y, z)";
                throw std::runtime_error(error.str());
            }
            // Gimbal-related fields are required only when the gimbal is enabled.
            if (enable_gimbal_actions_ &&
                (!node["hover_sec"] || !node["gimbal_pitch_deg"] ||
                    !node["gimbal_settle_sec"]))
            {
                std::ostringstream error;
                error << "Waypoint " << i << " is missing gimbal fields "
                    << "(hover_sec, gimbal_pitch_deg, gimbal_settle_sec) "
                    << "which are required when enable_gimbal is true";
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
            waypoint.hover_sec = node["hover_sec"] ? node["hover_sec"].as<double>() : 0.0;
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
            if (enable_gimbal_actions_ &&
                waypoint.gimbal_mode == GIMBAL_ANGLE && !node["gimbal_yaw_deg"])
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
            waypoint.gimbal_pitch_deg =
                node["gimbal_pitch_deg"] ? node["gimbal_pitch_deg"].as<double>() : 0.0;
            waypoint.gimbal_settle_sec =
                node["gimbal_settle_sec"] ? node["gimbal_settle_sec"].as<double>() : 0.0;
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

    // 里程计回调：保存世界系位置/速度，并记录接收时间供新鲜度判断。
    // remote 模式下额外做三项自检（断流、位置跳变、时间戳回退），
    // 任一成立就作废会话，避免用已经漂移的坐标系继续飞已上传的航线。
    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        const Eigen::Vector3d next(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
        if (mission_source_ == "remote" && odom_received_ && next.allFinite() &&
            ((remote_had_fresh_odom_ && !fresh(odom_receive_time_, odom_timeout_sec_)) ||
                (next - odom_position_).norm() > remote_reset_jump_m_ ||
                (!odom_stamp_.isZero() && (odom_stamp_ - msg->header.stamp).toSec() > 0.5)))
            invalidateRemoteSession("World odometry gap, jump or clock reset; recheck the map session");
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

    // ---- 状态回调：只记录快照与接收时间，不做业务判断 ---------------------
    // 真正的判断都在 fresh() / flightReady() 里按需做。
    // 飞控连接状态、解锁状态与飞行模式（mavros_msgs/State）
    void fcuCallback(const mavros_msgs::StateConstPtr& msg)
    {
        fcu_state_ = *msg;
        fcu_receive_time_ = ros::WallTime::now();
    }

    // 着陆状态（在地面 / 在空中 / 未知），用于判定"已落地"
    void extendedCallback(const mavros_msgs::ExtendedStateConstPtr& msg)
    {
        landed_state_ = msg->landed_state;
        extended_receive_time_ = ros::WallTime::now();
    }

    // px4ctrl 控制器状态机当前状态字符串（AUTO_HOVER / AUTO_TAKEOFF / AUTO_LAND ...）
    void controllerCallback(const std_msgs::StringConstPtr& msg)
    {
        controller_state_ = msg->data;
        controller_receive_time_ = ros::WallTime::now();
    }

    // 控制器是否已完成起飞准备（/px4ctrl/mission_ready）
    void readyCallback(const std_msgs::BoolConstPtr& msg)
    {
        controller_ready_ = msg->data;
        ready_receive_time_ = ros::WallTime::now();
    }

    // 规划器心跳：只要还有数据，就说明 traj_server 还活着
    void heartbeatCallback(const std_msgs::EmptyConstPtr&)
    {
        heartbeat_receive_time_ = ros::WallTime::now();
    }

    // "数据是否新鲜"的统一判定：时间戳非零，且距今不超过 timeout。
    bool fresh(const ros::WallTime& stamp, double timeout) const
    {
        return !stamp.isZero() && mission_guard::fresh(
            (ros::WallTime::now() - stamp).toSec(), timeout);
    }

    // 里程计双重校验：既要"最近收到过"（墙钟），也要"消息时间戳不超前、不滞后"（ROS 时间）。
    // 后者可以挡住时间源跳变造成的假新鲜。
    bool odomFresh() const
    {
        return odom_received_ && fresh(odom_receive_time_, odom_timeout_sec_) &&
            !odom_stamp_.isZero() && mission_guard::stampedFresh(
                (ros::Time::now() - odom_stamp_).toSec(), odom_timeout_sec_,
                odom_future_tolerance_sec_);
    }

    // 汇总飞行安全门：飞控/控制器状态新鲜、已连接、已解锁、OFFBOARD、
    // 处于空中、控制器状态可接受。
    // require_flight_ready_=false 只允许用于仿真启动，实机必须为 true。
    bool flightReady(bool starting) const
    {
        if (!require_flight_ready_)
            return true; // Only the simulator launch disables this gate.
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

    // 统一的异常终止：打印原因 → 锁定 mission_fault_ → 停止规划 → 关闭 CSV →
    // remote 模式下把状态置为 FAILED。
    // 锁定后不再接受任何新任务，必须落地并重启整套系统（这是刻意的安全设计）。
    void abortMission(const char* reason)
    {
        ROS_ERROR("Mission stopped: %s. Check aircraft/RC; land and restart the stack before a new mission.", reason);
        state_ = FINISHED;
        mission_fault_ = true;
        automatic_landing_ = false;
        if (uploaded_automatic_) abort_automatic_pub_.publish(std_msgs::Empty());
        arrival_candidate_time_ = ros::Time(0);
        planning_stop_pub_.publish(std_msgs::Empty());
        closeMissionCsv();
        if (mission_source_ == "remote")
        {
            remote_mission_.state = "FAILED";
            remote_mission_.reason = reason;
            publishRemoteStatus();
        }
    }

    // /move_base_simple/goal 触发：按当前 mission_source 分派到对应的启动路径。
    // 已闩锁降落、或已有任务在执行时，直接忽略。
    void startCallback(const geometry_msgs::PoseStamped::ConstPtr&)
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
        if (mission_source_ == "remote")
        {
            try { startRemoteMission(); }
            catch (const std::exception& error) { ROS_WARN("Remote start: %s", error.what()); }
            publishRemoteStatus();
            return;
        }
        if (mission_source_ == "clicked")
        {
            startClickedRoute();
            return;
        }
        startMission(mission_waypoints_, enable_gimbal_actions_);
    }

    // 由 RViz 点选坐标构造航点：高度默认取 clicked_point_height_，
    // 不驱动云台，悬停时间为 0。
    Waypoint makeClickedWaypoint(const geometry_msgs::Point& point, uint32_t id) const
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

    // RViz "Publish Point" 回调：把点变换到 world 系后追加到待飞航线并刷新可视化。
    // 会拒绝：航线已开始/已结束、超过点数上限、frame_id 缺失、坐标非有限。
    void clickedPointCallback(const geometry_msgs::PointStampedConstPtr& msg)
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
            catch (const tf::TransformException& error)
            {
                ROS_WARN("Cannot transform clicked point from '%s' to '%s': %s",
                         msg->header.frame_id.c_str(), goal_frame_id_.c_str(), error.what());
                return;
            }
        }

        const geometry_msgs::Point& point = point_in_goal_frame.point;
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

    // 外部（脚本或地面站）显式触发：开始执行已累积的 RViz 航线。
    void startClickedRouteCallback(const std_msgs::EmptyConstPtr&)
    {
        startClickedRoute();
    }

    // 真正开始点击航线：仅在 clicked 模式、已有点、未闩锁、状态空闲时成立。
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

    // 清空尚未执行的 RViz 航点（任务执行过程中不允许清空，需先落地）。
    void clearClickedRouteCallback(const std_msgs::EmptyConstPtr&)
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

    // 发布 RViz 可视化：航点点球、航线折线、编号文字，
    // 并清理上一轮留下的多余编号标记。
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
        points.action = clicked_waypoints_.empty()
                            ? visualization_msgs::Marker::DELETE
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
        line.action = clicked_waypoints_.size() < 2
                          ? visualization_msgs::Marker::DELETE
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

    // /back_trigger 触发：执行 YAML 中 test_back 定义的返航航线
    // （remote 模式没有预设返航线，需要走常规降落流程）。
    void backCallback(const geometry_msgs::PoseStamped::ConstPtr&)
    {
        if (mission_source_ == "remote")
        {
            ROS_WARN("Remote missions have no preset return route; use the existing landing/RC procedure");
            return;
        }
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

    // 启动一条航线的主入口（preset / clicked / remote 共用）。
    //
    // 先做完整安全检查：降落闩锁、故障锁、航线非空、里程计新鲜、飞行状态合格、
    // 规划器心跳、/goal 有订阅者；云台航线还要求云台话题有订阅者。
    // 任一条不满足就保留航线并返回 false，等用户确认悬停后再次触发——不静默失败。
    bool startMission(const std::vector<Waypoint>& waypoints, bool use_gimbal,
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

    // 把 active_waypoints_[current_index_] 发布到 /goal，并把状态置为 FLYING。
    // 这里也是"重新规划"的切换点：Diff-Planner 会从飞机当前状态规划到新目标。
    void publishCurrentGoal(bool retry = false)
    {
        const Waypoint& waypoint = active_waypoints_.at(current_index_);
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
        if (mission_source_ == "remote") remote_mission_.state = "FLYING";
        arrival_candidate_time_ = ros::Time(0);
        if (!retry) waypoint_retry_count_ = 0;
        ROS_INFO("Published waypoint %u goal: [x %.2f, y %.2f, z %.2f]",
                 waypoint.id, waypoint.x, waypoint.y, waypoint.z);
    }

    // 50 ms 主循环，执行顺序很重要：
    //   1) 维护远程状态与会话有效性
    //   2) 推进自动起降阶段
    //   3) 处理降落闩锁下的指令重发与"已落地"判定
    //   4) 逐项安全检查（地图、里程计、轨迹输出、飞行状态、规划器心跳），任一失败即中止
    //   5) 按状态机推进：FLYING → HOVERING →（WAITING_GIMBAL）→ 下一个航点
    // L1 地图门：降落阶段豁免；其余阶段连续失效超过 map_gate_debounce_sec_
    // 才中止。返回 false 表示已经中止任务。
    bool mapGateAllows()
    {
        const bool landing_phase = landing_latched_ || remote_mission_.state == "LANDING";
        if (landing_phase || mapCoordinatesReady())
        {
            map_gate_failed_since_ = 0.0;
            return true;
        }
        const double gate_now = ros::WallTime::now().toSec();
        if (mission_guard::sustained(gate_now, true, map_gate_failed_since_,
                                     map_gate_debounce_sec_))
        {
            abortMission("point-cloud coordinates or fence verification lost");
            return false;
        }
        ROS_WARN_THROTTLE(1.0, "Map/fence verification lost for %.2f s; "
                          "abort after %.2f s if it persists",
                          gate_now - map_gate_failed_since_, map_gate_debounce_sec_);
        return true;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        const ros::Time now = ros::Time::now();
        if (mission_source_ == "remote")
        {
            const bool valid = odomFresh();
            if (remote_had_fresh_odom_ && !valid)
                invalidateRemoteSession("World odometry became stale/invalid; recheck and upload again");
            remote_had_fresh_odom_ = valid;
            if ((ros::WallTime::now() - remote_status_time_).toSec() >= 0.2)
                publishRemoteStatus();
        }
        tickAutomaticFlight();

        // 地图/围栏门（L1）：连续失效超过 map_gate_debounce_sec_ 才中止。
        // 必须在下面的 TAKING_OFF 提前 return 之前，否则起飞阶段会绕过宽限
        // （旧实现里 automaticInputsReady(true) 一帧失效就中止）。
        if (state_ != FINISHED && !mapGateAllows()) return;

        if (remote_mission_.state == "TAKING_OFF") return;
        if (landing_latched_ && automatic_landing_ &&
            fresh(fcu_receive_time_, flight_state_timeout_sec_) &&
            fresh(extended_receive_time_, flight_state_timeout_sec_) &&
            !fcu_state_.armed &&
            landed_state_ == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
        {
            automatic_landing_ = false;
            corridor_active_ = false;
            ROS_INFO("Landing complete: ground + disarmed; LAND retries stopped. Restart stack for next flight.");
        }
        if (landing_latched_ && automatic_landing_ &&
            fresh(controller_receive_time_, .5) && controller_receive_time_ > landing_request_time_ &&
            controller_state_ == "AUTO_HOVER" &&
            (now - last_land_publish_time_).toSec() >= land_command_retry_sec_)
        {
            publishLandingCommand(!last_land_publish_time_.isZero());
        }

        if (state_ == FINISHED)
        {
            return;
        }
        if (!odomFresh())
        {
            ROS_ERROR("Odometry guard: valid_world_pose=%s stamp_zero=%s "
                      "receive_age=%.6f s stamp_age=%.6f s timeout=%.3f s "
                      "future_tolerance=%.3f s",
                      odom_received_ ? "true" : "false", odom_stamp_.isZero() ? "true" : "false",
                      (ros::WallTime::now() - odom_receive_time_).toSec(),
                      (ros::Time::now() - odom_stamp_).toSec(), odom_timeout_sec_,
                      odom_future_tolerance_sec_);
            abortMission("odometry invalid, stale or timestamp outside allowed range");
            return;
        }
        if (uploaded_automatic_ && !trajectoryOutputReady())
        {
            abortMission("Trajectory output unavailable or stop-latched during mission");
            return;
        }
        if (!flightReady(false))
        {
            ROS_ERROR("Flight guard: connected=%s armed=%s mode=%s landed_state=%u "
                      "controller=%s fcu_age=%.3f s extended_age=%.3f s "
                      "controller_age=%.3f s ready_age=%.3f s",
                      fcu_state_.connected ? "true" : "false", fcu_state_.armed ? "true" : "false",
                      fcu_state_.mode.c_str(), static_cast<unsigned int>(landed_state_),
                      controller_state_.c_str(),
                      (ros::WallTime::now() - fcu_receive_time_).toSec(),
                      (ros::WallTime::now() - extended_receive_time_).toSec(),
                      (ros::WallTime::now() - controller_receive_time_).toSec(),
                      (ros::WallTime::now() - ready_receive_time_).toSec());
            abortMission("controller or FCU flight state is no longer valid");
            return;
        }
        if (!fresh(heartbeat_receive_time_, 0.5))
        {
            ROS_ERROR("Planner heartbeat guard: receive_age=%.6f s timeout=0.500 s",
                      (ros::WallTime::now() - heartbeat_receive_time_).toSec());
            abortMission("planner heartbeat is no longer valid");
            return;
        }

        const Waypoint& waypoint = active_waypoints_.at(current_index_);

        if (state_ == FLYING)
        {
            if ((ros::WallTime::now() - goal_publish_time_).toSec() > waypoint_timeout_sec_)
            {
                // L1：先重发一次目标并重新计时，仍不到才中止。
                if (waypoint_retry_count_ < 1)
                {
                    ++waypoint_retry_count_;
                    ROS_WARN("Waypoint %u not reached within %.0f s; republishing the goal once",
                             waypoint.id, waypoint_timeout_sec_);
                    publishCurrentGoal(true);
                    return;
                }
                abortMission("waypoint timeout (goal may have been rejected, changed or unreachable)");
                return;
            }
            const Eigen::Vector3d target(waypoint.x, waypoint.y, waypoint.z);
            const double distance = (target - odom_position_).norm();
            const double speed = odom_velocity_.norm();

            // 到点判定要求"位置与速度同时进入容差"，并持续 arrival_stable_sec_，
            // 避免高速掠过目标点时被误判为已到达。
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
                    if (mission_source_ == "remote") remote_mission_.state = "HOVERING";
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
                if (mission_source_ == "remote") remote_mission_.state = "FLYING";
                arrival_candidate_time_ = ros::Time(0);
                return;
            }
            if ((now - hover_start_time_).toSec() < waypoint.hover_sec)
            {
                return;
            }

            // 只有"本航线需要云台"并且"该航点要求动云台"时，才进入云台等待状态
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
            // L0：云台是载荷，不参与飞行安全。超时只放弃这一动作，任务继续。
            ROS_WARN("Gimbal task for waypoint %u not confirmed within %.0f s; skipping it",
                     waypoint.id, gimbal_timeout_sec_);
            advanceWaypoint();
            return;
        }
        if (state_ == WAITING_GIMBAL &&
            (now - last_gimbal_publish_time_).toSec() >= gimbal_retry_sec_)
        {
            publishGimbalTask(true);
        }
    }

    // 下发云台任务。话题协议是 7 个 double 的定长数组：
    //   [waypoint_id, yaw_deg, pitch_deg, settle_sec, mode, yaw_min_deg, yaw_max_deg]
    // mode：0=angle，1=range。重发时 id 不变，供云台侧去重。
    void publishGimbalTask(bool retry)
    {
        const Waypoint& waypoint = active_waypoints_.at(current_index_);
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

    // 云台完成回报：必须与当前正在等待的航点 id 匹配，且到达时报文中的飞行状态
    // 依然有效，否则忽略或中止。完成后写入时间戳 CSV 并推进到下一个航点。
    void gimbalDoneCallback(const std_msgs::UInt32ConstPtr& msg)
    {
        if (state_ != WAITING_GIMBAL)
        {
            ROS_WARN("Ignoring gimbal_done %u because no gimbal task is pending", msg->data);
            return;
        }

        // 不在这里重复检查 odom/flightReady：50 ms 主循环已经在做，而且会给出
        // 具体原因（旧代码在这里只有一句笼统的中止）。
        const Waypoint& waypoint = active_waypoints_.at(current_index_);
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

    // 推进到下一个航点；已到航线末尾则结束本航线。
    // 自动往返模式下，航线飞完会先复检落地走廊，再请求自动降落。
    void advanceWaypoint()
    {
        ++current_index_;
        if (mission_source_ == "remote") remote_mission_.completed = current_index_;
        if (current_index_ >= active_waypoints_.size())
        {
            state_ = FINISHED;
            closeMissionCsv();
            if (mission_source_ == "remote")
            {
                if (uploaded_automatic_)
                {
                    const auto& home = remote_mission_.points.back();
                    const Eigen::Vector3d hover(home.x, home.y, home.z);
                    if (!verticalCorridorEndpointsClear(ground_origin_, hover))
                    {
                        abortMission("Return landing corridor is no longer clear");
                        return;
                    }
                    automatic_phase_time_ = ros::WallTime::now();
                    requestLanding(true);
                }
                else remote_mission_.state = "COMPLETED";
                publishRemoteStatus();
            }
            ROS_INFO("Waypoint mission finished, ros_time=%.3f", ros::Time::now().toSec());
            return;
        }
        publishCurrentGoal();
    }

    // 打开任务时间戳 CSV（覆盖写）并写表头。
    // 只记录"到达时刻 / 云台完成时刻"，用于事后做时序分析。
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

    // 追加一行任务时间戳记录（时间保留 3 位小数，即毫秒级）。
    void writeMissionCsv(const Waypoint& waypoint, const ros::Time& arrived,
                         const ros::Time& done)
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

    // 关闭 CSV（未打开时为空操作）。
    void closeMissionCsv()
    {
        if (mission_csv_.is_open())
        {
            mission_csv_.close();
        }
    }

    // PWM 档位匹配：允许 ±100 的偏差，避免摇杆/接收机抖动造成误判。
    static bool isInRcState(uint16_t state, uint16_t input)
    {
        return input > state - 100 && input < state + 100;
    }

    // 档位的可读名称，仅用于日志。
    static const char* rcStateName(RC_EIGHT_STATE state)
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

    // 把原始 PWM 归入 UP / MIDDLE / DOWN 三档；落在所有窗口之外时返回 false。
    static bool classifyRcState(uint16_t input, RC_EIGHT_STATE* state)
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

    // 发送 LAND 指令。retry=true 表示这是"控制器进入 AUTO_HOVER 后的补发"，
    // 因为 px4ctrl 只在特定状态下才会接受降落指令。
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

    // 请求降落（自动或人工）：闩锁降落标志、停止轨迹输出、记录请求时刻。
    // 闩锁之后本节点不再接受起飞/任务指令，必须落地并重启。
    void requestLanding(bool automatic)
    {
        state_ = FINISHED;
        closeMissionCsv();
        landing_latched_ = true;
        automatic_landing_ = automatic;
        corridor_result_wall_ = ros::WallTime::now().toSec();
        corridor_ok_ = true;
        if (mission_source_ == "remote")
        {
            remote_mission_.state = "LANDING";
            remote_mission_.reason = automatic ? "LAND requested" : "Manual RC landing/takeover";
            publishRemoteStatus();
        }
        last_land_publish_time_ = ros::Time(0);

        std_msgs::Empty stop;
        planning_stop_pub_.publish(stop);

        // 两个降落入口（航线飞完 / 地面站 /mission/land）都必须重置阶段起点，
        // 否则 age 会从起飞时刻开始算，长航线一按降落就立刻超时中止。
        automatic_phase_time_ = ros::WallTime::now();
        landing_contact_time_ = ros::WallTime();
        landing_request_time_ = ros::WallTime::now();

        if (!automatic_landing_)
        {
            ROS_WARN("RC channel 8: manual LAND selected because channel 6 is out of "
                "command mode; planner stopped and automatic LAND retry disabled");
        }
    }

    // /mission/land 触发：外部（地面站）请求自动降落。
    void landCallback(const std_msgs::EmptyConstPtr&)
    {
        requestLanding(true);
    }

    // 遥控器第 8 通道（拨杆）解析与动作分派——人工介入的最高优先级入口。
    //
    // 通道约定（必须与 px4ctrl 保持一致）：
    //   第 8 通道：UP=开始航线；MIDDLE=起飞（从 DOWN 拨上）/返航（从 UP 拨下）；
    //              DOWN=降落
    //   第 6 通道：>1750 表示处于"指令模式"，此时降落按自动降落处理（会重发 LAND）；
    //              否则视为人工接管，只停规划、不重发降落指令
    void rcCallback(const mavros_msgs::RCInConstPtr& msg)
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

    // ---- 成员变量 -------------------------------------------------------
    // 注意：状态新鲜度的时间戳一律用 ros::WallTime（不受 /use_sim_time 影响），
    //       而里程计的时间戳来自 ros::Time（消息头时间戳），两者用途不同，不要混用。
    ros::NodeHandle nh_;
    bool require_map_coordinates_ = false, map_ready_ = false;
    ros::WallTime map_ready_time_;
    ros::Subscriber map_ready_sub_;
    ros::NodeHandle pnh_;
    ros::Publisher point_pub_;
    ros::Publisher gimbal_task_pub_;
    ros::Publisher takeoff_land_pub_;
    ros::Publisher planning_stop_pub_, planning_resume_pub_;
    ros::Publisher startcommand_pub_;
    ros::Publisher backcommand_pub_;
    ros::Publisher clicked_markers_pub_;
    ros::Publisher remote_status_pub_;
    ros::ServiceServer remote_command_srv_;
    remote_mission::Mission remote_mission_;

    struct RemoteRequest
    {
        std::string id;
        std::vector<uint8_t> bytes;
        bool accepted;
        std::string reason;
    };

    std::deque<RemoteRequest> remote_requests_;
    bool remote_had_fresh_odom_ = false;
    double remote_reset_jump_m_;
    ros::WallTime remote_status_time_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gimbal_done_sub_;
    ros::Subscriber startcommand_sub_;
    ros::Subscriber backcommand_sub_;
    ros::Subscriber rc_sub_;
    ros::Subscriber clicked_point_sub_;
    ros::Subscriber start_clicked_route_sub_;
    ros::Subscriber clear_clicked_route_sub_;
    bool output_ready_ = false;
    ros::WallTime output_ready_time_;
    ros::Subscriber output_ready_sub_;
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
    ros::WallTime controller_receive_time_, ready_receive_time_, heartbeat_receive_time_, landing_request_time_;
    ros::WallTime goal_publish_time_, gimbal_wait_time_;
    ros::Time odom_stamp_;
    double odom_timeout_sec_, odom_future_tolerance_sec_, flight_state_timeout_sec_;
    double waypoint_timeout_sec_, gimbal_timeout_sec_;

    RC_EIGHT_STATE rc_eight_pre_;
    bool rc_initialized_;
    bool landing_latched_;
    bool automatic_landing_;
    bool automatic_roundtrip_ = false, uploaded_automatic_ = false;
    bool takeoff_ready_ = false, auto_control_ok_ = false, takeoff_seen_ = false;
    double takeoff_height_ = 0., takeoff_timeout_sec_, landing_timeout_sec_;
    double map_gate_debounce_sec_ = 1.5, map_gate_failed_since_ = 0.0;
    int waypoint_retry_count_ = 0;
    Eigen::Vector3d ground_origin_ = Eigen::Vector3d::Zero();
    ros::WallTime takeoff_ready_time_, auto_control_time_, takeoff_height_time_, automatic_phase_time_,
              auto_hover_since_, last_corridor_check_, landing_contact_time_;
    std::thread corridor_thread_;
    std::atomic<bool> corridor_stop_{false};
    std::atomic<bool> corridor_active_{false};
    std::atomic<bool> corridor_ok_{true};
    std::atomic<double> corridor_result_wall_{0.0};
    std::mutex corridor_target_mutex_;
    Eigen::Vector3d corridor_ground_{0., 0., 0.}, corridor_hover_{0., 0., 0.};
    ros::Subscriber takeoff_ready_sub_, auto_control_sub_, takeoff_height_sub_;
    ros::Publisher abort_automatic_pub_;
    std::ofstream mission_csv_;
};

// 进程入口：构造执行器（失败即打印致命日志并退出）→ 进入 ROS 自旋。
// 所有任务控制逻辑都在回调与定时器里，main 本身不做任何控制。
int main(int argc, char** argv)
{
    ros::init(argc, argv, "multipointplan_node");
    try
    {
        MultipointPlanner planner;
        ros::spin();
    }
    catch (const std::exception& error)
    {
        ROS_FATAL("Failed to start multipoint mission node: %s", error.what());
        return 1;
    }
    return 0;
}
