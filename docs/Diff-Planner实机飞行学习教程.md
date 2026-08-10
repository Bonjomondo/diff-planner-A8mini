# Diff-Planner 实机飞行学习教程

> 适用代码基线：`0c31bf7`（2026-08-08）  
> 主要对象：A8 mini / Orin NX、PX4、Livox MID-360、ROS1、Faster-LIO、Diff-Planner  
> 本文只讨论实机飞行链路，重点是 Diff-Planner 的数据流、代码结构、算法和实飞操作。

## 1. 先建立正确的整体认识

Diff-Planner 不是飞控，也不是定位算法。它位于定位建图和飞行控制之间：

1. Faster-LIO 与 EKF 给出无人机在 `world` 坐标系中的位置、速度和姿态；
2. Faster-LIO 给出注册到世界坐标系的激光点云；
3. Diff-Planner 把点云融合为局部占据栅格，接收目标点，持续生成无碰撞的五次多项式轨迹；
4. `traj_server` 以 100 Hz 对轨迹求值，生成位置、速度、加速度、jerk 和 yaw 指令；
5. `px4ctrl` 把这些平动期望量变成姿态与总推力，通过 MAVROS 发给 PX4。

```text
MID-360 点云 ──> Faster-LIO ──> /laserMapping/cloud_registered ──┐
                                                               │
PX4 IMU ──> MAVROS ──> Faster-LIO + EKF ──> /ekf/ekf_odom ─────┤
                                                               v
航点 /goal ─────────────────────────────────────────> Diff-Planner
                                                        │
                                                        │ 五次多项式轨迹
                                                        v
                                                   traj_server
                                                        │ /setpoints_cmd
                                                        v
                                                     px4ctrl
                                                        │ 姿态 + 推力
                                                        v
                                                   MAVROS / PX4
```

这里最重要的边界是：

- Diff-Planner 决定“未来几秒沿什么轨迹飞”；
- `px4ctrl` 决定“怎样跟踪这条轨迹”；
- PX4 完成最底层姿态、角速度和电机控制；
- 定位漂移、外参错误或坐标系错误不会被规划器自动修正，反而会让地图和轨迹一起出错。

## 2. 实机相关代码地图

建议按下列顺序阅读：

| 层级 | 关键文件 | 学习重点 |
|---|---|---|
| 一键启动 | [`sh_files/run_single_lio.sh`](../sh_files/run_single_lio.sh) | 实机各节点的启动顺序 |
| 实机参数入口 | [`run_exp_single_lio.launch`](../src/diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch) | 里程计、点云、速度、地图和规划范围 |
| 详细规划参数 | [`advanced_param_exp.xml`](../src/diff_planner/plan_manage/launch/include/advanced_param_exp.xml) | FSM、地图、优化器全部参数 |
| 规划入口 | [`diff_planner_node.cpp`](../src/diff_planner/plan_manage/src/diff_planner_node.cpp) | 创建并运行规划状态机 |
| 重规划状态机 | [`diff_replan_fsm.cpp`](../src/diff_planner/plan_manage/src/diff_replan_fsm.cpp) | 收目标、首次规划、周期重规划、安全停车 |
| 规划管理器 | [`planner_manager.cpp`](../src/diff_planner/plan_manage/src/planner_manager.cpp) | 全局参考、局部目标、初始化与结果保存 |
| 局部地图 | [`grid_map.cpp`](../src/diff_planner/plan_env/src/grid_map.cpp) | 点云 raycast、概率占据、膨胀和环形缓存 |
| A* | [`dyn_a_star.cpp`](../src/diff_planner/path_searching/src/dyn_a_star.cpp) | 为碰撞段寻找绕障拓扑和梯度方向 |
| 轨迹优化 | [`poly_traj_optimizer.cpp`](../src/diff_planner/traj_opt/src/poly_traj_optimizer.cpp) | MINCO、L-BFGS、代价函数和可行性复核 |
| 轨迹执行 | [`traj_server.cpp`](../src/diff_planner/plan_manage/src/traj_server.cpp) | 轨迹求值、yaw、停发和降落互锁 |
| 外环控制 | [`PX4CtrlFSM.cpp`](../src/realflight_modules/px4ctrl/src/PX4CtrlFSM.cpp) | 手动、悬停、指令控制、起飞和降落状态 |
| 航点任务 | [`multipointplan.cpp`](../src/user_command/multipoint/src/multipointplan.cpp) | 航点序列、到点判断、遥控器和云台握手 |

如果只想先读懂 Diff-Planner，本章表格中的前九项已经覆盖完整主链路。

## 3. 实机启动链路

### 3.1 `run_single_lio.sh` 实际启动了什么

脚本依次完成：

```text
MAVROS
  ↓
配置 PX4 MAVLink 消息频率
  ↓
Livox 驱动 + Faster-LIO
  ↓
LIO/PX4 IMU 融合 EKF
  ↓
Diff-Planner + traj_server
  ↓
px4ctrl
  ↓
multipoint 航点任务 + A8 mini 云台节点
  ↓
RViz
```

对应脚本代码是：

```bash
roslaunch mavros px4.launch
rosrun mavros mavcmd long 511 ...
roslaunch faster_lio mapping_mid360.launch
roslaunch ekf ekf_lidar.launch
roslaunch diff_planner run_exp_single_lio.launch
roslaunch px4ctrl run_ctrl_lio.launch
roslaunch multipoint multipointplan_exp_lio.launch
roslaunch diff_planner exp_rviz.launch
```

每个 `sleep` 都是在给上游节点留初始化时间。它不是严格的“就绪检测”：电脑负载高、雷达初始化慢或网络异常时，即使时间到了，下游节点也可能仍没有有效输入。因此实飞前必须检查话题，而不能只看 RViz 是否打开。

### 3.2 LIO 数据链路

[`mapping_mid360.launch`](../src/realflight_modules/faster-lio/launch/mapping_mid360.launch) 同时启动 Livox 驱动和 Faster-LIO。当前 [`mid360.yaml`](../src/realflight_modules/faster-lio/config/mid360.yaml) 有两个很关键的选择：

```yaml
common:
    lid_topic: "/livox/lidar"
    imu_topic: "/mavros/imu/data"

mapping:
    extrinsic_est_en: false
    world_transform_en: true
```

这意味着：

- Faster-LIO 使用的是 PX4 经 MAVROS 发布的 IMU，而不是 MID-360 自带 IMU；
- 雷达到 PX4 IMU 的外参完全依赖 `extrinsic_T` 和 `extrinsic_R`；
- `extrinsic_est_en: false` 时，错误外参不会在运行中被自动估计；
- `world_transform_T/R` 决定 LIO 世界系的额外变换，当前为单位变换。

Faster-LIO 发布 `/laserMapping/odometry`，随后 [`ekf_lidar.launch`](../src/realflight_modules/ekf_pose/launch/ekf_lidar.launch) 将它与 `/mavros/imu/data` 融合，输出 `/ekf/ekf_odom`。规划器、航点任务和控制器都使用这一个里程计，从而保持位置判断一致。

### 3.3 Diff-Planner 的实机输入

[`run_exp_single_lio.launch`](../src/diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch) 给规划器配置：

```xml
<arg name="odom_topic" value="/ekf/ekf_odom"/>
<arg name="cloud_topic" value="/laserMapping/cloud_registered"/>
<arg name="max_vel" value="0.5" />
<arg name="max_acc" value="3.0" />
<arg name="max_jer" value="20.0"/>
<arg name="planning_horizon" value="7.5" />
<arg name="map_resolution" value="0.15"/>
<arg name="inflation_size" value="0.3"/>
<arg name="flight_type" value="1" />
```

`flight_type=1` 表示规划器订阅 `/goal`，每次处理一个外部目标点。当前完整航点任务正是由 `multipointplan` 逐个向 `/goal` 发布目标。

## 4. ROS 话题与节点关系

### 4.1 主数据流

| 话题 | 发布者 | 订阅者 | 作用 |
|---|---|---|---|
| `/mavros/imu/data` | MAVROS | Faster-LIO、EKF、px4ctrl | 飞控 IMU |
| `/livox/lidar` | Livox 驱动 | Faster-LIO | 原始 Livox 点云 |
| `/laserMapping/odometry` | Faster-LIO | EKF | LIO 里程计 |
| `/laserMapping/cloud_registered` | Faster-LIO | GridMap | `world` 系注册点云 |
| `/ekf/ekf_odom` | EKF | Diff-Planner、px4ctrl、multipoint | 统一状态估计 |
| `/goal` | multipoint 或用户 | Diff-Planner | 单个三维目标点 |
| `/drone_0_planning/trajectory` | Diff-Planner | `traj_server` | 五次多项式系数和分段时间 |
| `/drone_0_traj_server/heartbeat` | Diff-Planner | `traj_server` | 规划器存活信号 |
| `/setpoints_cmd` | `traj_server` | px4ctrl | 位置、速度、加速度、jerk、yaw |
| `/mavros/setpoint_raw/attitude` | px4ctrl | MAVROS/PX4 | 姿态四元数与总推力 |
| `/planning/stop` | multipoint/用户 | `traj_server` | 立即停止轨迹指令输出 |
| `/px4ctrl/takeoff_land` | 脚本或 multipoint | px4ctrl、`traj_server` | 起飞/降落与轨迹输出互锁 |

注意实际轨迹话题是 `/drone_0_planning/trajectory`。launch 文件把两个节点的私有话题都重映射到了这个名字，不要只根据节点名猜话题路径。

### 4.2 目标触发不是同一个概念

本工程有两个容易混淆的话题：

- `/move_base_simple/goal`：对 `multipointplan` 来说只是“开始执行 YAML 航点任务”的触发器，消息中的坐标不会成为飞行目标；
- `/goal`：Diff-Planner 真正接收的三维目标点。

代码路径是：

```text
/move_base_simple/goal
    -> MultipointPlanner::startCallback()
    -> publishCurrentGoal()
    -> /goal
    -> DiffReplanFSM::waypointCallback()
```

调试单个规划目标时应直接发 `/goal`；执行 `points.yaml` 整套任务时才发 `/move_base_simple/goal`。

## 5. Diff-Planner 算法主线

### 5.1 入口和线程频率

[`diff_planner_node.cpp`](../src/diff_planner/plan_manage/src/diff_planner_node.cpp) 本身很薄：创建 `DiffReplanFSM`，调用 `init()`，然后 `ros::spin()`。

真正的运行节奏在 `DiffReplanFSM::init()`：

```cpp
exec_timer_ = nh.createTimer(ros::Duration(0.01),
                             &DiffReplanFSM::execFSMCallback, this);
safety_timer_ = nh.createTimer(ros::Duration(0.05),
                               &DiffReplanFSM::checkCollisionCallback, this);
```

因此：

- 主状态机按 100 Hz 被唤醒；
- 前向轨迹安全检查按 20 Hz 执行；
- `traj_server` 也按 100 Hz 对轨迹求值；
- `px4ctrl` 的上限频率在当前参数中是 200 Hz。

100 Hz 状态机并不等于每 10 ms 都完成一次 L-BFGS 优化。回调内部只有进入首次规划、重规划或碰撞处理状态时才调用优化器；普通 `EXEC_TRAJ` 周期主要做状态判断。

### 5.2 局部占据地图：点云怎样变成障碍物

LIO 实机使用 [`GridMap::cloudCallback()`](../src/diff_planner/plan_env/src/grid_map.cpp)：

1. 接收 `/laserMapping/cloud_registered`；
2. 丢弃 NaN/Inf 点；
3. 使用最新 `/ekf/ekf_odom` 得到无人机位置作为 raycast 原点；
4. 对每个点执行射线遍历；
5. 端点记为 hit，射线经过的体素记为 miss；
6. 使用 log-odds 累积占据概率；
7. 对确认占据的体素做三维膨胀。

核心更新可理解为：

```text
L_new = clamp(L_old + L_hit/miss, L_min, L_max)
occupied = (L_new >= L_occ)
```

当前概率参数来自 `advanced_param_exp.xml`：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `p_hit` | 0.65 | 一次命中增加占据置信度 |
| `p_miss` | 0.35 | 一次射线穿过降低占据置信度 |
| `p_min` / `p_max` | 0.12 / 0.90 | log-odds 上下限 |
| `p_occ` | 0.80 | 判为占据的概率阈值 |
| `fading_time` | 1000 s | 长时间未观测时缓慢淡出 |

当前分辨率为 0.15 m，膨胀距离为 0.30 m。代码计算：

```cpp
inf_grid = ceil((inflation - 1e-5) / resolution);
```

所以当前为 2 个体素，并在 x/y/z 三个方向按立方邻域膨胀。它不是精确欧氏球形膨胀；选参数时应把机体外廓、定位误差、控制跟踪误差和体素离散误差一起考虑。

#### 环形缓存

局部地图不是固定大地图，而是以无人机为中心移动的 ring buffer。当前 `local_map_size_x/y/z = 5.5/5.5/2.0` 在代码里被当作中心到边界的半范围，因此原始局部窗口约为：

```text
11 m × 11 m × 4 m
```

无人机移动时，旧边界体素被清空，内存地址通过环形索引复用。这让计算量与局部窗口大小有关，而不是与总飞行距离有关。

#### 当前地图实现的实机注意事项

- 点云坐标被直接当成世界坐标使用，`/laserMapping/cloud_registered` 必须与 `/ekf/ekf_odom` 共享一致的 `world` 系；
- 点云与里程计是独立订阅，不做时间同步。高延迟或急转弯时，raycast 原点与点云时刻不一致会造成障碍拖影；
- `getInflateOccupancy()` 对环形缓存外的 x/y 区域返回空闲，而不是未知占据；安全性依赖传感器范围、规划范围和持续更新；
- 高度虚拟墙当前由 `advanced_param_exp.xml` 实际写死为 `-0.1 < z < 3.0`；外层 launch 传入的 `virtual_ceil=2.0`、`virtual_ground=0.1` 没有生效；
- 当前编译的是 `grid_map.cpp`。launch 中的 `grid_map/max_ray_length` 没有被这个类读取；不要误以为修改它一定会改变 LIO 点云建图范围；
- `Depth Lost` 超时标志只在深度图回调中置位。LIO 点云路径不会设置 `flag_have_ever_received_depth_`，所以不能把这项检查当作 LIO 点云掉线保护。

最后一项尤其重要：实机必须额外监控 `/livox/lidar` 和 `/laserMapping/cloud_registered` 的频率，点云中断时应由操作员立即中止任务或接管。

### 5.3 收到目标后先生成“全局参考轨迹”

`DiffReplanFSM::waypointCallback()` 收到 `/goal` 后调用 `planNextWaypoint()`，进而调用：

```cpp
planner_manager_->planGlobalTrajWaypoints(
    odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
    one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
```

`planGlobalTrajWaypoints()` 用当前状态到目标点生成一条 minimum-jerk 五次多项式，期望巡航速度约为：

```cpp
des_vel = max_vel / 1.5;
```

必须注意：这条全局轨迹主要用于提供进度方向和局部目标，不是最终无碰撞轨迹。它可以穿过障碍物。真正避障发生在后续局部轨迹初始化、A* 引导和连续优化中。

### 5.4 滚动选择局部目标

每次规划时，`getLocalTarget()` 沿全局参考轨迹向前采样，寻找与当前起点距离达到 `planning_horizon` 的位置：

```cpp
if (dist >= planning_horizen) {
    local_target_pos = pos_t;
    break;
}
```

当前规划视野是 7.5 m。若全局终点已经落在视野内，则局部目标直接取最终目标，并把 `touch_goal=true`。

这构成滚动时域规划：

```text
当前状态 -> 未来约 7.5 m 的局部目标 -> 执行前一小段 -> 再规划
```

靠近最终目标时，代码使用制动距离判断局部目标速度是否应设为零：

```text
distance_to_goal < max_vel² / (2 * max_acc)
```

### 5.5 初始轨迹、碰撞段与 A*

`DiffPlannerManager::reboundReplan()` 分三步：

```text
STEP 1：生成初始 MINCO 轨迹
STEP 2：L-BFGS 优化
STEP 3：复核、保存并发布结果
```

首次规划或普通初始化时，先生成一条连接起终状态的多项式，再按 `polyTraj_piece_length / max_vel` 重新分段。当前值为：

```text
piece_length = 1.5 m
max_vel      = 0.5 m/s
初始每段时间 ts = 3.0 s
```

若连续失败，代码可以在起终点中间加入随机偏移点，尝试从不同初值重新优化。

随后 `finelyCheckAndSetConstraintPoints()` 对初始轨迹密采样，识别进入和离开膨胀障碍的区间。对每个碰撞区间调用三维 26 邻域 A*：

```cpp
ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
```

A* 在这里不是直接输出飞行轨迹。它的作用是：

1. 找到一条离散、无碰撞的绕障通路；
2. 找出该通路相对碰撞多项式控制点的方向；
3. 为控制点建立 `base_point + direction` 形式的障碍约束；
4. 让连续优化器获得“往障碍哪一侧弹开”的梯度。

因此 Diff-Planner 的绕障方式可以概括为：

```text
A* 找拓扑和方向，MINCO/L-BFGS 生成真正平滑、可执行的轨迹。
```

A* 本身有 0.2 s 搜索超时。起点或终点落入障碍时，`ConvertToIndexAndAdjustStartEndPoints()` 会沿起终点连线方向逐格推出障碍，再进行搜索。

### 5.6 MINCO 五次多项式和优化变量

每一段轨迹是五次多项式：

```text
p_i(t) = c0 + c1 t + c2 t² + c3 t³ + c4 t⁴ + c5 t⁵
```

它能连续计算：

- 位置 `p`；
- 速度 `v = p'`；
- 加速度 `a = p''`；
- jerk `j = p'''`。

优化器不直接把所有系数当作自由变量，而是优化内部连接点和每段时间，再由 MINCO 生成满足边界条件与连续性的系数。时间先映射到无约束虚变量，保证真实分段时间始终为正。

当前总代价可以概括为：

```text
J = J_smooth
  + J_obstacle
  + J_swarm
  + J_velocity/acceleration/jerk
  + J_point_spacing
  + weight_time × ΣT_i
```

对应代码集中在 `costFunctionCallback()` 和 `addPVAJGradCost2CT()`：

- `J_smooth`：MINCO 的平滑/jerk 代价；
- `J_obstacle`：控制点离障碍基准面的距离不足时施加惩罚；
- `J_swarm`：单机模式下通常没有有效的其他无人机轨迹；
- `J_velocity/acceleration/jerk`：超过设定上限时使用三次惩罚；
- `J_point_spacing`：正则化相邻约束点间距，避免离散点分布过坏；
- `J_time`：压缩总飞行时间。

当前关键权重：

| 参数 | 当前值 |
|---|---:|
| `weight_obstacle` | 10000 |
| `weight_obstacle_soft` | 5000 |
| `weight_feasibility` | 10000 |
| `weight_sqrvariance` | 10000 |
| `weight_time` | 10 |
| `obstacle_clearance` | 0.10 m |
| `obstacle_clearance_soft` | 0.50 m |

不要孤立理解 `obstacle_clearance=0.10`。规划使用的是已经膨胀 0.30 m 的地图，再叠加连续优化的硬、软距离惩罚，因此真实效果由地图膨胀、软距离、权重和跟踪误差共同决定。

#### Rebound 机制

L-BFGS 运行几次后，如果优化中的约束点又碰到新障碍，`roughlyCheckConstraintPoints()` 会重新调用 A*，更新障碍方向，并让当前优化提前退出。外层随后用新约束重启 L-BFGS。这就是代码里的 rebound：不是简单把点推出去一次，而是“优化—发现新碰撞—更新绕障方向—重新优化”。

### 5.7 优化完成后的二次复核

优化器返回并不等于轨迹立刻下发。代码还做两类硬检查。

#### 动力学可行性

`checkDynamicFeasibility()` 沿整条轨迹密采样，检查：

```text
|v| <= max_vel + vel_tolerance
|a| <= max_acc + acc_tolerance
```

当前参数是：

```text
max_vel=0.5, vel_tolerance=1.0
max_acc=3.0, acc_tolerance=1.0
```

这里的 tolerance 是绝对加法，不是百分比。也就是说最终硬检查允许速度到 1.5 m/s、加速度到 4.0 m/s²；优化代价仍会从 0.5 m/s 和 3.0 m/s² 开始惩罚。实机调参时必须理解这一区别。

#### 精细碰撞复核

`finelyCheckAndSetConstraintPoints()` 再次按地图分辨率和速度决定时间步长，对轨迹密采样。若仍碰撞，会增加约束并最多进行有限次重启；只有动力学与碰撞检查都通过，`flag_success` 才为真。

### 5.8 状态机如何持续重规划

主要状态如下：

```text
INIT
  -> WAIT_TARGET
  -> SEQUENTIAL_START / GEN_NEW_TRAJ
  -> EXEC_TRAJ
       -> REPLAN_TRAJ -> EXEC_TRAJ
       -> WAIT_TARGET       （到达最终目标）
       -> EMERGENCY_STOP    （碰撞、连续失败或卡死）
```

关键逻辑：

- `INIT`：没有里程计时不继续；
- `WAIT_TARGET`：等待 `/goal`；
- `SEQUENTIAL_START`：首次局部规划最多尝试 10 次；
- `EXEC_TRAJ`：轨迹执行超过 `thresh_replan_time=1.0 s` 后触发下一次局部重规划；
- `REPLAN_TRAJ`：优先从旧轨迹未来状态继续优化，失败后才重新多项式初始化或随机初始化；
- `EMERGENCY_STOP`：发布当前位置的零速度、零加速度保持轨迹。

从旧轨迹未来时刻取新的起点状态非常重要：

```cpp
start_pt_  = old_traj.getPos(t_cur);
start_vel_ = old_traj.getVel(t_cur);
start_acc_ = old_traj.getAcc(t_cur);
```

这样新旧轨迹在切换点保持状态连续，避免每秒重规划都产生位置或速度跳变。

#### 在线安全检查

`checkCollisionCallback()` 每 50 ms 检查当前轨迹未来采样点：

- 发现未来碰撞时先立即尝试局部重规划；
- 重规划失败且碰撞不足 `emergency_time=1.0 s` 时进入紧急停止；
- 碰撞还比较远时转入正常重规划；
- 连续规划失败超过 10 次时进入紧急停止；
- 大障碍导致长期无进展时触发 stuck detection。

当前卡死时间约为：

```text
1.5 × planning_horizon / max_vel
= 1.5 × 7.5 / 0.5
= 22.5 s
```

它不是瞬时保护，而是防止长时间在大障碍前徘徊。

#### 目标落在障碍中

`fsm/mondify_final_goal=true` 时，如果最终目标被占据，规划器会沿全局参考轨迹反向寻找较早的安全点并修改自己的最终目标。这能避免继续冲向障碍，但会产生一个任务层现象：

- Diff-Planner 可能安全停在修改后的点；
- `multipointplan` 仍按 YAML 中的原始坐标判断“是否到达”；
- 航点任务可能一直停在 `FLYING`。

遇到这种情况应查看日志中的 `modified final_goal`，确认目标是否落在膨胀地图内，并修改航点，而不是单纯放宽到点阈值。

## 6. 轨迹怎样变成飞控指令

### 6.1 轨迹消息

规划成功后，FSM 把轨迹序列化为 [`PolyTraj.msg`](../src/diff_planner/traj_utils/msg/PolyTraj.msg)：

```text
drone_id, traj_id, start_time,
order=5,
coef_x[], coef_y[], coef_z[],
duration[]
```

`traj_server` 检查阶数和数组长度，恢复出分段五次多项式对象。

### 6.2 100 Hz 轨迹采样

`traj_server::cmdCallback()` 根据：

```text
t_cur = now - trajectory.start_time
```

求出当前 `p/v/a/j`，发布 [`PositionCommand.msg`](../src/Utils/quadrotor_msgs/msg/PositionCommand.msg) 到 `/setpoints_cmd`。

当前 yaw 默认看向轨迹前方 `time_forward=1.0 s` 的位置：

```cpp
dir = traj.getPos(t_cur + time_forward) - current_pos;
yaw_target = atan2(dir.y(), dir.x());
```

代码还限制 yaw 变化率和变化加速度，并允许 `/planning/yaw` 在 0.5 s 有效期内覆盖自动朝向。

代码阅读时需要留意：`calculate_yaw()` 先算出了 `yawdot`，但函数返回前又把 `yaw_yawdot.second` 赋成 `yaw_temp`。因此当前 `/setpoints_cmd.yaw_dot` 实际可能携带目标 yaw，而不是计算出的 yaw 角速度。若后续控制或记录依赖 yaw-rate 前馈，应先修正并做地面验证。

### 6.3 心跳和停止互锁

`traj_server` 只有同时收到轨迹和规划器心跳才输出命令。心跳超过 0.5 s 未更新时，它会清除活动轨迹并停止继续输出。

收到以下任一消息也会停止：

- `/planning/stop`；
- `/px4ctrl/takeoff_land` 中的 `LAND`。

停止后，降落锁存会拒绝新轨迹；只有收到新的 `TAKEOFF` 才解锁，而且旧轨迹不会自动恢复。这一设计避免降落期间被晚到的轨迹重新拉回 `CMD_CTRL`。

### 6.4 px4ctrl 状态机

px4ctrl 的主状态关系是：

```text
MANUAL_CTRL
  -> AUTO_TAKEOFF -> AUTO_HOVER
  -> AUTO_HOVER -> CMD_CTRL
  -> CMD_CTRL -> AUTO_HOVER
  -> AUTO_HOVER -> AUTO_LAND -> MANUAL_CTRL
```

`CMD_CTRL` 中，控制器消费 `/setpoints_cmd`。如果指令超过 `msg_timeout/cmd=0.5 s` 没有更新，就退回 `AUTO_HOVER`，用当前里程计位置悬停。

控制算法 0 的核心可概括为：

```text
位置误差 + 速度误差
    -> 反馈加速度
    + 轨迹前馈加速度/jerk/yaw
    -> 期望姿态和总推力
    -> /mavros/setpoint_raw/attitude
```

当前控制参数在 [`ctrl_param_fpv.yaml`](../src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml)：

- 质量 `1.694 kg`；
- 位置增益 `Kp=[1.5,1.5,1.5]`；
- 速度增益 `Kv=[2,2,2]`；
- 自动起飞高度 `1.0 m`；
- 自动起降速度 `0.2 m/s`；
- 悬停油门近似值 `0.42`；
- 低电压阈值 `21.0 V`（6S）。

这些是具体机体参数，不是通用默认值。质量、桨、电机、电池、载荷或重心改变后，应先重新验证控制和推力映射，再提高规划速度。

## 7. 航点任务、遥控器与 A8 mini

### 7.1 航点如何进入规划器

[`points.yaml`](../src/user_command/multipoint/config/points.yaml) 中的每个航点依次经历：

```text
发布 /goal
  -> Diff-Planner 飞向目标
  -> 位置误差和速度同时达标，并持续稳定
  -> 悬停 hover_sec
  -> 执行云台任务
  -> 收到 gimbal_done
  -> 发布下一个 /goal
```

默认到点判据：

```text
distance <= 0.7 m
speed    <= 0.2 m/s
连续满足 0.5 s
```

这比只看距离可靠：无人机高速掠过航点时不会被误判为到达。

### 7.2 云台等待与规划器的关系

云台节点通过 UDP 访问 A8 mini，默认地址是 `192.168.144.25:37260`。飞行端进入 `WAITING_GIMBAL` 后不发布下一个目标；此时 Diff-Planner 已完成当前轨迹，px4ctrl 会转为悬停。

若云台没有返回 `/mission/gimbal_done`，航点任务会每 6 s 重发任务并一直等待。这是任务层的安全行为，不是 Diff-Planner 卡死。

只把 launch 参数 `start_gimbal_node:=false` 改成 false，并不会让航点任务跳过云台等待。若只学习 Diff-Planner 而不测试 A8 mini，最简单的方法是直接向 `/goal` 发布单个目标，绕过整套 multipoint/云台状态机。

### 7.3 遥控器通道逻辑

`multipointplan` 要求第 8 通道首次被识别时处于 DOWN，当前代码的 PWM 窗口为：

| 代码状态 | 中心 PWM | 转换动作 |
|---|---:|---|
| DOWN | 1999 | 初始状态；进入 DOWN 请求降落 |
| MIDDLE | 1499 | DOWN → MIDDLE 请求起飞；UP → MIDDLE 请求返程 |
| UP | 999 | MIDDLE → UP 开始航点任务 |

不同遥控器的物理拨杆方向可能相反，实飞前必须先看：

```bash
rostopic echo /mavros/rc/in
```

px4ctrl 另外使用：

- 第 5 通道 `channels[4]`：PWM 大于约 1750 才是 hover/offboard 模式；
- 第 6 通道 `channels[5]`：PWM 大于约 1750 才是 command-mode。

自动起飞要求两个模式开关正确、摇杆居中、里程计有效且机体被判断为已落地。

### 7.4 降落为什么要先停止轨迹

px4ctrl 只接受从 `AUTO_HOVER` 进入 `AUTO_LAND`。如果 `traj_server` 仍在连续发布 `/setpoints_cmd`，px4ctrl 会保持 `CMD_CTRL` 并拒绝 LAND。

当前修复链路是：

```text
第 8 通道进入 DOWN
  -> multipoint 锁存 /planning/stop
  -> traj_server 停止 PositionCommand
  -> px4ctrl 在 0.5 s 指令超时后回到 AUTO_HOVER
  -> multipoint 每 1 s 重发 LAND
  -> px4ctrl 接受并进入 AUTO_LAND
```

单独的 [`land.sh`](../sh_files/land.sh) 只发布一次 LAND。如果当时仍在 `CMD_CTRL`，第一条 LAND 可能只让 `traj_server` 停止输出，却被 px4ctrl 拒绝；之后没有第二条自动重试。更稳妥的实机流程是使用第 8 通道的锁存降落逻辑。若必须用话题方式，应先发布 `/planning/stop`，确认 `/setpoints_cmd` 已停止并等待超过 0.5 s，再发布 LAND，同时全程准备遥控接管。

第 6 通道拨出 command-mode 后，multipoint 会停止自动 LAND 重试，转为 RC 悬停/手动降落，避免自动和手动降落争抢。

## 8. 实机前配置

### 8.1 编译

在工作空间根目录执行：

```bash
catkin_make -DROS_EDITION=ROS1
source devel/setup.zsh
```

使用 Bash 时 source `devel/setup.bash`。

若工作空间曾移动，旧的 `build/`、`devel/` 中可能记录绝对路径，应按项目现有部署流程重新生成构建目录。不要同时 source 另一份包含同名 `diff_planner`、`multipoint` 或 `px4ctrl` 的工作空间，可用以下命令确认：

```bash
rospack find diff_planner
rospack find multipoint
rospack find px4ctrl
```

构建时如果在 `plan_env/CMakeLists.txt` 的 OpenCV 配置行失败，需要注意该文件含有机器相关的：

```cmake
include("~/Documents/opencv-4.6.0/build/OpenCVConfig.cmake")
```

`find_package(OpenCV REQUIRED)` 已经在它前面执行。应根据实机真实 OpenCV 安装位置清理或改正这条机器相关路径，而不是复制不存在的目录。

### 8.2 MID-360 网络

当前 [`MID360s_config.json`](../src/livox_ros_driver2/config/MID360s_config.json) 配置：

```text
机载电脑：192.168.1.50
雷达：    192.168.1.171
```

启动前至少确认：

```bash
ip -br addr
ping 192.168.1.171
```

IP、网口和 JSON 必须一致。点云偶发中断时还要检查网卡节能、线缆、供电和 UDP 丢包。

### 8.3 外参与坐标系

外参错误通常比规划参数错误更危险。建议先在桨叶拆除状态下验证：

1. 静止时 `/laserMapping/odometry` 和 `/ekf/ekf_odom` 速度接近零；
2. 手动沿机体前方移动，世界系位置变化方向与 RViz 一致；
3. 原地转动机体时，静态墙面不应跟着机体明显旋转或平移；
4. `/laserMapping/cloud_registered` 与里程计轨迹在同一坐标系对齐；
5. 起飞点附近的地面、墙和机体自身点云不会被错误标成飞行路径中的障碍。

如果墙面在转弯时出现双层、弧形拖影，应优先检查时间同步、外参和点云延迟，而不是先减小 `inflation_size`。

### 8.4 PX4 和控制参数

实机变更下列任一项后，都要重新低高度验证：

- 总质量和载荷；
- 电池节数或电压范围；
- 桨、电机或机架；
- IMU 安装方向；
- 遥控器通道映射；
- 悬停油门与推力模型；
- 位置/速度控制增益。

当前启动脚本包含：

```bash
echo '1' | sudo -S chmod 777 /dev/tty*
```

这是硬编码密码且授权范围过宽，只适合识别现有代码行为，不应作为长期部署方案。实机应通过 udev 规则或用户组权限只授权实际飞控串口，避免脚本保存明文密码和修改所有 TTY 设备。

## 9. 推荐的首次实飞学习流程

这套流程先验证单目标 Diff-Planner，再验证 YAML 航点和云台，从而容易定位问题属于哪一层。

### 第一步：准备一个保守场地

- 开阔、无人员、无细线和透明障碍；
- 起飞点到目标点之间留足横向和高度余量；
- 首次只使用约 1 m 高度、1～2 m 距离的目标；
- 安排一名操作员只负责遥控器和急停；
- 明确自动降落与手动接管两个方案。

### 第二步：使用调试启动脚本

```bash
source devel/setup.zsh
./sh_files/run_single_lio_debug.sh
```

它调用正常 LIO 启动脚本，同时保存控制台、ROS 日志、参数快照和关键 rosbag。飞行结束用 `Ctrl+C`，等待 rosbag 正常写完索引。

### 第三步：节点和链路检查

```bash
rosnode list
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/extended_state
rostopic hz /mavros/imu/data
rostopic hz /livox/lidar
rostopic hz /laserMapping/odometry
rostopic hz /laserMapping/cloud_registered
rostopic hz /ekf/ekf_odom
rostopic hz /drone_0_traj_server/heartbeat
```

此时尚未给目标，`/setpoints_cmd` 没有持续输出是正常的。必须确认：

- MAVROS 已连接；
- RC 数据有效；
- LIO 和 EKF 无 NaN、突跳和明显漂移；
- 注册点云更新正常；
- RViz 中膨胀障碍与真实环境一致；
- 规划器心跳正常。

### 第四步：起飞后只发一个直接目标

自动起飞可通过第 8 通道 DOWN → MIDDLE，或在安全条件满足时执行：

```bash
./sh_files/takeoff.sh
```

稳定悬停后，直接向 Diff-Planner 发一个近距离目标：

```bash
rostopic pub -1 /goal geometry_msgs/PoseStamped "header:
  frame_id: 'world'
pose:
  position: {x: 1.5, y: 0.0, z: 1.0}
  orientation: {w: 1.0}"
```

观察：

```bash
rostopic hz /drone_0_planning/trajectory
rostopic hz /setpoints_cmd
rostopic echo /setpoints_cmd
```

预期日志主线：

```text
Received goal
Replan 0
Success=yes
AUTO_HOVER(L2) -> CMD_CTRL(L3)
```

到达后，轨迹输出停止，px4ctrl 因指令超时回到 `AUTO_HOVER`。这属于正常的航点悬停交接。

### 第五步：验证规划停止和降落

在低高度先验证停止：

```bash
rostopic pub -1 /planning/stop std_msgs/Empty "{}"
```

确认 `/setpoints_cmd` 停止、px4ctrl 转为 `AUTO_HOVER`。然后按既定自动或手动方案降落。使用第 8 通道降落时，预期看到：

```text
[traj_server] planning stop received
[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!
[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND
```

### 第六步：再执行 YAML 航点任务

把 `points.yaml` 改成一个近距离航点，重新启动 `multipointplan` 使 YAML 被重新读取。确认 A8 mini 网络和云台 ACK 正常后，再用：

```bash
./sh_files/pub_trigger.sh
```

或第 8 通道 MIDDLE → UP 触发任务。一个航点完整通过后，再逐步增加航点数量、距离和障碍复杂度。

## 10. 参数调试方法

调参顺序应从感知和控制开始，最后才是优化器权重：

```text
定位/外参
  -> 地图质量
  -> 控制跟踪
  -> 速度和加速度上限
  -> 膨胀与安全距离
  -> 规划范围和重规划频率
  -> 优化权重
```

### 10.1 飞得不稳，不要先改规划权重

先比较 `/setpoints_cmd.position` 与 `/ekf/ekf_odom.pose`，以及期望速度与实际速度：

- 期望轨迹平滑但机体振荡：优先查控制参数、推力映射、延迟；
- 期望轨迹本身急弯或突变：再查规划和地图；
- 期望轨迹正常但点云随姿态漂移：查外参和时间同步。

### 10.2 障碍边缘距离不足

依次考虑：

1. 外参和定位误差是否已经让障碍位置偏移；
2. `inflation_size` 是否覆盖机体半径与跟踪误差；
3. `obstacle_clearance_soft` 是否足够；
4. `weight_obstacle_soft` 是否能压过时间和平滑代价；
5. 地图分辨率是否过粗。

不要只提高 `weight_obstacle`。如果地图位置就是错的，再高的权重也只会远离错误地图中的障碍。

### 10.3 频繁规划失败

查看是否出现：

```text
A-star error
Dynamic feasibility check failed
Local target in collision
replan fail too much
```

对应检查：

- A* 失败：局部地图是否被噪点堵死、膨胀是否过大、初末点是否在障碍中；
- 动力学失败：`max_vel/max_acc/max_jer` 是否相互匹配，时间权重是否过强；
- 局部目标碰撞：规划范围前端是否被障碍覆盖，目标和地图是否错位；
- 连续失败：降低任务复杂度，先恢复到单个近距离目标。

### 10.4 规划绕远或在大障碍前徘徊

- 全局参考只连接当前点和目标，没有全局障碍搜索；
- 局部优化每次只处理滚动视野；
- 面对超出局部窗口的大障碍，算法可能多次选择局部方向后才发现绕行代价很大；
- stuck detection 当前约 22.5 s 才判定长期无进展。

这类场景应扩大可感知与局部地图范围、重新设计航点，或引入真正的全局障碍路径层，而不是无限提高局部优化重试次数。

## 11. 日志与故障定位

### 11.1 优先看哪一层

| 现象 | 第一检查点 |
|---|---|
| RViz 没点云 | Livox 网络、`/livox/lidar`、Faster-LIO |
| 点云有但地图不动 | `/ekf/ekf_odom`、坐标系、GridMap 订阅 |
| 收到目标但无轨迹 | FSM 状态、目标是否在虚拟墙/障碍、A*/优化日志 |
| 有轨迹但无 `/setpoints_cmd` | `traj_server` 心跳、降落锁存、轨迹消息名 |
| 有 `/setpoints_cmd` 但不进入指令控制 | RC 第 5/6 通道、OFFBOARD、px4ctrl 状态 |
| 轨迹平滑但飞机振荡 | px4ctrl 增益、质量、推力模型、定位延迟 |
| 到点后不发下一个航点 | 到点速度阈值、云台 `gimbal_done`、目标被规划器修改 |
| LAND 被拒绝 | 是否仍在 `CMD_CTRL`、`/setpoints_cmd` 是否已停 |

### 11.2 调试日志目录

`run_single_lio_debug.sh` 默认写入：

```text
flight_logs/YYYYMMDD_HHMMSS/
```

主要文件：

- `console.log`：节点控制台输出；
- `key_events.log`：RC、航点、云台、规划停止和降落事件；
- `flight_debug.bag`：关键 ROS 话题；
- `rosparams.yaml`：当次实际参数；
- `points.yaml.snapshot`：当次航点快照；
- `metadata.txt`：Git 版本和环境信息。

当前调试脚本记录的是 `/drone_0_diff_planner_node/planning/trajectory`，而当前实机 launch 的实际轨迹话题是 `/drone_0_planning/trajectory`。如果需要在 bag 中分析规划多项式，应先通过 `rostopic list` 确认真实名字，并把实际话题加入录包列表。

### 11.3 一次问题的推荐时间线

每次复盘按时间对齐：

```text
目标或 RC 触发
  -> 规划器收到目标
  -> 轨迹发布
  -> /setpoints_cmd 开始
  -> px4ctrl 进入 CMD_CTRL
  -> 地图是否出现新障碍
  -> 是否重规划/紧急停止
  -> 指令是否停止
  -> px4ctrl 状态和 PX4 armed/landed 状态
```

只截取一条错误日志通常不足以确定根因。需要同时看目标、轨迹、控制命令、里程计、RC 和 px4ctrl 状态。

## 12. 当前代码中值得学习和警惕的细节

以下内容来自当前代码本身，修改参数或二次开发前应先处理清楚：

1. `fsm/mondify_final_goal` 的参数名拼写就是 `mondify`，改成 `modify` 不会被现有代码读取。
2. `manager/feasibility_tolerance` 被读入，但当前规划代码没有实际使用它；真正生效的是优化器的 `vel_tolerance` 和 `acc_tolerance`。
3. launch 设置了 `optimization/record_opt`，但 `PolyTrajOptimizer::setParam()` 没有读取该参数。
4. 实机 launch 的 `virtual_ceil/virtual_ground` 参数被 include 文件中的常数覆盖，当前有效值是 3.0 和 -0.1。
5. 当前 `grid_map.cpp` 不读取 `max_ray_length`，该参数不能直接约束 LIO 点云回调的 raycast 距离。
6. LIO 点云路径没有启用深度图路径的掉线超时标志，必须外部监控点云。
7. 环形缓存外被当作空闲，规划范围不应脱离可靠感知范围。
8. `traj_server` 的 `yaw_dot` 字段存在被目标 yaw 覆盖的代码路径。
9. 根 README 提到了 `sh_files/run_single_vio.sh` 和 `sh_files/run_vins.sh`，但当前仓库没有这两个脚本；现成的一键实机链路是 LIO。
10. `land.sh` 只发一次 LAND，从 `CMD_CTRL` 发起时可能缺少超时后的第二次请求。

这些细节不代表系统一定不能飞，而是说明实机工程必须以“当前真正执行的代码和话题”为准，不能只根据注释或参数名推断行为。

## 13. VIO 实机接口说明

仓库保留了 VIO 实机规划、控制和航点 launch：

- [`run_exp_single_vio.launch`](../src/diff_planner/plan_manage/launch/exp/run_exp_single_vio.launch)；
- [`run_ctrl_vio.launch`](../src/realflight_modules/px4ctrl/launch/run_ctrl_vio.launch)；
- [`multipointplan_exp_vio.launch`](../src/user_command/multipoint/launch/multipointplan_exp_vio.launch)。

主要差异是：

```text
里程计：/vins/imu_propagate
地图输入：/camera/depth/image_rect_raw
地图模式：深度图 + 同步里程计
```

使用前必须替换深度相机的 `fx/fy/cx/cy`，并确认深度尺度、VINS 外参、深度与里程计时间同步。当前仓库没有把 VINS、相机、规划、控制和航点整合成可直接运行的 `run_single_vio.sh`，因此需要根据实际传感器部署补齐启动链路。初学本项目时建议先把现成 LIO 实机链路完整跑通，再对照接口迁移。

## 14. 建议的代码学习练习

### 练习 1：追踪一个目标

从 `MultipointPlanner::publishCurrentGoal()` 开始，依次找到：

```text
DiffReplanFSM::waypointCallback()
DiffReplanFSM::planNextWaypoint()
DiffPlannerManager::planGlobalTrajWaypoints()
DiffReplanFSM::callReboundReplan()
DiffPlannerManager::reboundReplan()
PolyTrajOptimizer::optimizeTrajectory()
DiffReplanFSM::polyTraj2ROSMsg()
polyTrajCallback()
cmdCallback()
```

能解释每个函数的输入和输出，就掌握了主链路。

### 练习 2：追踪一个障碍点

从 `/laserMapping/cloud_registered` 的一个点开始，依次解释：

```text
cloudCallback()
raycastFromCloud()
setCacheOccupancy()
clearAndInflateLocalMap()
getInflateOccupancy()
finelyCheckAndSetConstraintPoints()
AstarSearch()
obstacleGradCostP()
```

重点回答：为什么 A* 路径没有直接发布给控制器？

### 练习 3：追踪一次重规划

记录旧轨迹的 `traj_id`，观察一秒后新轨迹如何从旧轨迹未来状态开始。理解这点后，再研究为什么局部重规划不会让 `/setpoints_cmd` 每秒跳回当前里程计位置。

### 练习 4：复盘一次安全停止

用调试日志对齐：碰撞检查、重规划失败、`EMERGENCY_STOP`、停止轨迹发布、px4ctrl 回到悬停。区分：

- 规划器紧急停止：保持当前位置；
- `/planning/stop`：停止轨迹服务器输出；
- 自动降落：px4ctrl 生成下降轨迹；
- 手动接管：退出 command-mode 后由操作员控制。

## 15. 最后记住的六句话

1. 全局轨迹是参考，局部优化轨迹才是真正的避障轨迹。
2. A* 提供绕障拓扑和梯度方向，不直接作为飞行轨迹。
3. 地图、定位和控制任何一层出错，规划器都无法独立保证安全。
4. 实机看参数必须追到代码读取点，不能只看 launch 参数名。
5. 降落前必须让 `PositionCommand` 停止，使 px4ctrl 回到 `AUTO_HOVER`。
6. 第一次实飞只做低高度、近距离、单目标，并保存完整日志。
