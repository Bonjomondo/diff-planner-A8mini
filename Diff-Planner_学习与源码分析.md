# Diff-Planner 学习与源码分析

> 本文基于当前仓库源码进行静态分析，面向希望理解、运行、调试和二次开发 Diff-Planner 的读者。重点不是逐文件翻译代码，而是建立“工程结构—运行链路—算法原理—参数—使用与排障”之间的联系。

## 1. 项目定位与结论先行

Diff-Planner 是一个 ROS1 下的无人机局部规划工作空间，源自 EGO-Planner-v2，并针对教育无人机、实机接口和稳定性做了适配。仓库不只包含“规划算法”，还包括：

- 目标点与多航点任务输入；
- 深度图/点云到局部概率占据栅格的建图；
- 全局参考轨迹、滚动局部目标和有限状态机；
- 3D A* 辅助的无 ESDF 轨迹避障；
- MINCO 风格的五次分段多项式轨迹和 L-BFGS 优化；
- 轨迹采样、偏航角生成和控制指令发布；
- 多机轨迹广播与时空避碰；
- 随机地图、局部感知和简化无人机仿真；
- LIO/VIO、PX4 控制器及用户任务接口。

最重要的理解是：

> **A* 不是最终轨迹规划器。** 系统先生成一条五次多项式初始轨迹；当初始轨迹穿过障碍物时，局部 3D A* 只负责寻找“从障碍物哪一侧绕开”的几何提示。之后仍由连续空间轨迹优化器同时优化路径形状和每段时间，最终输出可求位置、速度、加速度和 jerk 的光滑轨迹。

当前源码主链可概括为：

```text
用户目标 / 预设航点
        │
        ▼
全局最小 jerk 参考轨迹（不直接负责避障）
        │
        ▼
按 planning_horizon 截取滚动局部目标
        │
        ▼
生成局部五次多项式初值
        │
        ├─ 初值穿障 ─► 3D A* ─► 障碍基点与排斥方向
        │
        ▼
MINCO 参数化 + L-BFGS 联合优化空间点和分段时间
        │
        ▼
碰撞与动力学复核
        │
        ▼
PolyTraj ─► traj_server 100 Hz 采样 ─► PositionCommand
```

## 2. 仓库现状与分析边界

当前目录包含 24 个 ROS package。核心源代码齐全，但当前工作区并不是一份完整、可直接运行验证的 ROS 环境：

- 当前系统没有 `catkin_make`、`roslaunch`、`roscore` 和 `nvcc`；
- 当前目录没有 `.git` 元数据；
- `faster-lio` 和 `px4ctrl` 两个 Git submodule 目录为空；
- 静态 XML 检查发现一个无效的 package manifest；
- 仿真和部分用户接口 launch 文件还存在参数缺失，见第 14 节。

因此，本文对算法与工程调用关系的结论来自源码、CMake、launch、消息和脚本的交叉核对；运行命令给出了仓库设计用法，但当前快照尚不能直接完成端到端实测。

## 3. 工程结构

### 3.1 顶层目录

```text
Diff-Planner-main/
├── README.md                  项目简介和官方快速使用说明
├── LICENSE                    GPL-3.0
├── .gitmodules                faster-lio、px4ctrl 子模块声明
├── images/                    README 演示图
├── sh_files/                  实机、触发、返程、起降和录包脚本
└── src/                       catkin 工作空间源码
    ├── CMakeLists.txt         catkin 顶层 CMake 文件
    ├── diff_planner/          规划主算法
    ├── uav_simulator/         仿真地图、感知、动力学/控制
    ├── Utils/                 消息、RViz、可视化和工具包
    ├── user_command/          多航点任务接口
    └── realflight_modules/    faster-lio、px4ctrl 子模块
```

### 3.2 核心规划包

| package | 作用 | 建议优先阅读 |
|---|---|---|
| `diff_planner` | 主节点、重规划状态机、模块管理、轨迹服务器 | `diff_replan_fsm.cpp`、`planner_manager.cpp`、`traj_server.cpp` |
| `plan_env` | 深度/点云概率占据栅格、环形缓冲区、raycast、膨胀 | `grid_map.h`、`grid_map.cpp` |
| `path_searching` | 局部 3D A* | `dyn_a_star.cpp` |
| `traj_opt` | MINCO 轨迹工具、障碍约束生成、L-BFGS 优化 | `poly_traj_optimizer.cpp`、`poly_traj_utils.hpp` |
| `traj_utils` | 轨迹容器、自定义消息、RViz 可视化 | `plan_container.hpp`、`PolyTraj.msg`、`MINCOTraj.msg` |
| `swarm_bridge` | 多机轨迹/里程计等消息的 UDP/TCP 桥接 | `bridge_node_udp.cpp` |
| `drone_detect` | 从深度图中检测并擦除其他无人机 | `drone_detector.cpp` |

入口节点在 [`diff_planner_node.cpp`](../src/diff_planner/plan_manage/src/diff_planner_node.cpp)，它本身只创建 `DiffReplanFSM` 并进入 `ros::spin()`；真正的规划逻辑都在状态机和 manager 中。

### 3.3 仿真包

| package | 作用 | 当前主仿真是否使用 |
|---|---|---|
| `map_generator` | 生成柱体、圆环等随机点云地图 | 是 |
| `local_sensing_node` | 从全局地图构造局部点云；可选 CUDA 深度渲染 | 是 |
| `poscmd_2_odom` | 直接把期望位置指令转换成里程计 | 是 |
| `odom_visualization` | 无人机模型和轨迹可视化 | 是 |
| `so3_control` | SO(3) 几何控制器 | 主仿真 launch 中被注释 |
| `so3_quadrotor_simulator` | 四旋翼动力学积分 | 主仿真 launch 中被注释 |
| `mockamap` | 迷宫、Perlin 噪声等其他地图 | 独立工具 |

当前默认仿真是**运动学闭环**而不是严格动力学仿真：`traj_server` 发布期望状态，`poscmd_2_odom` 直接把期望位置、速度写入 odometry。它适合验证规划逻辑，但不能反映电机、姿态控制、延迟和跟踪误差。

### 3.4 工具、消息与用户接口

`Utils/` 中较重要的包包括：

- `quadrotor_msgs`：定义 `PositionCommand`、`TakeoffLand`、`SO3Command` 等消息；
- `rviz_plugins`：3D Nav Goal 等 RViz 工具；
- `manual_take_over`：规划指令与人工接管的切换；
- `odom_visualization`：里程计、模型、路径可视化；
- `moving_obstacles`、`random_goals`、`assign_goals`：实验辅助；
- `uav_utils`、`pose_utils`：坐标和几何工具。

`user_command/multipoint` 从 [`points.yaml`](../src/user_command/multipoint/config/points.yaml) 读取多航点、等待时间、偏航角和返程点，将任务逐点转换成规划器使用的 `/goal`。

### 3.5 实机子模块

[`.gitmodules`](../.gitmodules) 声明：

- `src/realflight_modules/faster-lio`：激光雷达定位建图；
- `src/realflight_modules/px4ctrl`：PX4 外环控制。

这两个目录在当前快照中为空。实机使用必须递归克隆源码或单独补齐相同版本的子模块。

## 4. 单机运行时的节点与数据流

### 4.1 仿真节点关系

以 `run_sim_single.launch` 为例：

```text
random_forest
  └─ /map_generator/global_cloud
             │
             ▼
pcl_render_node + poscmd_2_odom
  ├─ 局部点云 /drone_0_pcl_render_node/cloud
  └─ 里程计   /drone_0_visual_slam/odom
             │
             ▼
drone_0_diff_planner_node
  ├─ 局部概率地图
  ├─ FSM
  ├─ A*
  └─ MINCO/L-BFGS
             │ PolyTraj
             ▼
drone_0_traj_server
             │ PositionCommand
             ▼
manual_take_over
             │
             ▼
poscmd_2_odom（形成简化闭环）
```

单机手动目标由 RViz 的 3D Nav Goal 发布到 `/goal`。多航点模式则是：

```text
/move_base_simple/goal 或 pub_trigger.sh
        │
        ▼
multipointplan
        │ 逐点发布
        ▼
/goal ─► DiffReplanFSM
```

### 4.2 实机 LIO 链路

```text
Livox / IMU ─► faster-lio ─► EKF odometry
                         └─► registered point cloud
                                  │
                                  ▼
                              GridMap

目标 ─► Diff-Planner ─► PolyTraj ─► traj_server
                                       │
                                       ▼
                              /setpoints_cmd
                                       │
                                       ▼
                                   px4ctrl
                                       │
                                       ▼
                                  MAVROS / PX4
```

LIO 配置中使用：

- odometry：`/ekf/ekf_odom`；
- 点云：`/laserMapping/cloud_registered`；
- 输出控制参考：`/setpoints_cmd`。

### 4.3 实机 VIO 链路

VIO 模式使用：

- odometry：`/vins/imu_propagate`；
- 深度图：`/camera/depth/image_rect_raw`；
- 相机内参：`fx/fy/cx/cy`；
- 深度和 odometry 的近似时间同步。

与 LIO 最大区别是地图更新输入：LIO 直接融合世界坐标点云，VIO 将深度像素反投影到相机坐标系，再由相机位姿变换到世界坐标系。

## 5. ROS 接口

以下是主链中最值得关注的话题。实际全名受节点私有命名空间和 launch remap 影响。

| 方向 | 话题 | 消息 | 作用 |
|---|---|---|---|
| 输入 | `/goal` | `geometry_msgs/PoseStamped` | 手动或 multipoint 目标点 |
| 输入 | `/traj_start_trigger` | `geometry_msgs/PoseStamped` | 预设航点/多机任务触发 |
| 输入 | `~odom_world` | `nav_msgs/Odometry` | 规划状态估计 |
| 输入 | `~grid_map/odom` | `nav_msgs/Odometry` | 地图传感器位姿 |
| 输入 | `~grid_map/cloud` | `sensor_msgs/PointCloud2` | LIO 或仿真局部点云 |
| 输入 | `~grid_map/depth` | `sensor_msgs/Image` | 深度相机图像 |
| 输入 | `~grid_map/pose` | `geometry_msgs/PoseStamped` | 相机位姿输入 |
| 输入 | `/mandatory_stop_to_planner` | `std_msgs/Empty` | 强制急停 |
| 输入 | `/broadcast_traj_to_planner` | `traj_utils/MINCOTraj` | 其他无人机轨迹 |
| 输出 | `~planning/trajectory` | `traj_utils/PolyTraj` | 完整多项式系数，供本机执行 |
| 输出 | `/broadcast_traj_from_planner` | `traj_utils/MINCOTraj` | 紧凑轨迹参数，供多机广播 |
| 输出 | `~planning/heartbeat` | `std_msgs/Empty` | 规划器存活心跳 |
| 输出 | `~grid_map/occupancy` | `sensor_msgs/PointCloud2` | 原始占据体素可视化 |
| 输出 | `~grid_map/occupancy_inflate` | `sensor_msgs/PointCloud2` | 膨胀占据体素可视化 |
| 输出 | `position_cmd` | `quadrotor_msgs/PositionCommand` | 100 Hz 位置、速度、加速度、jerk、yaw |

两种轨迹消息用途不同：

- `PolyTraj` 携带每段的 18 个多项式系数和时长，便于 `traj_server` 直接求值；
- `MINCOTraj` 只携带首尾 P/V/A、内部节点和时长，接收方可重新构造轨迹，网络负载更低。

## 6. 地图：局部概率占据栅格

主实现是 [`grid_map.cpp`](../src/diff_planner/plan_env/src/grid_map.cpp)，`grid_map_bigmap.cpp` 当前没有被 CMake 编译，是保留的另一套实现，阅读时不要混淆。

### 6.1 移动环形缓冲区

地图不是固定大小的全局数组，而是以无人机/相机为中心移动的 3D ring buffer。

若参数为：

```xml
local_update_range_x = 5.5
local_update_range_y = 5.5
local_update_range_z = 2.0
resolution = 0.1
```

实际原始缓冲区范围约为 `11 m × 11 m × 4 m`，因为代码使用正负两个方向的半范围：

```text
ringbuffer_size = 2 × local_update_range / resolution
```

无人机跨过体素边界后，缓冲区只清理新移入的一层，而不整体搬移数组。全局体素索引通过取模映射到有限内存地址，这就是环形缓冲区节省内存和复制开销的原因。

### 6.2 深度图反投影

像素 `(u,v)`、深度 `d` 通过针孔模型变成相机坐标：

```text
x_c = (u - cx) d / fx
y_c = (v - cy) d / fy
z_c = d
```

再用相机旋转和平移变到世界坐标：

```text
p_w = R_wc p_c + t_wc
```

代码支持 `32FC1` 和 `16UC1`，并用 `k_depth_scaling_factor` 完成米与深度整数值的换算。`skip_pixel` 控制下采样，默认每隔 2 个像素处理一次。

### 6.3 Raycasting 与 log-odds 更新

每个观测点的射线终点被视为命中，传感器到终点之间的体素被视为空闲。概率更新使用 log-odds：

```text
L(p) = log(p / (1-p))
L_t = clamp(L_{t-1} + L_measurement, L_min, L_max)
```

使用 log-odds 后，反复观测只需做加法，并可用阈值判断占据状态。深度图和激光点云有各自的 `p_hit/p_miss/p_occ` 参数；当前新增的 `raycastFromCloud()` 为点云提供了沿射线清空自由空间的能力，避免只把点云终点堆成永久障碍。

### 6.4 障碍物膨胀和虚拟边界

原始占据体素会在 `obstacles_inflation` 范围内膨胀。轨迹规划读取的是膨胀地图，相当于把无人机安全半径转移到障碍物上。

需要注意：

- 膨胀最大限制为 4 个体素；
- 请求的膨胀超过 4 个体素时，代码会自动增大地图分辨率；
- `virtual_ground` 和 `virtual_ceil` 可构造上下虚拟墙；
- 局部缓冲区之外在当前实现中返回“自由”，而不是“未知/占据”。

规划器只对局部轨迹前约 `2/3` 做主要约束检查。默认仿真中传感范围约 5 m、局部地图半范围 5.5 m、规划视野 7.5 m，`7.5 × 2/3 = 5 m`，这三个数字是配套设计的，而不是互相矛盾。

### 6.5 地图超时保护

地图记录最后一次深度/点云更新时刻。超过 `odom_depth_timeout` 后，安全回调把状态切换到 `EMERGENCY_STOP`。这能防止传感器或定位掉线后继续沿旧地图高速飞行。

## 7. 全局参考轨迹与滚动局部目标

收到目标点后，`planGlobalTrajWaypoints()` 生成穿过目标点的全局最小 jerk 轨迹。这里的“全局”是参考意义上的：

- 它连接当前状态与最终目标；
- 它保证 P/V/A 边界连续；
- 它**不使用障碍物地图，不保证无碰撞**。

局部规划每次沿全局轨迹向前搜索，找到距离当前起点约 `planning_horizon` 的点作为 `local_target`。若已经接近全局末端，则直接把最终目标作为局部目标，并设置 `touch_goal=true`。

局部目标速度的处理：

- 离最终目标仍较远：采用全局参考轨迹在该处的速度；
- 已进入制动距离 `v_max²/(2a_max)`：目标速度设为 0。

这种设计把长距离导航拆成不断向前滚动的短问题，计算量与环境大小基本解耦。

## 8. 五次多项式与 MINCO 参数化

### 8.1 为什么用五次多项式

每个三维轨迹段为：

```text
p_i(t) = c_0 + c_1 t + c_2 t² + c_3 t³ + c_4 t⁴ + c_5 t⁵
```

对其求导可直接得到速度、加速度和 jerk。五次多项式有 6 组系数，足以在一段两端同时约束位置、速度和加速度。

### 8.2 最小 jerk

平滑代价为：

```text
J_smooth = Σ_i ∫₀ᵀⁱ ||p_i'''(t)||² dt
```

代码在 `MinJerkOpt::getTrajJerkCost()` 中直接计算该积分的闭式表达式。最小 jerk 常用于无人机，是因为：

- 位置、速度、加速度连续；
- 控制参考更平滑；
- 高频姿态/推力变化更少；
- jerk 又能直接作为 `PositionCommand` 的前馈量。

### 8.3 降维思想

优化器不直接把所有多项式系数作为自由变量，而是优化：

- `N-1` 个内部位置，每个 3 维；
- `N` 个分段时间。

总变量数为：

```text
3(N-1) + N = 4N - 3
```

给定首尾 P/V/A、内部位置和时间后，多项式系数由带状线性系统唯一解出。这正是 MINCO 类参数化高效的核心：在低维变量空间中优化，同时自动维持段间连续性。

为了保证时间始终为正，代码把无约束虚时间映射为真实时间，再通过链式法则把时间梯度传回 L-BFGS。

## 9. 初值、3D A* 与 rebound 避障

### 9.1 局部初值

局部初值有三种来源：

1. 首次规划或新目标：起点到局部目标的多项式；
2. 滚动重规划：沿用上一条局部最优轨迹的剩余部分，再接全局轨迹；
3. 连续失败：在中点附近加入随机横向/竖向偏移，尝试跳出坏的局部最优。

轨迹段数大致为：

```text
piece_num ≈ distance / polyTraj_piece_length
```

且至少为 2 段。每段初始时间约为：

```text
piece_time = polyTraj_piece_length / max_vel
```

### 9.2 为什么仍需要 A*

无 ESDF 的优化器不知道应该从障碍物左侧、右侧、上方还是下方绕行。如果只用“碰撞点向外推”的局部梯度，初始轨迹深穿障碍时方向可能不可靠。

代码先沿初始连续轨迹密集采样，识别每个“进入障碍—离开障碍”区段。对每个区段：

1. 取障碍段前后的自由控制点；
2. 在局部 3D 栅格中运行 A*；
3. 将 A* 路径与初始轨迹法平面求交；
4. 沿交点方向回扫到障碍边界；
5. 为对应约束点记录：
   - `base_point`：障碍边界基点；
   - `direction`：期望把轨迹推出障碍物的单位方向。

优化器之后不再跟踪整条 A* 折线，而是用这些局部半空间方向形成可微障碍代价。

### 9.3 A* 实现特点

[`dyn_a_star.cpp`](../src/diff_planner/path_searching/src/dyn_a_star.cpp) 的特点：

- 预分配 `100 × 100 × 100` 节点池；
- 搜索区域以当前起终点中点为中心；
- 26 邻域扩展；
- 对角距离启发式并加轻微 tie breaker；
- 单次搜索有 0.2 s 超时；
- 若起点或终点在障碍物内，会沿两点连线向外推到自由体素；
- 每轮通过 `rounds` 标记复用节点池，避免反复清空一百万个节点。

### 9.4 Rebound

L-BFGS 迭代过程中，如果轨迹又碰到原来没覆盖的新障碍，`roughlyCheckConstraintPoints()` 会重新运行局部 A*，生成新的排斥方向，并通过 early-exit 中止当前 L-BFGS。外层循环带着新约束重新启动优化，这就是 “rebound”。

它不是物理弹簧仿真，而是：

```text
发现新碰撞 → 重建可微避障方向 → 重启连续优化
```

## 10. L-BFGS 轨迹优化目标

优化总代价在代码中可理解为：

```text
J =
  J_jerk
  + J_obstacle
  + J_swarm
  + J_feasibility
  + J_spacing
  + w_time ΣT_i
```

### 10.1 障碍代价

对轨迹采样点 `p`、障碍基点 `b` 和排斥方向 `d`：

```text
distance = (p - b) · d
error = clearance - distance
```

当 `error > 0` 时加入三次惩罚。代码同时有：

- `weight_obstacle` + `obstacle_clearance`：较硬的近障约束；
- `weight_obstacle_soft` + `obstacle_clearance_soft`：更远距离的柔性引导。

由于地图本身已做膨胀，最终安全裕量由“地图膨胀 + 优化 clearance”共同决定。

### 10.2 动力学可行性

速度、加速度、jerk 超过上限时使用三次软惩罚，例如：

```text
J_v ∝ max(||v||² - v_max², 0)³
```

优化结束后还会沿整条轨迹重新密集采样，执行独立的硬复核：

```text
||v|| ≤ max_vel + vel_tolerance
||a|| ≤ max_acc + acc_tolerance
```

如果复核失败，最多重新优化 3 次。该后验检查是 Diff-Planner 相对原框架增加的重要安全补丁，避免软约束轻微违反后仍发布轨迹。

### 10.3 时间代价

`weight_time × ΣT_i` 鼓励更短的执行时间。它和动力学代价形成对抗：

- 时间太短：速度、加速度和 jerk 超限；
- 时间太长：时间代价上升；
- 优化器在两者间寻找平衡。

### 10.4 采样间距代价

代码函数名是 `distanceSqrVarianceWithGradCost2p()`，但当前实现并没有减去均值，实际更接近对相邻采样点距离四次方均值的惩罚。它会抑制采样点过分拉开，并间接改善数值稳定性；不要把当前实现严格解释成统计学方差。

### 10.5 多机避碰

对其他无人机在相同时刻的预测位置，使用竖直方向更宽松的椭球距离：

```text
d²_ellipsoid = dx² + dy² + dz² / 4
```

当距离小于双方 clearance 之和的放大值时加入三次惩罚。每轮优化后还会检查最小椭球距离；若仍太近，集群权重翻倍并重启优化。

### 10.6 数值积分

每个多项式段默认取 `constraint_points_perPiece = 5`，用梯形权重近似积分障碍、集群和动力学惩罚。增加该参数会提高约束采样密度，但也线性增加每次代价与梯度计算量。

## 11. 重规划状态机

状态定义在 [`diff_replan_fsm.h`](../src/diff_planner/plan_manage/include/plan_manage/diff_replan_fsm.h)：

```text
INIT
  │ 收到 odom
  ▼
WAIT_TARGET
  │ 收到目标和触发
  ▼
SEQUENTIAL_START
  │ 首次规划成功
  ▼
EXEC_TRAJ ─────────────┐
  │ 定时/碰撞需重规划   │ 新局部轨迹成功
  ▼                    │
REPLAN_TRAJ ───────────┘
  │ 连续失败或临近碰撞
  ▼
EMERGENCY_STOP
  ├─ 可恢复：GEN_NEW_TRAJ ─► EXEC_TRAJ
  └─ 需人工新目标：WAIT_TARGET
```

### 11.1 定时频率

- FSM 定时器：0.01 s，即 100 Hz；
- 安全碰撞检查：0.05 s，即 20 Hz；
- 地图更新：约 31.25 Hz；
- 地图可视化：8 Hz；
- 轨迹指令采样：100 Hz。

### 11.2 重规划起点连续性

滚动重规划时，起点不是当前 odometry 的瞬时值，而是当前正在执行轨迹在当前时刻的：

```text
start_pos = p_old(t)
start_vel = v_old(t)
start_acc = a_old(t)
```

这样新旧轨迹在切换时保持 P/V/A 连续，避免控制参考跳变。首次规划或重新起步才直接使用 odometry。

### 11.3 安全检查

安全线程对当前轨迹未来采样点检查：

- 膨胀地图碰撞；
- 与其他无人机预测轨迹的距离；
- 深度/点云和 odometry 更新超时。

若发现危险：

1. 先立即尝试一次局部重规划；
2. 若失败且碰撞时间小于 `emergency_time`，急停；
3. 否则转到 `REPLAN_TRAJ`。

急停轨迹是停留在当前位置的两段最小 jerk 轨迹，并以零速度、零加速度作为边界。

### 11.4 目标点落在障碍物内

启用 `fsm/mondify_final_goal` 时，系统沿已生成的全局参考轨迹从末端向前回退，寻找自由点，并把它改成新的最终目标。禁用该功能时，目标点占据会直接触发急停。

### 11.5 卡死与失败保护

当前版本增加了两类保护：

- 局部目标长时间变化小于 0.3 m，判为被大障碍物卡住；
- 沿起点到终点方向长期无进展，或绕行距离过大，判为徘徊；
- 连续重规划失败超过 10 次，进入急停。

`TARGET_STUCK_TIME` 约为：

```text
1.5 × planning_horizon / max_vel
```

因此调高视野或调低最大速度，会同时延长卡死判定时间。

## 12. 轨迹服务器与偏航控制

`traj_server` 接收 `PolyTraj`，恢复每段系数，并以 100 Hz 求：

- position；
- velocity；
- acceleration；
- jerk；
- yaw；
- yaw_dot。

默认 yaw 朝向 `time_forward` 秒后的轨迹位置：

```text
yaw_des = atan2(y_future - y_now, x_future - x_now)
```

并限制偏航角速度和偏航角加速度。`multipoint` 也可以通过 `/planning/yaw` 临时覆盖自动朝向；`yaw <= -100` 是“不强制指定”的哨兵值。

心跳是另一层故障保护：`traj_server` 超过 0.5 s 收不到规划器 heartbeat，会停止接受当前轨迹并发布当前位置悬停指令。

当前代码有一个需要特别注意的问题：`calculate_yaw()` 先正确算出 `yawdot`，随后又把返回 pair 的第二项覆盖为 `yaw_temp`。因此实际发布的 `PositionCommand.yaw_dot` 是目标 yaw 值而不是角速度。实机使用前应修复这一行并完成控制器联调。

## 13. 多机规划与通信

### 13.1 轨迹共享

每架无人机把本机轨迹压缩成 `MINCOTraj` 广播。接收端根据首尾状态、内部节点和时长重建五次轨迹，然后：

- 忽略自身轨迹；
- 检查轨迹阶数和消息维度；
- 丢弃重复或旧轨迹；
- 检查机器间时钟差；
- 忽略远离自身规划区域的轨迹；
- 将有效轨迹加入优化与在线碰撞检查。

时间同步非常重要：时间戳差大于 10 s 的轨迹会直接丢弃。实机集群应使用 NTP/chrony 等方式统一时钟。

### 13.2 顺序启动

`SEQUENTIAL_START` 试图让高 ID 无人机先收到低 ID 无人机轨迹，再开始规划，从而减少所有飞机同时发布互相冲突初值的概率。

当前 `have_recv_pre_agent_` 的循环逻辑在收到部分较低 ID 轨迹后就可能置为 `true`，未严格确认每个前序 ID 都有效。大规模集群使用前建议修正并做丢包测试。

### 13.3 UDP 桥

UDP 桥固定使用 8081 端口，把 ROS 序列化数据加上消息类型和长度后进行局域网广播，支持：

- odometry；
- 单机轨迹；
- stop；
- goal；
- joystick。

它轻量、延迟低，但没有确认重传、版本协商、加密和严格的包长度防护，更适合同构设备上的可信局域网实验。

### 13.4 TCP 桥现状

仓库有 TCP launch 和源码，但 `swarm_bridge/CMakeLists.txt` 中 `ENABLE_TCP` 默认为 `false`。因此 `run_swarm.sh` 启动 `bridge_node_tcp` 时，默认构建并不会生成这个可执行文件。若要使用 TCP，需要：

- 开启 `ENABLE_TCP`；
- 安装 ZeroMQ、`zmq`/`zmqpp` 开发依赖；
- 重新编译；
- 检查各无人机 IP 和地面站 IP。

## 14. 用户命令、多航点和返程

### 14.1 四种模式

代码中的参数拼写是 `fligt_type`，不是 `flight_type`，配置时必须沿用这个拼写。

| `fligt_type` | YAML 区域 | 每点格式 |
|---:|---|---|
| 1 | `test1` | `[x, y, z]` |
| 2 | `test2` | `[x, y, z, wait_time]` |
| 3 | `test3` | `[x, y, z, yaw]` |
| 4 | `test4` | `[x, y, z, yaw, wait_time]` |

返程点固定读取 `test_back`，格式为 `[x, y, z]`。

### 14.2 推进到下一个航点

节点持续比较当前 odometry 和活动航点的欧氏距离。当距离小于 `next_distance`：

- 可先阻塞等待 `wait_time`；
- 发布下一个 `/goal`；
- 最后一个点到达后可自动降落。

这是一层任务调度，不是规划器内部的全局多点联合优化。每次发布新 `/goal`，FSM 都重新生成一条全局参考和局部轨迹。

### 14.3 触发话题

- `/move_base_simple/goal`：开始执行正向任务；
- `/back_trigger`：切换到 `test_back`；
- `/planning/yaw`：发送当前航点的自定义 yaw；
- `/px4ctrl/takeoff_land`：起飞/降落；
- 遥控器第 8 通道：按次序执行起飞、前往、返程、降落。

## 15. 参数理解与调参顺序

### 15.1 关键参数

| 参数 | 含义 | 调大后的主要影响 |
|---|---|---|
| `grid_map/resolution` | 体素分辨率 | 更省内存、更快，但窄障碍和间隙表达更粗 |
| `grid_map/local_update_range_*` | 环形地图半范围 | 可看更远，但内存和地图更新代价快速增加 |
| `grid_map/obstacles_inflation` | 障碍膨胀 | 更安全，但可通行空间减少 |
| `fsm/planning_horizon` | 滚动局部目标距离 | 轨迹更有前瞻性，但优化问题更大 |
| `fsm/thresh_replan_time` | 定时重规划阈值 | 重规划更少，但对新障碍响应变慢 |
| `fsm/emergency_time` | 碰撞前急停阈值 | 更早急停，但可能更保守 |
| `manager/polyTraj_piece_length` | 初值每段大致长度 | 段数减少、速度快，但表达能力下降 |
| `optimization/constraint_points_perPiece` | 每段约束采样数 | 检查更细，但优化更慢 |
| `optimization/weight_obstacle*` | 避障权重 | 更排斥障碍，可能牺牲平滑和收敛 |
| `optimization/weight_feasibility` | 动力学惩罚 | 更遵守约束，可能增加时间 |
| `optimization/weight_time` | 总时长惩罚 | 更快，但更容易碰到动力学上限 |
| `optimization/vel_tolerance` | 后验速度容忍 | 更易通过复核，但会放宽安全边界 |
| `optimization/acc_tolerance` | 后验加速度容忍 | 同上 |
| `optimization/swarm_clearance` | 本机多机安全半径 | 多机更保守，狭窄区域更难规划 |

### 15.2 推荐调参顺序

1. 先确认坐标系、内参、深度尺度、点云和 odometry 时间同步正确；
2. 设定真实无人机半径对应的地图膨胀；
3. 用低 `max_vel/max_acc/max_jer` 完成空旷飞行；
4. 调地图分辨率、局部范围和 planning horizon；
5. 再调 obstacle clearance 和避障权重；
6. 最后调时间权重、动力学权重和容忍值；
7. 多机实验最后再加入 swarm clearance 与通信延迟。

不要用增大容忍值来掩盖定位抖动、控制跟踪差或时间同步问题。

## 16. 使用说明

### 16.1 建议环境

项目 README 声明支持 Ubuntu 16.04、18.04、20.04 对应的 ROS1。实际新部署优先使用仓库明确支持、依赖相对容易获得的 Ubuntu 20.04 + ROS Noetic，并使用 C++14。

主要依赖包括：

- ROS1/catkin、roscpp、message_filters；
- Eigen3；
- PCL；
- OpenCV、cv_bridge；
- yaml-cpp；
- RViz/Qt；
- MAVROS 与 `mavros_msgs`（实机和 multipoint）；
- 可选 CUDA（默认关闭）；
- 实机的 faster-lio、px4ctrl、相机/雷达驱动。

### 16.2 获取完整源码

```bash
git clone --recursive https://github.com/DifferentialRobotics/Diff-Planner.git
cd Diff-Planner
```

若已经是正常 Git clone：

```bash
git submodule update --init --recursive
```

当前提供的快照没有 `.git` 元数据，所以不能在这里直接执行 submodule update；应重新递归克隆或手动补齐两个子模块。

### 16.3 依赖与编译

在修复第 17 节的构建阻塞项后，可按 ROS 工作空间方式执行：

```bash
source /opt/ros/noetic/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

使用 zsh 时最后一行换成：

```zsh
source devel/setup.zsh
```

### 16.4 单机手动仿真

```bash
source devel/setup.bash
roslaunch diff_planner run_sim_single.launch
```

在 RViz 中使用 3D Nav Goal：

1. 左键确定 x-y；
2. 左键保持时配合右键上下拖动 z；
3. 松开后发布 `/goal`。

运行后建议先核对：

```bash
rostopic hz /drone_0_visual_slam/odom
rostopic hz /drone_0_pcl_render_node/cloud
rostopic hz /drone_0_planning/pos_cmd
rostopic echo -n 1 /drone_0_planning/trajectory
```

### 16.5 单机预设多航点仿真

编辑：

- [`points.yaml`](../src/user_command/multipoint/config/points.yaml)；
- [`multipointplan_sim.launch`](../src/user_command/multipoint/launch/multipointplan_sim.launch) 中的 `fligt_type` 和 `next_distance`。

启动仿真后，在另一个终端：

```bash
source devel/setup.bash
./sh_files/pub_trigger.sh
```

返程：

```bash
./sh_files/back.sh
```

### 16.6 多机仿真

在 [`run_sim_swarm.launch`](../src/diff_planner/plan_manage/launch/sim/run_sim_swarm.launch) 设置每架无人机的初始点和目标点：

```bash
source devel/setup.bash
roslaunch diff_planner run_sim_swarm.launch
```

另一个终端触发：

```bash
./sh_files/pub_swarm_trigger.sh
```

多机前应确认：

```bash
rostopic hz /broadcast_traj_from_planner
rostopic hz /broadcast_traj_to_planner
```

### 16.7 LIO 实机流程

以下流程只适用于已经完成硬件、MAVROS、LIO、EKF 和 px4ctrl 联调的机器：

1. 检查 `run_exp_single_lio.launch` 中点云、odometry、速度和地图参数；
2. 检查 `points.yaml`；
3. 检查 px4ctrl 的起飞高度和安全参数；
4. 遥控器保持可随时急停；
5. 启动整套节点；
6. 确认定位、点云、局部地图和 heartbeat；
7. 起飞；
8. 触发任务；
9. 需要时返程；
10. 降落、锁桨、停止节点。

当前实际脚本名是：

```bash
./sh_files/run_single_lio.sh
```

而 README 中有一处写成不存在的 `run_all_lio.sh`。

### 16.8 VIO 实机流程

先启动相机与 VINS，查看：

```bash
rostopic echo /camera/depth/camera_info
```

相机矩阵通常排列为：

```text
[fx, 0, cx,
  0, fy, cy,
  0,  0,  1]
```

把 `fx/fy/cx/cy` 写入 `run_exp_single_vio.launch`，再运行：

```bash
./sh_files/run_single_vio.sh
```

还必须确认深度图单位与 `k_depth_scaling_factor` 匹配，否则地图尺度会完全错误。

### 16.9 起飞、触发、返程和降落

```bash
./sh_files/takeoff.sh
./sh_files/pub_trigger.sh
./sh_files/back.sh
./sh_files/land.sh
```

这些脚本会向真实控制话题发布命令。实机上执行前必须确认桨叶区域清空、遥控器急停有效、坐标和任务点正确。

## 17. 当前仓库的已知问题与运行前检查

这些问题来自当前源码静态核对，不代表上游后续版本仍然存在。

### 17.1 会直接阻塞构建或启动的问题

1. `selected_points_publisher/package.xml` 的 XML 声明是：

   ```xml
   <?xml version="0.1.0"?>
   ```

   XML 版本应为 `1.0`。当前 manifest 无法解析，catkin 包发现阶段就可能失败。

2. `simulator.xml` 声明了无默认值的 `c_num`、`p_num`、`min_dist`，但 `run_in_sim.xml` include 时没有传入，而且当前文件也没有使用它们。应删除这些 arg、给默认值或在 include 时补齐。

3. `multipointplan.cpp` 强制读取 `auto_planning` 和 `auto_landing`；但仿真与 VIO multipoint launch 没有设置这两个参数，节点会打印 `Failed to get parameter` 后退出。应像 LIO launch 一样补上默认值。

4. 当前快照的 `faster-lio`、`px4ctrl` 子模块为空，实机代码和配置不完整。

5. `run_swarm.sh` 启动 TCP bridge，但 CMake 默认不构建 TCP bridge。

### 17.2 依赖声明和辅助包问题

- `multipoint` 使用 `mavros_msgs/RCIn`、`nav_msgs`、yaml-cpp，但 CMake/package manifest 没有完整声明；
- `rviz_plugins` 依赖 `multi_map_server_generate_messages_cpp`，对仅运行主规划器并非必要，却可能成为全工作空间构建阻塞项；
- `local_sensing_node` 的 package manifest 保留了 `svo_msgs`、`vikit_ros` 等旧依赖，而默认非 CUDA 构建未实际使用全部依赖；
- 多个 package 的 license 字段仍是 `TODO`，不影响算法但不利于重新发布。

如果只想学习核心规划，可先修复 manifest，并临时让无关辅助包加入 `CATKIN_IGNORE`，再逐个恢复。

### 17.3 代码层注意项

- `traj_server` 的 `yaw_dot` 返回值被错误覆盖，见第 12 节；
- 多机前序轨迹确认逻辑不严格，见第 13 节；
- 多航点等待使用阻塞 `sleep`，会暂停 multipoint 节点自己的其他回调；
- `grid_map_bigmap.*` 未编译，launch 中若干与 bigmap 对应的参数对当前 active map 无效；
- `manager/feasibility_tolerance`、`optimization/record_opt` 等参数当前读取后未用于主逻辑或根本未读取；
- 仿真默认不是动力学仿真；
- 实机脚本内硬编码了 sudo 密码并对 `/dev/tty*` 执行宽泛的 `chmod 777`。应删除明文密码，改用精确设备的 udev 规则；
- 当前目录没有 Git 元数据，无法确认 commit、分支或本地修改状态。

### 17.4 建议的启动前自检

```bash
roslaunch --files diff_planner run_sim_single.launch
roslaunch --nodes diff_planner run_sim_single.launch
roswtf
rqt_graph
```

运行时逐项确认：

- odometry 频率、时间戳、坐标轴和单位；
- 点云/深度图频率与 frame；
- 膨胀地图是否跟随无人机移动；
- `/goal` 是否被 FSM 接收；
- `PolyTraj` 是否持续更新；
- `PositionCommand` 是否为 100 Hz；
- heartbeat 是否稳定；
- 轨迹最大速度、加速度是否符合控制器能力；
- 多机时系统时间和网络广播是否正常。

## 18. 常见故障定位

### 18.1 一直停在 `INIT`

检查：

```bash
rostopic hz <odom_topic>
rostopic echo -n 1 <odom_topic>
```

重点看 launch remap 是否产生了重复的 `/drone_0_` 前缀。

### 18.2 一直停在 `WAIT_TARGET`

- 手动模式检查 `/goal`；
- 预设模式检查 `/traj_start_trigger`；
- 查看 `fsm/flight_type` 是 1 还是 2；
- 仿真 launch 中 `realworld_experiment=true`，所以仍需要目标/trigger 正常到达。

### 18.3 `Local target in collision`

可能原因：

- 地图膨胀过大；
- planning horizon 超过可信感知范围；
- 最终目标在障碍物中；
- 点云坐标系或深度尺度错误；
- 初值完全穿过大障碍，A* 0.2 s 内未找到路径。

可依次观察原始地图、膨胀地图、`init_list`、`a_star_list`、`optimal_list`。

### 18.4 频繁动力学复核失败

- 降低 `max_vel`；
- 增加 `weight_feasibility`；
- 降低 `weight_time`；
- 缩短 `polyTraj_piece_length`，提高轨迹表达能力；
- 检查起点 odometry 速度是否已经超限；
- 最后才考虑小幅增加 tolerance。

### 18.5 地图里障碍拖影或消失

- LIO 模式检查 `cloud_enable_raycast`；
- 深度模式检查相机内参与深度尺度；
- 检查 `p_hit/p_miss/p_occ`；
- 动态环境调小 `fading_time`；
- 静态 LIO 地图可禁用 fading；
- 检查 odometry 和传感器时间同步。

### 18.6 轨迹发布但飞机不动

顺着链路检查：

```text
PolyTraj
  → traj_server heartbeat
  → PositionCommand
  → manual_take_over
  → px4ctrl 状态
  → MAVROS/PX4 模式
```

不要只检查规划器日志。

## 19. 推荐源码阅读顺序

### 第一阶段：建立工程主线

1. [`run_sim_single.launch`](../src/diff_planner/plan_manage/launch/sim/run_sim_single.launch)
2. [`run_in_sim.xml`](../src/diff_planner/plan_manage/launch/include/run_in_sim.xml)
3. [`advanced_param_sim.xml`](../src/diff_planner/plan_manage/launch/include/advanced_param_sim.xml)
4. [`diff_planner_node.cpp`](../src/diff_planner/plan_manage/src/diff_planner_node.cpp)
5. [`diff_replan_fsm.cpp`](../src/diff_planner/plan_manage/src/diff_replan_fsm.cpp)

目标：能说清节点、话题和状态转移。

### 第二阶段：理解轨迹数据

1. [`plan_container.hpp`](../src/diff_planner/traj_utils/include/traj_utils/plan_container.hpp)
2. [`PolyTraj.msg`](../src/diff_planner/traj_utils/msg/PolyTraj.msg)
3. [`MINCOTraj.msg`](../src/diff_planner/traj_utils/msg/MINCOTraj.msg)
4. [`poly_traj_utils.hpp`](../src/diff_planner/traj_opt/include/optimizer/poly_traj_utils.hpp)
5. [`traj_server.cpp`](../src/diff_planner/plan_manage/src/traj_server.cpp)

目标：能从多项式系数手算 P/V/A/J，并理解首尾状态、内部点、分段时间如何确定轨迹。

### 第三阶段：理解局部规划

1. [`planner_manager.cpp`](../src/diff_planner/plan_manage/src/planner_manager.cpp)
2. [`poly_traj_optimizer.h`](../src/diff_planner/traj_opt/include/optimizer/poly_traj_optimizer.h)
3. [`poly_traj_optimizer.cpp`](../src/diff_planner/traj_opt/src/poly_traj_optimizer.cpp)
4. [`dyn_a_star.cpp`](../src/diff_planner/path_searching/src/dyn_a_star.cpp)

目标：能画出“初值—碰撞段—A*—方向约束—L-BFGS—复核”的完整流程。

### 第四阶段：理解感知地图

1. [`grid_map.h`](../src/diff_planner/plan_env/include/plan_env/grid_map.h)
2. [`grid_map.cpp`](../src/diff_planner/plan_env/src/grid_map.cpp)
3. [`raycast.cpp`](../src/diff_planner/plan_env/src/raycast.cpp)
4. [`pointcloud_render_node.cpp`](../src/uav_simulator/local_sensing/src/pointcloud_render_node.cpp)

目标：掌握体素索引、环形数组、log-odds、raycast 和 inflation。

### 第五阶段：扩展功能

- 多机：`RecvBroadcastMINCOTrajCallback()`、`swarmGradCostP()`、`swarm_bridge`；
- 实机：`run_exp_single_lio.launch`、`run_exp_single_vio.launch`、px4ctrl；
- 任务：`multipointplan.cpp`；
- 其他无人机检测：`drone_detect`。

## 20. 需要补充的基础知识

建议按以下顺序学习：

1. **ROS1**：node、topic、parameter、namespace、remap、launch、catkin；
2. **坐标变换**：旋转矩阵、四元数、齐次变换、相机坐标到世界坐标；
3. **栅格地图**：体素化、概率占据、log-odds、raycasting、障碍膨胀；
4. **图搜索**：A* 的 `g+h`、启发式、open/closed set、26 邻域；
5. **轨迹生成**：多项式、边界条件、连续性、minimum jerk；
6. **数值优化**：梯度、链式法则、软约束、L-BFGS、局部最优；
7. **滚动时域规划**：局部视野、重规划、轨迹拼接和安全检查；
8. **四旋翼控制**：差分平坦性、位置/速度/加速度/jerk 前馈、yaw；
9. **多机器人系统**：轨迹预测、时空冲突、时钟同步和通信丢包。

## 21. 适合做的二次开发

由易到难可选择：

1. 修复第 17 节的构建与 launch 问题，建立可重复 CI；
2. 为 FSM、轨迹消息和参数增加单元测试；
3. 修复 yaw_dot 并增加轨迹服务器测试；
4. 把 multipoint 阻塞等待改为非阻塞状态机；
5. 增加未知空间策略，不再默认把局部地图外视为自由；
6. 加入轨迹跟踪误差，把实机实际位置而不只是参考轨迹用于安全裕量；
7. 改进 A* 节点池和超时策略；
8. 增加网络丢包、延迟和时钟偏差仿真；
9. 恢复 SO(3) 控制器与动力学仿真，建立规划—控制联合测试；
10. 将 ROS1 接口迁移到 ROS2，并把参数和 QoS 显式化。

## 22. 一句话总结每个核心类

- `GridMap`：把深度/点云变成随无人机移动的膨胀概率体素地图；
- `AStar`：为穿障的局部初值找一个可行绕障方向；
- `MinJerkOpt`：由边界状态、内部点和时间高效生成五次最小 jerk 轨迹；
- `PolyTrajOptimizer`：对内部点和时间做可微联合优化；
- `DiffPlannerManager`：串联初值、A* 约束、优化、轨迹容器和碰撞检查；
- `DiffReplanFSM`：决定何时规划、重规划、执行、急停和接收新目标；
- `traj_server`：把多项式轨迹变成控制器可消费的 100 Hz 状态指令；
- `multipointplan`：把 YAML 任务和遥控器操作转换成逐个目标点；
- `swarm_bridge`：在机器间转发紧凑轨迹和其他 ROS 消息。

理解这九个对象及其数据流，就已经掌握了整个项目的主体。
