# Changelog

本文件记录 Diff-Planner A8 mini 分支的重要功能、配置和实机行为变更。

## Unreleased - 2026-10-03

- 实机 RViz 默认使用 `exp_single_lio.rviz`，从 42 机仿真配置切换为 10 项单机显示；
  保留 world、实机点云、EKF、膨胀地图与点击航线。新增 `rviz_config` / `rviz_args` 参数。
- debug 入口保存 RViz/LIO 配置快照、点云频率和 Ogre 渲染日志；修复采集器因 supervisor
  中间进程误判 launcher 退出、导致 CPU/GPU 诊断在启动前停止的问题。
- 新增 [20261003_162838 黑屏分析](docs/20261003_162838_RViz黑屏分析.md)。旧/新配置独立
  GPU 渲染均能显示 Grid；持续黑屏的桌面呈现原因及完整栈显示结果仍需核对。

## Unreleased - 2026-09-30

### Added / Changed

- **检测单仓整合**：A8 mini 实时检测的代码、类别表与模型并入 `multipoint` 包，
  正常启动不再依赖外部 `~/Documents/A8mini_Detction`。
  - 新增 `models/yolo11s.engine`、`models/yolo11s.names.json` 及
    `scripts/a8mini_labels.py`（类别名加载校验与画框）。
  - `a8mini_detection.launch` 新增 `model` 参数，默认
    `$(find multipoint)/models/yolo11s.engine`，可用 `model:=绝对路径` 切换；
    类别表须与模型同目录同基名（`custom.engine` ↔ `custom.names.json`），
    缺失、含占位名或编号不连续时在加载推理环境前报错。
  - `a8mini_detection.py` 移除 `repo_path` 及外部 `rtsp_capture.py`、
    `A8mini_RTSP_YOLO_Detection.py` 依赖；`a8mini_detection_node.py` 不再传递
    `repo_path`。
  - `a8mini_diagnostics.py` 增加 `sha256_file`，manifest 记录模型路径/大小/修改时间
    /SHA-256，源码快照改为本仓库检测、采集、录像、诊断、标签脚本及所用类别表。
  - `a8mini_detection.yaml` 的 `python_executable` 指向工作空间
    `.venv-a8mini/bin/python`，并移除 `repo_path`/`model` 键；`.venv-a8mini/`
    加入 `.gitignore`。
  - `CMakeLists.txt` 安装 `a8mini_labels.py` 与 `models` 目录。
- **本机环境重建 / OpenCV 修复**：删除 `plan_env/CMakeLists.txt` 中失效的
  `~/Documents/opencv-4.6.0` 路径引用，改用系统 OpenCV 重新构建整个 catkin 工作空间。
- **RViz 配置**：`exp.rviz` 新增 `lidar_map` 显示 `/laserMapping/cloud_registered`，
  并移除订阅空话题 `/camera/color/image_raw` 的根图像显示。
- **雷达 / 网络配置迁移到本机**：
  - `MID360_config.json`、`MID360s_config.json` 的 host IP 由 `192.168.1.50`
    改为 `192.168.1.5`，雷达 IP 由 `192.168.1.171` 改为 `192.168.1.190`。
  - `mapping_mid360.launch` 的 `user_config_path` 由 `MID360s_config.json`
    改为 `MID360_config.json`。
  - `mid360.yaml` 更新 IMU-LiDAR 外参，并新增 `publish/tf_world_frame: world`，
    对齐 EKF / 规划器 / RViz 的世界系。
- **文档**：新增 [检测单仓整合说明](A8mini/20260930_A8mini检测单仓整合说明.md)、
  [2026-09-29 全问题诊断报告](docs/20260929_全问题诊断报告.md) 和
  [达妙载板网口驱动说明](A8mini/达妙载板网口驱动_r8125_installed.md)；同步更新
  `A8mini/README.md`、`docs/A8mini实机操作指南.md` 及历史日志分析报告。

### Verification

- 独立检测单测、独立 catkin 编译与 install 布局检查通过。
- 本机宿主机 PyTorch CUDA 可用，仓库 engine 的 TensorRT 预热成功；只读 RTSP 检测
  约 25 s 收到 400 帧、处理 379 帧、重连 0 次；桌面显示运行约 22 s 稳定在
  24–25 FPS 并正常退出。
- 重建后规划、LIO、控制、任务节点动态库加载检查均无 `not found`。
- 完整飞行栈负载与实飞尚未验证。

## Unreleased - 2026-09-24

### Added / Changed
- **云台与检测硬件解耦及 points.yaml 集中配置**：
  - 在 `points.yaml` 及覆盖任务配置中新增 `enable_gimbal` 配置项（`true`/`false`），作为硬件总开关。
  - `run_single_lio.sh` 启动脚本自动优先读取 `points.yaml` 中的 `enable_gimbal` 与 `enable_realtime_detection`，统一作为单一配置源；环境变量 `A8MINI_START_GIMBAL_NODE` 和 `A8MINI_START_DETECTION` 仍保留作为临时覆盖。
  - 当 `enable_gimbal: false` 时，启动脚本自动关闭云台节点与 YOLO 实时识别节点（无云台则无 RTSP 视频流）。
  - `multipointplan` 节点解析 YAML 时支持从 `enable_gimbal` 自动同步云台执行模式；无云台时航点列表中的 `gimbal_*`（如 `gimbal_pitch_deg`、`gimbal_settle_sec`、`gimbal_yaw_deg`）及 `hover_sec` 字段变为可选（缺省自动填充为 0），无需配置云台参数即可执行航点飞行。

## Unreleased - 2026-09-17

- 根据 `20260916_163402` 修复地面误触任务、点击航线隐式回退 YAML、关闭载荷后仍等云台的问题。
- 实机默认 `mission_source=clicked`；YAML 任务需 `MISSION_SOURCE=preset`。新增控制器状态/就绪话题及定位时效、航点/云台超时检查。
- 新增本次进程树监督，Ctrl+C/HUP/启动失败清理检测与采集子进程，业务退出后再收尾 rosbag；检测窗口支持关闭按钮。
- 修复一键起飞完成判定、RC8 直接进入 UP、单次 LAND 在 CMD_CTRL 被拒后无重试、落地后 LAND 无限重发。
- 停止/降落锁不再被单次 TAKEOFF 消息清除；下一飞行需落地上锁后重启整栈。同次运行只允许一轮带云台预设任务，防止旧 ID 去重跳过动作。
- 修复轨迹心跳失效后继续输出旧指令、轨迹参数验证不足、短 RC 数组越界与目标坐标系检查缺失。
- 增加 [操作指南](docs/A8mini实机操作指南.md)、[验证和剩余审查项](docs/2026-09-17_修复-A8mini-report.md) 及退出/任务回归测试。尚未完成 ROS 编译、SITL 或实机验证。

## Unreleased - 2026-09-16

### Added

- 新增 `sh_files/one_click_takeoff.sh`，用于开机后执行单个文件启动完整的 LIO
  实机飞行栈并请求自动起飞。
- 一键入口会自动加载 `/opt/ros/noetic/setup.zsh` 和当前工作空间的
  `devel/setup.zsh`，不再需要手动执行 `cd` 或 `source`。
- 新增 `--start-only`（同时兼容 `--no-takeoff`）模式，仅启动飞行栈，不发送
  起飞指令，便于拆桨检查和现场调试。
- 新增 RViz 多点点选航线：使用 `Publish Point` 按顺序标记多个地图点，节点将其
  转换为连续航点并逐点交给 Diff-Planner 进行避障规划。
- 新增 `/mission/start_clicked_route` 和 `/mission/clear_clicked_route` 交互航线
  控制话题，以及 `/mission/clicked_waypoints` 编号点和连线可视化话题。

### Changed

- 一键脚本复用现有 `run_single_lio_debug.sh`，继续保存控制台、ROS 节点日志、
  rosbag、参数快照和关键事件摘要。
- 启动后增加就绪等待和连续稳定确认，要求关键节点（MAVROS、LIO、EKF、
  `px4ctrl`、`multipointplan`）、IMU、电池、里程计、起飞话题订阅和解锁服务
  均可用。
- 起飞前增加地面安全检查：飞控已连接且未解锁、ExtendedState 为地面状态、
  RC 摇杆居中、悬停/指令模式开关打开，并且 RC 8 通道处于 `DOWN`。
- 起飞指令只发布一次；脚本会等待 `armed=True` 且 MAVROS 进入离地/起飞状态
  后报告起飞确认，不会自动触发航点任务。
- 起飞确认超时不会盲目发送降落指令，而是保持飞行栈运行，提示操作员通过
  遥控器确认和接管。
- `multipointplan` 增加点击航点坐标系转换、固定飞行高度和最大点数配置；交互航线
  不执行 A8 mini 云台动作，适用于拆掉 A8 mini 后的飞行。
- RViz 配置默认显示交互航点标记；`run_single_lio.sh` 增加
  `A8MINI_START_GIMBAL_NODE` 和 `A8MINI_START_DETECTION` 环境变量，允许无 A8 mini
  启动完整 LIO 飞行栈。
- 更新 `README.md` 和 `A8mini/README.md`，补充一键起飞的使用方法、启动条件
  、`--start-only` 以及 RViz 点选多航点说明；同时记录项目原有 `3D Nav Goal`
  单点规划的操作逻辑。

### Usage

```bash
/home/q/Documents/diff-planner-A8miniV2/sh_files/one_click_takeoff.sh
```

只启动不自动起飞：

```bash
/home/q/Documents/diff-planner-A8miniV2/sh_files/one_click_takeoff.sh --start-only
```

### Verification

- `zsh -n sh_files/one_click_takeoff.sh`：通过。
- `git diff --check`：通过。
- `python3 -m unittest test_flight_diagnostics`（在 `sh_files/` 目录执行）：10 项通过。
- `catkin_make --pkg multipoint -j2`：通过。
- `roslaunch --nodes multipoint multipointplan_exp_lio.launch start_gimbal_node:=false
  start_detection:=false`：通过。
- 本次未连接真实飞机执行实飞测试。

### RViz 点选多航点

启动实机系统时拆掉 A8 mini：

```bash
A8MINI_START_GIMBAL_NODE=false \
A8MINI_START_DETECTION=false \
./sh_files/run_single_lio.sh
```

在 RViz 选择 **Publish Point**，按顺序点击地图点；确认后选择 **2D Nav Goal**
作为执行确认，或执行：

```bash
rostopic pub -1 /mission/start_clicked_route std_msgs/Empty '{}'
```

交互航线不会自动解锁或起飞。清空航线后才能重新标点：

```bash
rostopic pub -1 /mission/clear_clicked_route std_msgs/Empty '{}'
```

## Unreleased - 2026-08-08

### Fixed

- 修复降落时轨迹服务器仍以 100 Hz 持续发布 `PositionCommand`，
  导致 `px4ctrl` 无法从 `CMD_CTRL` 进入 `AUTO_HOVER` 的问题。轨迹
  服务器现在会在收到 LAND 后立即禁用指令输出并清除旧轨迹，
  收到 TAKEOFF 后才解锁并等待新轨迹；任务节点另以锁存的 `/planning/stop`
  保证自动和手动两种降落路径都会停止轨迹输出。
- 改进 `px4ctrl` 的降落中止日志，明确指出关闭 RC command-mode
  开关会使 `AUTO_LAND` 退回 RC 悬停控制。手动接管后，任务节点
  会停止重发 LAND，避免自动降落与手动降落反复抢占状态。
- 修复 `points.yaml` 中重复声明顶层 `waypoints` 键的问题。两个列表现已合并为
  一个序列，航点 1 和航点 2 都会被任务节点加载。
- 修复遥控器 8 通道只有严格经过 `UP -> MIDDLE -> DOWN` 才能触发降落的问题。
  现在任何有效状态进入 `DOWN` 都会请求降落，包括直接 `UP -> DOWN`。
- 修复 `px4ctrl` 处于 `CMD_CTRL` 时拒绝一次性 LAND 命令后不再降落的问题。
  降落请求现在会锁存，并按默认 1 秒周期持续重发，使控制器进入
  `AUTO_HOVER` 后仍能收到并接受 LAND。
- 降落请求会停止当前航点状态机、关闭任务 CSV，并拒绝后续主任务和返航触发，
  避免降落期间继续发布新航点。

### Added

- 新增 `sh_files/run_single_lio_debug.sh`，在执行原
  `sh_files/run_single_lio.sh` 的同时保存完整调试资料。
- 调试目录默认为 `flight_logs/YYYYMMDD_HHMMSS/`，包含：
  - `console.log`：完整控制台输出；
  - `key_events.log`：RC、航点、云台、起飞和降落事件摘要；
  - `flight_debug.bag`：关键飞行话题 rosbag；
  - `ros/`：各 ROS 节点日志；
  - YAML、launch、启动脚本和 Git 工作区快照；
  - ROS 参数、节点列表和话题列表。
- 新增 `land_command_retry_sec` launch/ROS 参数，默认值为 `1.0 s`。
- 新增 RC 通道 8 原始 PWM、识别位置、前一位置、初始化状态和降落锁存状态日志。

### Changed

- 通道 8 进入 `DOWN` 后，降落状态会一直锁存到 `multipointplan` 节点重启。
  这是安全行为，用于防止落地后再次误触发起飞。
- `flight_logs/` 已加入 `.gitignore`，避免飞行日志和 rosbag 进入版本控制。
- 更新 A8 mini 使用说明，补充调试启动、日志文件及降落重试行为。

### Active default mission

`points.yaml` 当前包含两个有效的水平 range 航点：

1. `(3.0, 0.0, 1.0)`；
2. `(4.0, 1.5, 1.0)`。

两个航点均保持 `pitch=0°`，水平扫描 `-135° -> +135°`。

### Upgrade notes

本次修改包含 `multipointplan.cpp` 变更，部署到实机后必须重新编译并重启：

```bash
catkin_make --pkg multipoint
source devel/setup.zsh
./sh_files/run_single_lio_debug.sh
```

仅修改 YAML 时不需要重新编译，但仍需重启 `multipointplan` 使配置重新加载。

## 2026-08-08 - Configuration cleanup

- 修正 `points.yaml` 字段注释，区分公共必填字段、`angle` 模式字段和
  `range` 模式字段。
- 增加可直接执行的水平 range 航点示例。

相关提交：`086ad4f`、`62b153c`。

## 2026-08-07 - Coverage missions and configurable gimbal range

### Added

- 新增 `gimbal_mode`：
  - `angle`：转到 `gimbal_yaw_deg`；
  - `range`：在 `gimbal_yaw_min_deg` 和 `gimbal_yaw_max_deg` 之间扫描。
- 新增每航点可配置的 `gimbal_yaw_min_deg`、`gimbal_yaw_max_deg`；省略时
  默认 `-135°` 和 `+135°`。
- 新增 5 m × 5 m 蛇形覆盖任务，共 9 个航点，相邻点间距 2.5 m。
- 新增 20 m × 20 m 蛇形覆盖任务，共 81 个航点，相邻点间距 2.5 m。
- 新增范围动作端点等待参数 `gimbal_range_move_wait_sec`，默认每端点 4 秒。

### Changed

- 云台任务消息扩展为
  `[id, yaw, pitch, settle, mode, yaw_min, yaw_max]`。
- Python 云台节点兼容旧 4 项、旧 5 项和新 7 项任务消息。
- 任务 CSV 增加 `gimbal_mode`、`yaw_min` 和 `yaw_max` 列。

相关提交：`ac4039a`。

## 2026-08-06 - A8 mini V1.0

- 新增 A8 mini SIYI UDP 云台控制节点。
- 新增航点到达、悬停、云台任务、完成确认和下一航点状态机。
- 新增 LIO/VIO/仿真 launch 集成。
- 新增任务到点和云台完成时间 CSV。
- 保留遥控器 8 通道起飞、任务、返航和降落操作流程。

相关提交：`802bb8c`、`aa21fed`。
