# Changelog

本文件记录 Diff-Planner A8 mini 分支的重要功能、配置和实机行为变更。

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
