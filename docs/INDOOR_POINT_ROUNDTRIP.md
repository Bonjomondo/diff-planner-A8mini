# 地图标点单点往返

分支 `codex/indoor-point-roundtrip`，基于 `zjc-argc` 的 `08d4ff2`。仅使用 `/home/ly/Documents/diff-planner-A8mini-gcs-mission`。不操作机载旧目录。

## 2026-10-18 验证与当前状态

实飞和自主任务测试目前暂停；没有完成自主往返实飞验收。协议/适配器单元测试、隔离 ROS 集成测试、真实规划器配合理想位姿跟踪模型的往返测试已通过。实际飞控在未解锁状态接受 OFFBOARD 后恢复自稳的地面联调已通过，不代表真实定位悬停验证。

新增 `px4ctrl/launch/run_ctrl_indoor_manual.launch` 作为后续人工起降测试入口：关闭自动起飞/解锁，保留遥控器，屏蔽该控制器的起降话题及解锁/重启服务；姿态输出和模式服务仍连接真实飞控，启动不等于仿真。该入口将机载手动速度限制为 0.3 m/s、姿态角限制为 20 度，其他机体/控制参数沿用原配置，仍须按实际机体核验。遥控器通过原逻辑进入定位悬停；结束后人工降落。

建图启动脚本补充 ATTITUDE_QUATERNION（MAVLink ID 31）的 5000 微秒请求，与既有 HIGHRES_IMU（105）请求配合。现场修复前姿态 IMU/EKF 约 50 Hz，修复后实测约 171 Hz，控制输出约 200 Hz；不保证每种链路都达到同一频率。请求被拒绝、飞控/MAVROS 重启后需要重新实测，不把收到单条消息等同于飞行输入就绪。

当前配置文件保存一次场地采集的内缩范围，但 `enabled: false`；运行任务和控制节点已停止。地图重启、重定位或更换场地后必须重新核对边界，不能直接启用保存的数值。

## 执行方式

地面站在本次 world 地图中选 P，记录人工起飞后的起始悬停点 R，上传 `[P, R]` 两点及 P 的悬停时长。upload 只缓存；start 经空中、定位、规划器和控制器检查后逐点发布 `/goal`。任务完成在 R 悬停，不自动降落。

新入口 `roslaunch multipoint indoor_roundtrip.launch` 只启动任务适配器/网关，关闭云台、识别和 RC 任务触发；**不启动定位、规划器、px4ctrl，不解锁、不起飞**。它启用强制地图坐标检查。不要把传感器一键建图接口当作飞行栈启动接口。

## 首次场地配置

`src/user_command/multipoint/config/indoor_fence.yaml` 保持 `enabled: false`。其中数值只记录 2026-10-18 某次持续 world 会话的内缩测试范围，不是可跨会话复用的实验室坐标；首次部署应清空并重新标定。缺少已启用的实际规划器围栏证明时任务仍被拒绝。

先标定当前 world 中围网、顶网、地面，按含桨尺寸及定位/跟踪误差扣除余量。填写允许**机体中心**到达的六个边界：

```text
bounds: [xmin, ymin, zmin, xmax, ymax, zmax]
```

完成实际数值核验后设 enabled 为 true。在启动本次规划器之前，将文件加载到根参数空间：

```bash
source /opt/ros/noetic/setup.bash
source /home/ly/Documents/diff-planner-A8mini-gcs-mission/devel/setup.bash
rosparam load "$(rospack find multipoint)/config/indoor_fence.yaml" /
```

更改边界后须重新启动规划器和任务适配器并重新标点；不在执行中热改围栏。不能仅依赖雷达识别稀疏网线。

`GridMap` 将围栏外区域视为占据。在围栏启用时，尚未通过射线观测的栅格也视为占据；观测掩码随滚动地图清理。只影响启用 indoor_mission 的规划器，原默认模式保留原行为。规划器发布实际采用的边界 `/indoor_mission/fence`，网关不因“参数文件存在”就认为围栏已经生效。

`/indoor_mission/check_endpoints` 使用 nav_msgs/GetPlan 服务类型，返回两个 pose 仅表示端点为已观测空闲、未膨胀占据且在围栏内，**不是一条预规划路径**。上传和开始都检查 P/R，规划可行性仍以实际规划器执行及任务状态为准。

## 坐标与生命周期

本版本限现有同一 world、零偏置 LIO/EKF 配置。`gcs_map_guard.py` 验证匹配时间的位姿一致性；不自动计算空间变换。10 个连续匹配样本、50 ms 配对容差、15 cm 位置容差、5 度姿态容差，超过 0.5 秒不再有效。时间回退、ROS 序号回退或 frame 错误撤销验证。

网关每 0.2 秒发布 `/indoor_mission/map_ready`。本开发分支的 remote 模式默认强制检查该信号，不能因误用了通用实机 launch 而绕过坐标保护；坐标/围栏验证丢失则终止并锁定任务失败。已有里程计/飞控/规划器异常检查继续生效。仅隔离的通用仿真/回归测试显式关闭地图检查，专用真实规划器测试仍开启。

protocol 2 增加可选布尔字段 `require_map_alignment`。新地面站发送 true；上传/开始前验证坐标、围栏和端点，拒绝不匹配的任务身份及边界变化。此模式不支持执行中 update；空闲时编辑应上传新任务。原通用协议的 JSON 形状继续兼容；本分支 remote 模式默认也在执行器侧强制坐标就绪，不配置实际围栏不能执行。

新增 cancel 操作，必须携带当前 map_session_id、mission_id、revision。取消 READY 会清空草稿并刷新会话；终止执行中的任务调用原规划停止路径，进入 FAILED，不能恢复，不自动降落。遥控器接管和飞控失联处理仍按原控制栈配置。

Wi-Fi 中断本身不停止机载任务；定位和控制正常时继续完成既有往返。地面站重连只查询/显示，不自动重放命令。重新定位/重启后必须重新核对坐标及任务，不能使用旧 PCD 航点。

## 构建与隔离测试

在新目录准备好 ROS1 雷达构建元数据后：

```bash
source /opt/ros/noetic/setup.bash
cmake -S src -B build -DROS_EDITION=ROS1
cmake --build build --target multipointplan diff_planner_node traj_server test_remote_mission test_mission_guard -- -j2 -l2
bash tools/test_indoor_roundtrip.sh
```

测试脚本创建独立 master `127.0.0.1:11411`，端口已占用则退出，不复用任何未知 master。不会向当前真实传感器使用的 11311 发布模拟飞控数据，不启动 MAVROS、PX4、电机或相机。脚本结束只清理自己创建的测试 master。

包含协议/ROS 适配器单元测试、原任务执行器回归、主动终止回归，以及真实 Diff-Planner/轨迹服务器配合**理想位姿跟踪模型**的单点往返测试。此测试不代表真实动力学、实际实验室障碍物或实机验收。

2026-10-17：上述协议/适配器 15 项测试、原任务及主动终止 ROS 集成测试、真实规划器理想跟踪往返测试已通过。规划器未知栅格预检曾暴露传感器原点未标记的问题，现只将当前传感器所在栅格标记为已知，不清除该格占据或膨胀信息。未连接实机飞行任务或解锁。

首次构建同时修复了 diff_planner_node/traj_server 缺少 catkin 消息生成依赖的问题。构建仍存在项目原有 OpenCV 4.2/4.5 动态库混用警告，真实规划器测试通过，但后续实机部署需核对依赖一致性。
