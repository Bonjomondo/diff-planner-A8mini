# A8 mini 航点连续录像任务

完整的代码实现、参数、编译运行和故障排查说明见
[《A8 mini 功能实现与使用说明》](A8mini功能实现与使用说明.md)。

本实现用于以下接线：A8 mini 的以太网口接入达妙载板，载板把相机网络接到机载电脑。云台控制不经过 PX4 UART；Python ROS 节点直接使用思翼以太网 SDK 的 UDP 协议。

## 网络准备

A8 mini 默认参数：

- 控制地址：`192.168.144.25:37260/UDP`

A8 mini 属于手册中的旧地址机型，常见 RTSP 地址为 `rtsp://192.168.144.25:8554/main.264`；不同出厂批次或改过配置的设备应以机身标签和调参助手显示为准。本任务只使用 UDP 控制地址，不依赖 RTSP。

给机载电脑连接达妙载板的网卡配置一个不冲突的同网段静态地址，例如 `192.168.144.30/24`。执行任务前先确认：

```bash
ping 192.168.144.25
```

如果相机 IP 已在思翼调参助手中修改，启动时同时传入实际地址，例如：

```bash
roslaunch multipoint multipointplan_exp_lio.launch camera_ip:=192.168.144.26
```

## 相机准备

在 UniGCS 或思翼调参助手中提前插入并格式化 TF 卡、配置录像分辨率，并关闭“开机自动录制”。ROS 节点根据 MAVROS 的真实飞行状态控制录像：进入 `TAKEOFF` 或 `IN_AIR` 后开始录像，确认 `ON_GROUND` 后停止；`LANDING` 期间保持录像，确保下降和接地过程也被保存。

## 构建与运行

```bash
catkin_make -DROS_EDITION=ROS1
source devel/setup.bash
roslaunch multipoint multipointplan_exp_lio.launch
```

VIO 定位时使用 `multipointplan_exp_vio.launch`。现有 `sh_files/run_single_lio.sh` 会启动 LIO 版本，并同时启动 A8 mini 云台节点。

任务仍沿用原工程的安全触发方式：遥控器 8 通道从中位拨到上位，或发送一次：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{}'
```

航点在 `src/user_command/multipoint/config/points.yaml` 中配置。`gimbal_mode`
支持 `angle`（转到 `gimbal_yaw_deg` 指定角度）和 `range`（保持配置的
pitch，从 `gimbal_yaw_min_deg` 扫到 `gimbal_yaw_max_deg`）两种模式；
两个范围字段省略时默认 `-135°` 和 `+135°`，`gimbal_mode` 省略时按旧
配置的 `angle` 处理。

已提供两份可直接切换的蛇形覆盖任务，所有相邻航点均相隔 2.5 m，且每点
执行一次 `range` 扫描：

```text
config/coverage_5x5.yaml     # 9 个航点
config/coverage_20x20.yaml   # 81 个航点
```

例如，LIO 实机使用 5 m × 5 m 任务：

```bash
roslaunch multipoint multipointplan_exp_lio.launch \
  yaml_path:=$(rospack find multipoint)/config/coverage_5x5.yaml
```

## 运行逻辑

Python 节点订阅 `/mavros/extended_state`。由于 SDK 的 `0x0C / 02` 是录像状态切换而不是独立的开始/停止命令，节点会先用 `0x0A` 查询 `record_sta`，仅在当前状态与目标状态不一致时发送切换命令，再轮询确认结果。重复的飞行状态不会把录像误切回去；相机缺少 TF 卡或状态确认失败时会打印错误并按 `record_retry_sec` 重试，但不会阻塞飞控回调。

录像成功切换后会发布锁存话题：

```text
/mission/recording_status  std_msgs/Bool
true = 正在录像，false = 已停止
```

`multipointplan` 同时满足位置误差和速度阈值并持续稳定 `arrival_stable_sec` 后，进入航点悬停。`hover_sec` 到时发布：

```text
/mission/gimbal_task  std_msgs/Float64MultiArray
[waypoint_id, yaw_deg, pitch_deg, settle_sec, mode, yaw_min_deg, yaw_max_deg]
```

`mode=0` 为指定角度，`mode=1` 为范围扫描。云台节点仍接受原来的四项
和五项消息；四项消息视为 `mode=0`，没有携带范围的五项消息采用
`-135°～+135°`。

指定角度模式下，Python 节点依次发送：

1. `0x0C / 03`：云台锁定模式；
2. `0x0E`：绝对 yaw、pitch（0.1° 精度）；
3. 等待 `move_wait_sec`（默认 2 秒）；
4. 保持 `settle_sec`；
5. 发布 `/mission/gimbal_done`。

范围模式下会依次向配置的 min/max 发送绝对角度命令，两端使用同一个
`gimbal_pitch_deg`，每段默认预留 4 秒转动时间。范围必须满足
`-135 <= min < max <= 135`。

飞行节点只接受当前航点相同编号的完成消息。在等待期间不会发布下一个 `/goal`；若完成消息丢失，会重发云台任务。Python 节点会对已完成编号去重并重新回复。

默认把时间对应关系写入：

```text
/tmp/a8mini_mission_timestamps.csv
```

可通过 launch 参数 `mission_csv_path` 修改位置。文件列为：

```text
waypoint_id,arrived_time,gimbal_done_time,yaw,pitch,gimbal_mode,yaw_min,yaw_max
```

## 不带飞机的检查

仿真 launch 使用 `dry_run` 云台后端，不会访问相机：

```bash
roslaunch multipoint multipointplan_sim.launch
```

可在仿真中检查录像状态映射：

```bash
rostopic echo /mission/recording_status
rostopic pub -1 /mavros/extended_state mavros_msgs/ExtendedState \
  "landed_state: 3"   # TAKEOFF，应输出 true
rostopic pub -1 /mavros/extended_state mavros_msgs/ExtendedState \
  "landed_state: 4"   # LANDING，保持 true
rostopic pub -1 /mavros/extended_state mavros_msgs/ExtendedState \
  "landed_state: 1"   # ON_GROUND，应输出 false
```

实机网络单独测试可以只启动 Python 节点并发布一个任务：

```bash
rosrun multipoint a8mini_gimbal_node.py
rostopic pub -1 /mission/gimbal_task std_msgs/Float64MultiArray \
  "data: [1, 0.0, -45.0, 2.0]"
rostopic echo /mission/gimbal_done
```

测试 `-90°～+90°` 范围扫描时发布七项消息，第五项 `1` 表示 `range`：

```bash
rostopic pub -1 /mission/gimbal_task std_msgs/Float64MultiArray \
  "data: [2, 0.0, -45.0, 1.0, 1.0, -90.0, 90.0]"
```

若 UDP 姿态命令未收到合法 ACK，节点不会发布完成消息，飞机会保持当前航点。此时检查网卡地址、相机 IP、载板网线和相机是否完成上电初始化。

## 实机调试日志

需要排查航点、遥控器拨杆、降落或云台问题时，使用调试启动脚本代替普通脚本：

本次降落故障的完整日志证据、根因和复测标准见
[降落失败 Bug/问题分析](./%E9%99%8D%E8%90%BD%E5%A4%B1%E8%B4%A5Bug%E9%97%AE%E9%A2%98%E5%88%86%E6%9E%90.md)。

```bash
source devel/setup.zsh
./sh_files/run_single_lio_debug.sh
```

它仍然执行原来的 `run_single_lio.sh`，但会在
`flight_logs/YYYYMMDD_HHMMSS/` 保存：

- `console.log`：全部启动命令和 ROS 节点控制台输出；
- `ros/`：由 `ROS_LOG_DIR` 收集的各 ROS 节点日志；
- `flight_debug.bag`：RC 输入、规划停止/降落命令、PX4 状态、里程计、控制指令、
  航点、轨迹、云台握手和录像状态等关键话题；
- `key_events.log`：从控制台日志自动提取的拨杆、航点、云台和降落事件；
- `points.yaml.snapshot`、launch 和启动脚本快照；
- ROS 参数、节点列表、话题列表和 Git 版本信息。

飞行结束时使用 `Ctrl+C`，等待脚本打印日志目录，保证 rosbag 正常写完索引。
快速筛选降落和航点事件：

```bash
grep -E "RC channel 8|LAND|AUTO_LAND|Reject AUTO_LAND|Published waypoint|mission finished" \
  flight_logs/YYYYMMDD_HHMMSS/console.log
rosbag info flight_logs/YYYYMMDD_HHMMSS/flight_debug.bag
```

通道 8 进入 DOWN 后，航点任务会被取消并锁存降落请求；轨迹服务器同时停止
发布 `PositionCommand`。在 `px4ctrl` 的 0.5 秒指令超时后，控制器会从
`CMD_CTRL` 转入 `AUTO_HOVER`，任务节点每秒重发的 LAND 随即触发
`AUTO_LAND`。需要自动降落时，保持 `px4ctrl` 的 mode 和 command-mode 两个
遥控开关开启。需要改为手动降落时，可将第 6 通道拨出 command-mode；任务节点
会停止继续重发 LAND，`px4ctrl` 稳定转入 RC 悬停控制，此后可用油门杆下降并按
正常手动流程落地、停桨。无论选择哪种方式，轨迹位置指令都会保持停止。为避免
落地后误触发再次起飞，降落锁存在节点重启前不会解除。

录像停止依据实际的 `ON_GROUND`，不依据 LAND 指令，因此自动或手动降落时都会保留完整下降过程。若 `/mavros/extended_state` 中断，节点会维持最后一次确认的录像目标，不会盲目发送切换命令。可用 `enable_auto_recording:=false` 临时关闭此功能。

协议帧格式、命令编号、CRC 和默认控制端口来自[思翼云台相机外部 SDK 协议 V0.1.1](https://siyi.biz/siyi_file/A8%20mini/SIYI_Gimbal_Camera_External_SDK_Protocol_Update_Log%20V0.1.1.pdf)。
