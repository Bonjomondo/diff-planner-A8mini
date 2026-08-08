# A8 mini 航点连续录像任务：功能实现与使用说明

## 1. 文档目的

本文档说明本次在 Diff-Planner 工程中为 A8 mini 实现了什么功能、代码如何组织，以及如何通过达妙载板网口连接、编译、运行和排查问题。

适用的硬件连接是：

```text
A8 mini 云台相机
    ↓ 网线
达妙 DM-ORIN NX 载板网口
    ↓ 载板网口对应的 Linux 网络接口
Orin NX 机载系统 / ROS1
```

云台控制不经过 PX4 的串口，也不经过 MAVROS 的 mount 话题。Python ROS 节点直接向 A8 mini 发送 SIYI UDP 协议数据。

## 2. 最终实现的任务流程

```text
A8 mini 上电，根据相机内部配置自动开始录像
    ↓
触发 multipointplan 航点任务
    ↓
发布第一个 /goal
    ↓
无人机到达航点，位置和速度连续稳定
    ↓
按 hover_sec 悬停
    ↓
发布 /mission/gimbal_task
    ↓
Python 节点通过网口设置锁定模式和绝对云台角度
    ↓
等待云台转动时间，再按 gimbal_settle_sec 保持目标方向
    ↓
发布 /mission/gimbal_done
    ↓
multipointplan 校验航点编号，记录 CSV，然后发布下一个 /goal
```

任务中没有以下逻辑：

- 拍照命令；
- 拍照完成反馈；
- 照片写卡等待；
- `take_photo` 和 `photo_wait_sec` 配置；
- ROS 开始或停止录像命令。

录像由 A8 mini 的“开机自动录像”功能负责，ROS 任务只负责飞航和转动云台。

## 3. 代码实现内容

### 3.1 飞行端状态机

文件：

```text
src/user_command/multipoint/src/multipointplan.cpp
```

实现了四个任务状态：

```cpp
enum MissionState
{
    FLYING,
    HOVERING,
    WAITING_GIMBAL,
    FINISHED
};
```

各状态的含义如下：

| 状态 | 含义 |
|---|---|
| `FLYING` | 无人机正在飞向当前航点 |
| `HOVERING` | 已稳定到达，正在执行 `hover_sec` |
| `WAITING_GIMBAL` | 已发布云台任务，等待相同航点编号的 `gimbal_done` |
| `FINISHED` | 航点任务未开始或已全部完成 |

飞行端具体实现了：

- 从 YAML 读取航点位置、悬停时间、云台动作模式、角度和稳定时间；
- 检查航点 ID 是否为唯一的正 UInt32 整数；
- 检查数值是否有效，并检查 A8 mini 角度范围；
- 同时使用位置误差和速度判断到点；
- 只有连续稳定 `arrival_stable_sec` 后才进入悬停状态；
- 非阻塞地执行 `hover_sec`，不在 ROS 回调中使用长时间 `sleep`；
- 发布云台任务后停在 `WAITING_GIMBAL`，不发布下一个 `/goal`；
- 只接受当前航点编号的 `/mission/gimbal_done`；
- 完成消息丢失时，按 `gimbal_retry_sec` 重新发布当前云台任务；
- 记录到点时间、云台完成时间和角度到 CSV；
- 保留原工程的遥控器 8 通道触发和可选 `test_back` 返程路线。

### 3.2 Python 云台节点

文件：

```text
src/user_command/multipoint/scripts/a8mini_gimbal_node.py
```

该节点实现了：

- SIYI 帧头、控制字节、小端数据长度、序列号、命令字和 CRC16-CCITT 打包/解包；
- 通过 `192.168.144.25:37260/UDP` 访问 A8 mini；
- 使用 `0x0C / 03` 把云台切到锁定模式；
- 使用 `0x0E` 发送绝对 yaw/pitch，角度单位是 `0.1°`；
- 支持指定角度和可配置起止角度的水平范围扫描两种动作；
- 校验 A8 mini 返回帧的长度、CRC 和命令字；
- UDP 超时重试，默认超时 `0.6 s`，重试 3 次；
- 用工作线程执行云台任务，避免阻塞 ROS 订阅回调；
- 校验任务长度、航点 ID、角度范围和等待时间；
- 对正在执行和已完成的航点 ID 去重；
- 收到重复的已完成任务时，不重复转动，直接再次发布 `gimbal_done`；
- 网络命令失败时不发布错误的完成消息，让飞机保持当前航点并等待重试；
- 提供 `dry_run` 模式，不连接实际相机也可以测试 ROS 流程。

A8 mini 角度限制为：

```text
yaw:   -135° ~ 135°
pitch:  -90° ~ 25°
```

### 3.3 ROS 话题

| 话题 | 类型 | 方向 | 作用 |
|---|---|---|---|
| `/mission/gimbal_task` | `std_msgs/Float64MultiArray` | 飞行端 → 云台端 | 发送 `[id, yaw, pitch, settle_sec, mode, yaw_min, yaw_max]`；`0=angle`，`1=range` |
| `/mission/gimbal_done` | `std_msgs/UInt32` | 云台端 → 飞行端 | 通知当前航点云台任务完成 |
| `/goal` | `geometry_msgs/PoseStamped` | `multipointplan` → Diff-Planner | 发布当前航点 |
| `odom_topic` | `nav_msgs/Odometry` | 定位端 → `multipointplan` | 判断位置误差和飞行速度 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | 用户 → `multipointplan` | 触发主航点任务 |
| `/back_trigger` | `geometry_msgs/PoseStamped` | 用户 → `multipointplan` | 触发可选返程路线 |

### 3.4 航点配置

文件：

```text
src/user_command/multipoint/config/points.yaml
```

默认示例：

```yaml
waypoints:
  - id: 1
    x: 3.0
    y: 0.0
    z: 1.5
    hover_sec: 2.0
    gimbal_mode: angle
    gimbal_yaw_deg: 0.0
    gimbal_pitch_deg: -45.0
    gimbal_settle_sec: 2.0

  - id: 2
    x: 6.0
    y: 1.0
    z: 1.5
    hover_sec: 2.0
    gimbal_mode: range
    gimbal_yaw_min_deg: -90.0
    gimbal_yaw_max_deg: 90.0
    gimbal_pitch_deg: -35.0
    gimbal_settle_sec: 2.0
```

字段含义：

| 字段 | 含义 |
|---|---|
| `id` | 航点唯一编号，必须为正整数 |
| `x/y/z` | `world` 坐标系下的航点位置，单位为米 |
| `hover_sec` | 稳定到点后，发布云台任务前的悬停时间 |
| `gimbal_mode` | `angle` 转到指定角度；`range` 在配置的起止角度间扫描；省略时为 `angle` |
| `gimbal_yaw_deg` | `angle` 模式的水平目标角度；`range` 模式可省略 |
| `gimbal_yaw_min_deg` | `range` 模式起始角度，默认 `-135°` |
| `gimbal_yaw_max_deg` | `range` 模式结束角度，默认 `+135°` |
| `gimbal_pitch_deg` | 云台俯仰目标角度，向下为负 |
| `gimbal_settle_sec` | 云台转动等待结束后，继续保持该方向的时间 |

`gimbal_settle_sec` 当前限制为 `0 ~ 60 s`。
范围字段必须满足 `-135 <= gimbal_yaw_min_deg < gimbal_yaw_max_deg <= 135`。

项目还提供两份 2.5 m 航点间距的蛇形覆盖任务：

```text
src/user_command/multipoint/config/coverage_5x5.yaml    # 5 m × 5 m，9 个点
src/user_command/multipoint/config/coverage_20x20.yaml  # 20 m × 20 m，81 个点
```

它们覆盖 `x/y=0~边长`，默认 `z=1.0 m`、pitch `-45°`，每个点都执行
`gimbal_mode: range`。实际飞行前应按场地原点、飞行高度和安全余量调整坐标。

如果需要使用原工程的返程触发，可以增加：

```yaml
test_back:
  - [0.0, 0.0, 1.0]
```

`test_back` 路线只执行飞行，不执行云台任务。

### 3.5 launch 和构建文件

已更新的文件包括：

```text
src/user_command/multipoint/CMakeLists.txt
src/user_command/multipoint/package.xml
src/user_command/multipoint/launch/multipointplan_exp_lio.launch
src/user_command/multipoint/launch/multipointplan_exp_vio.launch
src/user_command/multipoint/launch/multipointplan_sim.launch
```

实机 LIO/VIO launch 会同时启动：

```text
/multipointplan
/a8mini_gimbal
```

仿真 launch 会把云台节点设为 `dry_run=true`，不会向网络发送 A8 mini 命令。

## 4. 实机运行前准备

### 4.1 A8 mini 录像配置

在 UniGCS 或思翼调参工具中提前完成：

1. 插入 TF 卡；
2. 在相机中格式化 TF 卡；
3. 设置录像分辨率；
4. 开启“开机自动录像”；
5. 检查 TF 卡剩余容量；
6. 单独上电测试一次，确认确实自动生成了视频。

本 ROS 节点不会开始或停止录像。任务结束后应在相机端正常停止录像，或至少等待数秒再断电；不要在录像中直接拔出 TF 卡。

### 4.2 网络配置

A8 mini 默认控制参数：

```text
IP:       192.168.144.25
Protocol: UDP
Port:     37260
```

给机载电脑上连接达妙载板的网卡配置一个不冲突的同网段地址，例如：

```text
192.168.144.30/24
```

查看网卡名称：

```bash
ip -br addr
```

根据 DM-ORIN NX V2.X 载板手册，载板提供两路 SH1.0 8-pin 网口：一路为核心模组原生千兆网口，另一路通过 PCIe 扩展 RTL8125，最高支持 2.5G 和 PTP。运行 `ip -br addr` 时，要根据插线后的 link 状态确认 A8 mini 实际连接的那块网卡。如果使用 RTL8125 扩展口却没有出现对应 Linux 网卡，需先按载板手册附录检查 RTL8125 驱动，不是修改 ROS 代码。

如果只需要临时测试，可将 `<interface>` 替换为实际网卡名称：

```bash
sudo ip link set <interface> up
sudo ip addr add 192.168.144.30/24 dev <interface>
```

如果该网卡已有同网段地址，不要重复添加。正式使用时建议在 NetworkManager 中配置持久静态 IP。

上电后检查：

```bash
ping 192.168.144.25
```

必须先确认 ping 通，再调试 ROS 云台节点。如果 A8 mini 的 IP 已经在调参工具中修改，应在 launch 中传入修改后的地址。

## 5. 编译

### 5.1 正常编译

打开新终端，进入工作空间：

```bash
cd /home/q/Documents/diff-planner-A8mini
source /opt/ros/noetic/setup.zsh
catkin_make -DROS_EDITION=ROS1
source devel/setup.zsh
```

如果使用 Bash，把 `setup.zsh` 替换为 `setup.bash`。

`-DROS_EDITION=ROS1` 是为了让工程中的 Livox 驱动按 ROS1 方式编译。

### 5.2 工作空间曾经移动时

Catkin 的 `build` 缓存会记录工作空间绝对路径。如果出现以下类型错误：

```text
The current CMakeCache.txt directory is different from the directory where it was created
```

可以在新终端中使用新的生成目录，不覆盖旧缓存：

```bash
cd /home/q/Documents/diff-planner-A8mini
source /opt/ros/noetic/setup.zsh
catkin_make --build build_a8mini \
  -DCATKIN_DEVEL_PREFIX="$PWD/devel_a8mini" \
  -DROS_EDITION=ROS1
source devel_a8mini/setup.zsh
```

后续运行前要 source 与本次编译对应的 `devel_a8mini/setup.zsh`。不要同时 source 另一份包含同名 `multipoint` 功能包的旧工作空间。

## 6. 运行方法

### 6.1 先单独测试云台

实机首次调试时，建议先不启动飞机任务，只测试 A8 mini 云台节点。

终端 1：

```bash
cd /home/q/Documents/diff-planner-A8mini
source devel/setup.zsh
roscore
```

终端 2：

```bash
cd /home/q/Documents/diff-planner-A8mini
source devel/setup.zsh
rosrun multipoint a8mini_gimbal_node.py
```

如果相机 IP 不是默认地址：

```bash
rosrun multipoint a8mini_gimbal_node.py _camera_ip:=192.168.144.26
```

终端 3 监听完成消息：

```bash
source devel/setup.zsh
rostopic echo /mission/gimbal_done
```

终端 4 发布一个小角度测试任务：

```bash
source devel/setup.zsh
rostopic pub -1 /mission/gimbal_task std_msgs/Float64MultiArray \
  "data: [1, 0.0, -30.0, 2.0]"
```

正常情况下：

1. 云台转到 yaw `0°`、pitch `-30°`；
2. 节点等待默认转动时间 `2 s`；
3. 继续保持 `2 s`；
4. `/mission/gimbal_done` 输出 `data: 1`。

每个航点 ID 在 Python 节点的一次运行周期内只执行一次。如果想用相同 ID 再次测试实际转动，请重启 `a8mini_gimbal_node.py`，或者换一个新的航点 ID。

`-90°～+90°` 范围扫描测试（第五项 `1` 表示 `range`）：

```bash
rostopic pub -1 /mission/gimbal_task std_msgs/Float64MultiArray \
  "data: [2, 0.0, -45.0, 1.0, 1.0, -90.0, 90.0]"
```

旧的四项消息仍按 `angle` 模式解析；旧的五项 `range` 消息采用默认
`-135°～+135°` 范围。

### 6.2 不连接 A8 mini 的 dry-run 测试

需要测试整个 ROS 握手逻辑，但不希望操作真实云台时：

```bash
source devel/setup.zsh
roslaunch multipoint multipointplan_sim.launch
```

该 launch 中的 Python 节点会正常等待并回复 `gimbal_done`，但不会创建 A8 mini UDP 命令。

### 6.3 单独启动 LIO 实机航点节点

当 MAVROS、LIO、EKF、Diff-Planner 和 PX4 控制器已由其他方式启动后，执行：

```bash
source devel/setup.zsh
roslaunch multipoint multipointplan_exp_lio.launch
```

默认使用：

```text
里程计话题：/ekf/ekf_odom
相机 IP：  192.168.144.25
相机 UDP： 37260
```

修改过相机 IP 时：

```bash
roslaunch multipoint multipointplan_exp_lio.launch \
  camera_ip:=192.168.144.26
```

需要修改里程计话题时：

```bash
roslaunch multipoint multipointplan_exp_lio.launch \
  odom_topic:=/your/odometry/topic
```

### 6.4 使用 VIO 定位

```bash
source devel/setup.zsh
roslaunch multipoint multipointplan_exp_vio.launch
```

VIO launch 默认使用：

```text
/vins/imu_propagate
```

### 6.5 启动完整 LIO 实机系统

工程现有启动脚本已经包含 `multipointplan_exp_lio.launch`：

```bash
cd /home/q/Documents/diff-planner-A8mini
./sh_files/run_single_lio.sh
```

该脚本会依次启动 MAVROS、Livox LIO、EKF、Diff-Planner、PX4 控制器、航点节点和 RViz。脚本会访问串口并修改 `/dev/tty*` 权限，应在已确认飞控串口和权限配置后使用。

## 7. 触发任务

### 7.1 使用 ROS 话题触发

在所有飞行和云台节点启动后：

```bash
cd /home/q/Documents/diff-planner-A8mini
./sh_files/pub_trigger.sh
```

也可直接发布：

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped '{}'
```

触发消息的位置内容不会被当作航点；它只起“开始执行 `points.yaml`”的作用。

### 7.2 使用遥控器 8 通道

保留的默认逻辑是：

```text
8 通道下位（约 1999）→ 中位（约 1499）：起飞
8 通道中位 → 上位（约 999）：开始航点任务
8 通道上位 → 中位：执行 test_back
8 通道中位 → 下位：降落
```

第 8 通道进入下位后，轨迹服务器会先停止发布位置指令。需要
自动降落时，保持 `px4ctrl` 的 mode 和 command-mode 两个遥控开关
开启；控制器从 `CMD_CTRL` 退回 `AUTO_HOVER` 后，锁存并重发的 LAND
命令会启动降落。需要切换为手动降落时，将第 6 通道拨出
command-mode；任务节点会停止 LAND 重试，`px4ctrl` 稳定转入 RC 悬停
控制，此后使用油门杆下降并按正常手动流程落地、停桨。

不同遥控器的 PWM 方向可能相反，必须先通过以下命令确认实际数值：

```bash
rostopic echo /mavros/rc/in
```

如果不使用遥控器触发，启动时可以关闭：

```bash
roslaunch multipoint multipointplan_exp_lio.launch enable_rc:=false
```

### 7.3 返程触发

只有 `points.yaml` 中配置了 `test_back` 时才能使用：

```bash
./sh_files/back.sh
```

返程触发会取消正在执行的主航点任务，且返程点不执行云台动作。

## 8. launch 参数

LIO/VIO 实机 launch 支持下列参数：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `odom_topic` | LIO: `/ekf/ekf_odom` | 航点到达判断所用里程计 |
| `yaml_path` | 功能包内 `config/points.yaml` | 航点配置文件 |
| `next_distance` | `0.7` | 到点位置容差，单位 m |
| `velocity_tolerance` | `0.2` | 到点速度容差，单位 m/s |
| `arrival_stable_sec` | `0.5` | 位置和速度需要连续满足的时间 |
| `gimbal_retry_sec` | `6.0` | 未收到完成消息时重发任务的间隔 |
| `start_plan` | `1` | 是否订阅主任务触发 |
| `back_plan` | `1` | 是否订阅返程触发 |
| `enable_rc` | `true` | 是否使用遥控器 8 通道逻辑 |
| `mission_csv_path` | `/tmp/a8mini_mission_timestamps.csv` | 任务时间记录文件 |
| `start_gimbal_node` | `true` | 是否同时启动 Python 云台节点 |
| `camera_ip` | `192.168.144.25` | A8 mini IP |
| `camera_port` | `37260` | A8 mini UDP 控制端口 |
| `gimbal_move_wait_sec` | `2.0` | 发送角度后的固定转动等待时间 |
| `gimbal_range_move_wait_sec` | `4.0` | 范围扫描到每个端点预留的转动时间 |

例如，同时修改 IP、到点距离和 CSV 位置：

```bash
roslaunch multipoint multipointplan_exp_lio.launch \
  camera_ip:=192.168.144.26 \
  next_distance:=0.5 \
  mission_csv_path:=/tmp/grape_mission_01.csv
```

## 9. 时间日志和 CSV

`multipointplan` 会打印：

```text
Waypoint 1 arrived, ros_time=...
Waypoint 1 gimbal done, ros_time=...
```

Python 节点会打印：

```text
Waypoint 1 gimbal stable, ros_time=...
```

默认 CSV：

```text
/tmp/a8mini_mission_timestamps.csv
```

格式：

```csv
waypoint_id,arrived_time,gimbal_done_time,yaw,pitch,gimbal_mode,yaw_min,yaw_max
1,1754364005.200,1754364009.300,0.000,-45.000,angle,-135.000,135.000
2,1754364018.500,1754364027.600,0.000,-35.000,range,-90.000,90.000
```

`arrived_time` 是飞行端确认稳定到点的 ROS 时间，`gimbal_done_time` 是飞行端收到云台完成消息的 ROS 时间。可以使用这两个时间对应连续录像中的航点片段。

每次启动新的主航点任务时，默认 CSV 会重新创建。需要保留多次任务时，应为每次任务传入不同的 `mission_csv_path`，或者在再次执行前复制上一份 CSV。

## 10. 运行状态检查

启动后检查节点：

```bash
rosnode list | grep -E 'multipointplan|a8mini_gimbal'
```

检查话题：

```bash
rostopic info /mission/gimbal_task
rostopic info /mission/gimbal_done
```

检查里程计：

```bash
rostopic hz /ekf/ekf_odom
rostopic echo -n 1 /ekf/ekf_odom
```

查看云台任务：

```bash
rostopic echo /mission/gimbal_task
```

查看飞行目标：

```bash
rostopic echo /goal
```

## 11. 常见问题

### 11.1 `ping 192.168.144.25` 不通

检查：

- A8 mini 和达妙载板是否完成上电；
- 网线、载板接口和机载电脑网口是否连接正确；
- 机载电脑网卡是否有 `192.168.144.x/24` 地址；
- 机载电脑是否误用了 `192.168.144.25`，造成 IP 冲突；
- A8 mini IP 是否被改过；
- 同一机载电脑是否有多块 `192.168.144.0/24` 网卡导致路由错误。

### 11.2 云台不转，没有 `gimbal_done`

如果日志出现：

```text
SIYI command 0x0E timed out
```

说明 Python 节点没有收到有效 UDP ACK。应检查 IP、UDP 端口、防火墙、网线、A8 mini 上电状态和协议版本。

此时 `multipointplan` 会留在 `WAITING_GIMBAL`，继续保持当前 `/goal`，并按默认 `6 s` 周期重新发布云台任务。它不会因为云台通信失败而自动飞向下一个航点。

### 11.3 无人机到达附近，但不进入悬停状态

检查：

- `odom_topic` 是否与实际定位话题一致；
- 里程计位置与 `/goal` 是否使用同一坐标系；
- 位置误差是否小于 `next_distance`；
- 速度是否小于 `velocity_tolerance`；
- 条件是否连续满足了 `arrival_stable_sec`。

调试时可以适当放宽阈值，但实飞前应在开阔环境验证。

### 11.4 云台转动方向与预期相反

先使用小角度进行台架测试，确认 A8 mini 当前安装方向和角度正负号。本实现直接把 YAML 中的 yaw/pitch 作为 SIYI 绝对角度发送，不会根据机体安装方向自动取反。

### 11.5 第二次执行相同 ID 时云台没有重新转动

Python 节点会在内存中保留已完成 ID，用于处理飞行端重发和 ROS 丢包。如果需要重新执行整条任务，应重启云台节点，使已完成集合清空。

### 11.6 找到多个 `multipoint` 功能包

如果 `roslaunch` 报告同名功能包或加载了旧路径，执行：

```bash
rospack find multipoint
echo $CMAKE_PREFIX_PATH
```

打开新终端，先 source `/opt/ros/noetic/setup.zsh`，然后只 source 本工作空间对应的 `devel/setup.zsh` 或 `devel_a8mini/setup.zsh`。

## 12. 建议的首次实飞步骤

1. 拆除螺旋桨或保证无人机不会解锁，先单独检查 A8 mini 自动录像。
2. 设置机载电脑网卡 IP，确认能 ping 通 A8 mini。
3. 单独启动 `a8mini_gimbal_node.py`，用小角度任务检查云台方向和 `gimbal_done`。
4. 运行 `multipointplan_sim.launch`，检查航点、悬停、云台握手和 CSV 流程。
5. 把 `points.yaml` 改成低高度、近距离的测试航点。
6. 在开阔区域启动完整系统，确保随时可以遥控接管或中止任务。
7. 先测试一个航点，确认飞机在 `WAITING_GIMBAL` 时保持当前目标。
8. 确认视频和 CSV 时间能对应后，再扩展到完整航点路线。

## 13. 已完成的软件验证

当前实现已完成：

- `multipoint` ROS 功能包隔离编译；
- C++ 语法和链接检查；
- Python 语法检查；
- launch XML 检查；
- YAML 配置解析检查；
- 5 m × 5 m（9 点）和 20 m × 20 m（81 点）蛇形路径间距检查；
- SIYI 锁定模式和绝对角度官方示例帧字节校验；
- ROS 状态机联调：航点 1 到达后发布云台任务，收到 `gimbal_done=1` 后正确发布航点 2；
- CSV 到点和云台完成时间写入检查；
- Python dry-run 话题联调，正确返回指定航点 ID。

软件流程已打通。在真实飞行前，还需要在实际 A8 mini 上确认 UDP ACK、云台正负方向、机械限位、自动录像和无人机悬停稳定性。

## 14. 相关手册和协议

- [A8 mini 云台相机用户手册](./A8%20mini云台相机用户手册.pdf)；
- [达妙科技 DM-ORIN NX V2.X 载板使用说明书](./达妙科技DM-ORIN%20NX%20V2.X使用说明书V1.0.pdf)；
- [SIYI 云台相机外部 SDK 协议 V0.1.1](https://siyi.biz/siyi_file/A8%20mini/SIYI_Gimbal_Camera_External_SDK_Protocol_Update_Log%20V0.1.1.pdf)。





你现在运行 `./sh_files/run_single_lio.sh` 时，实际使用的是默认的 `points.yaml`。

入口在 [run_single_lio.sh](/Users/a111/Documents/UAV/diff-planner-A8mini/sh_files/run_single_lio.sh:15)：

```bash
roslaunch multipoint multipointplan_exp_lio.launch & sleep 2;
```

因为没传 `yaml_path`，所以使用 launch 文件里的默认值：

```xml
<arg name="yaml_path" default="$(find multipoint)/config/points.yaml" />
```

要切换 5×5 任务，把脚本第 15 行改成：

```bash
roslaunch multipoint multipointplan_exp_lio.launch \
  yaml_path:=$(rospack find multipoint)/config/coverage_5x5.yaml & sleep 2;
```

要切换 20×20：

```bash
roslaunch multipoint multipointplan_exp_lio.launch \
  yaml_path:=$(rospack find multipoint)/config/coverage_20x20.yaml & sleep 2;
```

使用普通航点：

```bash
roslaunch multipoint multipointplan_exp_lio.launch \
  yaml_path:=$(rospack find multipoint)/config/points.yaml & sleep 2;
```

改完以后仍然照常运行：

```bash
./sh_files/run_single_lio.sh
```

然后使用遥控器触发即可。配置文件只在 `multipointplan` 节点启动时读取，因此切换 YAML 后需要重启脚本或至少重启该节点，运行过程中修改不会立即生效。
