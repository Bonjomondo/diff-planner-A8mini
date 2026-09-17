# A8mini 实机操作指南（LIO / RViz，2026-09-17）

你描述的流程基本正确：**先把 RC8 从 DOWN 拨到 MIDDLE，请求自动起飞；确认已经悬停，再用 RViz 的 Publish Point 标航点，最后发出开始指令。** 标点可以提前在地面完成，但执行必须等起飞完成。这里“手动拨开关”触发的是程序自动解锁、爬升与悬停，不是用油门手动飞起来。

本指南对应本次修改后的源码，需在机载 Ubuntu 上重新编译。当前电脑仅完成离线回归，尚未完成 ROS 整体编译、SITL 或实机验收；验证步骤和审查中尚未解决的事项见[修复说明](2026-09-17_修复-A8mini-report.md)。**Ctrl+C 是停止程序，不能当作降落指令；先落地、确认上锁，再退出。**

## 1. 第一次更新后准备

在机载电脑工作空间根目录执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DROS_EDITION=ROS1
source devel/setup.bash
```

先按修复说明做不连接飞控的回归及地面拆桨检查，再使用下面的飞行步骤。不要同时运行上一轮残留节点或另一份独立 A8mini 检测程序。

启动脚本现在不会修改 `/dev/tty*` 权限。确认飞控 `/dev/ttyACM0` 对当前用户可读写；权限不足时在 Ubuntu 上一次性配置串口用户组，重新登录后再检查：

```bash
ls -l /dev/ttyACM0
id
# 仅当设备所属组为 dialout 且当前用户不在其中时执行：
sudo usermod -aG dialout "$USER"
```

RC 通道必须按实际 PWM 确认，不要只凭遥控器开关朝向判断：

| 输入 | 本项目含义 | 操作要求 |
| --- | --- | --- |
| CH1～CH4 | 四个摇杆 | 自动起飞时居中，约 1375～1625 |
| CH5 | PX4Ctrl 悬停模式许可 | 起飞前设为高值，>1750 |
| CH6 | PX4Ctrl 指令模式许可 | 起飞前设为高值，>1750 |
| CH8 DOWN | 高值，约 2000；这次日志是 1933 | 启动时必须先在此档 |
| CH8 MIDDLE | 中值，约 1500 | 从 DOWN 到此档请求起飞 |
| CH8 UP | 低值，约 1000 | 进入此档请求执行所选来源的航线 |

代码对 CH8 的有效窗口是三个中心值 `999 / 1499 / 1999` 各 ±100（不含边界）。这次日志的 CH8 一直为 DOWN，CH3 约为 1293；日志内没有起飞消息。CH5/CH6 建议在启动前就置于所需位置；如果控制器状态或 RC 映射不符合预期，停在地面检查，不要反复拨开关试飞。

## 2. 推荐流程：RC8 起飞，RViz 连续标点

### 第一步：启动系统，但不自动发起飞指令

带 A8mini 时，在工作空间根目录运行：

```bash
MISSION_SOURCE=clicked ./sh_files/one_click_takeoff.sh --start-only
```

此入口自动加载 ROS/工作空间、启动飞行栈并保存调试日志。等待“仅启动模式：未发送起飞指令”。此时 CH8 保持 DOWN。这个入口只启动一次，不要再运行另一份 `run_single_lio.sh`。

没有接 A8mini 时改用：

```bash
A8MINI_START_GIMBAL_NODE=false \
A8MINI_START_DETECTION=false \
MISSION_SOURCE=clicked \
./sh_files/one_click_takeoff.sh --start-only
```

这两个开关会关闭云台与检测进程，同时禁止任务等待云台。只把 `enable_auto_recording` 设为 false，**不会**关闭识别或云台。

### 第二步：请求起飞，等到悬停

把 **RC8 从 DOWN 拨到 MIDDLE，然后停在 MIDDLE**。不要连续拨到 UP。

当前控制参数为起飞高度 **相对起飞位置上升 1.0 m**、爬升速度 0.2 m/s，控制器另有约 3 秒电机预转阶段。以实际状态为准，不要靠固定倒计时判断完成。

在另一个终端加载工作空间后检查：

```bash
source devel/setup.bash
rostopic echo /px4ctrl/state
```

应看到 `AUTO_TAKEOFF` 最终进入 `AUTO_HOVER`。在查看话题的终端按 Ctrl+C 只结束这次查看；不要在启动飞行栈的终端按。进一步核对：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/extended_state
rostopic echo -n 1 /px4ctrl/mission_ready
```

要求 `armed: True`、`mode: OFFBOARD`、`landed_state: 2`（IN_AIR）、`data: True`，并目视确认悬停与地图/定位正常。`landed_state: 3` 只是正在起飞，`4` 是正在降落，都不是可以开始航线的确认。

如果起飞请求被拒绝，查看控制台原因；不要为了重新触发而在空中随意拨回 DOWN，因为进入 DOWN 会请求降落。

### 第三步：用 Publish Point 标航点

1. RViz 的 Fixed Frame 使用 `world`，确认地图和飞机位置合理。
2. 选择 **Publish Point**，依次点击地图上的目标位置。若工具点击一次后退出，重新选择该工具继续点击。
3. 核对带编号的橙色点，编号就是执行顺序。MarkerArray 话题为 `/mission/clicked_waypoints`。

默认所有点击航点使用 **world 坐标的 z=1.0 m**，不是离当前位置再升高 1 m，也不保证等于离地高度；与相对起飞高度是两个不同概念。默认不采用点击点的 z。需要更改时，在启动前调整 `multipointplan_exp_lio.launch` 的 `clicked_point_height`；单独启动任务节点也可传 `clicked_point_height:=1.2`，但不要和整栈入口重复启动。

还没执行时，如果点错，可以清空后重新标：

```bash
rostopic pub -1 /mission/clear_clicked_route std_msgs/Empty '{}'
```

### 第四步：明确开始执行

标好以后，下面三个方法任选一个，发一次即可：

| 方法 | 操作 |
| --- | --- |
| RViz | 用 **2D Nav Goal** 再点一下，作为开始按钮 |
| RC8 | 从 MIDDLE 拨到 UP |
| 终端 | 运行下面的命令 |

```bash
rostopic pub -1 /mission/start_clicked_route std_msgs/Empty '{}'
```

**2D Nav Goal 的点击坐标不加入航线，也不替代 Publish Point。** 修改后的点击模式在没有点、路线已执行或地面未起飞时，会拒绝开始，不会再偷偷执行 YAML 中的 `(0.5, 0, 1)`。

开始后，任务节点依次发布 `/goal`，Diff-Planner 生成避障轨迹，轨迹服务器发布 `/setpoints_cmd`，PX4Ctrl 在 CMD_CTRL 中跟踪轨迹。每个航点满足位置误差、速度及连续稳定时间后才推进。点击航线不执行云台扫描；独立的视频识别仍按启动配置运行。

实机默认 RViz 配置已移除直发 `/goal` 的 **3D Nav Goal** 工具，避免绕过上述任务检查。旧版自定义 RViz 配置若仍有它，请按本指南使用 Publish Point 和 2D Nav Goal。

### 第五步：完成、返程、降落

航线结束只表示航点执行完成，**不会自动降落**。完成后可以清空、重新标点并再次开始点击航线；新一轮开始仍须满足任务就绪条件。

需要返程时，可从 **UP 拨到 MIDDLE**，或在工作空间根目录运行：

```bash
./sh_files/back.sh
```

它执行 `points.yaml` 中的 `test_back`。当前值为 world 坐标 `(0, 0, 1)`，不是 PX4 原生 RTL，也不是自动识别的家位置；使用前确认这个坐标和路线适合现场。

需要降落时，可以把 RC8 从其他档位拨到 DOWN；CH6 保持指令模式时请求自动降落。如果 RC8 本来就在 DOWN，或者不希望再拨动开关，可运行：

```bash
./sh_files/land.sh
```

该脚本现在通过 `/mission/land` 通知任务节点：取消当前航线、停止轨迹输出、重发 LAND，等待 PX4Ctrl 从 CMD_CTRL 回到 AUTO_HOVER 后接收降落。确认 ON_GROUND 且 `armed: False` 后停止重发。CH6 退出指令模式时，RC 分支会停止自动 LAND 重试，后续按已验证的遥控接管流程处理。

**完成降落或任务故障锁止后，清空点击点不会解除控制锁。** 确认落地且上锁，退出并重新启动整套飞行栈，再进行下一次飞行；不要只重启 multipoint。

## 3. 可选：一键自动起飞

如果希望脚本在准备条件满足后主动发送一次起飞请求，省略 `--start-only`：

```bash
MISSION_SOURCE=clicked ./sh_files/one_click_takeoff.sh
```

这个命令会尝试自动起飞，不是单纯启动软件。启动前保持 CH8 DOWN、CH5/CH6 高值、摇杆居中。脚本只负责起飞，不会自动执行航线。等到确认完成悬停，再按第二节的 Publish Point → 开始执行流程操作。

自动起飞后 CH8 可能仍在 DOWN。此时可以直接用 2D Nav Goal 或终端开始航线；也支持从 DOWN 直接拨到 UP 请求开始，不再依赖 RC 是否采到中位。为避免混淆，首次验证建议采用 RViz/终端触发。此时若想降落，使用 `land.sh`，不用为了制造档位变化来回拨动 RC8。

若发送起飞消息的命令报错或起飞确认超时，脚本保留飞行栈供检查和遥控接管，不会自动重试起飞。

## 4. 可选：按 YAML 航点执行云台任务

启动前检查 `src/user_command/multipoint/config/points.yaml` 的航点、悬停时间和云台角度，显式选择 preset：

```bash
MISSION_SOURCE=preset ./sh_files/one_click_takeoff.sh --start-only
```

起飞并确认悬停后，用 RC8 UP、2D Nav Goal，或 `./sh_files/pub_trigger.sh` 开始 YAML 任务。不要使用 `/mission/start_clicked_route`；preset 模式会拒绝这个点击航线专用入口。

带云台的预设任务流程为：到点 → 稳定悬停 → 等待 hover_sec → 执行云台角度/扫描 → 收到完成回执 → 下一点。关闭 `A8MINI_START_GIMBAL_NODE` 时保留到点悬停，但跳过云台任务。当前旧云台协议按航点 ID 去重，因此本版限制每次运行只能启动一轮带云台的预设任务；重复执行需落地并重启整栈。

自动 TF 卡录像、实时识别、电脑保存带框视频是三个独立开关。当前 `points.yaml` 分别是 `false / true / false`，所以默认不自动录 TF 卡、不保存带框文件，但会启动识别。

## 5. A8mini 画面和退出

本次日志的 RTSP 收帧数一直为 0，网口 eth0 的 carrier 为 0，到相机地址的路由还走了无线网关。先查相机供电、接线、载板网口及同网段配置。在机载 Ubuntu 上可执行以下只读检查（网口名称按现场替换）：

```bash
ip -br address
ip route get 192.168.144.25
cat /sys/class/net/eth0/carrier
ping -c 3 192.168.144.25
```

相机默认地址、RTSP 和网口准备见 [A8mini README](../A8mini/README.md)。仅关闭识别窗口不会关闭飞行栈；它也不是飞行急停。

| 要结束什么 | 操作 |
| --- | --- |
| 只关检测窗口及其识别/采集进程 | 检测窗口中按 `q`、Esc，或点窗口关闭按钮 |
| 只停止 `rostopic echo` | 在运行 echo 的终端按 Ctrl+C |
| 退出本次完整飞行栈 | 落地上锁后，在原启动终端按一次 Ctrl+C，等待日志保存完成 |

退出时先清理本次飞行进程及检测子进程，随后收尾 rosbag 和诊断。遇到卡在原生库的子进程会有超时升级清理，可能需要数十秒。不要一看到窗口没立即消失就重开另一份程序。集成退出监督不管理你另外单独启动的旧版 `A8mini_RTSP_YOLO_Detection.py`。

**RViz 窗口与检测窗口不同：关闭 RViz 会使它的 roslaunch 退出，监督脚本随后停止整套飞行栈。** 因此关闭 RViz 也必须放在落地上锁后。只想停止识别时，关闭 A8mini 检测窗口即可。

## 6. 常见提示

| 提示/现象 | 含义与处理 |
| --- | --- |
| `Mission start rejected` | 任务未执行，检查定位、飞控、悬停状态、规划心跳和载荷节点；点位保留，就绪后重新触发 |
| `No RViz clicked points` | 当前是点击模式，先用 Publish Point 标点 |
| `route is already active or consumed` | 运行中不要改点；已完成则先清空再标 |
| `Mission stopped` | 定位/飞控/控制器/心跳失效，或到点/云台超时；不会用旧定位继续推进，处理飞机后落地重启 |
| 没有 `/px4ctrl/mission_ready` | 检查是否重新编译并运行了这份工作空间的 px4ctrl |
| `Waiting`、没有识别画面 | 查相机链路与 RTSP，不能以加载了 YOLO 模型推断相机已连通 |

默认 odom 超时 0.5 s、飞控状态超时 2.5 s、控制器/规划心跳超时 0.5 s、单航点飞行超时 120 s、云台等待超时 90 s。超过期限会停止任务并锁止后续派点；不要仅为消除告警而增大阈值。
