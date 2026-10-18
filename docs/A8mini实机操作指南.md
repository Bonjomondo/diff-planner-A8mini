# A8mini 实机操作指南（LIO / RViz，2026-09-17）

你描述的流程基本正确：**先把 RC8 从 DOWN 拨到 MIDDLE，请求自动起飞；确认已经悬停，再用 RViz 的 Publish Point 标航点，最后发出开始指令。** 标点可以提前在地面完成，但执行必须等起飞完成。这里“手动拨开关”触发的是程序自动解锁、爬升与悬停，不是用油门手动飞起来。

本指南对应当前源码，更新后需在机载 Ubuntu 上编译并加载新版节点。2026-10-18 更新增加
CH9/SWB 地面复位和二次起飞流程：相关节点编译、手势单元测试及模拟飞控/遥控器话题的
ROS 回归已通过，尚未实飞验收。早期问题背景见[修复说明](2026-09-17_修复-A8mini-report.md)。
**复位后远程模式必须重新上传完整航点，点选模式必须重新标点。**
**Ctrl+C 是停止程序，不能当作降落指令；先落地、确认上锁，再退出。**

## 1. 第一次更新后准备

在机载电脑工作空间根目录执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DROS_EDITION=ROS1
source devel/setup.bash
```

A8 mini 检测所需的 `yolo11s.engine`、类别表和检测辅助代码已随仓库提供，部署时不需另放 `A8mini_Detction` 目录。目标机仍须准备匹配的 PyTorch、Ultralytics、OpenCV、CUDA/TensorRT 环境；TensorRT 引擎跨 JetPack/TensorRT/GPU 环境的兼容性须在目标机核实。参数和故障边界见 [检测单仓整合说明](../A8mini/20260930_A8mini检测单仓整合说明.md)。

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
| CH9 / SwB | 降落/故障锁定后的地面任务复位 | 落地、未解锁、CH8 DOWN 时，拨到另一端保持 2 秒再拨回；完成后重新准备航点 |

代码对 CH8 的有效窗口是三个中心值 `999 / 1499 / 1999` 各 ±100（不含边界）。这次日志的 CH8 一直为 DOWN，CH3 约为 1293；日志内没有起飞消息。CH5/CH6 建议在启动前就置于所需位置；如果控制器状态或 RC 映射不符合预期，停在地面检查，不要反复拨开关试飞。

## 2. 推荐流程：RC8 起飞，RViz 连续标点

### 第一步：启动系统，但不自动发起飞指令

带 A8mini 时，在工作空间根目录运行：

```bash
MISSION_SOURCE=clicked ./sh_files/one_click_takeoff.sh --start-only
```

此入口自动加载 ROS/工作空间、启动飞行栈并保存调试日志。等待“仅启动模式：未发送起飞指令”。此时 CH8 保持 DOWN。这个入口只启动一次，不要再运行另一份 `run_single_lio.sh`。

没有接 A8mini 时：

推荐直接在 `src/user_command/multipoint/config/points.yaml` 中配置：
```yaml
enable_gimbal: false
```
启动脚本将自动检测该配置，自动关闭云台与 YOLO 实时识别节点，且航点文件中无需填写任何 `gimbal_*` 参数。

也可以通过环境变量临时覆盖：

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

### 第六步：SWB 复位、重新准备航点、再起飞

**完成降落或任务故障锁止后，清空点击点不会解除控制锁。** 新版支持 SWB 地面复位，
无需为二次起飞重启飞行栈。**复位会清除上一轮的机载航线、当前目标和轨迹；
远程模式必须重新上传完整航点，相同路线也要再上传一次。** 推荐全程在地面完成
复位和航点准备，再请求第二次起飞。

先完成一次遥控器配置：辅助通道设置 **九通：SwB**，并确认机载端实际收到随 SWB
变化的第九通道。遥控器菜单分配成功只代表发射端设置完成，还需接收机传出该通道。

每次降落锁定后按下列步骤操作：

1. 确认飞机已经落地，飞控显示 `armed: False`（电机已上锁）、`ON_GROUND`；
   SwG/CH8 保持 DOWN，飞控连接和遥控器数据正常。
2. SWB 从当前端档拨到另一端，保持 2 秒，随后拨回原端档。三档开关经过中档即可，
   需要在另一端保持；上→下→上或下→上→下均可。
3. 等待日志 `RC CH9: mission reset succeeded`。远程模式应回到 `IDLE`，航点数量为 0；
   这是正常的复位结果，表示等待重新上传，不是已经准备好航线。
4. 在地图中重新核对当前 `world` 原点、飞机位置、航点坐标、飞行高度和执行顺序。
   即使使用与上一轮相同的坐标，也要重新确认这些坐标适用于当前定位。
5. 按本次启动的任务来源准备航点：

   | 本次模式 | 必须重新执行的操作 | 操作完成后应看到什么 |
   | --- | --- | --- |
   | 地面站远程 `remote` | 等待新地图会话，重新上传本轮完整航点 | 本轮上传成功的机载回执，任务 `READY`，航点数与本轮一致 |
   | RViz 点选 `clicked` | 用 Publish Point 按顺序重新标点 | 重新出现带编号的橙色航点，数量、顺序和高度正确 |
   | YAML 预设 `preset` | 重新核对当前节点已加载的预设航点 | 确认预设路线和当前原点匹配；无需地面站上传 |

6. 航点准备完成后，满足 CH5/CH6 和摇杆回中条件，将 SwG 从 DOWN 拨到 MIDDLE，
   停在 MIDDLE 等待起飞完成。不要一次连续拨到 UP。
7. 确认飞机悬停、地图定位正常、控制器任务就绪，再从 MIDDLE 拨到 UP 或使用本模式
   对应的开始按钮，开始执行本轮航线。

远程模式的完整顺序是：**落地上锁 → SwG 下档 → SWB 往返复位 → 确认复位成功 →
核对原点 → 重新上传完整航点并确认 READY → SwG 中档起飞 → 悬停确认 → 开始任务。**
具体会话与上传确认见 [地面站二次飞行步骤](GCS动态航线接入.md#降落后用-swb-复位并重新上传航点)。

地面站仍显示上一轮的航线图形、曾经出现过“上传成功”、TCP 重新连接成功，都不代表
复位后机载端已重新收到航点。**必须看本轮上传回执和当前航点数量。** 未上传/未重新标点
时，起飞请求不会补回旧航线，开始任务会因没有可执行航点而被拒绝。

如果误在复位前上传了航线，成功复位会把它清掉，需要在复位之后再上传一次。
起飞或开始任务被拒绝时先查看原因；如果再次进入降落锁定，下一轮重新从本节第 1 步开始，
不要省略复位后的航点准备。预设 YAML 在节点启动时加载，单纯编辑文件再拨 SWB 不会
重新读取文件；已执行过的预设云台任务也仍受独立的一次性保护约束。

该手势仅在降落/故障锁定时生效，复位会清除旧航线和轨迹，不会解锁或起飞。
短拨、开机档位及未完成往返均不触发；中途解锁、离地、状态过期或 CH8 离开 DOWN
会取消，不能等落地后自动补执行。默认识别 CH9，可用 `rc_reset_channel:=0` 禁用。
若接收机只输出八个通道或 CH9 无有效信号，需先检查通道传输设置；终端仍可使用
`./sh_files/reset_mission.sh` 执行相同复位。规划器强制急停仍需检查并重启。

## 3. 可选：脚本自动起飞

默认入口不会自行起飞，需要你把 RC8 从 DOWN 拨到 MIDDLE。若想让脚本在准备条件
满足后主动发送一次起飞请求（无需拨 RC8），加 `--takeoff`：

```bash
MISSION_SOURCE=clicked ./sh_files/one_click_takeoff.sh --takeoff
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
