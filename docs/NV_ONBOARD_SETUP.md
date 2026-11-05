# nv 机载电脑：首次配置与手动启动指南

整理日期：2026-11-05，北京时间。

本文针对 `nv@192.168.2.105`，按《无人机自动往返_本轮交接与手动操作指南_2026-10-29.md》中“5. 手动启动：完整前置准备命令”的顺序整理，并参考《ly-无人机项目交接文档_更新至2026-10-28.md》。所有机载路径和硬件参数以本次对 nv 的只读核查为准。

本文初版仅整理操作步骤。2026-11-05 后续已完成新仓库首次完整编译、Livox SDK 枚举兼容修复、Bash/Zsh 环境加载验证，以及串口授权和必要服务部署。实际传感器建图和飞行栈未在本次部署中启动。第 1 节区分已完成事项和待验证事项；后续命令供操作者按需执行。

## 0. 日常开机后使用这两个入口

**本机首次部署现已完成，第 2～4 节保留作恢复和重部署参考，日常无需重复安装。** 开机后在 Windows `ssh nv`，进入机载终端后可直接执行下面命令，Bash、Zsh 均可，无需先手动 source ROS 环境。

启动雷达建图与点云：

```bash
~/.local/bin/nv-start-mapping
```

该入口确认后台服务运行后，以 `nv.yaml` 启动传感器建图；已有同 profile 且数据正常的建图可复用。点云转发服务 `agri-pointcloud-bridge.service` 和建图请求 API `agri-mapping-start.service` 已设为开机启动，用户无需保持 SSH 登录。API、桥接服务开机运行不代表传感器已经开始建图。

按第 5 节准备 EKF、规划器和控制器后，手动启动任务适配器与网关：

```bash
~/.local/bin/nv-start-gateway
```

该入口启动后台 `agri-task-gateway.service`，包含 `/multipointplan` 和 `/gcs_gateway`；网关服务没有设为开机自启，启动命令本身也不上传或开始任务。其余 EKF、规划器和控制器仍按第 5 节手动启动。只看点云时无需启动网关或后续飞行节点。

只读查看三个服务状态：

```bash
systemctl --user status agri-pointcloud-bridge.service agri-mapping-start.service agri-task-gateway.service --no-pager
```

本次已从地面站虚拟机验证 `127.0.0.1:37262` 点云协议和 `127.0.0.1:37263/status` 可透传到 nv。地面站现有隧道目标已为 `nv@192.168.2.105`；任务端口 `127.0.0.1:8765` 在手动启动网关后才有对应机载服务。本次只对任务 launch 和可执行程序做离线检查，未启动任务适配器或飞行控制器。

## 1. 当前设备与配置状态

| 项目 | nv 当前配置或状态 |
|---|---|
| SSH | `nv@192.168.2.105:22`；本次 Windows 电脑已验证公钥免密登录，别名 `nv` |
| 主机名 / 架构 | `ubuntu` / `aarch64`，Jetson Tegra234 |
| Wi-Fi | `TP-LINK_5G_C5A4`；`wlan0` 静态地址 `192.168.2.105/24` |
| Wi-Fi 网关 / DNS | `192.168.2.1`；NetworkManager 和 SSH 已启用开机启动 |
| 雷达有线网卡 | `eth1`，`192.168.9.50/24`；不要改成 ly/zzz 使用的 `192.168.1.5` |
| 雷达 | `MID360`，`192.168.9.88`；本次 ping 可达 |
| 雷达广播码 | `47MDM630020460` |
| 本次工作目录 | `/home/nv/Documents/diff-planner-A8mini` |
| 分支 / 核查时提交 | `codex/indoor-point-roundtrip` / `7b6aa2d`；后续以实际 `git status` 为准 |
| 建图 profile | `config/mapping_profiles/nv.yaml`，已新增且已通过离线校验 |
| IMU 来源 | `/mavros/imu/data`，因此建图也需要飞控正常连接 |
| 飞控连接 | `/dev/ttyTHS1:921600`，来自本机原 MAVROS 配置；不是 `/dev/ttyACM0:57600` |
| 外参来源 | `/home/nv/swarm/src/realflight_modules/faster-lio/config/mid360.yaml` 中现有外参；本次没有重新标定 |
| 已有基础环境 | ROS Noetic、catkin_make、CMake、Python3/PyYAML、Livox SDK2 静态库存在 |
| 新仓库构建状态 | 初次检查未编译；2026-11-05 后续已完整编译成功，`devel/setup.bash`、`devel/setup.zsh` 均已生成并验证加载 |
| 串口权限 | 已将 nv 加入 `dialout`；对当前设备增加 nv 读写 ACL，使旧会话立即可用；后续开机由持久组权限生效 |
| 建图服务 | 用户级 API 和点云桥均已安装、enabled、active；`Linger=yes`，未登录时也可启动 |
| 任务网关 | 已部署手动启动的用户服务 `agri-task-gateway.service`，默认 disabled/inactive |
| 控制参数 | 仓库当前质量 `1.453 kg`、悬停油门比例 `0.42`、低电压阈值 `21.0 V`；尚未确认适用于 nv 实际机体 |

已完成并验证 SSH 免密、Wi-Fi 静态地址、nv profile 离线生成、新仓库首次完整编译和环境加载，以及 API/点云桥服务和地面站隧道连通。8 项现有 profile 测试通过，不代表已经在 nv 上通过真实点云、悬停或自动往返验收。静态 Wi-Fi 与服务持久配置均已保存，尚未通过整机重启验证。

## 2. 连接与首次串口准备

### 2.1 Windows 连接机载

在 Windows PowerShell 执行：

```powershell
ssh nv
```

也可使用完整地址：

```powershell
ssh nv@192.168.2.105
```

Windows 须能访问该 Wi-Fi 所在局域网。固定地址绑定于 `TP-LINK_5G_C5A4` 连接配置，换到其他 Wi-Fi 不会自动沿用它。

### 2.2 统一使用 Bash

nv 默认终端是 Zsh，提示符可能是 `➜ ... git:(...)`。本文机载命令按 Bash 编写，**每个新机载终端先执行一次**：

```bash
bash --noprofile --norc
```

之后再执行后文的环境命令，避免将 Bash 的 `source setup.bash`、`read -p` 等直接复制到 Zsh。

### 2.3 给 nv 持久串口权限

在 nv 机载终端执行：

```bash
sudo usermod -aG dialout nv
```

组权限不会自动刷新到已经打开的 SSH 会话或用户服务管理器。首次部署时，保存工作、确认飞机在地面锁定后，可重启一次，同时核验静态 IP：

```bash
sudo reboot
```

等待启动后，从 Windows 重新 `ssh nv`，再进入 Bash 并检查：

```bash
id -nG
ls -l /dev/ttyTHS1
test -r /dev/ttyTHS1 && test -w /dev/ttyTHS1 && echo "串口读写权限正常"
ip -br -4 address show wlan0
ip -br -4 address show eth1
```

应看到 `dialout`、Wi-Fi 地址 `192.168.2.105/24`、雷达有线地址 `192.168.9.50/24`。若暂不重启，至少退出旧会话并重新登录；启动用户服务前还需确认其继承的新组权限。

串口仍异常时检查占用，不要执行旧脚本里对全部 `/dev/tty*` 的 `chmod 777`：

```bash
sudo fuser -v /dev/ttyTHS1
systemctl status nvgetty.service --no-pager
```

仅在确认某个登录服务实际占用 `/dev/ttyTHS1` 后，再处理该服务。本机检查时 `nvgetty` 虽 enabled，但 inactive；不能仅凭服务存在就判定飞控串口被占用。

## 3. 首次编译新工作空间

所有编译命令都在 nv 上执行；保留 `/home/nv/swarm`、`/home/nv/Diff-Planner` 等旧项目作为原始参数参考。

先确认分支和工作区，不用 `reset --hard` 或 `clean` 清除本地配置：

```bash
cd /home/nv/Documents/diff-planner-A8mini
git status --short --branch
git log -1 --oneline
cat config/mapping_profiles/nv.yaml
```

### 3.1 仅先验证雷达建图时

仓库有只构建雷达驱动和 Faster-LIO 的入口，能准备 Livox ROS1 的 `package.xml`：

```bash
cd /home/nv/Documents/diff-planner-A8mini
bash sh_files/build_mapping_only.sh
```

成功后可执行第 5.1、5.2 节验证传感器。**仅建图构建不保证已经生成 EKF、规划器、控制器和任务节点。**

### 3.2 准备完整手动启动时

新开已进入 Bash 的终端，清除其他工作空间的继承环境，再加载 ROS 基础环境：

```bash
cd /home/nv/Documents/diff-planner-A8mini
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash
```

首次克隆没有 Livox 的 `package.xml`，先准备 ROS1 清单。下面整个代码块复制执行；若已有不同清单，会明确停止该块而不覆盖：

```bash
(
  set -e
  cd /home/nv/Documents/diff-planner-A8mini
  driver=src/livox_ros_driver2
  if [ ! -f "$driver/package.xml" ]; then
    cp "$driver/package_ROS1.xml" "$driver/package.xml"
  elif ! cmp -s "$driver/package_ROS1.xml" "$driver/package.xml"; then
    echo "现有 Livox package.xml 与 ROS1 模板不同，请先核对"
    exit 1
  fi
  catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release -j2 -l2
)
```

**确认构建成功后**再加载新目录并检查包来源：

```bash
source /home/nv/Documents/diff-planner-A8mini/devel/setup.bash
rospack find faster_lio
rospack find livox_ros_driver2
rospack find ekf
rospack find diff_planner
rospack find px4ctrl
rospack find multipoint
```

这些路径都应位于 `/home/nv/Documents/diff-planner-A8mini/src/`。若出现 `/home/nv/swarm` 或其他旧目录，重新打开干净 Bash 并按第 5.1 节加载环境。

编译缺依赖时，先保存构建报错。可在已准备 ROS1 清单后检查声明的 ROS 依赖：

```bash
rosdep check --from-paths src --ignore-src --rosdistro noetic
```

若 rosdep 数据库已经初始化且源可用，可按缺失项安装：

```bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
```

rosdep 不能保证提供所有项目自带/第三方依赖；失败时按具体日志处理。本机已有 `/usr/local/lib/liblivox_lidar_sdk_static.a`，不需要为了本文先重装 SDK。2026-11-05 本机已完成首次完整编译；上述依赖排查命令用于后续环境变化或新依赖缺失时，不必因本文重复安装已有依赖。

首次构建曾因本机 SDK 缺少 `kLivoxLidarTypeMid360s` 枚举而失败。已在 `src/livox_ros_driver2/src/comm/pub_handler.cpp` 增加兼容检测：SDK 提供该枚举时保留识别，否则仅按其已支持的设备类型处理。兼容检测已通过本机 SDK、新旧枚举模型的独立编译检查，随后完整构建成功。该源码修复目前留在工作区，尚未提交。

## 4. 首次安装建图 API 与点云转发服务

下面统一采用 **nv 用户级服务**，状态和日志命令都加 `--user`。不要直接复制仓库内 `deploy/agri-mapping-start.service` 或旧 ly 系统级桥接单元，它们含旧用户和旧路径。

### 4.1 安装本仓库的建图启动 API

在 nv 上执行，须先完成建图相关构建，并确认没有正在运行的建图需要保留：

```bash
cd /home/nv/Documents/diff-planner-A8mini
bash deploy/install_mapping_start.sh --profile nv
sudo loginctl enable-linger nv
systemctl --user is-enabled agri-mapping-start.service
systemctl --user is-active agri-mapping-start.service
curl --max-time 5 http://127.0.0.1:37263/status
```

安装器会校验 profile、运行相关离线测试、生成用户级服务。`enable-linger` 让用户服务在未登录时也可运行。API 监听 `127.0.0.1:37263`，等待建图请求；安装 API 本身不会开始飞行任务。

### 4.2 从地面站源码复制点云桥

点云桥不在机载仓库中。本次核查其来源文件存在于地面站虚拟机：

```text
/home/cat/projects/agri_gcs_visual/deploy/pointcloud_bridge.py
```

在本次 Windows PowerShell 执行，使用已有 `agri-vm`、`nv` SSH 别名中转：

```powershell
ssh nv "mkdir -p /home/nv/.local/lib/agri-gcs"
scp -3 agri-vm:/home/cat/projects/agri_gcs_visual/deploy/pointcloud_bridge.py nv:/home/nv/.local/lib/agri-gcs/pointcloud_bridge.py
```

若以后虚拟机源码位置变更，先找到实际文件再替换源路径；不要用其他机器的服务文件代替桥接程序。

### 4.3 在 nv 上创建用户级点云桥服务

先确认目标文件不存在；如已有部署，先检查现有内容和服务状态。下面创建块会拒绝覆盖已有启动脚本或服务文件：

```bash
(
  set -e
  mkdir -p /home/nv/.local/lib/agri-gcs /home/nv/.config/systemd/user
  test -f /home/nv/.local/lib/agri-gcs/pointcloud_bridge.py
  test ! -e /home/nv/.local/lib/agri-gcs/start-pointcloud-bridge-nv.sh
  test ! -e /home/nv/.config/systemd/user/agri-pointcloud-bridge.service

  cat > /home/nv/.local/lib/agri-gcs/start-pointcloud-bridge-nv.sh <<'SCRIPT'
#!/bin/bash
set -e
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
source /opt/ros/noetic/setup.bash
source /home/nv/Documents/diff-planner-A8mini/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
exec /usr/bin/python3 -u /home/nv/.local/lib/agri-gcs/pointcloud_bridge.py \
  --bind 127.0.0.1 --port 37262 --cloud-source body
SCRIPT

  cat > /home/nv/.config/systemd/user/agri-pointcloud-bridge.service <<'SERVICE'
[Unit]
Description=nv read-only ROS point cloud bridge
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash /home/nv/.local/lib/agri-gcs/start-pointcloud-bridge-nv.sh
Environment=ROS_MASTER_URI=http://127.0.0.1:11311
Environment=ROS_IP=127.0.0.1
Restart=on-failure
RestartSec=3
NoNewPrivileges=true
MemoryMax=1536M
CPUQuota=100%
Nice=5

[Install]
WantedBy=default.target
SERVICE
)
```

创建成功或已核对现有文件后执行：

```bash
systemctl --user daemon-reload
systemctl --user enable --now agri-pointcloud-bridge.service
systemctl --user status agri-pointcloud-bridge.service --no-pager
ss -ltn | grep -E ':(37262|37263)[[:space:]]'
```

桥接器只读 `/laserMapping/cloud_registered_body` 和匹配时间的 `/laserMapping/odometry`，监听 `127.0.0.1:37262`。桥服务 active 仅表示程序在运行；ROS 或传感器未启动时可以处于等待状态，不能据此认定已有 LIVE 点云。

## 5. 手动启动：完整前置准备命令

首次配置完成后，每次开机按本节顺序启动。飞机保持地面锁定，确认供电、接线和遥控接管条件。**如果本次只需要雷达建图，完成 5.1、5.2 后即可停止增加节点。**

进入 5.6 之前，还需核对 nv 实际带电池质量、推力模型、悬停油门、通道方向和 PID。仓库里的 `1.453 kg / 0.42 / 21.0 V` 来自原配置，本次没有对 nv 实机确认或调参；本指南不把它们当作已验收值。

### 5.1 每个机载终端先加载环境

新 SSH 终端先执行第 2.2 节进入 Bash。看到 `➜` 且仍在 Zsh 时，不能直接加载下面的 `setup.bash`。先**单独执行**这一条并等待进入 Bash：

```bash
bash --noprofile --norc
```

进入 Bash 后，再复制下面这段。工作空间须先完成第 3 节编译，缺少 `devel/setup.bash` 时不能跳过编译：

```bash
cd /home/nv/Documents/diff-planner-A8mini
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash
source /home/nv/Documents/diff-planner-A8mini/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export AGRI_MAPPING_PROFILE=nv
```

若要保留当前 Zsh，只加载环境和运行建图脚本，可使用对应的 `.zsh` 版本：

```zsh
cd /home/nv/Documents/diff-planner-A8mini
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.zsh
source /home/nv/Documents/diff-planner-A8mini/devel/setup.zsh
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export AGRI_MAPPING_PROFILE=nv
```

后文包含 `read -p` 等 Bash 专用写法；执行完整手动启动步骤时仍按上面的两步进入 Bash 并重新加载环境。不要混用两种 shell 的 setup 文件。

先看已有节点：

```bash
rosnode list
```

`Unable to communicate with master` 可能只是 ROS master 尚未启动，下一步的建图脚本会按需启动它。若已有节点，先检查进程路径和状态，不重复运行同名节点，也不在空中排查重启。

### 5.2 终端 A：雷达建图与点云转发

先检查雷达网卡和连通性：

```bash
ip -br -4 address show eth1
ip route get 192.168.9.88
ping -c 2 -W 1 192.168.9.88
```

应经 `eth1`、源地址 `192.168.9.50` 访问雷达。然后检查已部署的用户服务，并启动 nv profile：

```bash
systemctl --user is-active agri-pointcloud-bridge.service agri-mapping-start.service &&
bash /home/nv/Documents/diff-planner-A8mini/sh_files/start_mapping_only.sh --profile nv
```

已安装但 inactive 的服务，可在地面锁定时启动后重新检查：

```bash
systemctl --user start agri-pointcloud-bridge.service agri-mapping-start.service
```

**只检查本机建图、暂不部署地面站桥接/API 时，也可直接执行建图脚本，不以两个服务 active 为前提：**

```bash
bash /home/nv/Documents/diff-planner-A8mini/sh_files/start_mapping_only.sh --profile nv
```

脚本启动 ROS master、MAVROS、Livox 驱动和 Faster-LIO；成功返回后，传感器进程留在后台。不要另外再开一份 Livox 驱动或 MAVROS。检查：

```bash
timeout 5 rostopic echo -n 1 /mavros/state
timeout 8 rostopic hz /mavros/imu/data
timeout 8 rostopic hz /laserMapping/cloud_registered_body
timeout 8 rostopic hz /laserMapping/odometry
rosparam get /agri_mapping/profile_digest
```

`timeout` 到时结束持续频率观察是正常行为。应实际看到 `connected: True` 和持续更新的数据，而不只是节点名。报错日志位于 `/home/nv/.local/state/agri-gcs/mapping-start/` 下的 `mavros.log`、`mapping.log` 等文件。

本机飞控经 UART `/dev/ttyTHS1:921600` 连接。脚本中的“检查 USB”是通用提示；在 nv 上应查实际 UART 接线、权限、供电、波特率和飞控对应 MAVLink 端口设置。本次未读取或修改飞控参数。

### 5.3 终端 B：EKF 定位

先执行 5.1，再启动：

```bash
roslaunch ekf ekf_lidar.launch
```

保持该终端运行。在另一个已加载环境的终端检查：

```bash
timeout 5 rostopic echo -n 1 /ekf/ekf_odom/pose/pose/position
timeout 5 rostopic echo -n 1 /ekf/ekf_odom/header
```

记录静止落地位置 `(X0,Y0,Z0)`。`Z0` 是定位参考点的 world 坐标，不能直接当物理地板高度，也不能默认等于 0。后续地图、围栏、起降点和目标必须属于同一次连续 world 会话。

### 5.4 终端 C：输入本次场地围栏并启动规划器

先执行 5.1。围栏顺序固定为 `[xmin, ymin, zmin, xmax, ymax, zmax]`。按原指南创建本地运行启动文件；下面块只在不存在时创建：

```bash
mkdir -p /home/nv/.local/state/agri-gcs/point-mission/manual
if [ ! -e /home/nv/.local/state/agri-gcs/point-mission/manual/planner.launch ]; then
cat > /home/nv/.local/state/agri-gcs/point-mission/manual/planner.launch <<'LAUNCH'
<launch>
  <arg name="bounds" />
  <arg name="floor" />
  <arg name="ceiling" />
  <param name="/indoor_mission/enabled" value="true" />
  <param name="/indoor_mission/skip_validation" value="false" />
  <rosparam param="/indoor_mission/bounds" subst_value="true">$(arg bounds)</rosparam>
  <include file="$(find diff_planner)/launch/exp/run_exp_single_lio.launch" />
  <param name="/drone_0_diff_planner_node/grid_map/virtual_ground" value="$(arg floor)" type="double" />
  <param name="/drone_0_diff_planner_node/grid_map/virtual_ceil" value="$(arg ceiling)" type="double" />
  <param name="/drone_0_diff_planner_node/grid_map/resolution" value="0.1" />
  <param name="/drone_0_diff_planner_node/grid_map/obstacles_inflation" value="0.3" />
  <param name="/drone_0_diff_planner_node/manager/max_vel" value="0.3" />
  <param name="/drone_0_diff_planner_node/optimization/max_vel" value="0.3" />
  <param name="/drone_0_diff_planner_node/manager/max_acc" value="0.8" />
  <param name="/drone_0_diff_planner_node/optimization/max_acc" value="0.8" />
  <param name="/drone_0_diff_planner_node/manager/max_jer" value="5.0" />
  <param name="/drone_0_diff_planner_node/fsm/mondify_final_goal" value="false" />
  <param name="/drone_0_diff_planner_node/fsm/waypoint_num" value="0" />
</launch>
LAUNCH
fi
```

其中速度等数值沿用参考指南的受限手动启动参数，不代表已对 nv 完成飞行整定；执行前仍需核对场地和机体条件。已有文件先检查内容：

```bash
cat /home/nv/.local/state/agri-gcs/point-mission/manual/planner.launch
```

根据当前 world 坐标和现场安全间距，人工输入六个边界。下面只检查输入是有限数字且各下界小于上界，**不能代替现场围栏标定**：

```bash
read -r -p "输入 xmin ymin zmin xmax ymax zmax：" xmin ymin zmin xmax ymax zmax
if python3 - "$xmin" "$ymin" "$zmin" "$xmax" "$ymax" "$zmax" <<'PY'
import math
import sys
try:
    v = [float(x) for x in sys.argv[1:]]
    if len(v) != 6 or not all(math.isfinite(x) for x in v):
        raise ValueError('需要六个有限数字')
    if not all(v[i] < v[i + 3] for i in range(3)):
        raise ValueError('各方向下界必须小于上界')
except ValueError as exc:
    sys.exit('围栏输入错误：' + str(exc))
PY
then
  roslaunch /home/nv/.local/state/agri-gcs/point-mission/manual/planner.launch \
    bounds:="[$xmin, $ymin, $zmin, $xmax, $ymax, $zmax]" \
    floor:="$zmin" ceiling:="$zmax"
else
  echo "未启动规划器，请重新核对围栏输入"
fi
```

保持终端 C 运行。在检查终端核对实际配置：

```bash
rosparam get /indoor_mission
timeout 5 rostopic echo -n 1 /indoor_mission/fence
rosservice list | grep /indoor_mission/check_endpoints
```

不能直接复用 ly/zzz 的历史坐标、旧地图或旧围栏；不要设置 `skip_validation=true` 绕过未知空间和围栏检查。修改参数后是否生效要核对实际运行节点，不能认为修改文件就改变了正在运行的地图。

### 5.5 检查电池遥测频率

在检查终端执行：

```bash
timeout 8 rostopic hz /mavros/battery
```

若频率不足以满足现有新鲜度检查，可以请求约 5 Hz 电池遥测，再实际测量：

```bash
rosrun mavros mavcmd long 511 147 200000 0 0 0 0 0
timeout 8 rostopic hz /mavros/battery
```

这是 MAVLink 消息频率请求，不是起飞指令；飞控重启后可能需要重新请求。不要通过放宽新鲜度阈值来掩盖数据缺失。

### 5.6 终端 D：自动起降控制器

**仅在 nv 机体控制参数已核对、现场允许准备完整飞行栈时执行。** 先执行 5.1，再启动：

```bash
roslaunch px4ctrl run_ctrl_indoor_automatic.launch takeoff_height:=0.5
```

保持运行。`0.5` 是相对记录地面位置向上升的米数，来自参考指南，应按本次现场确认；不是绝对 world Z。该入口保留遥控、要求显式起飞请求，但它会启用自动解锁/起降能力，应与后续开始任务操作区分。

### 5.7 终端 E：自动往返任务适配器与网关

已部署网关启动入口，执行：

```bash
~/.local/bin/nv-start-gateway
```

该入口在后台启动 `/multipointplan` 和 `/gcs_gateway`，网关监听机载 `127.0.0.1:8765`；它不替代传感器、EKF、规划器或控制器。无需保持此终端运行。不要再并行执行旧的 `roslaunch multipoint indoor_roundtrip.launch`，以免重复任务节点。

查看启动结果：

```bash
systemctl --user status agri-task-gateway.service --no-pager
```

检查：

```bash
timeout 5 rostopic echo -n 1 /mission/remote/status
timeout 5 rostopic echo -n 1 /mavros/state
timeout 5 rostopic echo -n 1 /mavros/extended_state
timeout 5 rostopic echo -n 1 /px4ctrl/takeoff_ready
timeout 5 rostopic echo -n 1 /px4ctrl/automatic_control_ok
timeout 5 rostopic echo -n 1 /drone_0_traj_server/output_ready
ss -ltn | grep -E ':(8765|37262|37263)[[:space:]]'
```

地面准备时应保持 `armed: False`，飞控连接正常；`landed_state: 1` 表示飞控报告在地面。未上传时任务通常为 `IDLE`；满足条件后才会有 `takeoff_ready: True`。地面未解锁时 `flight_ready: False` 可以是正常状态。

### 5.8 终端 F：在上传和开始前录包

先执行 5.1，再执行：

```bash
mkdir -p /home/nv/flight_logs
rosbag record \
  -O /home/nv/flight_logs/roundtrip_$(date +%Y%m%d_%H%M%S).bag \
  /mission/remote/status \
  /mavros/state /mavros/extended_state /mavros/rc/in /mavros/battery \
  /mavros/imu/data \
  /ekf/ekf_odom /laserMapping/odometry \
  /px4ctrl/state /px4ctrl/mission_ready /px4ctrl/takeoff_ready \
  /px4ctrl/automatic_control_ok /px4ctrl/takeoff_height \
  /px4ctrl/takeoff_land /px4ctrl/abort_automatic \
  /goal /setpoints_cmd /planning/yaw /planning/stop \
  /drone_0_planning/trajectory \
  /drone_0_traj_server/heartbeat /drone_0_traj_server/output_ready \
  /indoor_mission/fence /indoor_mission/map_ready \
  /rosout_agg
```

保持录包终端运行，确认 `.bag.active` 在写入。落地锁定后按 `Ctrl+C` 正常结束，等待完成后再断电。此清单在参考指南基础上增加 `/mavros/imu/data`；没有记录高带宽原始点云，也不保证所有状态话题都已发布，需结合实际 `rostopic list` 核对。

### 5.9 全部启动后的节点对照

在新检查终端执行 5.1，再查看：

```bash
rosnode list | sort
```

| 节点 | 作用 | 来源 |
|---|---|---|
| `/rosout` | ROS 日志 | 5.2 的 ROS master |
| `/mavros` | 飞控状态、IMU、遥控、电池 | 5.2 |
| `/livox_lidar_publisher2` | MID360 驱动 | 5.2 |
| `/laserMapping` | Faster-LIO 位姿和点云 | 5.2 |
| `/agri_pointcloud_bridge_…` | 地面站点云转发，匿名后缀可能变化 | 第 4 节用户服务 |
| `/ekf` | 融合定位 | 5.3 |
| `/drone_0_diff_planner_node` | 规划、局部地图和围栏 | 5.4 |
| `/drone_0_traj_server` | 轨迹转换成位置控制指令 | 5.4 |
| `/px4ctrl` | 自动起降和飞行控制 | 5.6 |
| `/multipointplan` | 往返任务状态机 | 5.7 |
| `/gcs_gateway` | 任务网关 | 5.7 |
| `/record_…` | 数据录制 | 5.8，开启录包后才有 |

只完成建图时，没有 EKF、规划器、控制器和任务网关是正常的。节点名存在不代表数据正常，可进一步检查：

```bash
timeout 5 rosnode ping -c 1 /laserMapping
timeout 5 rosnode ping -c 1 /px4ctrl
rosnode info /px4ctrl
pgrep -af '(^|/)(mavros_node|livox_ros_driver2_node|run_mapping_online|ekf|diff_planner_node|traj_server|px4ctrl_node|multipointplan|gcs_gateway_node.py)([[:space:]]|$)'
```

仅准备建图时无需执行控制器检查。若进程路径同时指向新旧仓库，先在地面查明重复来源。同名 ROS 节点会互相替换，不能只用节点列表判断有没有重复进程。

## 6. 地面站虚拟机连接 nv

本机端口均监听机载回环地址，通过 SSH 转发给地面站。本次核查现有虚拟机隧道已使用各自密钥连接 nv，且主机指纹与 nv 一致；API 和点云桥透传已验证。本节余下步骤用于其他虚拟机或重配连接，当前虚拟机无需重复执行。Windows 和虚拟机各有独立 SSH 密钥。

### 6.1 在地面站虚拟机配置自己的 nv 密钥

以下命令在地面站 Ubuntu 虚拟机的终端中执行，不在 nv 上执行：

```bash
test -f ~/.ssh/agri_nv_ed25519 || ssh-keygen -t ed25519 -f ~/.ssh/agri_nv_ed25519 -C agri-gcs-nv
ssh-copy-id -i ~/.ssh/agri_nv_ed25519.pub nv@192.168.2.105
ssh -i ~/.ssh/agri_nv_ed25519 -o IdentitiesOnly=yes -o BatchMode=yes nv@192.168.2.105 hostname
```

首次认证按提示输入口令，不在本文保存密码。首次确认主机身份时，本次读取的 nv ED25519 主机公钥指纹为：

```text
SHA256:YMil4HQElcBMY8AnKsuwEJf50RHpKM+AA96CyRNGIiw
```

如果主机重装或密钥变化，应重新核验，不能直接忽略主机密钥冲突。已有只允许转发旧飞机端口的受限密钥，不一定可用于本节全部端口。

### 6.2 手动建立三个端口的隧道

先检查虚拟机本地端口和旧飞机隧道：

```bash
ss -ltn | grep -E ':(8765|37262|37263)[[:space:]]'
systemctl --user status agri-mapping-tunnel.service agri-task-tunnel.service --no-pager
```

如确认本次要从旧机切换到 nv，且旧机当前无需这些连接，在虚拟机停止旧隧道；不要让旧的 ly/zzz 隧道占着相同端口：

```bash
systemctl --user stop agri-mapping-tunnel.service agri-task-tunnel.service
```

再在虚拟机启动下列前台隧道并保持终端运行：

```bash
ssh -N -i ~/.ssh/agri_nv_ed25519 \
  -o IdentitiesOnly=yes -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=30 -o ServerAliveCountMax=3 \
  -L 127.0.0.1:37262:127.0.0.1:37262 \
  -L 127.0.0.1:37263:127.0.0.1:37263 \
  -L 127.0.0.1:8765:127.0.0.1:8765 \
  nv@192.168.2.105
```

| 地面站虚拟机本地端口 | nv 对应服务 | 何时有数据 |
|---|---|---|
| `127.0.0.1:37262` | 点云桥 | 第 4 节部署桥后可连接；5.2 后核验 LIVE 数据 |
| `127.0.0.1:37263` | 建图启动 API | 第 4.1 节部署后 |
| `127.0.0.1:8765` | 任务网关 | 第 5.7 节启动后 |

地面站“机载电脑地址”填写 `192.168.2.105`，点云连接填写 `127.0.0.1:37262`，任务网关填写 `127.0.0.1:8765`。实际自动隧道脚本若仍绑定 ly/zzz 用户或主机别名，需要另行改为 nv；本节手动隧道不会修改那些服务的持久配置。

新开虚拟机终端可以只读检查 API：

```bash
curl --max-time 5 http://127.0.0.1:37263/status
```

地面站应显示本次真实 LIVE 点云和新鲜位姿。重启建图后重新确认起降点、目标和围栏，不复用上一次 world 会话的标点。上传任务和“确认自动起飞”是两个独立操作；本文没有代替操作者执行开始或起降。

## 7. 日志和常见问题

在 nv 已进入 Bash 的终端查看用户服务日志：

```bash
journalctl --user -u agri-mapping-start.service -n 60 --no-pager
journalctl --user -u agri-pointcloud-bridge.service -n 60 --no-pager
tail -n 60 /home/nv/.local/state/agri-gcs/mapping-start/mavros.log
tail -n 60 /home/nv/.local/state/agri-gcs/mapping-start/mapping.log
```

| 现象 | 先检查 |
|---|---|
| `新工作目录尚未编译` | 第 3 节，确认新仓库的 `devel/setup.bash` 存在 |
| `Permission denied` 打开 `/dev/ttyTHS1` | `id -nG`、dialout、是否在授权前的旧会话/旧用户服务中运行 |
| 飞控未连接但串口存在 | UART 接线、供电、波特率、飞控端 MAVLink 配置、是否被其他进程占用 |
| 雷达 ping 不通 | eth1 是否为 `192.168.9.50/24`、雷达电源/网线、路由是否走 eth1 |
| 点云超时 | `nv.yaml`、`mapping.log`、广播码、雷达与飞控 IMU 数据是否持续 |
| 服务提示 `Unit ... not found` | 先按第 4 节安装；本方案是用户级服务，命令要加 `--user` |
| 服务 active 但没有 LIVE 地图 | 分别检查真实 ROS 点云、桥、SSH 隧道和地面站，不能只看服务状态 |
| `Address already in use` | 查虚拟机旧隧道/手动 SSH，或机载同端口的旧服务 |
| profile 指纹不一致 | 在地面核对正在运行的 profile；不直接覆盖运行配置或空中切换 |
| `flight_ready: False` | 地面未解锁时可正常；同时检查地面 `takeoff_ready`、数据新鲜度和任务状态 |
| 终端提示符出现 `✗` | Git 工作区有未提交/未跟踪文件；用 `git status --short` 查看，不代表 ROS 出错 |

Wi-Fi 静态配置已保存在当前 NetworkManager 连接中。无需每次开机重新设置，可只读核对：

```bash
nmcli -f connection.id,connection.autoconnect,ipv4.method,ipv4.addresses,ipv4.gateway,ipv4.dns \
  connection show uuid 4f15c62b-50d9-46f5-9ec3-fd46792b4860
```

应为 `manual` 和 `192.168.2.105/24`。本次修改前的 Wi-Fi 配置备份位于 `/var/backups/nv-static-ip.mQOYm5sP/`，目录仅 root 可读；自动回退定时任务已取消。

## 8. 本文的验证范围

- 已核对 nv 的连接、网络、原项目参数、串口节点、仓库脚本、launch 入口和地面站点云桥来源。
- 已完成 nv profile 的默认选择、结构、来源外参和生成配置校验，现有 8 项 profile 测试通过。
- 后续已完成首次完整构建、SDK 兼容检查，以及 Bash/Zsh 环境加载检查；主要建图、定位、规划、控制和任务二进制已生成。
- 后续已完成串口组权限、当前会话设备 ACL、用户服务 lingering、API/点云桥开机部署，以及任务网关的手动启动入口。API 返回 idle；点云桥返回真实模式的协议握手，当前等待传感器启动。
- 已从现有地面站虚拟机验证 API 和点云桥隧道；任务网关仅做 unit、launch、可执行程序检查，本次未启动它。
- 本文命令块按所属 shell 做离线语法核查；嵌入 Python、Bash 启动脚本、规划器 XML、systemd 用户服务单元和仓库引用路径也通过离线核查。语法通过不等于命令已经执行或设备通过验收。
- 待现场执行并确认：整机重启后的自动连接/服务状态、飞控连接、真实点云与位姿、任务网关实际运行、LIO/EKF 一致性、nv 控制参数及实机表现。

建图和任务准备应按上述实际结果逐步推进，不把 ly/zzz 的历史实飞、旧围栏或旧参数当作 nv 的验收记录。
