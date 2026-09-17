<img src="images/nus_logo.png" alt="nus logo" align="right" height="80" />

# Diff-Planner

## 概述
**Diff-Planner** 是为**微分智飞**公司旗下教育无人机子品牌**非凸空间**适配的单机导航避障算法。其基于开源算法 **[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)** ，并由原班人马深度参与算法优化。在继承 **EGO-Planner** 优秀框架的基础上，针对教育无人机平台的特殊需求进行了全面适配和增强，旨在提供更稳定、更可靠的科研体验。

<p align="center">
  <img src="images/navigation.gif" alt="nav" width="600" />
</p>

## 算法优化
- **Diff-Planner** 在 **[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)** 基础上做了多处优化，包括：
>+ **修复**局部规划时 A* 终点在障碍物里，尝试沿 A* 起始方向推出障碍物时的bug。

>+ **修复**优化过程中频繁打印局部目标点在障碍物里（"Local target in collision, skip this planning."），规划器卡死的bug。

>+ **修复**在状态机中使用 **planNextWaypoint()** 导致状态机卡死的bug。

>+ **新增**规划优化异常检测， 避免动力学不可行的轨迹发出，新增了动力学容忍值（[advanced_param.xml](src/diff_planner/plan_manage/launch/include/advanced_param.xml)中设置）：\
\<param name="optimization/vel_tolerance" value="1.0" type="double"/>  \
\<param name="optimization/acc_tolerance" value="1.0" type="double"/>

>+ **修复**遇到大障碍物后，无人机在大障碍物面前反复徘徊卡死的bug，同时增加是否使用大障碍物检测的开关（[advanced_param.xml](src/diff_planner/plan_manage/launch/include/advanced_param.xml)中设置）：\
>\<param name="fsm/enable_stuck_detect" value="true"/> 

>+ **新增**优化失败次数过多的处理。

>+ **新增**激光雷达建图 **raycast** 版本，使激光雷达建图更加稳定。

>+ **新增**用户接口 **user_command** 功能包，使用户能以多种方式设置途径点以及设置返程点。

>+ **traj_server** 节点**新增**yaw角控制接口，用户可根据需要在规划过程中控制无人机yaw角。

- **本项目会长期维护并根据用户反馈持续优化。**

## 运行环境
本项目基于ROS1开发，请根据所使用ubuntu版本安装对应版本ROS1，支持ubuntu16.04, 18.04和20.04。


## 仿真运行步骤

### 1. 下载源码并编译:
```
git clone https://github.com/DifferentialRobotics/Diff-Planner.git
cd Diff-Planner
catkin_make
```

### 2. 单机rviz手动指点飞行：
```
cd Diff-Planner
source devel/setup.zsh # 如果使用bash终端，则执行: source devel/setup.bash
roslaunch diff_planner run_sim_single.launch
```
使用rviz中的**3D Nav Goal**插件，在地图上按住左键选择目标点x-y平面位置，按住左键不松手同时按住右键上下拖动调整目标点z轴位置，之后松开鼠标即发送目标点，无人机开始规划。
<p align="center">
  <img src="images/rviz_test.gif" alt="rviz_tes" width="600" />
</p>

### 2.1 RViz连续点选航线

仿真使用 `roslaunch diff_planner run_sim_single.launch mission_source:=clicked`，
实机 LIO 入口现默认 `mission_source=clicked`。启动 `multipointplan` 后，可以用 RViz 的 **Publish Point** 工具在建好的地图上
连续点击多个点。节点会按点击顺序把这些点作为航点，逐点发布到 `/goal`，由
Diff-Planner 对相邻航点之间的路段进行避障规划；点位会显示为带编号的橙色标记，
默认飞行高度为 1.0 m。

点击完成后，用 RViz 的 **2D Nav Goal** 工具再点一次作为“开始执行”触发，或者执行：

```bash
rostopic pub -1 /mission/start_clicked_route std_msgs/Empty '{}'
```

清空当前点选航线后才能重新标点：

```bash
rostopic pub -1 /mission/clear_clicked_route std_msgs/Empty '{}'
```

点选航线不会自动解锁或起飞；应先按原有流程起飞并确认定位、地图和遥控器接管正常。
实机不使用 A8 mini 时，启动航点节点应同时关闭云台和识别：

```bash
A8MINI_START_GIMBAL_NODE=false \
A8MINI_START_DETECTION=false \
./sh_files/run_single_lio.sh
```

如果只单独启动 `multipointplan_exp_lio.launch`，则直接传入
`start_gimbal_node:=false start_detection:=false`。


### 3. 单机预设点飞行：
在 **[points.yaml](src/user_command/multipoint/config/points.yaml)** 的 `waypoints`
列表中配置航点坐标、悬停时间和云台动作；`gimbal_mode: angle` 转到指定
角度，`gimbal_mode: range` 按 `gimbal_yaw_min_deg` 和
`gimbal_yaw_max_deg` 扫描指定水平范围。另有
`coverage_5x5.yaml` 和 `coverage_20x20.yaml` 两份 2.5 m 间距蛇形覆盖任务；
可选的 `test_back` 用于配置返程点。之后通过以下指令执行任务：
```
cd Diff-Planner
source devel/setup.zsh
roslaunch diff_planner run_sim_single.launch
cd Diff-Planner #新建终端
./sh_files/pub_trigger.sh #开始执行任务 或在rviz中用2D Nav Goal插件在地图任意位置点击也能开始执行任务
./sh_files/back.sh #开始返程规划
```
注：仿真启动文件使用 dry-run 云台后端，不会向 A8 mini 发送网络命令。
实机下与达妙载板网口的连接和调试方法见
**[A8 mini 航点连续录像任务](A8mini/README.md)**。

### 4. 集群预设点飞行：
在 **[run_sim_swarm.launch](src/diff_planner/plan_manage/launch/sim/run_sim_swarm.launch)** 中设置每架无人机的目标点 **target0_x/y/z**，之后通过以下指令执行任务：
```
cd Diff-Planner
source devel/setup.zsh
roslaunch diff_planner run_sim_swarm.launch
cd Diff-Planner #新建终端
./sh_files/pub_swarm_trigger.sh #开始执行任务
```


## 实机运行教程

本次修复后的逐步流程见 **[A8mini 实机操作指南](docs/A8mini实机操作指南.md)**，
日志依据、测试结果和尚未闭环的审查项见 [20260916 日志修复说明](docs/2026-09-17_修复-A8mini-report.md)。
常用顺序是 **RC8 DOWN→MIDDLE 请求起飞 → 确认悬停 → Publish Point 标点 →
2D Nav Goal 或 RC8 UP 开始执行**。实机默认点击模式不再回退到 YAML；
需要 YAML 任务时启动前设置 `MISSION_SOURCE=preset`。

### A8 mini 连续录像航点任务

A8 mini 通过达妙载板网口接入机载电脑时，可使用新增的航点—悬停—云台握手流程。配置、网络参数和测试方法见 [A8mini/README.md](A8mini/README.md)。

### 0.深度相机内参替换
若要使用**视觉定位**下规划，需要先在 **[run_exp_single_vio.launch](src/diff_planner/plan_manage/launch/exp/run_exp_single_vio.launch)** 中替换深度相机内参 **cx/cy/fx/fy**，内参查看方式：
```
cd Diff-Planner
./sh_files/run_vins.sh
rostopic echo /camera/depth/camera_info
```
消息中的K矩阵即为深度相机内参，注意矩阵中的顺序为 **fx/cx/fy/cy**。

### 1. 雷达定位下规划：
```
cd Diff-Planner
./sh_files/run_single_lio.sh #请先按照配套的产品手册教程配置途径点位
```

如果希望开机后只执行一个文件完成“启动飞行栈并自动起飞”，可直接运行：

```bash
cd /home/q/Documents/diff-planner-A8miniV2
./sh_files/one_click_takeoff.sh
```

该入口会自动加载 ROS 和当前工作空间，不需要另外执行 `source`。它会等待
MAVROS、LIO/EKF、RC、里程计和飞控地面状态满足条件后才发送一次起飞指令；
脚本随后保持运行，按 `Ctrl+C` 可正常停止节点并保存调试日志。只启动系统而不
发送起飞指令时使用：

```bash
./sh_files/one_click_takeoff.sh --start-only
```

起飞前仍需确认桨叶、场地、电池和遥控器安全；遥控器摇杆应居中，模式/指令开关
应打开，RC 8 通道应处于 DOWN。该脚本只负责起飞，不会自动开始航点任务；
确认 OFFBOARD、IN_AIR 和 `/px4ctrl/mission_ready=True` 后，先标点再将 RC8 拨到 UP，
或手动发布任务触发消息。落地上锁后再 Ctrl+C 退出；Ctrl+C 不负责降落。

### 2. 视觉定位下规划：
```
cd Diff-Planner
./sh_files/run_single_vio.sh #请先按照配套的产品手册教程配置途径点位
```

起飞、规划、返航、降落方式详见与实机配套的**产品手册**。

## 致谢与声明
本项目在开发过程中参考并使用了 **[EGO-Planner-v2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)**，特此感谢浙江大学 **FAST-Lab** 团队的开源贡献。

相关代码均严格遵循原项目的开源许可协议使用，用户在使用本项目时，请务必遵守相应的许可证条款。

# Q&A
请随时提交问题或讨论，我们会在看到问题后尽快回复。
