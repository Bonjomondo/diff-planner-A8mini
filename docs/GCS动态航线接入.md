# 地面站动态航线接入

`mission_source=remote` 接收地面站的完整 `world` 航线，经人工起飞、悬停确认后逐点交给 Diff-Planner，支持执行中替换未完成航点，并回传当前航点、悬停阶段和完成数。

本机已验证独立 C++ 航线逻辑、Python TCP 通信和 ROS 适配器单元测试；相关任务、规划和轨迹
节点已完成编译，SWB 复位及二次起飞已通过模拟飞控/遥控器话题的 ROS 回归。Qt 页面联调
和实机验收尚未完成。ACK 表示任务执行器接受命令，不代表避障规划已经成功。

## 与前端的对应关系

对接 [agri_gcs_visual 的 feat/indoor-local-mission 分支](https://github.com/gyyhh274/agri_gcs_visual/tree/feat/indoor-local-mission)，协议依据为 [GroundLink.cpp](https://github.com/gyyhh274/agri_gcs_visual/blob/feat/indoor-local-mission/src/GroundLink.cpp) 与 [INDOOR_MISSION.md](https://github.com/gyyhh274/agri_gcs_visual/blob/feat/indoor-local-mission/docs/INDOOR_MISSION.md)。

| 地面站操作 | 机载行为 |
| --- | --- |
| 地图标点、核对本次定位坐标 | 前端编辑 `world` 米制航点，Z 为定位坐标高度 |
| 上传航线 `upload` | 原子校验并缓存完整航线，返回 `READY`；不解锁、不起飞、不发布 `/goal` |
| 人工确认起飞 | 沿用 RC8 DOWN → MIDDLE 和 px4ctrl 流程 |
| 开始任务 `start` | 核对会话、任务 ID、版本、定位、飞控和规划器状态，发布第一个 `/goal` |
| 动态更新 `update` | 保留指定索引之前的航点，原子替换后缀；可更新当前飞行目标或后续航点 |
| 地面站查看进度 | 每 0.5 s 收到遥测，包含任务状态、当前航点、完成数和 `world` 位姿 |

前端当前页面已经实现上传、开始和查询；尚未实现执行中编辑/更新按钮。本次修改后端，`update` 为兼容 protocol 2 的扩展，可通过下文示例或 ROS 服务调用，后续前端可按相同协议增加入口。`update` 成功后机载返回新版本号，前端必须以此为准。

## 编译与启动

建议在 Ubuntu 20.04 / ROS Noetic 的现有工作空间运行。新增依赖为 `message_generation`、`message_runtime`、`sensor_msgs`，已写入包声明。

```bash
catkin_make
source devel/setup.bash
```

### 仿真

```bash
roslaunch diff_planner run_sim_single.launch mission_source:=remote
```

该入口自动启动任务执行器和 TCP 网关。仿真初始位置约为 `(-15, 0, 1)`，先在附近选择少量航点。仿真沿用原启动文件的 `require_flight_ready=false`，需要有效里程计和规划器心跳即可开始；实机入口始终开启飞行状态检查。

### 实机 LIO

在已经编译的工作空间中，通过原完整飞行栈入口启动：

```bash
MISSION_SOURCE=remote \
A8MINI_START_GIMBAL_NODE=false \
A8MINI_START_DETECTION=false \
./sh_files/one_click_takeoff.sh
```

默认等待 RC8 DOWN → MIDDLE 请求起飞。上传航线后，确认无人机悬停、定位正常，等待地面站显示任务就绪，再点击“开始任务”。RC8 UP 和原 `/move_base_simple/goal` 触发也可开始已上传的航线，执行相同检查。结束任务后继续悬停；降落仍按原遥控器流程操作。

若 LIO、Diff-Planner、MAVROS 和 px4ctrl 已经单独启动，只启动任务模块：

```bash
roslaunch multipoint multipointplan_exp_lio.launch \
  mission_source:=remote start_gimbal_node:=false start_detection:=false
```

两种启动方式选其一，避免重复启动任务执行器和网关。VIO 入口 `multipointplan_exp_vio.launch` 同样接受 `mission_source:=remote`。远程航线不包含云台动作，也不依赖预设 YAML 航线；需要相机录像/识别时仍可按原配置启动对应节点。

### 连接地面站

网关只监听 loopback，默认 `127.0.0.1:8765`。在地面站电脑设置 `ONBOARD_SSH` 为已经能够登录的机载 SSH 地址，再执行：

```bash
ssh -N -L 8765:127.0.0.1:8765 "${ONBOARD_SSH:?请先设置机载 SSH 登录地址}"
```

地面站连接设置填写 `127.0.0.1:8765`。同机仿真无需隧道。旧网关占用 8765 时应先停止旧实例，或通过任务 launch 的 `gcs_port` 参数选择其他端口，并同步修改隧道和地面站设置。连接断开后重连、查询状态，不自动重新开始任务。

## 降落后用 SWB 复位并重新上传航点

**每次 SWB 复位成功后，必须重新上传本轮完整航点；即使继续飞相同路线，也要重新上传。**
复位会清除机载航线、旧目标、旧轨迹和任务进度，远程任务回到 `IDLE`，同时生成新的
`map_session_id`。地面站本地保留的航点图形不等于机载端已接受本轮航线。

先将遥控器辅助通道设为 **九通：SwB**，确认接收机实际向 MAVROS 输出有效 CH9。
以下顺序适用于落地后的下一轮，也适用于起飞被拒绝、拨回下档进入降落锁定后的恢复：

1. **确认可以复位。** 飞机已落地、未解锁（`armed: False`），SwG/CH8 保持 DOWN，
   飞控连接、落地状态和遥控器数据正常。正在降落或电机仍解锁时不能复位。
2. **完成 SWB 手势。** 从当前端档拨到另一端，保持 2 秒，随后及时拨回原端档。
   上→下→上和下→上→下均支持；三档开关需要到另一端，中间档不算保持。
3. **确认本次复位成功。** 日志出现 `RC CH9: mission reset succeeded`；查询机载任务
   应为 `IDLE`、`total=0`、`revision=0`、任务 ID 为空，并取得新的 `map_session_id`。
   若仍是 `LANDING`/`FAILED`，先处理拒绝原因，不能把拨过开关当作成功。
4. **重新核对原点和完整航点。** 检查当前地图中的飞机位置、`world` 坐标原点、航点
   顺序、XY 坐标和 Z 高度。会话变化不代表定位原点一定发生移动，但仍需人工核对。
   可以复用核对后的相同坐标，不能复用上一轮“机载已上传”的结论。
5. **重新执行完整上传。** 使用新会话发起新的 `upload`，建议生成新的任务编号。
   新请求必须使用新的 `request_id`，`revision=1`，`frame_id=world`，`waypoints`
   包含本轮全部航点。复位后不要用 `start` 或 `update` 代替完整上传。
6. **确认本轮上传被机载接受。** 收到本轮上传的 `accepted=true` 回执，机载状态为
   `READY`；核对 `map_session_id`、任务编号和版本属于本轮，`total` 与本轮航点数一致。
   地面未起飞时 `flight_ready=false` 可以是正常现象，不能因此把已接受的上传当成失败。
7. **单独请求起飞。** 满足 CH5/CH6 与摇杆回中条件后，将 SwG 从 DOWN 拨到 MIDDLE，
   停在 MIDDLE，等待自动起飞和悬停完成。
8. **悬停确认后开始。** 确认定位正常、任务就绪，再点击“开始任务”或将 SwG 从
   MIDDLE 拨到 UP。开始时必须使用本轮已接受的会话、任务编号和准确版本。

建议在地面完成第 1～6 步后再起飞。起飞请求与航线上传相互独立：**能请求起飞不代表
航点已上传；未重新上传时不能开始旧航线。** SWB 只做复位，不会解锁、起飞、上传航点
或自动开始任务；SwG 中档起飞也不会替你恢复上一轮航点。

| 本轮阶段 | 机载状态与航点 | 操作确认 |
| --- | --- | --- |
| 降落锁定 | `LANDING`，不接受新任务操作 | 落地、上锁后完成 SWB 手势 |
| 复位完成 | `IDLE`，`total=0`，新会话 | 重新核对原点并完整上传 |
| 新航线上传成功 | `READY`，航点数为本轮上传数量 | 核对本轮回执后请求起飞 |
| 起飞并悬停 | 航线仍为 `READY`，飞行条件满足后任务就绪 | 单独发出开始任务指令 |
| 本轮开始执行 | `FLYING`/`HOVERING` | 按本轮航点推进 |

注意两种容易误判的情况：

- **先上传、后复位：** 成功复位会清空刚上传的航点，因此必须在最后一次成功复位之后再上传。
- **重发旧上传请求：** 执行器保留请求去重缓存。旧 `request_id` 的重试可能返回旧操作的
  `accepted=true`，同时附带当前 `IDLE`、`total=0` 的状态；它不会重新写入航点。
  本轮应使用新的请求 ID 发起上传，并以当前任务状态和航点数量确认成功。

终端备用方式是 `./sh_files/reset_mission.sh`，仅在返回 `success: True` 时表示复位成功。
它同样会清除航点，之后仍必须执行上述重新上传步骤。仅 TCP 重连或重复按开始不会复位。
两次飞行之间只要又成功复位一次，就要再次按新会话重新上传。

## 消息与 ROS 接口

TCP 使用 UTF-8 JSON，每行一帧，单帧最多 65536 字节（不含换行），单任务最多 200 航点。网关支持分包、连续多帧、最多 4 个并发客户端；非法、超大或慢客户端不会停止任务执行器。

| 字段 | 约定 |
| --- | --- |
| `type` / `protocol` | 命令为 `command` / `2` |
| `request_id` | 每次新操作生成唯一 ID；同一请求重试必须保留原 ID 和内容 |
| `operation` | `upload`、`start`、`get_status`、`update` |
| `map_session_id` | 从当前机载遥测取得，变更后重新核对坐标并上传 |
| `mission_id` | 上传时生成新的 ID；开始、更新使用机载已接受的 ID |
| `revision` | 上传为 1；开始必须等于当前版本；更新必须等于当前版本 + 1 |
| `frame_id` | 上传、更新必须为 `world` |
| `waypoints` | 上传为完整数组；更新为要替换的后缀数组 |
| `replace_from` | 仅更新使用，**从 0 开始的数组索引，不是航点 ID** |

每个航点必须包含 `id`、`x`、`y`、`z`、`hover_sec`。ID 是唯一的正 UInt32；坐标和时间必须为有限数值，拒绝字符串数字、布尔值、NaN、Infinity 和重复 JSON 键。悬停时间范围为 0–60 s。有效 `world` 定位是上传前提，但地面未起飞时可以上传。

ROS 侧接口：

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/mission/remote/command` | `multipoint/MissionCommand` 服务 | 接受命令，返回 `accepted`、`reason`、完整 `mission` 快照 |
| `/mission/remote/status` | `multipoint/MissionStatus`，5 Hz、latched | 权威任务状态，由 TCP 网关转发 |
| `/goal` | `geometry_msgs/PoseStamped` | 当前目标点，Diff-Planner 按原入口规划/重新规划 |
| `/planning/stop` | `std_msgs/Empty` | 沿用原有定位/飞控/规划器异常或降落停止逻辑 |
| `/mission/reset` | `std_srvs/Trigger` 服务 | 落地、未解锁、RC8 下档且状态新鲜时显式复位；清除旧航线并生成新的地图会话 |

降落锁定后，确认落地、未解锁、CH8/SwG 下档且状态正常，用 CH9/SwB 从原端档
拨到另一端保持 2 秒，再及时拨回原端档，可完成同一复位；短拨及空中操作无效。
遥控器需设置“九通：SwB”，接收机需实际向 MAVROS 输出第九通道。
也可在机载端运行 `./sh_files/reset_mission.sh`，确认响应 `success: True`。
随后必须查询新 `map_session_id`、重新核对原点并**重新上传完整航点**；相同路线也要再上传。
本轮上传被接受、任务回到 `READY` 后，再按原流程人工起飞、悬停确认、开始任务。
完整操作与回执检查见[降落后用 SWB 复位并重新上传航点](#降落后用-swb-复位并重新上传航点)。
复位不会触发起飞，TCP 重连、重复 `start` 和 RC8 中档都不能代替复位。
遥控器手势和机载 ROS 服务复用相同复位检查，现有 protocol 2 的四种操作保持不变。

TCP ACK 为 `type=ack`、`protocol=2`，包含原 `request_id`、布尔 `accepted`、`reason` 和 `mission`。ACK 来自 ROS 执行器的服务响应，不在网关提前确认。执行器缓存最近 256 个请求结果；重试返回原接受结果和**当前**任务状态，相同 ID 携带不同有效内容会被拒绝。当前定位会话内已经使用的任务 ID 也不能再次上传，避免旧航线重放。

遥测固定为 `type=telemetry`、`protocol=2`、`server=agri-onboard-ros`，顶层包含 `fcu_connected`、`armed`、`mode`、`battery_percent`、`mission_ready`、`odom_fresh`、`world_x/y/z`。电池未知为 -1，已知时为 0–100。

嵌套 `mission` 包含 `frame_id`、`map_session_id`、`mission_id`、`revision`、`state`、`current_waypoint_id`、`completed`、`total`、`reason`、`odom_fresh`、`flight_ready`、`world_x/y/z`、`max_abs_xy`、`min_z`、`max_z`，与当前 GroundLink 的必需字段一致。

状态为 `IDLE → READY → FLYING ↔ HOVERING → COMPLETED`；失败为 `FAILED`，降落为 `LANDING`。`completed` 仅在稳定到达且悬停动作完成后增加。READY 和终态的 `current_waypoint_id=0`；完成任务后 `completed=total`。完成后需上传新任务才能再次执行。

## 动态更新后续航点

`update` 替换 `[replace_from, 原航点数组末尾)`，前缀保持不变。服务回调与任务推进在同一个 ROS 回调线程中执行，校验失败不会修改航线、版本或进度。

| 当前状态 | 允许更新范围 | 对规划器的影响 |
| --- | --- | --- |
| READY | 从任意有效索引替换 | 仅更新缓存，开始时使用新版本 |
| FLYING | `replace_from >= completed` | 等于 completed 时立即发送新 `/goal`；大于时到后续航点再发送 |
| HOVERING | `replace_from > completed` | 当前悬停动作保持，后续目标按新航线执行 |
| COMPLETED / FAILED / LANDING | 拒绝 | 不修改已有任务 |

索引等于 `total` 表示追加；空后缀可删除未来航点，但不能删除当前执行点，也不能产生空任务。新后缀和保留前缀的 ID 必须整体唯一。若更新命令到达时任务已推进，机载会按最新进度再次检查并拒绝过时更新。

以下示例通过现有连接查询任务，再替换当前目标之后的航点。**示例会实际修改任务，请先在仿真验证，并按地图调整 `new_points`。** 它不发送起飞或开始命令。

```python
import json
import socket
import uuid

with socket.create_connection(("127.0.0.1", 8765), timeout=5) as sock:
    stream = sock.makefile("rb")

    def command(operation, **fields):
        request_id = str(uuid.uuid4())
        payload = dict(type="command", protocol=2, request_id=request_id,
                       operation=operation, map_session_id="", mission_id="", revision=0)
        payload.update(fields)
        sock.sendall(json.dumps(payload, allow_nan=False).encode("utf-8") + b"\n")
        while True:
            line = stream.readline()
            if not line:
                raise ConnectionError("机载链路断开，请重连查询结果")
            message = json.loads(line)
            if message.get("type") == "ack" and message.get("request_id") == request_id:
                return message

    current = command("get_status")["mission"]
    if current["state"] not in ("FLYING", "HOVERING"):
        raise RuntimeError("先在仿真地面站上传并开始航线")
    new_points = [dict(id=101, x=-11.0, y=1.0, z=1.0, hover_sec=2.0),
                  dict(id=102, x=-9.0, y=1.0, z=1.0, hover_sec=2.0)]
    result = command("update", map_session_id=current["map_session_id"],
                     mission_id=current["mission_id"], revision=current["revision"] + 1,
                     frame_id="world", replace_from=current["completed"] + 1,
                     waypoints=new_points)
    print(json.dumps(result, ensure_ascii=False, indent=2))
```

若需要立即改变正在飞向的目标，必须在 `FLYING` 时使用 `replace_from=current["completed"]`；原当前目标被后缀中的第一个点取代，Diff-Planner 从当前运动状态重新规划。仅修改未来航点时，不中断当前路段。

## 参数与异常处理

| 任务 launch 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `gcs_bind_host` | `127.0.0.1` | 必须为 loopback，通过 SSH 隧道连接 |
| `gcs_port` | `8765` | TCP 监听端口 |
| `remote_max_abs_xy` | `30.0` | X/Y 绝对值上限，单位 m |
| `remote_min_z` / `remote_max_z` | `0.3` / `2.5` | 接受的 world.z 范围，单位 m |
| `remote_reset_jump_m` | `5.0` | 相邻里程计位置跳变阈值，单位 m |

坐标限制是任务接口的范围检查，不是障碍物检查；应与实际定位范围、地面高度和规划器虚拟围栏一致。规划器仍负责障碍物和动力学可行性，目标拒收/不可达会由原航点超时机制停止任务。

开始任务要求新鲜 `world` 里程计、规划器心跳与 `/goal` 订阅者，以及 FCU connected、armed、OFFBOARD、IN_AIR、控制器 AUTO_HOVER/CMD_CTRL 和 mission_ready。未满足时拒绝开始并保留 READY 航线，条件恢复后使用**新的 request_id** 重试。相同 request_id 的拒绝结果也会被缓存。

每次任务节点重启生成新定位会话；定位失效/超时、位置大幅跳变或时间戳回退超过 0.5 s 也会更新会话。未开始的航线被清空；执行中的任务停止并进入 FAILED，需降落、未解锁、RC8 下档后调用 `/mission/reset`，重新核对原点并上传。小幅/快速的定位原点重置未必能被检测，重启定位后仍应人工核对坐标并重启完整定位、规划和控制链路。

仅 Wi-Fi/TCP 断开时，已经开始的任务在定位和控制正常的情况下继续执行。此版没有可恢复暂停机制，不自动把断网解释为停机或降落。网关超过 1 s 未收到执行器状态时，关闭遥测中的就绪/定位有效标志并拒绝新控制命令；ROS 调用丢失回复会提示结果未知，应查询最新状态。

远程模式不接收 RViz 点击航线，也不使用 YAML 的返程航线。避免其他节点同时向 `/goal` 发布目标。降落、人工接管、原有异常停止仍由原飞行控制流程处理；本协议不提供解锁、起飞、暂停、取消或降落命令。

## 验证

无需 ROS 的检查，在仓库根目录运行：

```bash
g++ -std=c++11 -Wall -Wextra -Werror -pedantic \
  src/user_command/multipoint/test/test_remote_mission.cpp -o /tmp/test_remote_mission_gcs
/tmp/test_remote_mission_gcs
python3 -m unittest discover -s src/user_command/multipoint/test -p 'test_gcs*.py' -v
```

覆盖航线原子替换、版本/会话限制、已完成前缀保护、坐标范围、当前飞行目标/悬停保护、TCP 分帧、超大帧、多个客户端、ACK 关联、ROS 请求映射、过期状态和电池未知值。

在 ROS Noetic 环境运行真实执行器与网关的集成测试：

```bash
catkin_make
source devel/setup.bash
rostest multipoint remote_mission.test
catkin_make run_tests_multipoint
catkin_test_results build/test_results/multipoint
```

集成测试只发布模拟飞控、定位和规划器心跳，没有真实飞控/电机/相机/规划器，覆盖：上传不出目标/起飞指令、地面开始拒绝、空中开始、重连重试去重、后续替换、当前目标重新发布、进度完成、定位丢失停止。最后还需使用真实 Diff-Planner 仿真验证新目标产生轨迹，再在 Qt 页面验证上传/确认/开始/进度的完整流程。
