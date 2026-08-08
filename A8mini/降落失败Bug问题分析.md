# A8 mini 降落失败 Bug/问题分析

## 1. 问题摘要

本次飞行中，第 8 通道已正常触发 LAND，但无人机没有进入自动降落。
`px4ctrl` 持续报错：

```text
Reject AUTO_LAND, which must be triggered in AUTO_HOVER.
Stop sending control commands for longer than 0.500000s...
```

根本原因是：任务节点只停止了自己的航点状态机，没有停止已经在
运行的轨迹服务器。`traj_server` 仍然以 100 Hz 向 `px4ctrl` 发布
`PositionCommand`，导致控制器一直处于 `CMD_CTRL`，无法满足自动降落必须
先进入 `AUTO_HOVER` 的条件。

后续拨动第 6 通道是操作员主动切换手动降落，不是误操作。日志显示
手动降落最终成功，但旧逻辑仍在每秒重发 LAND，造成自动降落与手动
接管反复争抢控制状态。

## 2. 分析对象

- 日志目录：`flight_logs/20260808_163825/`
- 飞行时间：2026-08-08 16:38:26 ~ 16:42:10
- 日志对应代码版本：`866fefea37244ce57db17873e19ae9e460fc2c79`
- ROS：Noetic
- 里程计：`/ekf/ekf_odom`
- 位置指令：`/setpoints_cmd`
- 起降指令：`/px4ctrl/takeoff_land`

## 3. 飞行时间线

| 时间 | 关键事件 | 结果 |
|---|---|---|
| 16:39:23 | 第 8 通道 DOWN → MIDDLE | 起飞指令正常发布 |
| 16:39:50 | 第 8 通道 MIDDLE → UP | 开始航点任务 |
| 16:40:30 | 航点 2 云台任务完成 | 主任务完成 |
| 16:40:32 | 第 8 通道 UP → MIDDLE | 开始执行返程点 `(0, 0, 1)` |
| 16:40:47 | 第 8 通道进入 DOWN | LAND 已发布，但被 `px4ctrl` 拒绝 |
| 16:40:47 ~ 16:41:37 | 任务节点每秒重发 LAND | 每次都因仍处于 `CMD_CTRL` 被拒绝 |
| 16:41:38 | 第 6 通道从约 1933 切换为约 1065 | 操作员主动进入 RC 悬停/手动降落 |
| 16:41:42 | 油门开始明显下压 | 高度开始持续下降 |
| 16:41:57.061 | `/mavros/state` 的 `armed` 变为 `false` | 飞控已上锁（disarm）、电机停转 |
| 16:41:57.214 | `/mavros/extended_state` 变为 `ON_GROUND` | 手动降落完成 |

## 4. 日志证据

### 4.1 LAND 命令已正常发布

`/px4ctrl/takeoff_land` 记录了 1 条 TAKEOFF 和 80 条 LAND。首条 LAND 时间为
16:40:47.498，说明第 8 通道识别和 LAND 发布链路正常。

### 4.2 LAND 后位置指令没有停止

rosbag 中 `/setpoints_cmd` 共有 13,858 条消息。LAND 发布后仍有约
8,250 条位置指令，持续到 rosbag 结束。指令一直保持返程终点：

```text
position: [0, 0, 1]
velocity: [0, 0, 0]
publish rate: 100 Hz
```

因为 `px4ctrl` 的 `msg_timeout/cmd` 为 0.5 秒，而新指令每 0.01 秒就会
到达一次，所以 `cmd_is_received()` 始终为真，控制器不可能从
`CMD_CTRL` 自动退回 `AUTO_HOVER`。

### 4.3 `px4ctrl` 状态机的拒绝符合设计

`px4ctrl` 只允许从 `AUTO_HOVER` 进入 `AUTO_LAND`。当它仍处于
`CMD_CTRL` 时收到 LAND，会输出：

```text
[px4ctrl] Reject AUTO_LAND, which must be triggered in AUTO_HOVER.
Stop sending control commands for longer than 0.500000s...
```

因此错误不在 LAND 消息格式，也不在降落速度参数，而在上游位置
指令没有停止。

### 4.4 第 6 通道是手动接管，且手动降落成功

第 6 通道切换后，`px4ctrl` 从 `CMD_CTRL` 进入 `AUTO_HOVER`，允许遥控
杆修改悬停目标。油门值从约 1436 逐步降至 1330、1110 和 1065，里程计
高度从约 1.0 m 下降至约 0.1 m。最终飞控上锁（disarm）并报告已落地。

但旧代码中 `landing_latched_` 仍然每秒发布 LAND，会导致以下循环：

```text
AUTO_HOVER -> AUTO_LAND -> AUTO_HOVER
```

进入 `AUTO_LAND` 后，`px4ctrl` 发现第 6 通道已离开 command-mode，因此立即
返回 RC 悬停。这没有阻止本次手动降落，但属于不必要且不清晰的状态
争抢。

## 5. 根因分析

### 根因一：任务取消没有传递到轨迹输出层

降落时，`multipointplan` 执行了：

- 将自身航点状态设为 `FINISHED`；
- 关闭任务 CSV；
- 停止发布新航点；
- 发布并重试 LAND。

但是，已发布的返程目标已在 `diff_planner` 和 `traj_server` 中形成轨迹。
`traj_server` 是独立节点，不知道 `multipointplan` 的内部状态已取消，所以
仍按设计保持最后位置并持续发布指令。

### 根因二：LAND 重试机制只重试结果，没有解除前置条件

旧修复通过每秒重发 LAND，希望 `px4ctrl` 进入 `AUTO_HOVER` 后接受下一条
LAND。但只要 100 Hz 位置指令不停，`AUTO_HOVER` 的前置条件就永远无法
成立，因此无限重试也无法解决问题。

### 根因三：自动降落和手动降落没有明确仲裁

第 8 通道已锁存自动 LAND，但第 6 通道离开 command-mode 表示操作员
需要 RC 手动接管。旧代码没有用第 6 通道状态停止 LAND 重试，两条路径
因此同时活跃。

## 6. 修复方案

### 6.1 增加明确的轨迹停止握手

`multipointplan` 在任何降落路径中都会发布锁存的 `/planning/stop`。
`traj_server` 收到后会：

- 立即停止发布 `PositionCommand`；
- 将 `receive_traj_` 置为 `false`；
- 清除当前 yaw 控制状态；
- 拒绝降落期间新到达的轨迹；
- 直到收到 TAKEOFF 后才解锁，且解锁后必须等待新轨迹。

`traj_server` 也直接监听 LAND，保证通过其他脚本或节点发送 LAND 时同样
会停止轨迹指令。

### 6.2 分离自动降落与手动降落

- 第 6 通道仍在 command-mode：执行自动降落，LAND 保持锁存和重试。
- 第 6 通道已离开 command-mode：仅停止规划指令，不再发布自动 LAND，
  由 `px4ctrl` 进入 RC 悬停/手动降落。
- 如果自动降落过程中第 6 通道被切出 command-mode，当前飞行周期内会
  永久停止 LAND 重试，避免重新切换开关后意外恢复自动降落。

### 6.3 改进诊断信息

- `px4ctrl` 日志明确说明第 6 通道使自动降落转为 RC 悬停/手动降落。
- 调试 rosbag 新增 `/planning/stop`。
- `key_events.log` 新增第 6 通道、轨迹停止和 `traj_server` 关键日志。

## 7. 修复后状态流程

### 7.1 自动降落

```text
第 8 通道进入 DOWN
        |
        +--> 取消航点任务
        +--> 锁存 /planning/stop
        +--> traj_server 停止 100 Hz PositionCommand
        +--> 发布 LAND
                         |
                         +--> CMD_CTRL 等待 cmd 超时 0.5 s
                         +--> AUTO_HOVER
                         +--> 下一次 LAND 重试
                         +--> AUTO_LAND
                         +--> 落地、自动上锁（disarm）
```

由于 ROS 回调时序存在小幅差异，修复后首条 LAND 仍有可能在位置指令
超时前被拒绝一次，这属于正常过渡。默认 1 秒后的重试应该被接受。

### 7.2 手动降落

```text
第 6 通道离开 command-mode
        |
第 8 通道进入 DOWN
        |
        +--> 取消航点任务
        +--> 锁存 /planning/stop
        +--> traj_server 停止 PositionCommand
        +--> 禁用自动 LAND 重试
        +--> RC 悬停/手动降落
        +--> 操作员使用油门下降并按正常手动流程停桨
```

## 8. 验证结果

### 8.1 编译检查

以下软件包均已编译通过：

```text
multipoint
diff_planner
px4ctrl
```

### 8.2 无硬件 ROS 消息回归

| 测试项 | 结果 |
|---|---|
| 合成轨迹启动后是否发布位置指令 | 通过，正常输出 |
| 收到 `/planning/stop` 后是否停止输出 | 通过，连续 2 秒无消息 |
| 收到 LAND 后是否停止输出 | 通过，连续 2 秒无消息 |
| TAKEOFF 是否会错误恢复旧轨迹 | 通过，旧轨迹没有恢复 |
| TAKEOFF 后发布新轨迹是否恢复输出 | 通过 |
| 自动降落是否锁存和重试 LAND | 通过 |
| 第 6 通道手动接管后是否停止 LAND 重试 | 通过，连续 2 秒无 LAND |

## 9. 实飞复测要求

当前已完成编译和 ROS 消息级回归，仍需要进行一次安全的低高度实飞。
建议使用：

```bash
./sh_files/run_single_lio_debug.sh
```

自动降落复测时：

1. 起飞高度控制在约 1 m，保持开阔、无人环境。
2. 保持第 6 通道在 command-mode。
3. 第 8 通道进入 DOWN。
4. 确认 `/setpoints_cmd` 在 LAND 后立即停止。
5. 允许首条 LAND 在 `CMD_CTRL` 中被拒绝一次，但随后应在约
   0.5 ~ 1.5 秒内出现：

```text
[traj_server] planning stop received...
[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!
[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND
```

6. 确认高度持续下降，落地后 `armed=false`、`landed_state=ON_GROUND`。

手动降落复测时：

1. 将第 6 通道拨出 command-mode。
2. 将第 8 通道拨到 DOWN，取消规划轨迹。
3. 确认日志出现 `automatic LAND retry stopped`。
4. 确认不再出现每秒 LAND 重发。
5. 使用遥控杆下降，落地后按正常手动流程停桨。

## 10. 结论

本次问题不是 LAND 指令丢失，也不是第 6 通道误操作。它是一个跨节点
状态协调 Bug：任务层认为任务已取消，但轨迹输出层仍然持续发送位置
指令，使 `px4ctrl` 无法满足自动降落前置条件。

修复后，任务取消会明确传递到轨迹服务器，并对自动降落和第 6 通道
手动接管进行互斥处理。编译和消息级回归已通过，剩余工作为低高度
实飞验收。
