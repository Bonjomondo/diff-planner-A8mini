# 地面站自动启动建图

机载代码仅位于 `/home/ly/Documents/diff-planner-A8mini-gcs-mission`（`zjc-argc`）。不操作旧目录。

地面站打开后，经已有 SSH 密钥建立 `37262` 点云和 `37263` 启动接口的本机转发。默认自动发送 `POST /ensure`，随后查询 `GET /status`，完成后持续显示真实点云。自动启动可以在地面站“实时建图 → 连接”中关闭，关闭后可用“启动建图”按钮手动请求。

机载 `agri-mapping-start.service` 开机运行 API，**不会开机直接启动雷达**。API 只监听 `127.0.0.1:37263`，只允许固定的传感器建图脚本。飞行任务网关及只读地图桥不变。请求不能传入 shell、路径、ROS 节点名、解锁或起飞操作。

```json
{"protocol":1,"operation":"ensure_mapping"}
```

启动范围：roscore、MAVROS、MID-360 和 Faster-LIO；MAVROS 请求 HIGHRES_IMU 200 Hz，不发送解锁、模式切换或起飞命令。脚本仅加载新工作目录的 ROS 环境并校验包来源与二进制。已在运行且有新点云时直接复用；已存在但无点云的建图节点或单独占用雷达的驱动会明确报错，不强制杀进程。

相邻并发请求合并；脚本使用 flock 跨进程防重。步骤都有等待超时，总启动检查有 110 秒看门狗。失败时仅清理由该次请求创建的进程组，保留原来已运行的节点。一次启动失败不会自动反复拉起传感器，排除供电、USB 或网线问题后点击“启动建图”重试。

关闭地面站或取消自动启动不会停止已运行建图。当前不提供远程停止接口，避免网络断开或界面重启影响已有定位。

## 部署

先在新目录配置并构建建图所需的目标（不必构建飞行控制栈）：

```bash
cd /home/ly/Documents/diff-planner-A8mini-gcs-mission
bash sh_files/build_mapping_only.sh
bash deploy/install_mapping_start.sh
```

构建脚本从驱动的 `package_ROS1.xml` 生成缺少的 `package.xml` 并设置 `ROS_EDITION=ROS1`。不执行驱动原有 `build.sh`，因为它会删除工作空间的构建输出。脚本只构建建图相关二进制，不编译任务执行器或飞控控制器。

虚拟机更新地面站的 `deploy/mapping_tunnel.py` 并重启用户级 `agri-mapping-tunnel.service`。机载 `~/.ssh/authorized_keys` 中对应虚拟机地图公钥原有限制保留，仅增加 `permitopen="127.0.0.1:37263"`，不能开放任意端口或 shell。

```bash
systemctl status agri-mapping-start.service --no-pager
journalctl -u agri-mapping-start.service -n 60 --no-pager
curl --fail http://127.0.0.1:37263/status
```

API 状态为 `idle/starting/ready/error`。`ready` 表示启动检查通过；地面站地图的 LIVE 以持续收到真实点云为准，不用 API 状态替代数据新鲜度。

传感器日志：`~/.local/state/agri-gcs/mapping-start/{roscore,mavros,mapping}.log`，ROS 日志在同目录 `ros/log`。机载重启后 API 回到 idle，打开地面站会重新发起请求。机载不可达时地面站间隔重试，不需要重新打开界面。
