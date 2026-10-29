# 每架无人机的雷达建图配置

地面站的“启动建图”和命令行统一使用 `sh_files/start_mapping_only.sh`。
每架无人机只编辑一份 YAML，不再修改启动脚本、Livox JSON 或 Faster-LIO 算法配置。

## 选择配置

默认按当前 Linux 登录用户读取 `config/mapping_profiles/<用户名>.yaml`：

- `ly`：`ly.yaml`，旧无人机已有外参、雷达 `192.168.1.190`。
- `zzz`：`zzz.yaml`，新无人机原项目已验证外参、雷达 `192.168.1.193`。

找不到当前用户的配置就报错，不会自动使用其他机型的标定。用户名只是默认映射；同名用户管理多架机器时，必须显式选择各自的配置。

显式指定的优先级：命令行 `--profile` > 环境变量 `AGRI_MAPPING_PROFILE` > 登录用户名。

```bash
bash sh_files/start_mapping_only.sh                  # 当前用户配置
bash sh_files/start_mapping_only.sh --profile zzz    # 指定配置名
AGRI_MAPPING_PROFILE=zzz bash sh_files/start_mapping_only.sh
bash sh_files/start_mapping_only.sh --profile /absolute/path/aircraft.yaml
```

运行 API 时可在 systemd 服务中设置 `Environment=AGRI_MAPPING_PROFILE=zzz`；省略则按服务运行用户选择。HTTP 请求不能修改配置路径或机型。

## 修改哪些参数

复制已有 YAML 为 `<新用户名>.yaml`（或新的机型名），填写：

| 参数 | 含义 |
|---|---|
| `lidar.model` | SDK 配置格式：`MID360` 或 `Mid360s`，名称区分大小写。按设备/本机 SDK 的已验证格式选择；新机 zzz 使用 `Mid360s` 才能接收数据。 |
| `lidar.ip` | 雷达本身的 IPv4 地址 |
| `lidar.host_ip` | 机载连接雷达的有线网卡地址，不是 Wi-Fi 地址 |
| `lidar.broadcast_code` | 雷达广播码/序列号，沿用设备已确认的值 |
| `lidar.publish_freq` | 发布频率（Hz），常用 10 |
| `imu.topic` | `/mavros/imu/data` 或 `/livox/imu`；更换 IMU 来源必须使用对应标定 |
| `imu.fcu_url` | MAVROS 飞控连接，例如 `/dev/ttyACM0:57600`；使用雷达 IMU 时不会启动 MAVROS |
| `extrinsics.extrinsic_T` | 雷达坐标到所选 IMU 坐标的平移，三个数，单位米 |
| `extrinsics.extrinsic_R` | 同一变换的 3×3 旋转矩阵，按行展开为九个数；不是欧拉角 |

变换方向与 Faster-LIO 保持一致：`p_IMU = R × p_LiDAR + T`。配置检查只验证数字、维度和旋转矩阵结构，不代表完成标定或飞行验收。

`AGRI_MAPPING_PROFILE` 也允许指定仓库外的绝对路径，便于保存各机本地标定。

## 生效与运行状态

启动器先校验 YAML，再以原有算法配置和驱动模板生成专属运行文件。文件位于：

```text
~/.local/state/agri-gcs/mapping-start/profiles/<配置指纹>/
  lidar.json
  lio.yaml
  profile.json
```

不会覆盖仓库内的原始 JSON/YAML，也不会覆盖另一份运行配置。启动日志打印所选 YAML 路径；运行配置指纹可查询：

```bash
rosparam get /agri_mapping/profile_digest
```

已运行的建图只能在配置指纹一致时直接复用。改过参数、换机型或旧建图未记录配置时，启动器会拒绝复用，并要求先在地面停止建图再启动。不会一边建图一边偷偷切换外参。直接重启启动 API 可能停止其托管的传感器节点，须先确认飞机在地面且电机锁定。

`mapping_mid360.launch` 新增了 `lidar_config`、`lio_config`、`profile_digest` 参数供统一启动脚本传入。直接运行旧 launch 的默认参数仍为兼容旧流程；按用户选择机型时应使用上面的统一脚本，不要直接调用旧 launch。

本机服务安装可用 `bash deploy/install_mapping_start.sh [--profile 配置名或绝对路径]`。该安装器使用当前用户的 systemd 服务，避免写死 `ly` 或仓库目录；若仍有旧的系统级同名服务运行，会提示先迁移，避免端口冲突。

## 验证

离线配置测试（不会启动 ROS 或雷达）：

```bash
python3 -m unittest discover -s tests -p 'test_mapping_profiles.py' -v
```

硬件检查仍需核验点云和 world 位姿持续更新。两个 YAML 是两架现有机体的不同标定，不能仅因用户名相同就移植到另一架机体。
