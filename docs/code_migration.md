# 动捕代码迁移与使用

2026-10-28 从 `/home/current/syz/mocap` 工作区迁移，保持原来的 `src/natnet_zenoh`、`src/mocap_tools`、`src/acquisition`、`src/mocap_runtime`、`tests/natnet`、`tests/tools` 路径和模块名。源文件校验值见 `licenses/migration_manifest.json`。旧仓库保持不变。

## 范围

- 完整复制 NatNet SDK/适配器/发布订阅/队列和相应测试，保留 SDK 许可与版权。
- 复制 `track_rigid.py` 和 `kinematics.py`。刚体记录拒绝覆盖已有文件；修复 duration 与 stats 共用计时导致无法自动停止的问题，并显式持有订阅句柄、清理资源。
- `alignment.py` 从旧逻辑提取并改写为离线 `interpolate_rigid_poses`，支持任意图像目标时刻；不依赖 Manus、原生扩展或固定60Hz。禁止外推和跨长缺口/无效端点插值。
- `object_offset.py` 提取并改写为 `pose_matrix`、`camera_world_pose`、`world_points_to_camera`，只应用已知外参。未迁移旧 HDF5 管线、Manus SDK、个人标定、MANO 模型和机器人 TCP 标定。
- 新增精简命令入口，保留 `publish mocap`、`subscribe mocap`、`track-rigid` 命名；没有迁移旧项目的其余子命令。

## 安装

在本项目根目录，用独立 Python 环境安装：

```bash
pixi install
./mocap --help
```

`./mocap` 默认使用本项目的 Pixi Python；测试使用 `pixi run test -q`。

## 采集

旧仓库记录的 Windows 地址为 `169.254.213.247`，Ubuntu 为 `169.254.1.1`，这些是历史配置，运行前核对当前机器地址。Motive Tracker 3.4.1.1 Final 中启用 NatNet Unicast、Rigid Bodies 和所需 marker 流。需要原始 marker 时核对 Labeled/Unlabeled 输出。建议同时保存 Motive .tak。

先在本项目启动 Zenoh Router：`./mocap router`。如果 7447 端口已有 Router，确认其配置后复用。Router 运行时，在本项目两个终端分别执行，IP 参数替换为实际地址：

```bash
./mocap publish mocap --motive-server-ip 169.254.213.247 --natnet-client-ip 169.254.1.1 --zenoh-endpoint tcp/127.0.0.1:7447
```

```bash
./mocap subscribe mocap --connect-endpoint tcp/127.0.0.1:7447 --output ego_data/session_001/mocap.jsonl
```

完整帧 JSONL 保存所有传输的 marker 和刚体。输出文件已存在时默认拒绝覆盖；此文件是后续离线评测的主要原始输入。发布队列仍可能丢帧，检查 frame_number 和 publisher_dropped_frames，JSONL 不替代 .tak。

可以在另一个终端查看或记录 `ego`：

```bash
./mocap track-rigid record --names ego --duration 30 --output ego_data/session_001/ego.jsonl
```

`ego` 名称由 publisher 的刚体描述映射解析，区分大小写；也可用 `--ids`。刚体 JSONL 是辅助数据，不包含所有原始 marker，不替代完整帧记录。历史示例频率不是当前测得频率。

## 时间与坐标约定

- `motive_timestamp`：保留 SDK 原值，不默认与 Ego 共享零点。
- `publisher_received_time_ns`：Ubuntu 发布进程的 monotonic 接收时间。
- 刚体记录中的 `t_ns`：订阅端 monotonic 接收时间，**不是曝光时刻**。
- `natnet_timing`：附加保存 SDK 的曝光中点、数据接收、发送及 precision timestamp 原始字段。缺失字段为 null；tick_frequency_hz 为 null，须结合实际 SDK/服务器信息核实，不能将原始 ticks 直接当纳秒。
- wire schema_version 保持1，新增字段为可选扩展，兼容旧消息；原有校验器仍校验核心字段。
- 四元数 wire 顺序为 xyzw，`kinematics.py` 内部数学函数为 wxyz；必须显式转换。
- 旧 schema 的 `motive_x_forward_z_up_right_handed` 是配置约定，程序不自动检测或变换坐标轴。采集前核实 Motive 实际轴向与单位。

此版本保存同步依据，但**尚未实现或测得 Ego↔Motive 的离线时间映射**。仓库保留插值工具，但最新配对策略是统一到 PC 时间后取最近邻，不使用插值函数生成标签；不能以网络接收时间估计代替图像曝光验证。

## 外参

`T_AB` 将 B 系列向量变换到 A 系，位置单位 m。使用 `camera_world_pose(T_WB, T_BC)` 得到相机世界位姿；使用 `world_points_to_camera(points, T_WB, T_BC)` 变换参考点。

`T_BC` 必须通过独立标定求出；本次没有创建虚假的单位外参，也没有移植标定板检测/PnP 求解。重新安装 marker 或修改 ego 刚体 pivot/轴后应重新核验。

## 验证边界

自动化测试覆盖离线 NatNet 解包、收发、字段保留、刚体选取、文件保护、插值及变换方向。未连接 Windows/Motive 或 Ego 设备，未验证实机采集、同步精度和外参精度。
