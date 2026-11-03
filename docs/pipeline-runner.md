# 从三 H5 重建、按阶段续跑

## 一条命令

```bash
./mocap pipeline run --session 20261030_182714_db5456 \
  --profile config/pipeline/20261030_182714_db5456.json
```

顺序为视觉推理 → SLAM → MANO拟合 → 固定初始化与参考缓存导出。三个算法阶段互不依赖，最终导出依赖三者；当前串行执行，避免 GPU/内存争用。

首次明确接管已有完整处理结果时加 `--adopt-existing`。接管记录会注明 `adopted_existing`，表示已有产物在当前输入指纹下建立基线，**不是重新推理，也不能反向证明历史产物与当前算法版本一致**。本条会话已完成这一步，后续直接运行以上命令。

```bash
# 只执行到 SLAM；下次默认继续完成后续阶段
./mocap pipeline run --session 20261030_182714_db5456 \
  --profile config/pipeline/20261030_182714_db5456.json --until slam

# 强制重算视觉，完成后导出会根据变化失效；其他算法独立判断复用
./mocap pipeline run --session 20261030_182714_db5456 \
  --profile config/pipeline/20261030_182714_db5456.json --force vision
```

## 续跑规则

- `data/sessions/<ID>/pipeline.json` 保存每阶段状态、输入指纹、输出版本、输出指纹、来源和日志路径。
- 只有 `complete`、输入指纹相同、输出仍完整且指纹相同才跳过。仅有文件或上次退出码为零均不够。
- 输入指纹基于实际 H5 输入组、视频、配置、适配代码和 profile 声明的算法代码/权重；不对含有输出的整个 ego.h5 做输入哈希，避免自身写回导致永久失效。
- 依赖的输出变化使导出阶段失效。当前配置/核心适配代码变化采用保守失效策略，可能使多个算法重算。
- 每次尝试使用新版本名，原 H5 版本不覆盖。失败留下日志及工作目录，再运行同命令可跳过此前成功阶段并重试失败阶段。
- 断点粒度是**阶段**，不承诺恢复神经网络中间帧或求解器内部迭代。若进程在写完 H5 后、保存 checkpoint 前中断，可能重算该阶段并保留旧版本。
- 会话锁禁止同一会话同时运行两条流水线。Ctrl+C/SIGTERM 会记录失败并终止适配器子进程组；异常断电留下 running 时，下次重新执行该阶段。
- 计算日志位于 `reports/session_<ID>/pipeline-<stage-version>.log`；大型算法中间文件位于 profile 指定的工作区和算法仓库，不增加 data 下的结果目录树。

## 三 H5 的唯一数据来源

| 输入 | 用途 |
|---|---|
| ego.h5 原始 RGB 时间、视觉顶点、SLAM 位姿、运行时 okvis.yaml | 算法实体、相机坐标轴、原始帧时间 |
| mocap.h5 原始 marker 与 time_s、MANO 顶点和拓扑 | 动捕设备点云、手部虚影、真实相对采样时间 |
| aligned.h5 保存的设备刚体轨迹、候选安装外参和时间偏移 | 参考设备位姿及初始化候选参数 |

不再读取旧 `slam.json`、`meta.json`、`sensor_frames.json` 或二进制参考缓存作为输入。参考最近邻配对保留索引、时间差和覆盖标志；超出覆盖范围隐藏，而不是把末帧冒充对应观测。设备与手部参考有效性分开。两路网格使用各自 H5 内的 faces，避免默认拓扑相同。

每次固定初始化另存为 `aligned.h5::/processing/initialization/initial-<fingerprint>/`，保存 T_WL、SLAM设备轨迹、有效性、原始 RGB 时间、mocap 索引和配对时间差。同指纹版本复用。Web 缓存可以丢弃后重建：

```bash
./mocap pipeline export --session 20261030_182714_db5456 \
  --slam-run okvis-20261101 --vision-run sch-v21-temporal-v1 \
  --mano-run moshpp-v1 --registration-run okvis-20261101 --body-run v1 \
  --output-dir /tmp/ego-h5-rebuilt
```

`export` 只组装已有 H5 处理结果；`run` 负责算法执行、写回 H5 和导出。

## 适配边界与候选参数

当前提供的 profile 绑定本条会话及本机环境：M900 双目、head_000022 IMU 适配配置、MANO_L_20 左手20点标签。新设备或不同标签不能只换 session ID；必须确认对应 profile。配置中 external solver 路径明确，运行器不自动安装或修改算法环境。

当前 aligned.h5 中已保存的 world_preview 刚体参考和 slam_registration 候选参数是必要输入。本轮不重新估计物理安装外参、不做整段重新配准。新会话若没有这些输入会明确失败，不能套用本条会话的世界变换。物理参数确认后应新增标定版本。

视觉输入适配核对视频帧数与时间戳，必要时裁剪无时间戳的尾帧；沿用现有相机标定与候选模型设置。MANO适配从 mocap.h5 提取左手 marker，保留缺失值，沿用皮肤距离等既有候选假设。OKVIS 使用独立环境并写回最终 BA 位姿，不能称为实时在线评估。

## 2026-11-02 SLAM 源码替换记录

- `Third_Repo/orbslam3_mocap-OKVIS2-X-imu`：用户移入的旧代码目录，历史 `okvis-20261101` 来自替换之前的代码，不能当作新版输出。
- `Third_Repo/orbslam3_mocap-OKVIS2-X`：当前最新代码。profile 的执行路径仍指向这里；已取消 `existing_versions.slam`，防止首次 `--adopt-existing` 将旧产物接管为新版结果。
- 新适配层默认导出 `trajectory_cam0_*.csv` / `trajectory_cam0.tum`，位姿为 `T_worldFLU_cam0FLU`。原始 `okvis2-*_trajectory.csv` 仍为估计器 IMU 位姿。本项目当前 `okvis_pose_to_h5.py` 读取后者，因此不能直接把 cam0 文件传给旧 IMU 变换链。
- 替换时检查：新目录缺少 `.pixi/envs/default/bin/python` 和 `build/okvis_app_synchronous`；`head_to_okvis.py` 的 ORB 参考默认路径为另一机器的 `/home/wh/rk3576/orbslam3_mocap`。该次检查的环境、参考路径和输出契约缺项现已由下节的新版接入解决。
- 现有 H5、候选标定、pipeline checkpoint 与展示结果未重算。profile 已变化，后续 `run` 会重新评估指纹；完成新版适配前请勿把当前展示视为新代码的验证结果。

## 新版 cam0 FLU 接入（2026-11-02）

当前 profile 设置 `slam_pose_frame: cam0_flu`，重新运行新版求解器后导入 `trajectory_cam0_ba.csv`（无 BA 文件时为 final）。H5 仍存于 `/processing/okvis/<version>`，但明确标记 `body_frame=cam0_flu`、`transform=T_worldFLU_cam0FLU`，并保存 `world_frame.json` 和源码指纹。旧 IMU 版本保持原义，不覆盖。

设 F 为 cam0 FLU、C 为左相机光学 RDF，`T_CF` 仅旋转：

```text
R_CF = [[0,-1,0], [0,0,-1], [1,0,0]]
T_FC = inverse(T_CF)
T_FB = T_FC × inverse(T_IC) × T_IB
T_WL = T_WB_mocap(t0) × inverse(T_LF(t0) × T_FB)
设备：T_WB_slam(t) = T_WL × T_LF(t) × T_FB
视觉手：p_W(t) = T_WL × T_LF(t) × T_FC × p_C(t)
```

T_IB、T_IC、时间偏移均保持之前的候选值；T_WL 只在首个合格时刻初始化一次。新版世界原点是默认轨迹第一帧 cam0 光心；初始化采样仍采用首个 Good 且动捕质量合格的时刻，不强行把命令开始时刻当世界原点。

展示中左光心局部坐标轴跟随 cam0 FLU，IMU和刚体轴保持各自物理定义。传感器安装关系仍从固定外参推导。导出器按 H5 的 body_frame 分派转换，未知类型直接报错。

环境：新版缺失的 external/supereight2 依赖使用本机旧目录内已有的依赖源码补齐，估计器从新版源码重新编译；复用本机 Pixi 缓存依赖环境。build 位于 `/data/ego-hand-work/build/okvis-cam0-20261102`，避免系统盘空间不足。数据读取/旧设备标定模块及许可证随适配器补齐；不再依赖另一机器 `/home/wh/...` 的路径。

### 本条数据实际重跑结果

2026-11-02 已由统一 pipeline 真实运行新版求解器，H5 版本为 `slam-0641eb9bc683-1`：1291 个 cam0 FLU 位姿、约43秒，1290 Good / 0 Marginal / 1 Lost，最终 BA 完成，visual_lost_warnings=0。有效显示帧1289/1293；视觉左手186/1293。状态为 executed，视觉/MANO仍为明确接管的历史结果。

候选安装矩阵与时间偏移经逐值核对保持不变。旧 `okvis-20261101` 保留；源码、轨迹、世界FLU定义与相机配置均保存在新版本的 metadata/provenance 中。重复运行已验证按指纹复用。
