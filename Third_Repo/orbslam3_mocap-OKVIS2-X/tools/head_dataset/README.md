# Head dataset adapter

构建需要包含 Git 子模块的完整源码。本地目录是无 `.git` 的源码拷贝；原记录的公开克隆地址当前不可访问，获取原分支应使用有权限的仓库地址。依赖与构建方法见下文「构建」。

源码不包含本机 `datasets/`、`results/`、`build/` 或 `okvis2-local-deps/`。Python 适配脚本依赖 NumPy、OpenCV、h5py、PyYAML，诊断脚本还使用 SciPy。相机布局和 IMU 输入约定脚本已从 `orbslam3_mocap-feature-imu-static-init/Examples/Stereo-Inertial/scripts/` 复制到本目录，保留原 ORB 工程可运行；其许可见 `ORB_REFERENCE_LICENSE`（GPL-3.0）。

`profile_preferences.json` 记录的是本机验证过的输入路径和缓存。首次克隆没有转换缓存时会先导出原参数；要重建009优化配置，可先导出数据，再按下文 `prepare_drift_profile.py --gyro-noise 0.001 --acc-noise 0.01` 创建独立变体，将其导出文件夹传给 `run_head.sh`。验证结论不自动推广到其他数据集。

图形运行入口：`pixi run bash run_head.sh /数据集文件夹路径`（在本仓库根目录执行）。默认显示相机特征匹配窗口，并打开浏览器三维轨迹界面，替换原来的二维俯视图。结束后按相机窗口内任意键让命令退出，浏览器回放仍可使用。图形开关写入本次结果目录的 `okvis_gui.yaml`，原始导出配置保留。直接调用底层 `run.sh` 仍默认无窗口；可设置 `OKVIS_GUI=1` 开启，设置 `OKVIS_GUI_HOLD=0` 可在运行结束后自动关闭相机窗口。

运行产物放在原始数据集的 `slam/` 中，与 `video/` 同级。新转换缓存放在 `slam/exports/`，每次运行单独创建 `slam/runs/日期时间_随机后缀/`，其中保存轨迹CSV/TUM、地图、日志、统计、配置、三维服务信息和 `replay_3d.html`。已有工程内转换缓存可继续复用；新结果仍写到原始数据集下。传入导出目录时通过 `adapter_report.json` 找到原始数据集。底层 `run.sh` 未指定第二个结果目录参数时也采用此布局；显式第二参数可指定其他位置。

```text
head_000003/
├── head_000003.h5
├── video/
└── slam/
    ├── exports/     # 转换后的相机/IMU数据
    └── runs/
        └── 日期时间_后缀/
            ├── run.log
            ├── summary.json
            ├── trajectory_imu.tum
            ├── okvis2-slam-final-ba_trajectory.csv
            ├── okvis2-slam-final_map.csv
            └── replay_3d.html
```

## 2026-10-13 漂移优化

修复了双目特征描述器首次初始化时跨相机写入的线程竞争。每个提取线程现在只初始化其持锁的相机。重新编译后，009 使用 `sigma_g_c=0.001`、`sigma_a_c=0.01` 的独立配置，替代之前的 0.007、0.05；不改变像素、坐标轴、外参或运动轨迹。010 保留原噪声配置，因为较强的 IMU 权重在010上使Z变化增大。

`profile_preferences.json` 只为已验证的原始数据路径指定优先配置，入口仍为 `run_head.sh /数据集文件夹`。直接传入导出目录会使用该目录本身的配置。新数据默认保持原参数，不自动套用009的调整。每次运行创建新结果目录，旧配置和旧结果保留。

009 的独立图像/陀螺仪极线检查使用58组背景匹配，最小残差候选为−20ms，分组为−28/−24ms，因此继续保留原−22.2ms，不把弱观测拟合成新的精确标定。噪声参数根据相邻IMU样本的稳健差分估计提出，再通过完整运行验证；这不是正式的Allan噪声或六面标定。

躯干遮罩也做了完整对照：单独启用会显著减少有效背景特征，009首尾距离反而增至1.39m，故没有默认启用。相关可选支持已保留，并修正上游mask极性错误（0排除、非0有效）；BRISK检测后、描述前过滤遮罩特征。`test_camera_mask.cpp` 检查mask极性、尺寸、投影与特征过滤。

原009无窗口运行本来已有约4.2cm首尾距离，图形运行曾达到79cm，因此不能只据首尾距离声称精度提升多少倍。对照报告保留旧无窗口结果、旧图形结果、线程修复单独结果和新参数重复结果。首尾距离是运动估计的一个观测量，没有真值时不等于定位误差，也不保证每次随机RANSAC/并行优化完全一致。

可创建额外独立参数对照，图像以符号链接复用：

```bash
python3 tools/head_dataset/prepare_drift_profile.py \
  --base datasets/head_000009_pinhole --output datasets/new_trial \
  --gyro-noise 0.001 --acc-noise 0.01
```

## 三维可视化与后期回放

```bash
# 传入运行结果目录，或原始数据集目录（自动查找最近的运行）
/home/wh/rk3576/OKVIS2-X/view_result.sh /home/wh/下载/head_0/head_000009
```

界面提供三维拖动旋转、平移、滚轮缩放、XY/XZ/YZ视图、视角按钮、时间滑条、播放速度、IMU姿态轴、XYZ时间曲线、相机同步帧和最终地图点云。选择不同运行可比较同一数据的重复运行；不同运行的世界航向可能不同，没有自动跨运行对齐。

实时读取每2秒更新。刷新保留视角与坐标范围；初始实时范围为XYZ各±2m，越界时手动点击“适配范围”。比例按米一致，不夸大Z轴，也不自动将运动轨迹旋转成水平。最终地图仅显示轨迹附近最多15000个点，避免远距离离群点占满视野。

“实时输出”按时间戳保留首次发布的位姿，避免最终优化回调把最后一帧历史输出覆盖。结束时全局轨迹、最终优化轨迹可独立选择，并叠加实时输出。轨迹差异和起终点距离不等于真值误差。

运行结束后生成 `replay_3d.html`。双击可离线查看轨迹、姿态、曲线和点云，无需网络；独立HTML不包含全部视频帧。用 `view_result.sh` 在本机回放可同时看原始转换图像。界面也可点击“保存离线回放HTML”。手动导出：

```bash
python3 /home/wh/rk3576/OKVIS2-X/tools/head_dataset/viewer.py --result 结果目录 --export
```

三维服务只监听127.0.0.1。单独运行 `view_result.sh` 时 Ctrl+C 关闭服务；随SLAM自动启动的服务PID记录在 `结果目录/viewer.pid`，关闭时可执行 `kill "$(cat 结果目录/viewer.pid)"`。URL记录在 `viewer_url.txt`。浏览器关闭不会停止SLAM。

本地 Plotly JavaScript 已打包，不依赖CDN。服务依赖Python与现有NumPy；界面支持WebGL浏览器。

此目录适配 `head-v3 H5 + video/*.mp4`。输入数据只读。默认使用 RGB 双目和 imu0，640×553、140°水平视野的虚拟针孔图像。暂不使用 IR、深度网络、GNSS、LiDAR 或语义建图。

## 构建（Ubuntu 24.04，无需 sudo）

当前稀疏双目惯性目标不使用 ROS2、Torch、RealSense 或 CUDA。先保证仓库的 Git 子模块已初始化（包括 `supereight2/third_party/srl_projection`）；仅复制源码目录不会复制子模块内容。本目录的源码拷贝没有 `.git` 元数据，原文所列 GitHub 分支地址当前返回 404；此处构建使用公开的上游 OKVIS2-X 子模块版本，不能据此确认与原分支的 Git 提交完全一致。要严格复现原分支，请从可访问的原始仓库获取带 Git 子模块记录的完整源码。

```bash
cd /path/to/orbslam3_mocap-OKVIS2-X
pixi install
pixi run bash tools/head_dataset/build.sh
pixi run build/okvis_app_synchronous
```

最后一条命令不传参数时输出 `Usage: ... configuration-yaml-file dataset-folder`，用于检查可执行程序及动态库能否加载。`pixi.toml` 锁定开发依赖，`.pixi/config.toml` 将环境移到源码目录外，以避免 CMake 导出接口拒绝源码树内的依赖头文件。以前的系统包/本地 GeographicLib 安装仍可直接运行 `bash tools/head_dataset/build.sh`。

转换新数据时直接运行 `pixi run bash run_head.sh /path/to/head_dataset`；若只导出不运行，可调用下方 `head_to_okvis.py`。已导出的数据不再依赖外部 ORB 工程路径。

## 导出并运行

```bash
pixi run python tools/head_dataset/head_to_okvis.py \
  --dataset /path/to/head_000009 \
  --output /path/to/new_export \
  --acc-noise 0.05
pixi run bash tools/head_dataset/run.sh /path/to/new_export
```

输出目录必须不存在，防止覆盖旧结果。`--projection native` 保留原生鱼眼缩放；OKVIS 的 pinhole+equidistant 无法可靠表示后半球射线，默认针孔重投影更稳妥。重投影不改变光学轴或外参，也不将双目强行整平。

## 定义来源

- 从本目录内置的 `dataset_layout.py` 和 `merge_dataset_camera_settings.py` 加载运行时相机绑定、内参、规范外参和 IMU 输入约定，不再依赖本机下载目录。
- `cam0=RGB_left`，`cam1=RGB_right`，单位为 m、rad/s、m/s²。按 H5 的 `frame_index` 解码，不能以数组行号代替视频帧号；视频可能含额外帧。
- 数据集外参 `T_CB` 表示相机←rig，IMU 配置 `T_SB` 表示 IMU←rig。OKVIS 的相机外参为 `T_SC = T_SB @ inverse(T_CB)`。输出位姿默认属于 IMU；不能直接当成 ORB 的相机位姿比较。
- 沿用 ORB 的 `imu_input_calibration` 解析 M900 legacy profile：IMU轴无需再次变换，已做六面校正的数据不重复扣偏置。未标定时间偏移时沿用该 profile 的 −0.0222 s，直接加到 IMU 时间戳；OKVIS 的 `image_delay=0`，避免双重校正。此偏移是参考工程的假设，不能视为本序列实测标定。`--imu-offset 0` 可做无偏移对照。
- 相机时间戳保留 CLOCK_MONOTONIC 原始纳秒值，不用 MP4 FPS 或 PTS 生成；双目唯一近邻匹配限差1ms，记录 `frame_mapping.csv`。只保留两端有25ms IMU覆盖的帧。
- 默认噪声来自内嵌标定。此次实测显式使用 `--acc-noise 0.05`，与009 ORB恢复配置一致；元数据原值为0.0012。两者均不是此次重新测出的噪声参数。
- `adapter_report.json` 保留偏移、外参定义、噪声选择及原始六面校正质量警告。

## 输出与验证范围

运行目录中的 `run.log`、`okvis2-slam-final_trajectory.csv`、`okvis2-slam-final-ba_trajectory.csv` 与 `okvis2-slam-final_map.csv` 来自上游。应用额外导出 `tracking_quality.csv`：quality 0=Good、1=Marginal、2=Lost，后台更新可能重复同一 state_id，统计时按 state_id 去重。

轨迹存在不代表视觉跟踪正常：丢跟踪时 IMU 仍可输出预测位姿。应同时检查质量状态、初始化/重初始化日志、轨迹有限值、覆盖和异常跳变。两组数据没有已确认的真值轨迹，不能据此给出定位精度或成功率保证。

`run.sh` 成功退出后调用 `summarize_run.py` 生成 `summary.json` 和 `trajectory_imu.tum`。也可手动运行 `python3 tools/head_dataset/summarize_run.py 导出目录 运行结果目录`。TUM 的时间为原始单次启动时钟的秒数，不是 Unix 时间；世界坐标系由估计器初始化，Z轴朝上，位姿属于 IMU。

`verify_export.py` 是针对此次两组 M900 数据的集成检查，验证外参链、原始帧索引/纳秒时间映射、首帧像素与重投影、IMU单位和偏移符号；它不是其他传感器配置的通用校验器。
