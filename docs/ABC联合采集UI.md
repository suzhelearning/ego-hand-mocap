# ABC 联合采集 UI

## 启动

在 ego_hand_ground_truth 工作区运行：

```bash
.pixi/envs/default/bin/python tools/joint_collection_ui.py --port 8775
```

打开 http://127.0.0.1:8775 。复用 `/home/current/code/emg_glove_ws` 的采集员页、被试页、人员登记、动作库与 ABC 输入。此入口直接读取 Current 串口，无需 ROS；原手套与非联合模式保持原入口。

默认网络参数取 `configs/network/lab.json`。腕带按 USB 序列号绑定：左尾号 `70`、右尾号 `6C`。可用 `--left-emg-port`、`--right-emg-port` 覆盖；`--emg-ui-root` 指定其他 UI 工作区。

## 操作

1. 选择参与者、生成动作序列，完成原有踏板校准和采前检查。
2. 点击“开始演示”：连接 Ego 与 NatNet，启动 Ego / Windows 时钟探针；腕带连接保持运行。
3. A：开始当前动作的肌电文件、独立 Motive Take 与独立 Ego 录制。启动需要数秒，等待启动提示消失再执行动作。
4. B：结束并保存本动作。校验 Windows Take，导出并校验 Ego 文件，保存肌电单向粗映射。保存完成后只切换演示，下一动作仍需 A。
5. 录制中 C：沿用原动作纠错流程；确认后 B 保存。待录时 C：结束整轮并保存汇总。停止整轮按钮可收尾当前动作。

启动/保存期间显示忙碌提示并拒绝其他操作，不排队重复 ABC。设备异常时保存已有数据并标记失败；不在原轮内静默重连。结束这一轮、排查并重连后再开始新轮。

## 保存结构与时间

默认根目录 `data/joint_ui`。保留原 UI 的 `日期/参与者/动作名_时间.h5` 与同名标签目录；单手动作继续保留另一只手的配对文件。

每动作标签目录新增：

- `joint/manifest.json`：该动作 Take 路径、大小和 SHA256，Ego 文件、起停证据及完整性状态。
- `joint/ego/`：Ego 原始 H5 及各路视频，已校验 SHA256。UI 的 Ego 窗口显示左右 RGB 实时预览。
- `joint/mocap.h5`：该动作 NatNet 刚体及原始曝光时钟。
- `joint/ego_control.json`、`motive_control.json`：控制操作记录。

Take 留在 Windows 原采集目录（默认 `~/Desktop/syz/ego-hand/remote`），每动作独立 `.tak`，不复制到 Ubuntu；关联以 manifest 为准，不能按文件名猜对应动作。

整轮的 `joint_collection_<id>/clocks/ego.jsonl` 和 `windows.jsonl` 连续保存四时间戳探针。每个动作的 `collection_directory` 指向同一整轮目录。动作之间不重建时钟探针；原始时钟不被修改。Ego / 动捕保留原时钟和对时证据，供离线仿射映射使用。

肌电原始设备时间、读取批次时间和连接 epoch 保留。每侧各自拟合单向粗映射，原始 EMG 和 IMU 共用该侧映射；写入各数据组下的 `coarse_alignment/pc_timestamp_ns` 与 `clock_extrapolated`。时间域是本机 CLOCK_MONOTONIC，不是日期时间。每动作用本段读取证据拟合，不重置设备时钟、不重新握手；不足以拟合的短片段明确记 `insufficient_data`。未知固定传输/传感器延迟仍存在，`physical_sync_valid=false`，不能当作精确物理同步。

A/B 是动作标签边界；不同设备依次起停，原始文件可能包含边界余量。使用标签和映射后的时间裁剪，不能简单按各文件第一帧对齐。

## 验证记录（2026-11-04）

- 联合后端回归 116 项通过；UI、Current、动作流程相关回归 93 项通过。7 项 ROS launch 环境相关用例未运行，本机采用无 ROS 入口。
- 真实双动作采集：`data/joint_ui_validation/20261104/UI_TEST/`。两段独立 Take / Ego / 双侧 EMG，Take 与 Ego 校验成功；同轮探针对象与腕带连接 epoch 未改变。
- 浏览器确认双侧 1000 SPS、EMG0 v1 数据到达及联合模式提示。ABC 验收由程序调用同一输入处理路径完成，未声称实际踩过物理踏板。
- 这批是软件联调数据；手动放行采前检查有记录，不代表生理肌电质量合格。

补充检查：修复异常收尾后，再次完成两段真实采集（18:30 的两段）。扩展纠错/力动作/重连检查 42 项通过，另有两项旧纠错测试仍读取已废弃的 `collection_session.json` 位置，在未修改的 HEAD 副本中也同样失败；一项 Weili ROS 模块测试缺运行依赖。本次未改变旧汇总结构。

当前电脑部分原动作演示视频尚未缓存，例如 `fingers`；UI 会阻止缺视频动作开始。可先在手动列表选择文字动作 `fingerfreeform`、`unconstrained`，或按原项目 `docs/ORIGINAL_ACTION_TERMINAL.md` 准备完整视频缓存。这与传感器连接/联合保存无关。

## 本机 PCsensor 脚踏板

现场确认输出顺序为左 `R`、中 `S`、右 `D`。联合模式中将 R/S/D 分别映射为 A/B/C（开始、保存、纠错或结束），同时保留原 ABC 键盘输入。页面刷新后生效；输入框内键入字母不会触发采集。D 的松开事件也对应 C，保持原纠错键的防抖行为。页面采前校准显示两套按键对应关系。


Ego 实时预览：在“Tab 2 · 实时数据”的 Ego 窗口显示 RGB 左右画面。后端只读订阅现有 5580/5581 H264 发布流，不发送录制/相机参数命令；预览缩至640像素宽、约4帧/秒，原始录像不改变。断流超过1.5秒拒绝返回旧图。页面隐藏时不请求图片，后台保持有限缓存。通过本机带令牌接口读取，不上传云端。新增依赖 `pip install -e '.[preview]'`；本机已安装。验证：两路JPEG连续变化、未授权请求403、EMG仍就绪；36项相关测试通过。
