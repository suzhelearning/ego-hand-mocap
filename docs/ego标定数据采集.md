# Ego 外参标定：采集与保存

本阶段连续保存动捕帧，按人工指定的静止窗口分组，并归档对应的相机图片；不计算时间同步或外参。静止状态由操作人员保证，程序尚未判断姿态是否稳定。

前提：Zenoh Router 和本项目 NatNet publisher 正在运行，且 Motive 中 `ego` 有效。选择与实际相机一致的 `--camera-id`；默认为 `cam0_optical`，不要据默认值假定其他相机的坐标系。

```bash
./mocap calibrate ego record --output calibration/ego_session_001 --camera-id cam0_optical
```

首次运行前在项目根目录执行 `pixi install`；`./mocap` 默认使用本项目的 Pixi 环境。输出目录必须不存在，避免覆盖。

相机对准固定标定板，头戴停稳后输入 `s` 开始当前组；保持静止、采集相机图像后输入 `e` 结束。改变位置和朝向，重复上述步骤；输入 `q` 结束。程序自动按名称解析 `ego`；如果无法获取名称映射，可加 `--rigid-id 10`（使用实际 ID）。

也可用 `--duration 2` 自动采一组，用于检查链路。中断时保留已收到的数据，session 状态为 interrupted。

当前系统没有已知的实时相机控制接口：在每组静止期间通过 Ego 自身采集方式录图，随后导出图像，记录帧号、真实采集时间戳和设备时钟域（含 boot_id）。停止录制后，将图像导入相应组，例如：

```bash
./mocap calibrate ego attach-image --session calibration/ego_session_001 --group 0001 \
  --image /path/to/frame.png --timestamp-ns 123456789 --frame-index 42 \
  --clock-domain ego_CLOCK_MONOTONIC_boot_id
```

时间戳和帧号须替换为实际值。图像与组的对应由人工保证，保存为 `manual_static_pose_group`，不表示已完成跨设备同步。图像不自动从视频或 H5 提取；每组当前归档一张，不覆盖已有图像。保留原视频、H5、相机内参、畸变参数、图像旋转镜像处理和标定板类型、方格尺寸，用于后续求解。

目录结构：

```text
session.json             会话状态、刚体名称/ID、相机 ID、各组计数
mocap.jsonl              完整 NatNet 帧及接收时间，包含 marker 和所有刚体
events.jsonl             分组开始/结束事件及接收时钟时间
samples/0001/ego.jsonl   窗口内每帧目标刚体、原始时间字段及缺失/有效性
samples/0001/sample.json 窗口边界、接收帧数和有效帧数
samples/0001/camera/     手动归档图片及时间、时钟域、帧号和 SHA-256
```

`receiver_time_ns` 和 `publisher_received_time_ns` 是 Linux 接收时钟，不是图像或 OptiTrack 曝光时间；NatNet 高精度字段保留原始 ticks，频率未经确认。`usable_rigid_data` 仅表示窗口有至少一帧有效刚体，不代表静止、同步或标定质量合格。无刚体帧和无效帧仍被保存。正式采集建议同时保存 Motive `.tak`，并检查帧号和丢帧计数。
