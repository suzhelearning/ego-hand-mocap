# MOTION 本地可视化

## 启动

从项目根目录运行：

```bash
./mocap visualize
# 或
pixi run motion
# 或使用已有环境
.pixi/envs/default/bin/python -m motion_web
```

打开 http://127.0.0.1:8765 。用 `--port 8766` 更换端口，`--root /path/to/project` 指定包含 `ego_data/`、`mocap_data/` 的目录。服务只监听本机；上传在内存解析，不保存或修改原文件。

JSON 和模拟示例不需要新增 Python 依赖。读取 H5 需安装：

```bash
.pixi/envs/default/bin/python -m pip install -e '.[motion]'
```

页面使用随包分发的 Three.js 0.170.0 与 OrbitControls，无 CDN 和构建步骤，许可证位于 `src/motion_web/static/vendor/THREE-LICENSE.txt`。

## 操作

1. 分别选择 EGO / MOCAP 本地文件，或从会话数据下拉框选择并加载。网页按会话发现处理后的 EGO 手部 JSON、对齐 H5 和原始 NatNet 刚体 H5。
2. 没有数据时点击“加载模拟动作”，展示合成双手弯曲运动。页面持续显示模拟标记。
3. EGO 是绿色不透明球体/骨段，MOCAP 是紫色半透明球体/骨段；虚影可穿透实体显示，便于比对。当前不是 MANO 表面网格。
4. 左键旋转、右键平移、滚轮缩放；“适配视角”按有效轨迹范围重新构图。
5. 支持暂停、循环、倍速、逐帧、时间轴、两路显示开关和 GT 透明度。逐帧优先使用 EGO 采样时间，没有 EGO 时使用 MOCAP。
6. 两路时间各自首帧归零。MOCAP 正偏移表示延后，负偏移表示提前。最近邻原始采样，无插值，超过容差或范围隐藏，缺失点/无效帧不沿用旧姿态。

**这只是预览时间轴，不证明同步。** 即使坐标系名称一致，也不代表标定正确。不同坐标系会告警，但仍原样叠加以便检查。真实对比前应在离线处理阶段完成时间映射、坐标变换和单位核实；本版不计算误差分数。

## motion JSON v1

每个文件为一路动作。最小示例：

```json
{
  "version": 1,
  "name": "EGO right hand",
  "kind": "points",
  "unit": "m",
  "coordinate_system": "camera",
  "bones": [[0, 1]],
  "frames": [
    {"time_s": 0.0, "points": [[0, 0, 0], [0, 0.03, 0]], "valid": true},
    {"time_s": 0.033333, "points": [[0.01, 0, 0], null], "valid": true}
  ]
}
```

- `version=1`，`unit` 为 `m` / `cm` / `mm`，`coordinate_system` 为明确非空名称。浏览器统一使用米，Y 轴朝上；如源数据轴定义不同，需要离线变换，不按名称自动猜测。
- `time_s` 是有限数且严格递增，单位秒。建议导出相对秒数避免大时间戳的浮点精度损失。API 返回 `source_start_s` 保留文件首时间。
- 每帧点数量保持一致，最多128点；缺失点用 `null`，整帧无效用 `valid:false`。骨骼边引用从0开始的点索引，缺失任一端点时不绘制。
- `bones` 可省略，只显示点。21点顺序由数据提供者确定，页面不猜测或强制重新排序。
- `synthetic:true` 标记合成数据，真实数据不应设置此标记。
- 最多50,000帧，单文件最多64 MiB；这是短片段预览器，长录制请先裁剪。JSON 在内存完整加载。

## 原始格式支持边界

- `schema=ego-natnet-raw-rigid-v1` 的 H5：读取 `frames/motive_timestamp`、`rigid_bodies/*/position` 和 `tracking_valid`，以刚体中心点预览。只显示位置，不显示刚体朝向；**不是手部 Ground Truth**。使用 `position_unit`，不假定采样已与 EGO 同步。
- 原始 EGO H5：当前包含图像/IMU 的文件不会自动变成动作；明确报错并要求先输出 motion JSON。算法21点接口确定后可增加专用适配器。
- C3D：本版不解析，不实现 marker→MANO→21点；给出转换提示。原始 marker 不等同于关节中心。
- 数据目录中的 JSON 元数据/summary 文件不一定是动作 JSON，选中后会显示格式校验错误。

## 验证

```bash
.pixi/envs/default/bin/python -m pytest
node --test tests/motion/timeline.test.mjs
```

真实 EGO/MOCAP 配对手部数据尚未验证；模拟示例只能验证加载、渲染与交互。

## 会话数据加载

`data/sessions/<session_id>/runs/<run_id>/aligned/aligned.h5` 的对齐H5显示在MOCAP下拉框并标记“已对齐”；`raw/mocap/` 中的原始刚体H5也可单独预览。读取左RGB统一时间轴上的最近邻动捕刚体中心，以首帧为0秒；时钟外推、缺失或跟踪无效的点隐藏。当前仍显示刚体位置，不显示姿态轴、手部骨架或RGB视频，也不能作为EGO算法结果进行误差比较。没有生成手部输出的运行不会被当成 EGO 手部结果列出。旧目录保留作迁移回退，但网页优先发现会话目录，避免同一资产重复出现。
