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

打开 http://127.0.0.1:8765 。用 `--port 8766` 更换端口，`--root /path/to/project` 指定项目目录，或 `--root /path/to/project/data` 指定数据目录；默认使用本项目 `data/`。服务只监听本机；上传在内存解析，不保存或修改原文件。

JSON 和模拟示例不需要新增 Python 依赖。读取 H5 需安装：

```bash
.pixi/envs/default/bin/python -m pip install -e '.[motion]'
```

页面使用随包分发的 Three.js 0.170.0 与 OrbitControls，无 CDN 和构建步骤，许可证位于 `src/motion_web/static/vendor/THREE-LICENSE.txt`。

## 操作

1. 分别选择 EGO / MOCAP 本地文件，或从会话数据下拉框选择并加载。网页按会话发现处理后的 EGO 手部 JSON、对齐 H5 和原始 NatNet 刚体 H5。
2. 没有数据时点击“加载模拟动作”，展示合成双手弯曲运动。页面持续显示模拟标记。
3. EGO 是绿色不透明实体，MOCAP 是紫色半透明虚影；点骨架使用球体/骨段，H5 内的 MANO/视觉网格使用三角形表面。虚影可穿透实体显示。
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
node --test tests/motion/*.test.mjs
```

真实 EGO/MOCAP 配对手部精度尚未验证；模拟示例只能验证加载、渲染与交互。2026-10-30 已用真实 MANO 网格验证新路径加载。

## 会话数据加载

`data/sessions/<session_id>/runs/<run_id>/aligned/aligned.h5` 的对齐H5显示在MOCAP下拉框并标记“已对齐”；`raw/mocap/` 中的原始刚体H5也可单独预览。读取左RGB统一时间轴上的最近邻动捕刚体中心，以首帧为0秒；时钟外推、缺失或跟踪无效的点隐藏。当前仍显示刚体位置，不显示姿态轴、手部骨架或RGB视频，也不能作为EGO算法结果进行误差比较。没有生成手部输出的运行不会被当成 EGO 手部结果列出。旧目录保留作迁移回退，但网页优先发现会话目录，避免同一资产重复出现。


## 2026-10-30：会话三 H5 与内部版本

当前目录为 `data/sessions/<session_id>/{ego.h5,mocap.h5,aligned.h5}`。页面显示实际数据根目录，点击“刷新会话数据”可重新扫描新录制/新处理版本；菜单按会话分组，显式选择版本，不默认加载历史结果。

- `ego.h5::processing/vision/<version>`：EGO 视觉处理版本，已保存网格时可显示实体；没有视觉结果时显示空状态，不把图像/IMU当动作。
- `mocap.h5::processing/mano/<version>`：MANO 网格，作为 MOCAP 虚影显示。
- `mocap.h5` 原始数据：兼容 NatNet 刚体与 C3D 导入后的 marker；marker 不自动连成骨架。
- `aligned.h5::processing/alignment/<version>`：按左RGB时间轴显示最近邻刚体结果。
- 兼容旧 `runs/<run>/ego/hands/*.json`、旧对齐文件以及 `unassigned/` 中的动捕样例。

网格接口为 `mesh/vertices(F,V,3)`、整数 `mesh/faces(T,3)`；单位和坐标系取自版本属性 `units` / `coordinate_frame`。时间使用 `arrays/time_s(F)`，或标量 `arrays/mocap_frame_rate` / `arrays/frame_rate` 生成相对秒数，不猜测帧率。可选 `arrays/valid(F)` 为布尔帧掩码；不完整版本不列出，缺少网格或时间信息时明确报错。最多4096顶点、8192三角形、200万帧顶点样本。网格顶点不是21关节。

目录加载直接读取所需 H5 数据集，不将大型图像 H5 整体读入内存；浏览器上传仍限制64 MiB。处理版本请从服务器会话目录选择，单独上传H5默认预览原始层。

已验证会话 `20261030_152704_299c41` 的 `moshpp-20261030-v1`：1328帧、778顶点、1538三角形、约11.058秒。没有改写源数据。

## 加载时转换到动捕世界系（2026-10-30）

页面默认“动捕世界系（加载时转换）”。Python在返回可视化数据前完成变换，源H5、JSON、配置均只读，不要求提前生成空间转换H5。切换显示坐标系或时间配对版本会清空旧画面，需要重新加载，避免保留旧设置下的姿态。

### 相机系 → 动捕世界系

当前接入左RGB光学系 `cam0_optical`：

```text
p_W[i] = T_WB[paired_rgb_index[i]] @ T_BC @ p_C[i]
```

默认外参为 `config/ego_left_rgb_extrinsics.json`；启动时可用 `./mocap visualize --extrinsics /path/to/calibration.json` 指定。文件须明确 `from_frame`、`to_frame`、`rigid_body_id`、`position_unit=m`、`T_BC`、`status`。此入口当前只关联 `RGB_left`；其他相机不得借用左RGB外参。项目现有外参状态是 `candidate_not_independently_validated`，页面始终标注，不能视为高精度真值。头戴刚体系/安装状态必须与标定一致。

同会话 `aligned.h5::processing/alignment/<version>/cameras/RGB_left` 提供原始 `rgb_index`、`rgb_pc_ns`、`clock_pair_usable` 及相应 `rigid_bodies/<rigid_body_id>/{position,quaternion_xyzw,usable}`。使用严格的原始RGB索引查表，不用相对首帧时间猜测配对，不插值、不固定使用首帧位姿。只有一个完成版本时自动选用；多个版本时在页面“相机转换使用的时间配对”中显式选择。不允许跨会话使用时间配对。

视觉H5需保存 `arrays/rgb_index(F)`，motion JSON需在每帧保存 `rgb_index`。该字段是原始RGB帧索引，不是裁剪后重新编号的动作序号。缺少索引或找不到配对时加载失败；无效跟踪/时钟外推对应帧隐藏。网格面拓扑不变，全部顶点按同一帧变换。输出保留RGB配对时间轴；如果只有RGB子集，首帧可能大于零。

### 已是世界坐标 / 其他源坐标系

- `mocap_world`：直接显示，明确标注“已声明为动捕世界系”，不重复乘外参，不据此宣称精度已验证。
- `ego_world`、`c3d_native` 等：使用**本会话** `visualization.json` 中明确提供的刚体世界变换。没有配置就提示缺少变换，可切回原坐标预览。不会猜测C3D轴方向、SLAM世界对齐或腕部绝对位置。

配置结构示例（`T_WSource` 必须替换为实际标定得到的4×4矩阵，不能填占位单位阵用于评估）：

```text
{
  "world_transforms": {
    "ego_world": {
      "position_unit": "m",
      "status": "你的验证状态",
      "T_WSource": 实际4×4刚体变换矩阵
    }
  }
}
```

对SLAM/EGO世界系，每段初始化都要提供本段变换，不自动用逐帧动捕修正SLAM漂移；腕部相对坐标还需先恢复绝对腕部位置和尺度。文件中的点由加载器先转换为米，配置平移也必须为米。

浏览器上传文件没有会话上下文：已在 `mocap_world` 的文件可直接显示；需要转换的文件请放到相应会话目录并从目录加载，或使用原坐标预览。JSON文件可通过 `/api/load?path=sessions/<id>/file.json&target=mocap_world` 加载；目录菜单保留已定义的资产规则。

API可附加 `target=mocap_world`、`alignment_version=<version>` 与 `alignment_session=<session_id>`；未传target保持原坐标接口兼容。返回 `spatial` 转换状态；动态转换与同版本对齐刚体有相同 `timeline_id`。空间坐标一致不自动证明两路同步，页面对不同会话/不同时间轴/人工偏移继续提示。最新C3D/Take尚缺时间锚点的会话不会被假装配对完成。
