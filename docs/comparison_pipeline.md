# 单一统一对比 Pipeline

## 两条完整路线

mocap-benchmark：EGO marker恢复设备刚体；hand marker经MANO恢复手部参考。设备与手以虚影展示，后续增加obj。

EGO算法：RGB+IMU输入新版OKVIS，导出cam0 FLU位姿T_LF；视觉手算法得到相机系p_C。设备与手以实体展示，视觉手永远使用SLAM位姿。

## 采集到可视化

1. 独立安装标定：确定IMU、左右相机和marker刚体之间固定变换；明确时间、单位和关节定义。
2. ./mocap record：先启动Motive录制，再启动EGO；保留时钟探针。此刻不重置mocap世界，也不创建SLAM世界。
3. 回传EGO与已标注C3D，保留原始时间索引和哈希；分别离线运行mocap参考、视觉手和SLAM。
4. SLAM初始化建立L；选首个跟踪有效且有对应mocap位姿的输出t0，以时间对应而非控制命令对应。
5. 固定T_WL = T_WB_mocap(t0) @ inverse(T_LF(t0) @ T_FB)。初始化mocap位姿允许时间插值，但不把插值宣称为原始观测。
6. 后续设备 = T_WL @ T_LF @ T_FB；手 = T_WL @ T_LF @ T_FC @ p_C。后续mocap只作参考，不修改算法轨迹。
7. 在单一Z-up场景对比两条路线，保留原点/坐标轴辅助层及缺失有效性；不自由缩放或逐帧贴合。

T_AB将B坐标变为A坐标；W为动捕世界、L为SLAM世界、I为IMU、B为marker刚体、C为相机。I与B刚性安装；L与运动中的B不是固定关系。T_IB = T_IC @ inverse(T_BC)。新版F为cam0 FLU；T_FC将光学RDF变为FLU，T_FB = T_FC @ inverse(T_IC) @ T_IB。历史IMU轨迹按其H5标记使用原链，不混用。

## 当前实现与边界

tools/build_initialization_comparison.py读取已有会话OKVIS、视觉手和动捕位姿；生成报告与Web缓存，并向aligned.h5新增初始化/配对版本，不覆盖历史版本。初始化使用原始OKVIS位姿，不用已全段对齐的设备轨迹反推世界变换。

20261030_182714_db5456：首个Good输出为索引1，EGO t=0.099958s，动捕t=0.618485s。初始化关系保存到reports/session_<ID>/world3d/initialization.json。初始化后的T_WL固定，设备和视觉手仅来自SLAM。

当前T_IB与时间偏移仍复用之前全段拟合结果，存在同序列信息参与，不能称为独立评估。正式实验需独立安装外参和时间锚；最终BA本身也使用整段数据，不能冒充纯在线轨迹。旧全段配准误差不适用新方案。obj尚未接入。

三H5职责保持：ego.h5保存传感器/视觉/OKVIS；mocap.h5保存参考；aligned.h5保存时钟、配对与历史处理版本。之前三个对比模式已取消。

## 质量检查与复现

物理参数确认前，保持候选安装外参及时间偏移不变。初始化导出增加以下检查：

- 时间戳必须严格递增；不外推，不跨无效跟踪帧插值。恰好命中有效采样点时不依赖前一帧状态。
- 从 RGB 起点起默认 2 秒内选择首个合格 SLAM 位姿，其对应动捕插值两端的 marker RMS 均须不超过 3 mm。
- 默认插值最大采样间隔 0.1 秒；初始化之前的算法帧隐藏。数值非有限的视觉顶点不展示。
- 门槛是可调整的工程默认值，不代表传感器精度认证。无合格初始化则退出，不退回全段配准。
- `initialization.json` 保存处理版本、实际读取的数据指纹、导出代码指纹、质量门槛、有效帧覆盖率及待确认事项。覆盖率是可用性，不是准确率。
- 当前动捕时间读取 mocap.h5 的 raw/frames/time_s；仍是相对时间，物理时间锚保持候选状态。

```bash
.pixi/envs/default/bin/python tools/build_initialization_comparison.py \
  --session 20261030_182714_db5456 \
  --slam-run okvis-20261101 \
  --vision-run sch-v21-temporal-v1 \
  --max-initial-delay-s 2 \
  --max-marker-rms-mm 3 \
  --max-gap-s 0.1 \
  --mocap-rate-hz 120

.pixi/envs/default/bin/python -m unittest discover \
  -s tests/tools -p test_comparison_quality.py
```

本命令消费已有离线结果及本会话 Web 资源，不负责重跑 OKVIS 或视觉推理；不修改原始 H5。明天确认物理参数后，需要将参数接入独立标定配置并重建初始化结果。

## 架构与入口更新

正式实现已迁入 `src/mocap_pipeline`：`inspection` 检查依赖，`quality` 处理采样有效性，`comparison` 计算固定初始化并导出，`static` 提供版本化展示模板。旧 tools 文件仅为兼容入口。

优先使用 `./mocap pipeline status --session <ID>` 与 `./mocap pipeline export --session <ID>`。导出不依赖旧 index.html，单版本可自动发现；多版本必须显式选择。参考缓存与 H5 帧时间及数据尺寸不一致时拒绝导出，避免混用结果。完整审查见 [架构梳理](architecture-review.md)。

## 三 H5 重建与断点续跑更新

`pipeline run` 已统一视觉、SLAM、MANO和导出；状态保存为会话内的 pipeline.json，详细流程见 [运行器说明](pipeline-runner.md)。前述“导出依赖已有 Web 缓存”和“按120 Hz生成动捕时间轴”为历史实现，当前已取消：直接读取 H5 的 raw/frames/time_s、MANO 网格、刚体轨迹和候选标定重建全部展示资源，初始化结果也写回 aligned.h5 的版本组。
