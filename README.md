# Ego 手部识别动捕 Ground Truth 项目

通过 OptiTrack 采集头戴刚体和手部 marker，拟合 MANO 后生成21点三维参考结果，与 Ego 图像算法的21点估计做离线对比；保留原始配对数据，支持后续算法优化。

## 当前状态

联合采集、五次会话的离线时钟配对和两段EGO OKVIS位姿处理已有产物。此迁移把原始输入与历史处理结果归入会话目录；手部 C3D→MANO→21点、相机到头戴刚体外参及最终误差评测仍待实施，现有材料不包含已完成的手部精度实验。

头戴录制时持续运动。Ego 每次录制初始化自己的局部世界系，不能用初始头戴位姿代替逐帧动捕轨迹。Ego21点究竟输出在当前头戴系还是本段世界系，仍待确认。

## 分工

- 孙祎喆：动捕链路、头戴与手部 marker、相机刚体标定、MANO参考结果和离线对齐。
- 石晨浩：Ego手部识别、21点输出接口及联合评估；实际模型版本待提供。

## 文件导航

- `data/sessions/<session_id>/`：一次采集的数据、对时依据与处理运行；原始数据位于 `raw/`，采集日志位于 `capture/`，离线运行位于 `runs/<run_id>/`。
- `data/unassigned/`：尚不能证明属于哪个会话的历史文件。
- `data/samples/`：格式测试样例与历史采集冒烟数据，不自动参与评测。
- `reports/migrations/layout-20261029-v1/`：本次文件映射、SHA-256核对和迁移结果。
- `docs/数据目录重组_20261029.md`：当前路径规范、迁移统计和历史目录兼容说明。

- `docs/联合数据采集.md`：运行 `./mocap record` 一次启动 EGO 与 Ubuntu 动捕 H5 录制，Ctrl+C 停止并保存；定时录制用 `./mocap record --duration 5`。

- `docs/项目数据流.md`：当前三类数据、PC 时间基准、最近邻配对、坐标变换与21点评测的统一说明。

- `docs/关键上下文与接手说明.md`：后续接手优先阅读，含用户确认事实、关键约定、风险及待确认接口。

- `docs/项目信息同步文档.md`：飞书文档可读快照，包括具体内参、畸变、外参、分工与接口。
- `docs/时间同步方案.md`：本轮新增的离线同步实施建议。
- `docs/feishu_snapshot.json`、`docs/feishu_snapshot.xml`：保留文档版本与块结构的原始快照。
- `calibration/devices/head_000205/extracted-from-h5/`：从该设备H5提取的标定快照。
- `src/`：项目自身的采集、对齐、可视化与位姿工具。
- `configs/`：网络与处理配置；采集网络默认值位于 `configs/network/lab.json`。
- `deploy/zenoh/`：Zenoh Router ACL配置。
- `results/`：保留的旧版处理记录与来源文件；新的离线结果写在对应会话的 `runs/` 下。
- `Third_Repo/`：暂保留在原位置，外部OKVIS构建产物记录了该绝对路径；未经重建验证不移动。
- `manifest.json`：来源、快照版本与已存文件校验值。

## 数据目录约定

```text
data/sessions/<session_id>/
├── session.json
├── raw/{ego,mocap,clocks}/
├── capture/
└── runs/<run_id>/{ego,mocap,sync,aligned,evaluation,logs,work}/
```

采集ID沿用原始会话名；每次重处理使用新run ID。原始输入和旧路径均保留，本次迁移为复制，摘要与文件哈希记录在迁移报告中。

## 坐标变换

T_AB 表示将B系点变到A系。W=动捕世界系，B=头戴marker刚体系，C=算法实际使用的相机/头戴系，E=本次Ego初始化世界系。

相机/头戴系输出：p_W_Ego(t)=T_WB(t)·T_BC·p_C_Ego(t)。

Ego世界系输出：p_W_Ego(t)=T_WE·p_E_Ego(t)，每段录制建立对应关系；若有T_EC(t)，则T_WE=T_WB(t)·T_BC·inverse(T_EC(t))。漂移和重定位须独立处理。

H5参考系为cam0_optical；固定相机间外参方向为cam0→cami。单位m，齐次列向量左乘，矩阵按行存储。T_BC尚需标定。

## 下一步

1. 明确Ego输出坐标系、真实尺度、左右手及21点定义、图像采集时间戳。
2. 保存逐帧头戴刚体位姿与有效性，确定marker布局及MANO表面对应。
3. 录制同步动作，估计跨设备时间映射，并独立验证图像曝光时刻。
4. 确认畸变模型与旋转镜像图像域，完成相机—刚体外参标定。
5. 实现MANO拟合、质量控制和配对评测。

## 来源

飞书：https://vcnqhq28cfdm.feishu.cn/docx/SyU7dqjtloMpSKxOMXZcFXKRnac

本地整理日期：2026-10-28（Asia/Shanghai）。时间同步方案来自本轮后续讨论，尚未同步回飞书。此目录为本地快照，不自动跟随飞书更新。

## 动捕代码入口（2026-10-28 迁移）

已从旧 mocap 仓库迁移 NatNet 采集、刚体记录、位姿数学函数及测试，保留模块路径与命名。安装、采集命令、时间字段语义和适配范围见 `docs/code_migration.md`。

目前可以接收并记录原始动捕流；五次会话已有离线时钟映射和最近邻配对。相机外参标定求解、marker→MANO拟合与最终精度评测仍待实现。

Ego 外参标定的数据采集命令和格式见 `docs/ego标定数据采集.md`。当前支持实时记录动捕、静止窗口分组，以及录制结束后人工归档相机图像。

## MOTION Web 可视化

运行 `./mocap visualize`，打开 http://127.0.0.1:8765 。支持 EGO 不透明实体与 MOCAP 半透明虚影叠加、双路文件加载、统一播放时间轴；没有动作数据时可点击“加载模拟动作”。当前支持统一 motion JSON 与 NatNet 刚体 H5 位置预览，尚未实现原始 Ego H5 手部重建或 C3D/MANO 拟合。格式、可选 H5 依赖和操作说明见 [docs/motion_web.md](docs/motion_web.md)。
