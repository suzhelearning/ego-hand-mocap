# 项目目录与数据生命周期设计

日期：2026-10-29。状态：已实施；原始旧路径保留，详见 `docs/数据目录重组_20261029.md`。

## 目标与范围

项目根目录为 `/home/current/syz/ego_hand_ground_truth`。采用“代码按职责组织，数据按采集会话聚合，网络节点通过配置关联”。一次实验可整体追溯、复制、归档；离线重跑不覆盖已有结果。

本次包含目录迁移、路径解析、清单适配、命令兼容、文档和相关验证。不实现新的手部识别、MANO拟合、空间标定求解或误差评测；为这些阶段预留位置，不创建虚假产物。不改变原始时间戳、坐标语义和历史采集状态。不重命名现有 Python 包。

## 方案比较与选择

1. 按设备组织：归属清楚，但同次实验分散，跨设备追溯困难。
2. 按处理阶段组织：适合批处理，但单次实验搬运和归档复杂。
3. 按会话聚合、内部按来源和阶段分层：便于完整追溯，代价是统一改造路径解析。

用户已选择方案3。以下字段、迁移策略和兼容细节是本设计提出的工程决策。

## 网络节点与职责

- EGO：ADB录制控制与文件导出；USB网络用于时钟探针；产生原始视频、H5及设备元数据。
- Windows/Motive：NatNet提供刚体与曝光计数；SSH用于Windows时钟探针；Motive take和C3D独立录制、导出与归档。当前联合采集不控制Motive take录制。
- Ubuntu：联合采集协调、时钟探针存储、数据汇聚、离线计算和本机可视化。
- Zenoh：保留已有发布订阅链路，单独管理部署；不成为直接NatNet联合采集的强制依赖。

设备名表示来源，session_id表示一次采集，run_id表示一次离线运行。IP和远端路径只出现在配置和来源记录中，不参与目录命名。

## 顶层布局

```text
ego_hand_ground_truth/
├── README.md
├── mocap
├── pyproject.toml
├── pixi.toml
├── pixi.lock
├── configs/
│   ├── network/
│   ├── capture/
│   ├── pipelines/
│   └── visualization/
├── src/                        # 保留当前包名
├── tools/                      # 转换、导出、诊断入口
├── deploy/
│   ├── ubuntu/
│   ├── windows/
│   ├── ego/
│   └── zenoh/
├── third_party/                # 外部项目内部目录不改动
├── tests/
├── licenses/
├── data/
│   ├── sessions/<session_id>/
│   ├── unassigned/<source_id>/
│   └── samples/<sample_id>/
├── calibration/
│   ├── devices/<device_id>/<calibration_id>/
│   └── rigs/<rig_id>/<calibration_id>/
├── reports/                    # 跨会话报告、迁移审计
├── docs/
│   ├── architecture/
│   ├── formats/
│   ├── guides/
│   ├── history/
│   └── superpowers/
└── .cache/                     # 可再生缓存
```

`.pixi/`保留在原位；build、egg-info、pytest缓存等生成目录不纳入数据归档，也不在此次迁移中未经核实删除。顶层manifest.json是已有来源快照，保留原件；不替代会话清单。许可证随依赖保留。空的未来功能目录无需预先创建。

## 会话与运行布局

```text
data/sessions/<session_id>/
├── session.json
├── raw/
│   ├── ego/                    # 保留设备导出的内部文件名和结构
│   ├── mocap/
│   │   ├── natnet.h5
│   │   └── motive/             # 有证据关联到本次采集的tak/c3d
│   └── clocks/{ego,windows}.jsonl
├── capture/                    # 事件、控制响应、采集日志
├── calibration/references.json
└── runs/<run_id>/
    ├── run.json
    ├── config/                 # 实际使用配置快照
    ├── ego/
    │   ├── okvis/{pose.h5,diagnostics/}
    │   └── hands/joints21.h5
    ├── mocap/{markers/,mano/}
    ├── sync/                   # 模型、左右配对CSV、统计
    ├── aligned/aligned.h5
    ├── evaluation/
    ├── visualization/scene.json
    ├── logs/
    └── work/                   # 可重建的大型中间文件
```

raw在采集结束后不可由离线流程改写。后续导入同次Motive文件属于可追溯的追加操作，记录导入时间、来源和哈希，不能替换已有文件。视频不嵌入aligned.h5或复制到可视化目录。

历史session_id保持不变。新session_id默认采用现有时间加唯一后缀方式；run_id建议为日期时间加唯一后缀，不根据算法名猜测配置。ID必须是安全的单路径组件，拒绝路径穿越。已存在ID拒绝覆盖。

每个run可以只执行一个或多个阶段，不要求全部算法齐备。run创建后记录running状态，结束为complete或failed；complete仅表示所请求阶段执行成功。阶段分别记录not_run、running、complete、failed、reused；质量验证独立记录，不从执行成功推断精度已验证。运行结束后保持不可变，重试生成新run。

## 清单、路径与来源规范

session.json新增明确schema版本，包含session_id、原始采集状态和错误、设备身份、采集配置快照引用、原始资产索引、哈希、原有录制身份、时钟域说明和追加导入历史。迁移不得丢弃旧字段，旧清单原件保存于迁移审计目录。

run.json包含schema版本、session_id、run_id、请求阶段、阶段状态、输入资产及哈希、代码来源版本、依赖版本、实际配置、标定引用、产物索引及哈希、失败原因和质量标记。没有Git版本时明确记为不可用，可用相关源文件哈希记录来源，不伪造提交信息。

路径统一规则：

- session.json里的本地路径以所在会话目录为基准。
- run.json里的本运行产物路径以所在run目录为基准；会话原始输入通过资产ID引用，不拼接任意相对上跳路径。
- 跨运行复用通过session_id、run_id、资产ID及哈希定位；显式记录依赖，不使用latest作为可复现输入。
- 标定通过设备/rig标识、calibration_id及哈希定位。单独导出会话包时包含所引用标定和跨运行依赖，导出前验证依赖闭合。
- Windows路径、设备SD路径等远端来源字符串原样保留，不能当成本地路径解析。
- 文件访问始终受批准的数据根目录限制，解析后检查真实路径，包括符号链接；清单不能扩大Web服务的可访问范围。

标定references.json表达采集时已知关联；运行使用不同标定版本时在run.json记录实际选择，不回改旧运行。marker重新安装后不能自动沿用旧rig外参。

## 配置与代码职责

合并config和configs。configs/network/lab.yaml记录EGO地址/序列号、Motive地址、Windows SSH别名、归档目录和可选Zenoh端点。命令行参数优先于配置，配置优先于兼容默认值。启动时记录实际生效的非敏感配置；密钥不进入项目清单。

Zenoh ACL与节点启动部署文件归deploy/zenoh，网络端点归network配置。tools/start_router.sh若迁移到deploy，旧调用入口需同步更新或保留轻量转发。Third_Repo改为third_party之前审计构建配置、绝对路径和本地修改，不盲目重建或删除外部构建产物。

统一路径解析器集中处理项目根、数据根、会话、运行、标定和旧格式。具体模块归属由实施计划确定；不在各脚本重复拼目录。现有./mocap record、./mocap visualize保持可用，--output-root继续表示项目输出根目录。离线工具增加显式会话/运行选择，移除写死的alignment_20261029等目录。

## 可视化消费规则

界面按会话→运行→资产类型选择。只展示实际存在且支持解析的资产；原始EGO H5、C3D不能被标成已支持的手部动作。已有模拟示例保留明确模拟标签。

对齐结果通过明确的统一时间轴和原始帧关联播放，不对两路分别首帧归零后声称完成同步。普通原始预览保留预览语义。此迁移不新增RGB渲染、姿态轴或MANO渲染能力。

## 旧目录映射

| 旧位置 | 新位置/规则 |
|---|---|
| capture_sessions/<id>/session.json | data/sessions/<id>/session.json |
| capture_sessions/<id>/sync | 同会话raw/clocks |
| 采集事件、控制响应和日志 | 同会话capture |
| data/ego_data/<id> | 同会话raw/ego |
| data/mocap_data/<id>.h5 | 同会话raw/mocap/natnet.h5 |
| results/alignment_20261029/<id> | 经来源核验的历史run/sync |
| results/okvis_pose/<id> | 经来源核验的历史run/ego/okvis、logs、work |
| data/ego_pose_h5/* | 对应历史run/ego/okvis/pose.h5 |
| data/aligned_h5/<id> | 对应历史run/aligned |
| 格式样例egotest_001.c3d | data/samples/c3d-format-egotest-001 |
| 其他归属不明录制 | data/unassigned/<source_id>，记录来源，不强配会话 |
| 历史时钟审查实验 | 按真实内容归实验数据或reports，保留输入引用 |

不同旧处理结果不能仅按日期归入一个run。无法证明同一次处理的结果分为多个历史run，以显式依赖连接；未知配置、版本或阶段状态标为unknown，不补造成功证据。

## 迁移与回滚策略

1. 只读盘点：所有文件、大小、哈希、旧清单、H5引用、脚本路径、构建依赖及磁盘余量。产出reports/migrations/<migration_id>/中的计划与冲突报告。
2. 先实现新旧清单读取和新路径解析，再在临时目录验证新写入格式。迁移工具默认dry-run；执行显式指定计划。
3. 对每个会话复制到临时目标并校验，完成后同文件系统内原子改名到最终会话目录。空间不足时停止并报告，不自行删除源文件或改用不可回滚策略。
4. 原始文件逐字节保留。派生H5若含旧绝对路径，优先由加载器根据来源资产映射解析；确需改写则生成新的派生产物并记录来源哈希，保留旧文件。历史报告中的旧路径保留为历史证据，通过迁移映射解释。
5. 更新入口、实时说明和Web数据发现；新旧同一会话同时存在时只显示新布局，来源不一致则报冲突，不能静默合并。旧路径兼容作为过渡读取能力，不依赖批量符号链接。
6. 验证后报告迁移状态。旧源文件本轮保留，不自动删除；退役旧源是后续明确操作。回滚恢复旧入口/配置和清单读取，源文件仍在；新采集会话独立保留，不能随回滚丢弃。

迁移记录逐项包含旧路径、新路径、前后哈希、处理方式和结果。重复执行不得覆盖，已完成且哈希相同可跳过，冲突停止。用户已有未知文件、外部仓库本地修改和构建产物不擅自清理。

## 验收标准

- 原始H5、MP4、C3D及探针文件迁移前后哈希一致，采集状态和时间语义不变。
- 会话清单、运行输入及标定依赖全部可解析；移动完整会话包后仍可定位内部文件。
- 使用已有会话回归离线时间配对，索引、时间差和有效性与原结果一致；对齐H5的数值字段不变。
- OKVIS现有产物可读取并保持数值不变，不为路径迁移重新运行昂贵SLAM。
- Web能发现新布局、加载已有支持的数据，并拒绝清单路径越界；失败或未实现阶段不会显示为成功。
- 新采集路径通过自动化测试验证；真实设备联通的采集冒烟只在设备可用且适合录制时执行，未执行需明确记录。
- 对受影响采集、路径、导出、可视化功能运行适当回归检查；无需为纯文档搬迁新增测试。
- README和操作指南描述当前实现；历史报告保留日期与当时结论。

## 后续步骤

实施按“路径与清单兼容→新写入与读取→数据迁移→验证与文档”推进。外部OKVIS仓库因构建产物含当前绝对路径，继续留在 `Third_Repo/`；迁移后的副本没有引入新路径依赖。
