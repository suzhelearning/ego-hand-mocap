# SCH v2.1 · 时序 MANO 精简推理包

**时序 MANO 是 v2.1 的 `mano_temporal` 阶段，不是独立模型。**
本包已固定该阶段及已验证参数，直接从双目视频运行 AnyHand-WiLoR 前端和时序 MANO 优化。
仅保留推理及必要依赖，无 Git 仓库、评测入口、看板、数据集和旧结果。

## 1. 放权重

把单独提供的三个文件放到以下位置：

```text
weights/anyhand/wilor/anyhand_wilor.ckpt
WiLoR/pretrained_models/detector.pt
WiLoR/mano_data/MANO_RIGHT.pkl
```

小型模型配置和 MANO 均值参数已包含。安装脚本会自动建立 AnyHand 的资产引用。
需要的是 **AnyHand-WiLoR checkpoint**，不能用原版 WiLoR checkpoint 替代。

## 2. 安装

适用 Linux / RTX 5090；Python 3.10、PyTorch 2.7.1 + CUDA 12.8。

```bash
sudo apt-get install -y git ffmpeg libegl1 libgl1 libglib2.0-0 build-essential
curl -LsSf https://astral.sh/uv/install.sh | sh  # 已有 uv 则跳过
bash setup.sh
```

首次安装依赖需联网，只建立一个包内环境 `.venvs/anyhand`。

## 3. 推理

填写 `configs/input.example.json` 中的视频、标定、时间戳路径和同步信息，然后：

```bash
./run_benchmark.sh --config configs/input.example.json --output my_run_001
```

相对路径基于包根目录。默认跑完整视频，建议先用一对短视频测试。
时间戳 CSV 表头为 `frame_id,timestamp_ns`，每帧一行，frame_id 从 0 连续递增，
时间严格递增，行数匹配视频；左右时间必须处于同一时间基准。

标定结构见 `configs/calibration.schema.example.json`，示例数值仅用于展示格式。
支持 KB4 / FISHEYE624，尺寸需匹配视频。
`T_left_right` 将右相机坐标变换到左相机坐标，平移单位米。

结果：`ego_hand_benchmark/results/my_run_001/`。
`run.json` 记录状态和输出路径，`reconstruction/` 包含关键点、MANO 参数/网格和预览视频。
已有实验名不会被覆盖；解码缓存位于 `data/derived/`。

核心代码在 `handpose_pipeline/pipelines/sch_v2_hand/`，配置在 `configs/sch_v21_temporal.json`。
这是一套离线视频推理流程。代码快照来源见 `PROVENANCE.json`；第三方许可随源码保留。
