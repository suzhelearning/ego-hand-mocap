# 对齐H5

每个会话子目录包含一个 `<会话名>_aligned.h5`。左右相机分别位于 `cameras/RGB_left`、`cameras/RGB_right`，每行对应一个原始RGB帧。

- `rgb_cif_ns` / `rgb_pc_ns`：原始EGO帧时间和映射后的PC时间。
- `mocap_index` / `mocap_frame_number`：原始动捕H5行索引和NatNet帧号。
- `mocap_exposure_ticks` / `mocap_pc_ns`：动捕曝光计数和映射后的PC时间。
- `delta_ns`：动捕映射时间减RGB映射时间。
- `clock_extrapolated`、`within_mocap_coverage`：外推及覆盖标记。
- `clock_pair_usable`：在动捕覆盖内且未进行时钟外推；不代表光学同步已验证。
- `rigid_bodies/<id>/`：最近邻原始刚体position（米）、quaternion_xyzw、tracking_valid、present、mean_error；usable额外要求跟踪有效。
- `metadata/`：完整时钟模型和对齐统计JSON。

RGB图像像素不嵌入此文件；相机属性video_path关联原MP4，文件属性记录原始EGO和动捕H5路径。位姿没有插值，仍使用原动捕坐标系；未进行空间外参变换。物理曝光相位尚未标定，physical_exposure_alignment_valid为False。

原failed采集状态保留为source_capture_status；本文件complete仅表示打包完整。已逐相机核验原始RGB时间戳及每个所选动捕字段与源H5完全一致。
