# EGO 设备实时可视化实施计划

目标：NatNet EGO 刚体实时驱动 head_ring_description URDF，在 Z-up 页面显示 B、base、left_cam。
用户确认 OBJ 与 URDF base 一致；GL xyz=(-2,-13,3)mm，GO PYR=(-1,0,10)deg。
结构：ego_live 模块负责安装变换、URDF 清单、最新帧缓冲和本机 HTTP；复用 NatNetStream 和 Three.js。
不改变 Motive 刚体定义、不启动录制、不替换离线候选标定。几何偏移保持候选。

- [x] 测试局部安装平移随刚体旋转、Motive PYR 与 URDF RPY 分离、跟踪过期隐藏。
- [x] 实现配置、URDF 解析和只保留最新帧的 NatNet 接收。
- [x] 实现本机实时页面，加载六个 STL、显示三个坐标轴和跟踪状态。
- [x] 接入 ./mocap live-device，运行测试及实际连接，浏览器验证并更新 README。
