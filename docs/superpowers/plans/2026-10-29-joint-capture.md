# 联合采集实现计划

> 按 executing-plans 在本会话内实施，完成后独立审查。当前目录非Git仓库，不创建worktree或提交。

目标：交付 ./mocap record [--duration 秒]，可启动、停止、保存两端文件并记录会话和对时依据。
设计：../specs/2026-10-29-joint-capture-design.md
技术：现有Python、NatNet、h5py、ADB、SSH、TCP；不依赖临时/tmp环境或results脚本。

- [x] 任务1：src/joint_capture/storage.py、natnet.py；先测ServerInfo非法字段、H5批量追加/缺失刚体/不覆盖，再实现流式原始记录。
- [x] 任务2：ego.py、clock.py；设备状态所有权、文件路径边界与四时间戳采样；外部接口用明确测试替身/回环TCP测试。
- [x] 任务3：session.py、cli.py与mocap命令接入；先测协调启动/正常停止/错误清理/已有录制保护，实施定时和Ctrl+C、会话文件与锁。
- [x] 任务4：补依赖和用户说明，运行全测试、独立审查、实机5秒验收；保存验收结果并给出可直接运行的命令。

检查重点：无数据不启动Ego；SDK退出不能挂起；只停止本会话录制；中途异常不丢已收到数据；加密导出不泄露凭据；完成必须检查双侧文件存在且帧数非零；新运行不覆盖任何历史录制。

验收证据：docs/联合数据采集_验收_20261029.md；用法：docs/联合数据采集.md。
