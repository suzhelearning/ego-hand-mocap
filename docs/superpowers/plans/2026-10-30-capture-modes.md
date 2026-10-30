# 两种联合采集模式实施计划

目标：`mocap record --mode h5|take`，默认Windows Take。用户已要求实施。

设计：共用EGO生命周期、探针、会话锁和导出。Take模式通过NatNet命令控制Windows Motive，以实时帧recording位验证转换；SSH用于确认共享Windows保存目录及核验指定Take，不复制Take、不生成动捕H5。Windows根目录默认远端用户的~/Desktop/syz/ego-hand/remote，可显式覆盖。已有录制拒绝接管；中途失败尽力收尾并记录失败，停止不确定时不能报告成功。

- [x] 为命令参数、两模式隔离、Take异常收尾编写测试。
- [x] 实现NatNet命令应答和实时录制状态，Windows文件验证控制器。
- [x] 接入会话生命周期；Take引用以remote_assets记录，禁止伪造mocap_h5。
- [x] 更新文档，运行完整Python和前端测试，审查差异。

验证边界：本轮自动化使用本地协议服务/模拟设备，不在无操作人员确认的情况下启动真实硬件录制。需实机验收Motive版本、Take扩展名和网络设置。

验证结果：159项Python测试、2项前端测试通过；git diff --check通过。独立审查提出停止UDP丢包风险，已加入最多3次停止尝试及新状态帧确认，补充丢请求、丢应答和重试耗尽测试。尚未实际启动Motive/EGO进行Take录制。

后续调整：Windows根目录复用，取消会话子目录；Take用北京时间含微秒时间点命名。核验只读取本次精确文件，同名存在则拒绝。
