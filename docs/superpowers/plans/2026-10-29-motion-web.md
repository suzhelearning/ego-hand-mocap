# Motion Web Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 实现本地双路动作加载与三维叠加播放。
**Architecture:** Python 校验/适配动作数据并提供 HTTP 接口，Three.js 静态客户端呈现点骨架。
**Tech Stack:** Python 标准库，h5py 可选，Three.js 本地资源，原生 ES modules。
**Spec:** docs/superpowers/specs/2026-10-29-motion-web-design.md

## Global Constraints
- 不改原始数据；默认只监听本机；限制读取在指定数据根目录。
- 明示模拟数据和未同步坐标/时间状态；不将刚体位置当作手部关节。

## Review Focus
- 非有限数、重复/倒序时间与错误点/骨骼结构应拒绝。
- 路径穿越与超大请求不可读取任意本地文件。
- 缺失点、无效帧及时间轴边界不能显示滞留姿态。
- 两路加载失败不能破坏已加载数据，替换后时间轴要重算。
- 启动和静态资源无需外部网络；小屏控件可访问。

## Tasks
- [x] 1. tests/motion/test_data.py 先验证数据校验、单位转换、H5缺失支持、demo，再实现 src/motion_web/data.py。
- [x] 2. tests/motion/test_server.py 验证文件限制与真实 HTTP 加载；实现 server.py 和 CLI，加入包静态资源。
- [x] 3. tests/motion/timeline.test.mjs 验证最近邻、时间越界、偏移；实现静态页面、播放器与 Three.js 渲染。
- [x] 4. 跑完整 pytest、Node 测试、浏览器验证，记录依赖和启动方法。

## Execution ledger
- 当前目录不是 Git 仓库，直接在工作目录编辑，不创建 worktree 或提交。
- 用户已确认推荐方案，按现有授权直接实施，不重复索取流程批准。

- 完成：102 项 Python 测试、2 项 Node 时间轴测试通过；wheel 包含静态资源与许可证。
- 浏览器已验证：空状态、模拟双手、连续播放、逐帧、NatNet 600帧加载、原始EGO拒绝提示、正时间偏移隐藏越界样本。
- 独立审查发现非法 unit 类型/超大整数导致异常逃逸，已复现并增加解析及 HTTP 回归测试修复。
- 真实配对手部数据与精度评测尚未验证，属于本次明确边界。
