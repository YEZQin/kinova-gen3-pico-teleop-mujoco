# 可复用机器人遥操作 Skills 设计

日期：2026-07-26  
状态：用户已批准。

## 目标

把本项目中可跨对话复用的硬件配置、安全控制、仿真验证和发布经验安装为
个人 Codex Skills。Skills 安装到 `C:\Users\qqxx\.codex\skills`，不依赖当前
对话历史，也不把项目专用细节塞进一个过大的 Skill。

## Skill 边界

### `operating-pico-xr-bridge`

触发于 PICO 左手柄、XRoboToolkit PC Service、WSL/Python binding、
`PXREASetting.ini`、TCP 60061、Grip、位姿和时间戳问题。

- 负责 Windows → WSL → Python 的输入链路。
- 提供分层诊断和安全的只读检查。
- 不负责 MuJoCo IK 或 Kortex 机械臂运动。

### `validating-kinova-gen3-mujoco`

触发于 Kinova Gen3 MuJoCo/MJCF、6DoF 映射、DLS IK、Viewer、headless、
跟踪误差和有限状态验证。

- 负责在无实体机械臂时验证控制链。
- 强调模型契约、Grip 相对锚定、误差测量和回归测试。
- 不把仿真通过描述为实机通过。

### `operating-kinova-gen3-kortex`

触发于 Kortex Python API/wheel、固件兼容、有线 TCP 10000、Gen3 实体
6DoF Twist、Stop/watchdog、`MOVE` 门控和实机验收。

- 固化 wheel/固件/Python 匹配、环境密码、低速启动和实体急停边界。
- 禁止自动 Home、绕过门控、用软件 watchdog 替代物理安全层。
- 将“离线验证”和“实体验证”明确分开。

### `publishing-robot-teleop-project`

触发于将机器人遥操作项目打包、写 README、发布 GitHub 分支/Draft PR、
提供下载方式和组织实机验证。

- 核对许可证、来源、秘密、测试证据和可复现命令。
- 默认在实机验证前保留稳定主线，用公开分支和 Draft PR 收集验证。
- 不把存在冲突的 PR 或未验证代码描述为已可合并。

## 内容结构

每个 Skill 包含：

- `SKILL.md`：小于 500 词的核心决策和流程。
- `agents/openai.yaml`：显示名、简短描述和默认提示。
- `references/`：只放详细命令、检查表和安全契约；由 Skill 按需读取。

不创建自动连接或运动脚本。硬件操作的关键风险在环境、状态和现场判断，
脚本化会增加误触发；只保留可复制的只读检查和显式人工确认命令。

## 验证策略

四个 Skill 逐个部署：

1. 用不加载 Skill 的新代理执行典型任务，记录缺漏作为 RED。
2. 使用官方 `init_skill.py` 初始化。
3. 编写最小 Skill 和参考文件。
4. 运行 `quick_validate.py` 并扫描占位符。
5. 用新代理显式加载 Skill 执行同类任务，验证 GREEN。
6. 审查输出是否越权、遗漏安全门或混淆仿真/实体。
7. 当前 Skill 通过后才创建下一个。

安装完成后通过目录扫描和元数据校验确认 Codex 可发现它们。若当前会话的
Skill 列表不会热刷新，明确提示用户在新对话中自动加载。
