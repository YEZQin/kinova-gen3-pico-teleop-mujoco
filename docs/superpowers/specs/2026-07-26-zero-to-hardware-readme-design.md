# 从零部署 Gen3 实机控制 README 设计

日期：2026-07-26  
状态：用户已批准采用推荐环境并要求直接实施。

## 目标

让第一次接触本项目的实机验证人员，仅依靠 README 即可从一台 Windows
电脑开始，完成项目下载、PICO 链路、Kortex Python API、机械臂有线网络、
分层自检和首次低速遥操作。

## 方案选择

- **采用：WSL2 Ubuntu 22.04 + Python 3.10 统一运行。** PICO PC Service
  留在 Windows；XR Python binding、项目和 Kortex wheel 安装在同一个 WSL
  venv。这与项目当前已验证的 PICO 数据链一致，依赖和启动命令只有一套。
- 不采用 Windows 原生双 SDK：需要另一套 XR native library 构建和路径说明，
  当前项目未验证。
- 不采用容器：USB/头显、Windows PC Service、WSLg 和机器人网卡路由会增加
  排障层级。

## README 结构

在实体章节增加一条连续的“从零到首次实机控制”路径：

1. 从公开 GitHub 实机分支克隆项目。
2. 安装 WSL2 Ubuntu 22.04、Python 3.10、venv、编译工具和 `netcat`。
3. 创建统一 venv，安装项目并先完成 pytest/MuJoCo dry-run。
4. 按既有章节配置 Windows PICO PC Service 和 WSL XRoboToolkit binding。
5. 在 Web App 记录机器人固件；从 Kinova 官方渠道获取匹配固件和受支持
   Python 的 Kortex wheel，并在同一 venv 用 pip 安装。
6. 验证 Kortex 所需 import；不把官方 examples 仓库误当成可安装 SDK。
7. 使用独立有线网卡，将主机配置到机器人同一网段；从 WSL 验证 TCP 10000。
8. 依次执行 MuJoCo、XR、Kortex import/port 四层检查，任何一层失败都不得输入
   `MOVE`。
9. 完成实体急停、Web App 低速和工作空间检查，以 `0.01 m/s`、`2 deg/s`
   启动，精确输入 `MOVE`，首次左 Grip 建立锚点。
10. 提供退出、重开和常见 wheel/网络/密码/时间戳故障排查。

## 安全与准确性

- 不固定推荐某个 Kortex wheel 版本；wheel 必须与目标机器人固件和该 wheel
  支持的 Python 匹配。官方旧示例中的固定 wheel 只作为安装方法参考。
- 不在 README 中出现真实密码；只用交互式环境变量。
- 不提供自动 Home、运动型 SDK example 或绕过 `MOVE` 的验证脚本。
- TCP 可达和 import 成功不等于允许运动；首次运行仍必须由现场安全负责人执行。
- `200 ms` 仅描述主机锁存停止请求和阻断新命令，不承诺机械臂物理停止时间。

## 验证

- 检查 README 中的参数与 `python -m kinova_teleop.main --help` 一致。
- 检查所有本地路径均使用克隆目录变量，不含作者机器路径。
- 检查 GitHub、Kinova 安装文档和 Kortex examples 链接有效。
- 运行 pytest、2000 步 MuJoCo dry-run、compileall 和 `git diff --check`。
