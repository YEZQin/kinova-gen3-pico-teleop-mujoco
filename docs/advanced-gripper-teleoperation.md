# Advanced PICO teleoperation with a proportional gripper / 高级 PICO 比例夹爪遥操

This is a separate, opt-in profile for an operator who has already completed the public [zero-to-first-motion workflow](../README.md) or [中文公开首动流程](../README_CN.md). The public lifecycle and its generated launcher remain translation-only with no gripper writes. This guide adds a parameterized launcher; it does not replace any public prerequisite, package gate, onsite checklist, or Stop procedure.

本文是独立的可选高级模式。使用前必须完成公开[零到首动流程](../README.md)或[中文流程](../README_CN.md)。公开生命周期和生成的启动器仍为仅平移且不写入夹爪。本指南只增加参数化启动器，不替代任何公开前置条件、包门禁、现场检查清单或 Stop 流程。

## Evidence and safety boundary / 证据与安全边界

The advanced profile is documented and automatically checked, but is **not hardware-observed** and **not hardware-validated**. It is not a claim of safe physical operation. Keep the physical E-stop and Web Stop reachable, retain the second observer and all nine onsite checks, keep people clear, and investigate every latched fault before a new attempt. Kinova firmware limits and all operator controls remain in force.

该高级模式已有文档与自动化检查，但**尚未完成硬件现场观察**，也**未完成硬件验证**；这不代表可安全进行实体运行。必须保持实体 E-stop 与 Web Stop 可达，保留第二观察员和全部九项现场检查，让人员远离，并在每个锁存故障后调查原因才可再次尝试。Kinova 固件限制及全部操作者控制始终有效。

For the first gripper trial, use no payload, begin with a small Trigger value, and keep the physical E-stop and Web Stop reachable.

首次夹爪试验必须空载，从较小的 Trigger 值开始，并保持实体 E-stop 与 Web Stop 可达。

## Prerequisites and measured inputs / 前置条件与实测输入

Use artifacts produced afresh for this exact robot, calibration, package, and session. Do not reuse another robot's lease, preflight report, calibration, or bounds. Measure an inclusive absolute XYZ box around the current TCP pose after the public T0 and physical inspection. Every bound must be finite, and each minimum must be strictly less than its corresponding maximum.

使用为当前机器人、校准、包和会话新鲜生成的产物。不得复用其他机器人的 lease、预检报告、校准或边界。完成公开 T0 与实体检查后，围绕当前 TCP 位姿测量包含端点的绝对 XYZ 盒；每个数值必须有限，且每个最小值必须严格小于对应最大值。

Enter the artifact paths and the measured values when prompted. The variables keep the launch command bound to the operator's local artifacts rather than reusable example paths.

在提示时输入产物路径和实测数值。变量使启动命令绑定到操作者本地的产物，而不是复用示例路径。

```powershell
$robotHost = Read-Host 'Private IPv4 address of this Gen3'
$robotUser = Read-Host 'Kortex user name for this Gen3'
$motionLease = Read-Host 'Path to the fresh local motion-lease artifact'
$preflightReport = Read-Host 'Path to the fresh local reviewed preflight report'
$operatorCalibration = Read-Host 'Path to the captured operator calibration'

[double[]]$workspaceMin = @(
  [double](Read-Host 'Measured absolute X minimum (m)'),
  [double](Read-Host 'Measured absolute Y minimum (m)'),
  [double](Read-Host 'Measured absolute Z minimum (m)')
)
[double[]]$workspaceMax = @(
  [double](Read-Host 'Measured absolute X maximum (m)'),
  [double](Read-Host 'Measured absolute Y maximum (m)'),
  [double](Read-Host 'Measured absolute Z maximum (m)')
)

& .\scripts\start_gen3_pico_teleop.ps1 `
  -RobotHost $robotHost `
  -RobotUser $robotUser `
  -MotionLease $motionLease `
  -PreflightReport $preflightReport `
  -OperatorCalibration $operatorCalibration `
  -WorkspaceMin $workspaceMin `
  -WorkspaceMax $workspaceMax `
  -Scale 1.0 `
  -MaxLinearSpeed 0.05 `
  -EnableGripper `
  -PythonPath .\.venv-kortex\Scripts\python.exe
```

## Operator semantics / 操作语义

Before a password prompt, the launcher validates the artifact-bound package and requires fresh **PICO V2** samples with Trigger capability while Grip remains released. The password is masked and available only to the Python child. In that child, enter exactly one `MOVE`; do not move before that confirmation.

在密码提示前，启动器验证绑定产物的包，并要求新鲜的 **PICO V2** 采样具备 Trigger 能力且 Grip 始终释放。密码被掩码且只提供给 Python 子进程。在子进程中精确输入一次 `MOVE`；确认前不得运动。

Grip is the arm-motion clutch. Grip activation establishes an anchor frame only: it sends neither an arm motion command nor a gripper command, so it cannot create an arm jump or change the gripper. Proportional Trigger commands begin only on subsequent active samples. Then hold Grip to follow the controller. While Grip is held, the left index Trigger commands the Gen3 gripper proportionally: Trigger `0.0` requests open and Trigger `1.0` requests closed. Grip release holds the last/current gripper command and stops arm following; it is not permission to ignore any Stop or fault.

Grip 是机械臂运动离合。激活 Grip 只建立锚点帧：它既不发送机械臂运动命令，也不发送夹爪命令，因此不会造成机械臂跳变或改变夹爪。比例 Trigger 命令只从后续的有效活动采样开始。随后持续按住 Grip 才跟随控制器。按住 Grip 时，左手食指 Trigger 按比例命令 Gen3 夹爪：Trigger `0.0` 请求张开，Trigger `1.0` 请求闭合。松开 Grip 会停止机械臂跟随并保持最后/当前夹爪命令；这不允许忽略任何 Stop 或故障。

For workspace handling, each requested Cartesian target is projected into the measured XYZ box by a coordinate-wise clamp: a coordinate below its minimum becomes that minimum, a coordinate above its maximum becomes that maximum, and a coordinate already inside is unchanged. The arm follows this projected target rather than treating the software boundary as an automatic exit. This projection neither enlarges the physically inspected workspace nor overrides Kinova limits, the physical E-stop, Web Stop, or the operator's responsibility to stop.

工作区处理会将每个请求的笛卡尔目标投影到实测 XYZ 盒中：逐坐标 clamp，低于最小值的坐标变为最小值，高于最大值的坐标变为最大值，盒内坐标保持不变。机械臂跟随投影后的目标，而不是把软件边界视为自动退出条件。该投影不会扩大已实体检查的工作区，也不会覆盖 Kinova 限制、实体 E-stop、Web Stop 或操作者的停止责任。

A recoverable stale input, workspace boundary projection/contact, and shutdown/cleanup send no automatic gripper command and hold the current gripper position. No automatic open occurs in any of these cases. Resolve the condition and follow the normal admission and operator controls before another commanded motion.

可恢复的陈旧输入、工作区边界投影/接触以及关闭/清理均不发送自动夹爪命令，并保持当前夹爪位置；这些情况都不会自动张开夹爪。排除条件后，必须重新遵循正常门禁与操作者控制，才可再次执行命令运动。
