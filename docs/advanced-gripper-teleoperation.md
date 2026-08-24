# Advanced PICO teleoperation with a binary Trigger gripper / 高级 PICO 二值 Trigger 夹爪遥操

This is a separate, opt-in profile for an operator who has already completed the public [zero-to-first-motion workflow](../README.md) or [中文公开首动流程](../README_CN.md). The public lifecycle and its generated launcher remain translation-only with no gripper writes. This guide adds a parameterized launcher; it does not replace any public prerequisite, package gate, onsite checklist, or Stop procedure.

本文是独立的可选高级模式。使用前必须完成公开[零到首动流程](../README.md)或[中文流程](../README_CN.md)。公开生命周期和生成的启动器仍为仅平移且不写入夹爪。本指南只增加参数化启动器，不替代任何公开前置条件、包门禁、现场检查清单或 Stop 流程。

## Evidence and safety boundary / 证据与安全边界

The tuned binary gripper profile was hardware-observed on one Gen3 L53 / PICO 4 Ultra installation at core control commit `b40eb57`; see [the preserved baseline note](hardware-observed-b40eb57.md). That observation is installation-local evidence only: it is **not safety-rated**, **not production-safe**, and **not universal hardware validation**. Keep the physical E-stop and Web Stop reachable, retain the second observer and all nine onsite checks, keep people clear, and investigate every latched fault before a new attempt. Kinova firmware limits and all operator controls remain in force.

调优后的二值夹爪模式已在一套 Gen3 L53 / PICO 4 Ultra 安装上以核心控制提交 `b40eb57` 进行现场观察；见[保留的基线记录](hardware-observed-b40eb57.md)。该观察仅是安装级证据：它**不是安全认证**、**不是生产安全声明**，也**不是通用硬件验证**。必须保持实体 E-stop 与 Web Stop 可达，保留第二观察员和全部九项现场检查，让人员远离，并在每个锁存故障后调查原因才可再次尝试。Kinova 固件限制及全部操作者控制始终有效。

For the first gripper trial, keep the gripper empty, begin at `Trigger <= 0.9`, and cross the `0.9` threshold deliberately while keeping the physical E-stop and Web Stop reachable.

首次夹爪试验保持夹爪空载，从 `Trigger <= 0.9` 开始，再有意识地越过 `0.9` 阈值，同时保持实体 E-stop 与 Web Stop 可达。

## Prerequisites and measured inputs / 前置条件与实测输入

Use artifacts produced afresh for this exact robot, calibration, package, and session. Do not reuse another robot's lease, preflight report, calibration, or bounds. Measure an inclusive absolute XYZ box around the current TCP pose after the public T0 and physical inspection. Every bound must be finite, and each minimum must be strictly less than its corresponding maximum.

使用为当前机器人、校准、包和会话新鲜生成的产物。不得复用其他机器人的 lease、预检报告、校准或边界。完成公开 T0 与实体检查后，围绕当前 TCP 位姿测量包含端点的绝对 XYZ 盒；每个数值必须有限，且每个最小值必须严格小于对应最大值。

The recommended reproducible path is the tuned session launcher below. It creates a fresh `sessions/<run>/local-config` directory, copies the current operator calibration into it, runs read-only T0, generates a fresh local package, then starts the tuned advanced launcher. This avoids stale `local-config` overwrite failures while keeping every artifact local.

推荐使用下方调优会话启动器。它会创建新的 `sessions/<run>/local-config` 目录，把当前操作者校准复制进去，执行只读 T0，生成新的本地包，然后启动调优高级遥操。这样可避免旧 `local-config` 覆盖失败，同时所有产物仍保留在本地。

```powershell
$operatorCalibration = Read-Host 'Path to the captured operator calibration'
$robotHost = Read-Host 'Private IPv4 address of this Gen3'

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

& .\scripts\start_gen3_pico_tuned_session.ps1 `
  -RobotHost $robotHost `
  -RobotUser admin `
  -OperatorCalibration $operatorCalibration `
  -WorkspaceMin $workspaceMin `
  -WorkspaceMax $workspaceMax `
  -LeaseOwner operator `
  -Scale 1.0 `
  -MaxLinearSpeed 0.05 `
  -LinearGain 1.5 `
  -TranslationAxisGain ([double[]]@(-2.0,1.0,1.0)) `
  -GripperBinaryThreshold 0.9 `
  -ConfirmPhysicalChecks `
  -PythonPath .\.venv-kortex\Scripts\python.exe
```

If you already have a fresh motion lease, reviewed preflight report, teleop profile, and calibration in the same reviewed local package, you may instead call `scripts/start_gen3_pico_teleop.ps1` directly with the same tuned parameters. The session launcher is preferred for repeatable runs because it prevents accidental reuse or overwrite of stale local artifacts.

若已拥有同一已复核本地包中的新鲜 motion lease、reviewed preflight report、teleop profile 和校准，也可以用相同调优参数直接调用 `scripts/start_gen3_pico_teleop.ps1`。为便于反复复现，优先使用会话启动器，因为它能避免误复用或覆盖旧本地产物。

The equivalent direct low-level call must remain artifact-bound with `-MotionLease`, `-PreflightReport`, `-OperatorCalibration`, `-RunId`, `-LeaseOwner`, `-WorkspaceMin`, `-WorkspaceMax`, and `-EnableGripper`, plus the same `-Scale 1.0`, `-MaxLinearSpeed 0.05`, `-LinearGain 1.5`, `-TranslationAxisGain ([double[]]@(-2.0,1.0,1.0))`, and `-GripperBinaryThreshold 0.9` settings. Do not add `-InvertTranslation` to this observed preset.

等价的低层直接调用仍必须绑定 `-MotionLease`、`-PreflightReport`、`-OperatorCalibration`、`-RunId`、`-LeaseOwner`、`-WorkspaceMin`、`-WorkspaceMax` 和 `-EnableGripper` 等本地产物，并使用相同的 `-Scale 1.0`、`-MaxLinearSpeed 0.05`、`-LinearGain 1.5`、`-TranslationAxisGain ([double[]]@(-2.0,1.0,1.0))` 与 `-GripperBinaryThreshold 0.9` 设置。不要在该已观察预设中额外加入 `-InvertTranslation`。

## Operator semantics / 操作语义

Before a password prompt, the launcher validates the artifact-bound package and requires fresh **PICO V2** samples with Trigger capability while Grip remains released. The password is masked and available only to the Python child. In that child, enter exactly one `MOVE`; do not move before that confirmation.

在密码提示前，启动器验证绑定产物的包，并要求新鲜的 **PICO V2** 采样具备 Trigger 能力且 Grip 始终释放。密码被掩码且只提供给 Python 子进程。在子进程中精确输入一次 `MOVE`；确认前不得运动。

Grip is the arm-motion clutch. Grip activation establishes an anchor frame only: it sends neither an arm motion command nor a gripper command, so it cannot create an arm jump or change the gripper. Gripper commands begin only on subsequent active samples. In the tuned binary profile, Trigger above `0.9` commands `0.99` closed; Trigger at or below `0.9` commands `0.01` open. Grip release holds the last/current gripper command and stops arm following; it is not permission to ignore any Stop or fault.

The underlying proportional endpoints remain Trigger `0.0` for open and Trigger `1.0` for closed; the tuned hardware profile deliberately reduces them to the binary threshold above.

Proportional Trigger commands begin only on subsequent active samples in the underlying mode; the tuned binary command is applied at the same post-anchor point.

`GripperTriggerMin` and `GripperTriggerMax` calibrate the observed PICO Trigger travel. Values at or below the configured minimum map to gripper position `0.01`; values at or above the configured maximum map to `0.99`; values between them map linearly. Kortex receives normalized positions, not raw percentages.

The tuned preset uses no `InvertTranslation`, `LinearGain 1.5`, and `TranslationAxisGain @(-2.0,1.0,1.0)`. `TranslationAxisGain` is applied after operator calibration; negative X reverses and boosts forward/back without changing Y/Z. `MaxLinearSpeed` remains the unchanged hard velocity cap. A changed Trigger target starts a 1.25-second burst that reasserts the current gripper position every active control cycle, then deduplicates. Status output reports `gripper_target`; evidence records `gripper_commanded` only after an actual Kortex gripper RPC succeeds.

Grip 是机械臂运动离合。激活 Grip 只建立锚点帧：它既不发送机械臂运动命令，也不发送夹爪命令，因此不会造成机械臂跳变或改变夹爪。夹爪命令只从后续有效活动采样开始。调优二值模式下，Trigger 大于 `0.9` 命令 `0.99` 闭合；Trigger 小于等于 `0.9` 命令 `0.01` 张开。松开 Grip 会停止机械臂跟随并保持最后/当前夹爪命令；这不允许忽略任何 Stop 或故障。

`GripperTriggerMin` 和 `GripperTriggerMax` 用于标定 PICO Trigger 的实测行程。小于等于最小值时映射到夹爪位置 `0.01`，大于等于最大值时映射到 `0.99`，中间线性插值。Kortex 接收归一化位置，而不是裸百分数。

调优预设不使用 `InvertTranslation`，采用 `LinearGain 1.5` 与 `TranslationAxisGain @(-2.0,1.0,1.0)`。`TranslationAxisGain` 在操作者校准后应用，负 X 会反转并增强前后而不改变 Y/Z。`MaxLinearSpeed` 仍是未改变的速度硬上限。Trigger 目标变化会启动 1.25 秒 burst，在 Grip 有效的每个控制周期重申当前夹爪位置，随后去重。终端状态显示 `gripper_target`；只有 Kortex 夹爪 RPC 实际成功后，evidence 才记录 `gripper_commanded`。

For workspace handling, each requested Cartesian target is projected into the measured XYZ box by a coordinate-wise clamp: a coordinate below its minimum becomes that minimum, a coordinate above its maximum becomes that maximum, and a coordinate already inside is unchanged. The arm follows this projected target rather than treating the software boundary as an automatic exit. This projection neither enlarges the physically inspected workspace nor overrides Kinova limits, the physical E-stop, Web Stop, or the operator's responsibility to stop.

工作区处理会将每个请求的笛卡尔目标投影到实测 XYZ 盒中：逐坐标 clamp，低于最小值的坐标变为最小值，高于最大值的坐标变为最大值，盒内坐标保持不变。机械臂跟随投影后的目标，而不是把软件边界视为自动退出条件。该投影不会扩大已实体检查的工作区，也不会覆盖 Kinova 限制、实体 E-stop、Web Stop 或操作者的停止责任。

A recoverable stale input, workspace boundary projection/contact, and shutdown/cleanup send no automatic gripper command and hold the current gripper position. No automatic open occurs in any of these cases. Resolve the condition and follow the normal admission and operator controls before another commanded motion.

可恢复的陈旧输入、工作区边界投影/接触以及关闭/清理均不发送自动夹爪命令，并保持当前夹爪位置；这些情况都不会自动张开夹爪。排除条件后，必须重新遵循正常门禁与操作者控制，才可再次执行命令运动。
