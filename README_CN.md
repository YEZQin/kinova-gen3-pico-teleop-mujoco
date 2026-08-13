# PICO 到 Kinova Gen3 的受控遥操作

[English](README.md) | [证据等级](docs/evidence-levels.md) | [PICO 详情](docs/pico-udp-quickstart.md) | [Kortex 详情](docs/kortex-hardware-quickstart.md)

本公开流程从一台干净的 Windows 电脑开始，逐步到达经过明确门禁的首次运动流程。支持对象为 **Kinova Gen3 L53、7 自由度、固件 2.8.0-5**，搭配 **Kortex 2.8 / `kortex_api` 2.8.0.post5** 和 PICO 4 或 PICO 4 Ultra 左手柄。默认且推荐的路径是 MuJoCo。

> **安全边界。** 本项目不是安全认证系统、生产安全系统、无人值守系统，也不宣称所有硬件均已验证。首次运动必须有可触及的物理急停/Web Stop 和第二观察员。Stop RPC 返回并不等于机械臂已经物理静止。公开首跑仅允许平移：不控制视觉模块，也不写夹爪命令。最终扩展/非对称配置仅为 **offline verified and not hardware-validated**，不得作相反宣称。

## 链路与约束

`PICO 左手柄 → Unity OpenXR Bridge → UDP 15031 发现 → Python 准入/Grip 离合 → MuJoCo（默认）或显式门禁的 Kortex Cartesian Twist`。

Grip 是离合器：先释放到低于 `0.8`，再按到高于 `0.9` 才会建立新锚点。受控硬件模式为 40 Hz，陈旧输入超时不超过 `0.2 s`，角速度上限 `2 deg/s`，你选择的线速度不超过 `0.02 m/s`。陈旧/非法/换源输入、watchdog、工作区或 RPC 故障都会锁存 Stop；必须现场排除后才能重新运行。

## 开始前

请使用原生 Windows 10/11 PowerShell 5.1+（不使用 WSL）、Git、CPython **3.11.x**，以及用于 USB 安装的 Android platform tools/ADB。若选择从源码构建 APK，请通过 Unity Hub 安装 Unity `2022.3.62f3c1`，并勾选 Android Build Support、SDK/NDK 和 OpenJDK。桥接源码固定 PICO OpenXR SDK `3aa3e62bff41df618529eeb60ff02c29a515dafe`。

APK 和 Kortex wheel 是带 SHA-256 校验的 `v0.2.0-rc.1` 发布资产，不是仓库二进制文件。发布存在前，不得替换文件或绕过校验。[release/public-release-assets.json](release/public-release-assets.json) 跟踪其预期名称、URL、哈希和大小；当前仓库的 APK 声明是发布占位，不能安装。

将头显和 PC 放入同一受信任 LAN/VLAN；不要设置固定 PICO 地址。如果 Windows Firewall 提示，仅在受信任网络配置文件中允许 Python 入站 UDP `15031`。不要创建宽泛规则、把端口暴露给公用网络、使用 `adb tcpip`、`adb connect` 或 ADB reverse；脚本不会创建防火墙规则。

## 从零到运行：严格顺序

从新的 PowerShell 窗口执行。仅将明确标出的本地变量替换为**你的**机器人测量值。绝不复制其他机器人的工作区、校准、报告、租约或证据文件。

```powershell
# LIFECYCLE: clone
git clone --branch codex/public-gen3-pico-hardware-teleop https://github.com/YEZQin/kinova-gen3-pico-teleop-mujoco.git kinova-gen3-pico-teleop
Set-Location .\kinova-gen3-pico-teleop
```

若不能使用 Git，请从同一发布分支下载 ZIP 并解压到新的短路径本地目录；不要复制开发工作区。该分支/tag 与 release 是发布目标，使用前必须确认其已存在。

```powershell
# LIFECYCLE: bootstrap
.\scripts\bootstrap_public_teleop.ps1
```

Bootstrap 验证受跟踪的发布清单、资产身份/SHA-256，创建 `.venv-kortex`，先安装 wheel 再安装项目，固定 protobuf 3.20.0，并进行导入/离线检查。它不连接机器人、不保留凭据、不修改防火墙。

```powershell
# LIFECYCLE: apk-install
.\scripts\bootstrap_public_teleop.ps1 -InstallApk
```

该操作要求恰好一台已授权 ADB 设备。也可以在安装 Unity 后本地构建 APK，并只安装到已授权设备：

```powershell
$unityPath = 'C:\Program Files\Unity\Hub\Editor\2022.3.62f3c1\Editor\Unity.exe'
.\scripts\build_pico_udp_bridge.ps1 -UnityPath $unityPath -Install
```

在头显中打开 **Kinova PICO Bridge**。若 ADB 无法启动它，请从头显应用库手动打开；不要为了让 ADB 工作而改变网络拓扑。

```powershell
# LIFECYCLE: pico-gate
.\.venv-kortex\Scripts\python.exe -m kinova_teleop.main --input pico-udp --check-input --samples 10 --check-timeout 15
```

移动处于 tracked 状态的左手柄，并在本门禁中完全释放 Grip。它必须报告新鲜样本和一次释放；该检查不会建立机器人连接。

```powershell
# LIFECYCLE: mujoco-finite
.\.venv-kortex\Scripts\python.exe -m kinova_teleop.main --dry-run --headless --steps 2000
```

仅在输出包含 `finite_state=true` 后继续。这是仿真证据，不是硬件证据。

```powershell
# LIFECYCLE: calibration
.\scripts\capture_pico_operator_calibration.ps1 -OutputPath local-config\operator-axes.json
```

在 Grip 释放的状态下依次完成 neutral/right/neutral/up/neutral/forward 提示，并确认本地校准正确对应右、上、前。它只使用 PICO UDP，绝不连接 Kortex。

```powershell
# LIFECYCLE: t0
$robotIp = Read-Host 'Private IPv4 address of this Gen3'
$robotUser = Read-Host 'Kortex user name for this Gen3'
.\scripts\prepare_gen3_hardware.ps1 -RobotIp $robotIp -RobotUser $robotUser -Output local-config\t0.json
```

最后一条命令会通过掩码密码提示完成本会话只读 T0 观察，随后清除子进程凭据。`admin` 可以是设备的用户名，绝不是密码。确认 L53/7DoF、固件 `2.8.0-5`、ready/manual-control 状态和有限 TCP 位姿；任一不符立即停止。

围绕**你的实测 T0 TCP 位姿**测量并实体检查完整扫掠体积。选择包含该点的紧凑、闭区间绝对 XYZ 边界；不得重用过宽或他人的边界。

```powershell
# LIFECYCLE: local-package
$workspaceMin = @(<your-measured-x-min-m>, <your-measured-y-min-m>, <your-measured-z-min-m>)
$workspaceMax = @(<your-measured-x-max-m>, <your-measured-y-max-m>, <your-measured-z-max-m>)
$owner = Read-Host 'Responsible operator identifier'
.\.venv-kortex\Scripts\python.exe -m kinova_teleop.public_hardware_setup package --t0 local-config\t0.json --calibration local-config\operator-axes.json --workspace-min $workspaceMin --workspace-max $workspaceMax --linear-speed 0.005 --owner $owner --output-dir local-config --workspace-clear --physical-estop-reachable --teach-pendant-stop-reachable --second-observer-present --cable-slack-checked --device-fixture-checked --speed-level-checked --workspace-bounds-checked --load-tcp-checked
```

包生成器是离线的：不接收密码，不建立 Kortex 连接。只有九项实体检查全为真时才创建新本地工件。不要覆盖旧包；需要修正时请使用新的本地配置。

```powershell
# LIFECYCLE: offline-validation
$profile = Get-Content local-config\teleop-profile.json -Raw | ConvertFrom-Json
$profileRoot = Split-Path -Parent (Resolve-Path local-config\teleop-profile.json)
& .\.venv-kortex\Scripts\python.exe -m kinova_teleop.main --backend kortex --enable-hardware --input pico-udp --robot-ip $profile.robot_ip --robot-user $profile.robot_user --workspace-min $profile.workspace_min_m --workspace-max $profile.workspace_max_m --max-linear-speed $profile.max_linear_speed_m_s --max-angular-speed-deg 2 --translation-only --expanded-translation-envelope --responsive-translation-profile --recover-stale-input --stale-timeout 0.2 --control-hz 40 --scale 0.8 --operator-calibration (Join-Path $profileRoot $profile.calibration) --motion-lease (Join-Path $profileRoot $profile.motion_lease) --preflight-report (Join-Path $profileRoot $profile.preflight_report) --run-id $profile.run_id --lease-owner $profile.lease_owner --validate-motion-package
```

该命令纯离线验证包：不读取密码，不建立 PICO 或 Kortex 连接。只有成功后才可进入下一节。

## 现场首动：九项检查、启动、离合、Stop

启动器继续前，必须实体确认包命令所表示的九项条件：工作区净空；急停可达；示教器/Web Stop 可达；第二观察员；线缆余量；设备/夹具；速度等级；已检查的边界；负载/TCP。T0 后及每次中断后均要复核。将机械臂置于安全姿态，保持 Grip 释放，按毫米尺度每次只测试一个平移轴。

```powershell
# LIFECYCLE: physical-checklist
Read-Host 'Confirm all nine onsite checks, then press Enter'
```

```powershell
# LIFECYCLE: hardware-launcher
.\scripts\start_generated_gen3_teleop.ps1 -Profile local-config\teleop-profile.json -PythonPath .\.venv-kortex\Scripts\python.exe
```

```powershell
# LIFECYCLE: grip-release
Write-Host 'Keep PICO Grip fully released (< 0.8) until the input gate succeeds.'
```

受控启动器先验证，再要求输入 `HARDWARE-READY`，然后以不回显、不持久化的方式提示密码。仅其子进程提供精确的运动确认。

```powershell
# LIFECYCLE: move
Write-Host 'Inside the guarded program, type exactly: MOVE'
```

将 Grip 按过 `0.9` 一次以锚定；第一次按下不得跳变。每次仅移动一个轴几毫米，观察方向和速度，然后松开 Grip。首跑路径刻意禁用夹爪写入和视觉控制。

```powershell
# LIFECYCLE: stop-troubleshooting
Write-Host 'Release Grip; for any unexpected motion use physical E-stop/Web Stop, then Ctrl+C.'
```

若发生陈旧输入、UDP、换源、工作区、watchdog 或 Stop 故障：保持人员远离，按需要使用实体 Stop，不要在锁存故障后重新启动，检查线缆/网络/夹具，再从适当门禁重做。PICO 故障检查头显应用是否在前台、左手柄 tracking、受信任 LAN/VLAN、VPN/AP 隔离和窄范围 UDP 15031 规则。Kortex 故障时不得随意更换固件或 SDK；确认必需 wheel/固件组合并重新执行只读 T0。

## 证据、许可证与开发

在解释任何结果前先阅读 [docs/evidence-levels.md](docs/evidence-levels.md)。它区分自动化/离线、安装现场 PICO、只读 T0 和现场观察证据。最终扩展/非对称配置仍是 offline verified and not hardware-validated。

项目自有源码采用 MIT 许可证（[LICENSE](LICENSE)）。Kortex 再分发声明、准确上游身份及 PICO/Unity 声明引用位于 [release/THIRD_PARTY_NOTICES.txt](release/THIRD_PARTY_NOTICES.txt)。未经许可证核查，不得再分发 SDK、APK、固件、日志、私有设备数据或依赖项。

仅开发检查（不运行硬件启动器）：

```powershell
.\.venv-kortex\Scripts\python.exe -m pytest -q
.\.venv-kortex\Scripts\python.exe -m kinova_teleop.main --dry-run --headless --steps 2000
git diff --check
```

简要顺序：clone → 校验 bootstrap 资产 → 安装/打开 APK → PICO gate → MuJoCo finite → calibration → read-only T0 → 本地测量/package → offline validation → 九项实体检查 → guarded launcher → release/press Grip → 精确 `MOVE` → Stop 并调查所有故障。
