# PICO 左手柄遥操作 Kinova Gen3（MuJoCo 仿真 + 门禁式 Kortex 实体后端）

本项目在 **Windows 原生 Python** 中接收 PICO 左手柄位姿，通过局域网
UDP 自动发现链路控制 MuJoCo 中的 Kinova Gen3 7DoF 模型。首次构建并安装
PICO 应用、之后日常启动分别使用：

```powershell
$unityPath = 'C:\Program Files\Unity\Hub\Editor\2022.3.62f3c1\Editor\Unity.exe'
.\scripts\build_pico_udp_bridge.ps1 -UnityPath $unityPath -Install
.\scripts\start_pico_udp_teleop.ps1
```

> **安全边界：** 默认命令只控制
> `kinova_gen3_mujoco/teleop_scene.xml` 仿真，不连接、初始化或命令任何
> Kinova 实体机器人。仓库包含一个门禁式 Kortex 实体后端：它**未在真机
> 验证**，且只有同时给出 `--backend kortex --enable-hardware`、设置
> `KINOVA_PASSWORD` 并在终端精确输入 `MOVE` 后才会连接实体机械臂。

详细的首次配置、PICO 手动启动、健康检查和排障步骤见
[PICO UDP 快速开始](docs/pico-udp-quickstart.md)；实体模式的前置条件、
安全行为和首次验收清单见
[Kortex 实体模式快速开始](docs/kortex-hardware-quickstart.md)。

## 推荐链路和操作语义

```text
PICO 左手柄
  -> PICO Unity OpenXR bridge
  -> UDP 15031 自动 DISCOVER/READY（不配置固定 Windows IP）
  -> PicoUdpInput
  -> Grip 安全离合与相对 6DoF 映射
  -> EndEffectorTargetBackend
  -> MuJoCoBackend（默认）或 KortexBackend（显式门禁）
  -> Kinova Gen3 MJCF 仿真 / 实体机械臂
```

- 只读取左手柄；摇杆和 A/B/X/Y 不使用。MuJoCo 模式下 Trigger 也不
  使用；Kortex 实体模式下可选 `--gripper`，由 Trigger 按比例驱动夹爪
  （需要发送协议 V2 的 PICO 应用，旧 V1 应用按 Trigger=0 处理）。
- 启动或链路恢复后必须先把 Grip 松到 `< 0.8`，再按到 `> 0.9` 才能运动。
- 按下时捕获手柄和仿真末端当前位姿，因此激活瞬间不跳变。
- 持续按住时映射完整相对 6DoF：平移默认缩放 `0.5`，相对旋转为 1:1。
- 松开 Grip 后保持最后目标；重新摆放手柄、再按下可从新参考位姿继续。
- 无效包、倒序/重复包、未跟踪或超过 `0.2 s` 的陈旧输入都会解除离合并
  保持目标。恢复后不能沿用仍按住的 Grip，必须再次松开再按下。

## 要求和固定版本

- Windows 10/11、PowerShell 5.1 或更高版本。
- Windows 原生 Python `>=3.10`；推荐仓库根目录的 `.venv`。
- Unity `2022.3.62f3c1`，包含 Android Build Support、SDK/NDK 和 OpenJDK。
- PICO Unity OpenXR SDK 固定在提交
  `3aa3e62bff41df618529eeb60ff02c29a515dafe`（release 1.4.0）。
- PICO OS `5.13.0` 或更高版本；已验证目标为 A9210、Android 14。
- PICO 和 Windows 位于允许 UDP 广播/单播互通的同一 LAN/VLAN。
- 可选 ADB 默认路径为 `C:\adb\adb.exe`；没有 ADB 也可在头显中手动启动。

推荐路径不需要 WSL、XRoboToolkit PC Service 或
`xrobotoolkit_sdk`。

## Windows Python 环境

在仓库根目录运行：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
```

先验证 MuJoCo、Gen3 模型和控制安全性：

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --dry-run --headless --steps 5000
```

成功的 dry-run 以此行结束：

```text
completed steps=5000 finite_state=true
```

## 构建、安装和日常启动

首次构建/安装：

```powershell
$unityPath = 'C:\Program Files\Unity\Hub\Editor\2022.3.62f3c1\Editor\Unity.exe'
.\scripts\build_pico_udp_bridge.ps1 -UnityPath $unityPath -Install
```

脚本先运行 Unity EditMode 测试，再构建 ARM64/IL2CPP APK，复制到被 Git
忽略的 `artifacts/kinova-pico-udp-bridge.apk`，最后只向已授权的 USB
设备安装。若连接了多个已授权设备，请显式指定：

```powershell
.\scripts\build_pico_udp_bridge.ps1 -UnityPath $unityPath -Install `
  -DeviceSerial YOUR_PICO_SERIAL
```

日常使用：

```powershell
.\scripts\start_pico_udp_teleop.ps1
```

脚本在检测到唯一 PICO USB 设备时自动启动
`com.yezqin.kinovapicobridge`，随后要求 20 个新鲜、tracked 的样本和至少
一次 Grip `< 0.8`，门禁通过后才打开 MuJoCo Viewer。若 ADB 不可用，请在
PICO 应用库中手动打开 **Kinova PICO Bridge**，再运行：

```powershell
.\scripts\start_pico_udp_teleop.ps1 -ManualPicoStart
```

关闭 Viewer 或按 `Ctrl+C` 退出。程序会释放 UDP 和 MuJoCo 资源。

## 输入健康检查和比例调节

只检查 100 个 PICO 数据样本而不加载 MuJoCo：

```powershell
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --input pico-udp --check-input --samples 100 --check-timeout 60
```

每行会显示 `left position`、`quat_xyzw`、`grip` 和递增的
`timestamp_ns`。移动、旋转左手柄并按下/松开 Grip 时，对应数值应变化；
四元数应保持有限且归一化。输入检查必须看到一次 Grip `< 0.8` 才会成功。

若仿真末端平移过大，降低比例：

```powershell
.\scripts\start_pico_udp_teleop.ps1 -Scale 0.25
```

`-Scale` 只缩放平移；姿态仍使用完整相对旋转。

## 网络、跟踪和安全排障

- 首次出现 Windows 防火墙提示时，只在受信任的专用网络允许 Python。
  Windows 必须允许 Python 接收 UDP `15031`。
- PICO 与 PC 必须在同一 LAN/VLAN；关闭会拦截局域网的 VPN，避免访客
  Wi-Fi、AP/client isolation 或不同 VLAN。
- 不配置 PICO/PC 固定 IP：PICO 广播 `KINOVA_DISCOVER_V1`，Windows 在
  UDP `15031` 回应 `KINOVA_READY_V1`，随后锁定第一个有效数据源。
- 超时通常表示应用未启动、左手柄休眠/未跟踪、防火墙阻止 UDP 或网络隔离。
- 若 PICO 仍停留在旧版 controller bridge 或系统手柄/HandDialog，ADB
  启动意图可能被当前 VR 应用拦截。退出旧应用并关闭系统对话框，在头显中
  手动打开 **Kinova PICO Bridge**，然后重新运行
  `.\scripts\start_pico_udp_teleop.ps1 -ManualPicoStart`。
- `Grip release` 错误表示检查期间没有观察到 `< 0.8`；完全松开 Grip 后重试。
- `stale=true`、`untracked`、非法/倒序数据或网络断开都会保持最后目标；
  链路恢复后先松开再按下 Grip。
- 支持的日常恢复流程是：结束当前 Windows 控制进程并退出头显应用；下次重新
  打开两端后，从 Grip 完全松开的新会话开始。不要把“应用重启后继续保持
  Grip 按下并无缝接管上一会话”作为操作流程。
- UDP `15031` 已被占用时，退出占用进程后重试；程序不会切换随机端口。

脚本不会自动创建永久防火墙规则，也不会使用 ADB Reverse、`adb tcpip`
或 `adb connect`。

## 模型和后端边界

本项目固定使用 `kinova_gen3_mujoco/` 中的 Gen3 7DoF MJCF。
旧工作区中的 `kinova/kinova.urdf` 实际描述 JACO2 J2S6S200，不得作为
Gen3 模型使用。

`EndEffectorTargetBackend` 把控制逻辑与具体后端分离。`MuJoCoBackend`
是默认后端；`KortexBackend` 是门禁式实体后端，内建 0.03 m/s、5 deg/s
硬限速、0.2 s 命令看门狗、Stop 级联和启动期 `ClearFaults` 故障恢复，
但**尚未在真机验证**。启用条件、默认参数和首次验收清单见
[Kortex 实体模式快速开始](docs/kortex-hardware-quickstart.md)；不满足
其中安全条件时，不要使用实体模式。

## XRoboToolkit 兼容入口（非推荐）

`--input xrobotoolkit` 仅为旧环境保留。它依赖 WSL、XRoboToolkit PC
Service 和 `xrobotoolkit_sdk`，不属于本项目推荐或验收路径。新部署请使用
默认的 Windows 原生 `pico-udp` 链路。

## 开发验证

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --dry-run --headless --steps 5000
git diff --check
```

生成的 Unity `Library/`、`Temp/`、`Logs/`、`UserSettings/`、APK 和
`artifacts/` 均不应提交。
## 已有本地环境：直接启动并控制 Kinova Gen3

以下命令均从仓库根目录的 **Windows PowerShell** 运行。该项目仅在 Windows 原生 Python
中工作；不需要 WSL、XRoboToolkit PC Service，也不使用固定 IP。PICO 和 PC 必须在允许 UDP
广播/单播的同一 LAN/VLAN 上，Windows 仅需允许 Python 接收 UDP `15031`。

如当前环境尚未安装项目依赖，先执行一次：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
```

首次构建并安装 PICO APK 前，将 `$unityPath` 改为本机安装的 **Unity 2022.3.62f3c1**
`Unity.exe` 路径；连接并授权唯一的 PICO USB 设备后执行：

```powershell
$unityPath = 'C:\Program Files\Unity\Hub\Editor\2022.3.62f3c1\Editor\Unity.exe'
.\scripts\build_pico_udp_bridge.ps1 -UnityPath $unityPath -Install
```

日常使用时，左手柄唤醒并处于 tracked 状态后，先**完全松开左手 Grip**，再启动：

```powershell
.\scripts\start_pico_udp_teleop.ps1
```

脚本会在 ADB 可用且只发现一个已授权 PICO USB 设备时自动启动 `Kinova PICO Bridge`，随后进行
UDP 输入预检，确认至少一次 Grip `< 0.8` 后才打开 MuJoCo Viewer。若没有 ADB、自动启动被 VR
前台应用拦截，或你已在头显中手动打开应用，请使用手动回退：

```powershell
.\scripts\start_pico_udp_teleop.ps1 -ManualPicoStart
```

只检查输入而不加载 MuJoCo：

```powershell
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --input pico-udp --check-input --samples 100 --check-timeout 60
```

平移幅度过大时调小比例（姿态仍为完整的相对 6DoF 旋转）：

```powershell
.\scripts\start_pico_udp_teleop.ps1 -Scale 0.25
```

控制期间始终使用**左手 Grip**：启动、断链或应用重启后必须先释放到 `< 0.8`，再按下到 `> 0.9`；
不能保持按住 Grip 跨会话接管。关闭 MuJoCo Viewer 或按 `Ctrl+C` 即可停止，程序会释放 UDP 和
MuJoCo 资源。

即使日常口头称其为“机械臂 URDF”，本控制实现实际加载的是
`kinova_gen3_mujoco/teleop_scene.xml`：其中为 MuJoCo 生成/整理的 Kinova Gen3 7DoF **MJCF**
模型。它不是旧目录 `kinova/kinova.urdf` 所描述的 JACO2 J2S6S200，后者不得作为 Gen3 使用。
以上默认命令仅控制该 MuJoCo 仿真，绝不连接、初始化或命令任何实体 Kinova 机器人；
实体模式必须显式走 [Kortex 实体模式快速开始](docs/kortex-hardware-quickstart.md) 的门禁流程。
