# PICO UDP 快速开始（Windows + MuJoCo）

```powershell
$unityPath = 'C:\Program Files\Unity\Hub\Editor\2022.3.62f3c1\Editor\Unity.exe'
.\scripts\build_pico_udp_bridge.ps1 -UnityPath $unityPath -Install
.\scripts\start_pico_udp_teleop.ps1
```

第一条命令用于首次构建并安装 PICO 应用；之后通常只需第二条日常命令。
本流程仅驱动 Kinova Gen3 的 MuJoCo 仿真，不连接或命令任何实体机器人。

## 1. 一次性要求

- Windows 10/11 和 PowerShell 5.1+。
- Windows 原生 Python `>=3.10`。
- Unity `2022.3.62f3c1`，安装 Android Build Support、SDK/NDK、OpenJDK。
- PICO Unity OpenXR SDK 固定提交：
  `3aa3e62bff41df618529eeb60ff02c29a515dafe`（release 1.4.0）。
- PICO OS `5.13.0` 或更高；目标 A9210/Android 14。
- PICO 与 PC 在允许广播/单播互通的同一 LAN/VLAN。
- 可选 ADB 位于 `C:\adb\adb.exe`；没有 ADB 可手动启动应用。

推荐流程不需要 WSL、XRoboToolkit PC Service 或
`xrobotoolkit_sdk`。

创建 Windows 虚拟环境：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
```

## 2. 构建和安装 PICO 应用

连接并在头显中授权 USB 调试后运行：

```powershell
$unityPath = 'C:\Program Files\Unity\Hub\Editor\2022.3.62f3c1\Editor\Unity.exe'
.\scripts\build_pico_udp_bridge.ps1 -UnityPath $unityPath -Install
```

脚本按固定 Unity 版本运行 EditMode 测试和 Android ARM64/IL2CPP 构建，
输出 `artifacts/kinova-pico-udp-bridge.apk`，并要求恰好一个已授权 USB
设备。多个设备连接时显式限定序列号：

```powershell
.\scripts\build_pico_udp_bridge.ps1 -UnityPath $unityPath -Install `
  -DeviceSerial YOUR_PICO_SERIAL
```

只构建、不安装：

```powershell
.\scripts\build_pico_udp_bridge.ps1 -UnityPath $unityPath
```

## 3. 自动启动和手动回退

日常启动：

```powershell
.\scripts\start_pico_udp_teleop.ps1
```

若 ADB 能看到唯一 PICO USB 设备，脚本自动启动包
`com.yezqin.kinovapicobridge`。如果 ADB 缺失、离线或未连接：

1. 在 PICO 应用库中手动打开 **Kinova PICO Bridge**。
2. 在 PC 上运行：

```powershell
.\scripts\start_pico_udp_teleop.ps1 -ManualPicoStart
```

需要显式选择已授权设备时：

```powershell
.\scripts\start_pico_udp_teleop.ps1 `
  -DeviceSerial YOUR_PICO_SERIAL
```

脚本先检查 20 个不同的新鲜 tracked 样本和至少一次 Grip 释放，再启动
MuJoCo Viewer。关闭 Viewer 或按 `Ctrl+C` 可安全退出。

## 4. UDP 自动发现和防火墙

无需填写 PICO 或 Windows 固定 IP。Windows 监听
`0.0.0.0:15031/UDP`；PICO 广播 `KINOVA_DISCOVER_V1`，Windows 回复
`KINOVA_READY_V1`，之后接收器锁定第一个发送有效数据的 PICO 源。

首次出现 Windows 防火墙提示时，只在受信任的专用网络允许 Python。
若超时：

- 确认 PICO 与 PC 在同一 LAN/VLAN；
- 禁用拦截局域网的 VPN；
- 避免访客 Wi-Fi、AP/client isolation 或跨 VLAN；
- 允许 Python 入站接收 UDP `15031`；
- 确认 **Kinova PICO Bridge** 正在前台运行；
- 唤醒并拿起左手柄，确认它处于 tracked 状态。

脚本不会自动修改永久防火墙规则，也不使用 ADB Reverse、`adb tcpip`
或 `adb connect`。

若 ADB 显示启动成功，但头显仍停留在旧版 controller bridge 或系统
手柄/HandDialog，说明当前 VR 应用拦截了启动意图。退出旧应用、关闭系统
对话框，在头显应用库中手动打开 **Kinova PICO Bridge**，然后重新运行：

```powershell
.\scripts\start_pico_udp_teleop.ps1 -ManualPicoStart
```

## 5. 输入预检和预期输出

只验证 100 个真机样本，不加载 MuJoCo：

```powershell
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --input pico-udp --check-input --samples 100 --check-timeout 60
```

预期每个接受的样本打印一行：

```text
left position=[+0.12345 +1.23456 -0.34567] quat_xyzw=[+0.00000 +0.00000 +0.00000 +1.00000] grip=0.000 timestamp_ns=123456789000
```

操作左手柄时检查：

- 平移时 `position` 变化；
- 旋转时 `quat_xyzw` 变化，四元数保持有限且归一化；
- 按压 Grip 时 `grip > 0.9`，完全松开时 `grip < 0.8`；
- `timestamp_ns` 在 100 行中持续前进；
- 没有持续 duplicate/out-of-order、untracked 或 stale 失败。

预检必须看到至少一次 Grip `< 0.8`。若报
`no Grip release observed`，完全松开后重试。若超时，按上一节检查网络、
应用和左手柄 tracking。

## 6. 左 Grip 相对 6DoF 操作

1. 启动后保持 Grip 松开，直到系统看到 `< 0.8`。
2. 把左手柄移到舒适位置，再按住 Grip 到 `> 0.9`。
3. 激活瞬间同时捕获手柄与仿真末端参考位姿，不产生目标跳变。
4. 持续按住并移动、旋转左手柄：末端跟随完整相对 6DoF。
5. 默认平移比例是 `0.5`；相对旋转为 1:1。
6. 松开 Grip 到 `< 0.8`：保持最后目标，可重新摆放手柄。
7. 再次按下时，从新的两端参考位姿继续。

平移幅度过大时：

```powershell
.\scripts\start_pico_udp_teleop.ps1 -Scale 0.25
```

`-Scale` 不缩放姿态。

## 7. Stale、未跟踪和恢复门禁

重复/倒序或非法数据、`tracked=false`、应用暂停、网络断开，或超过
`0.2 s` 没有有效新包，都会使输入无效、解除 Grip 离合并保持最后目标。

网络/应用恢复时，即使 Grip 仍按住也不能运动。必须先松开到 `< 0.8`，
再按到 `> 0.9` 才能建立新会话。运行输出中的
`status: clutch=... stale=... ik_converged=...` 用于观察这些状态转换。

日常使用按“一次启动、一次稳定控制会话”操作：本次完成后关闭 Windows
控制进程和头显应用；下次重新打开两端，并保持 Grip 松开，等进入 READY
后再按下操作。支持的是这种 release-first 新会话；不要求、也不要依赖应用
热重启时保持 Grip 按下并延续上一会话。

## 8. 无头 MuJoCo 安全自检

此命令使用确定性合成输入，只加载 Gen3 仿真：

```powershell
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --dry-run --headless --steps 5000
```

成功时：

```text
completed steps=5000 finite_state=true
```

项目使用 `kinova_gen3_mujoco/teleop_scene.xml` 的 Gen3 7DoF MJCF。
旧 `kinova/kinova.urdf` 是 JACO2 J2S6S200，不可当作 Gen3 使用。

## 9. 当前限制和实体后端边界

本快速开始只覆盖默认的 `MuJoCoBackend` 仿真路径。仓库另含门禁式
`KortexBackend` 实体后端（未真机验证），启用条件、安全行为与验收清单见
[Kortex 实体模式快速开始](kortex-hardware-quickstart.md)；未满足其安全
条件时，不要使用 `--backend kortex`。

旧 `--input xrobotoolkit` 是非推荐兼容入口，需要 WSL、XRoboToolkit
PC Service 和 `xrobotoolkit_sdk`；推荐路径始终是 Windows 原生
`--input pico-udp`。
