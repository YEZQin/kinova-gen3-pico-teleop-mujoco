# PICO XRoboToolkit USB/ADB 与 Wi‑Fi 自动切换设计

## 1. 目标

为 Kinova Gen3 MuJoCo 遥操项目增加一套 Windows 侧连接管理器，使 PICO 头显在 USB 接入时优先通过 ADB Reverse 隧道传输 XRoboToolkit 数据，并在 USB 拔出后继续通过无线 ADB 连接传输，避免用户手动修改地址或重新配对。

同时将 `C:\adb` 加入当前 Windows 用户的 `PATH`，让新终端可以直接执行 `adb`。

本设计控制的是以下完整链路：

```text
PICO 左手柄
  → com.xrobotoolkit.client
  → PICO 127.0.0.1:12345 / :13579
  → ADB Reverse（USB 或无线 ADB transport）
  → Windows XRoboToolkit PC Service
  → Windows gRPC :60061
  → WSL xrobotoolkit_sdk
  → MuJoCo Kinova Gen3
```

## 2. 已验证的本机条件

- ADB 路径：`C:\adb\adb.exe`
- ADB 版本：`1.0.41`，Platform Tools `36.0.2-14143358`
- PICO 型号：A9210
- USB 序列号：`PA921BMGK7300311G`
- USB ADB 已授权
- 当前 PICO Wi‑Fi 地址：`192.168.1.4`
- 官方应用：`com.xrobotoolkit.client` v1.1.1
- 备用应用：`com.xrobotoolkit.pico.tcp1` v1.1.3，APK 中含硬编码地址 `192.168.1.104`
- 旧自定义应用：`com.yezqin.l20picobridge` v1.0，不作为 XRoboToolkit PC Service 协议替代品
- 官方客户端 APK 显示数据 TCP 端口 `12345`，命令/流控制端口 `13579`
- WSL 使用 Ubuntu 22.04；MuJoCo 和项目测试环境已存在

PICO 的 Wi‑Fi 地址只能作为运行时发现结果，不能硬编码进项目。

## 3. 方案选择

### 3.1 采用方案

使用官方 `com.xrobotoolkit.client`，将其 PC 地址一次性设置为 `127.0.0.1`。Windows 连接管理器负责：

1. 发现已授权的 USB PICO；
2. 读取 `wlan0` IPv4 地址；
3. 通过 USB 执行 `adb tcpip 5555`；
4. 建立 `<PICO_IP>:5555` 无线 ADB transport；
5. 分别在当前可用的 USB与无线 transport 上配置：
   - `reverse tcp:12345 tcp:12345`
   - `reverse tcp:13579 tcp:13579`
6. 持续监控 transport；
7. USB 拔出后保留或重建无线 transport 上的 Reverse 规则；
8. USB 再插入时恢复双 transport 准备状态。

官方客户端始终访问本机回环地址，因此 USB/Wi‑Fi 切换不会要求修改 Unity 应用中的服务器地址。

ADB 无法把已经建立的 TCP socket 从 USB transport 原地迁移到无线 transport。拔线瞬间原连接会断开，官方客户端需要在无线 Reverse 上重新连接同一个 `127.0.0.1` 端点。目标是在 5 秒内自动恢复；这段窗口由现有 XR stale 检测和执行器目标保持覆盖。若官方客户端在有界等待后没有自行重连，监控器通过无线 ADB 重新启动该应用一次。

### 3.2 不采用的方案

- 不使用 `com.xrobotoolkit.pico.tcp1`：其旧 IP 硬编码会在 DHCP 地址变化后失效。
- 不通过坐标点击自动修改 Unity UI：PICO VR 界面无法由 UIAutomator 稳定定位。
- 不新建 PICO APK：当前官方 TCP 客户端已能配合 ADB Reverse，无需维护第二套 XR 协议实现。

## 4. 组件设计

### 4.1 Windows 用户 PATH 配置

一次性 PowerShell 安装脚本读取当前用户级 `Path`：

- 若已包含规范化的 `C:\adb`，保持不变；
- 否则仅追加 `C:\adb`；
- 不覆盖系统级或用户级既有条目；
- 更新当前脚本进程的 `PATH`；
- 写入用户环境变量，使新 PowerShell、CMD 和应用可以直接调用 `adb`；
- 输出“当前终端”和“新终端”的生效说明。

不使用可能截断长 PATH 的字符串拼接式 `setx PATH ...`。

### 4.2 Python ADB 领域层

新增平台无关、可单元测试的模块，负责：

- 解析 `adb devices -l`；
- 区分 USB 序列号和 `<IPv4>:5555` 无线序列号；
- 解析 `ip -4 addr show wlan0` 与 `ip route`；
- 生成 `tcpip`、`connect`、`reverse` 和状态检查命令；
- 表示 transport 状态、Reverse 状态及错误原因；
- 拒绝 `unauthorized`、`offline`、多台 USB PICO 未指定设备等歧义状态。

Python 模块不直接决定 PowerShell 窗口生命周期，也不修改系统 PATH。

### 4.3 Windows 连接管理器

新增 PowerShell 入口，负责真实设备编排：

```powershell
.\scripts\pico_adb_transport.ps1 -Prepare
.\scripts\pico_adb_transport.ps1 -Monitor
.\scripts\pico_adb_transport.ps1 -Status
```

职责：

- 自动定位 `adb`，优先 PATH，回退 `C:\adb\adb.exe`；
- 调用 Python 领域层或稳定的命令接口；
- `-Prepare` 完成一次 USB → TCP/IP → 双 Reverse 准备；
- `-Monitor` 每秒检查 transport，只有状态变化时输出日志；
- `-Status` 只读显示 USB、无线、PICO IP、两个 Reverse 和 PC Service 端口；
- 保存最后成功的 PICO IP 到忽略跟踪的本地状态文件；
- 监控中若 Wi‑Fi IP 改变，使用 USB 重新发现并替换旧连接；
- 对 Ctrl+C 正常退出，不关闭 PC Service，不禁用头显的 ADB TCP/IP。

### 4.4 快速启动入口

新增 Windows 快速启动脚本，顺序为：

1. 检查 ADB；
2. 准备并启动 transport 监控器；
3. 启动前检查 Windows PC Service gRPC 端口 `60061`；启动 PICO 客户端后再诊断 `12345`、`13579` 的实际连接/监听状态；
4. 检查官方 PICO 客户端是否已安装，必要时通过 `am start` 启动；
5. 生成/更新 WSL 项目根目录的 `PXREASetting.ini`；
6. 先运行有限样本的 `--check-xr`；
7. 成功后启动 MuJoCo 遥操。

默认不自动启动未知路径的 PC Service 可执行文件；若用户在本地配置文件中提供其路径，脚本才可以启动它。

## 5. 传输状态机

状态定义：

- `NO_DEVICE`：无 USB 或无线 ADB；
- `USB_ONLY`：USB 已授权，无无线 transport；
- `DUAL_READY`：USB 与无线 transport 均在线，两个 transport 的 Reverse 规则已配置；
- `WIFI_ONLY`：USB 已拔出，无线 transport 在线且 Reverse 完整；
- `DEGRADED`：存在 transport，但端口规则、PC Service 或 PICO IP 不完整；
- `UNAUTHORIZED`：检测到 USB 设备但未授权；
- `AMBIGUOUS`：检测到多台 USB PICO且未指定序列号。

主要转换：

```text
NO_DEVICE → USB_ONLY → DUAL_READY → WIFI_ONLY
                         ↑              │
                         └── USB 重插 ──┘
```

任何状态下发现 Reverse 缺失都会尝试幂等重建。无线连接断开时，只有 USB 仍在线才能重新发现可能变化的 IP；两者都离线则进入 `NO_DEVICE` 并等待。

## 6. 故障与安全行为

- `adb unauthorized`：明确提示用户在头显内确认 RSA，不继续配置。
- 多台 USB Android/PICO：要求使用 `-Serial` 指定，避免修改错误设备。
- `wlan0` 无 IPv4：保持 USB 隧道可用，但标记不能拔线。
- `adb tcpip 5555` 重启 adbd 后短暂离线：有界重试，不重复无限刷屏。
- `adb connect` 失败：保留 USB Reverse，提示 Wi‑Fi 同网段和防火墙检查。
- PC Service 的 `60061` 缺失：不启动遥操并明确报错。
- `12345/13579` 的服务端行为来自已安装 PICO 客户端及 PC Service 运行时，不能只根据 APK 字符串推断全部监听时序；连接管理器始终建立两条 Reverse，但把这两个 Windows 端口作为握手后的诊断项，避免因延迟监听形成启动死锁。
- `60061` 与 `12345/13579` 分工明确：
  - `12345/13579` 是 PICO 客户端到 PC Service；
  - `60061` 是 WSL Python SDK 到 PC Service。
- 不自动开启公网防火墙规则。
- 不把 PICO IP、用户路径、SDK配置或运行日志提交到 Git。
- MuJoCo 控制器现有 stale timestamp、Grip 离合和 IK 保持策略继续作为最终运动安全层。

## 7. 用户操作

首次：

1. 运行 PATH 安装脚本；
2. 新开终端确认 `adb version`；
3. USB 接入 PICO 并确认 RSA；
4. 在官方 XRoboToolkit 客户端中把服务器地址设置为 `127.0.0.1`，数据端口 `12345`；
5. 启动 Windows PC Service；
6. 执行快速启动脚本。

日常：

1. 启动 PC Service；
2. 接入 USB；
3. 运行一个快速启动命令；
4. 状态达到 `DUAL_READY` 后可以拔线；
5. 状态应转为 `WIFI_ONLY`，XR 时间戳继续递增，MuJoCo 不中断。

## 8. 测试

### 8.1 自动化测试

- ADB devices 输出解析；
- USB/无线 transport 分类；
- unauthorized/offline/多设备；
- `wlan0` IPv4 解析；
- Reverse 规则解析；
- 状态机转换；
- 命令幂等性；
- 端口缺失诊断；
- CLI dry-run 使用假 ADB runner，不接触真实设备；
- 现有 30 项 MuJoCo 测试继续通过。

### 8.2 本机验收

在当前 PICO A9210 上验证：

1. `adb` 在新 PowerShell 中可直接执行；
2. USB 接入后识别 `PA921BMGK7300311G`；
3. 自动发现 PICO IP，不依赖固定 `192.168.1.4`；
4. USB 和 `<IP>:5555` 同时在线；
5. 两个 transport 都存在 `12345`、`13579` Reverse；
6. 官方客户端连接 `127.0.0.1` 后，`--check-xr` 时间戳递增；
7. 拔掉 USB，ADB 列表只保留无线 transport；
8. Reverse 规则仍存在或被监控器重建；
9. `--check-xr` 时间戳在不超过 5 秒的切换窗口后恢复递增；
10. 切换窗口内 MuJoCo 保持上一有效目标，随后左手柄 6DoF 遥操自动恢复；
11. USB 重插后回到 `DUAL_READY`。

## 9. 完成标准

- 用户 PATH 安全追加 `C:\adb`；
- 项目提供可测试的 ADB transport 管理逻辑；
- 官方 PICO 客户端无需在 USB/Wi‑Fi 间修改 IP；
- 接线后可一条命令准备遥操；
- 拔线后无线 ADB Reverse 自动接管；
- 连接失败时机械臂保持且错误可操作；
- README 提供首次配置、日常启动、拔线验收和恢复方法；
- Windows 和 WSL 自动化测试全部通过。
