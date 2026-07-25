# PICO UDP Kinova Gen3 6DoF 遥操作设计

**日期：** 2026-07-25  
**状态：** 用户已逐节确认，等待书面规格终审

## 1. 目标

在 Windows 原生环境中，通过 PICO 左手柄控制 MuJoCo 中的 Kinova Gen3
7DoF 机械臂末端。用户按住左 Grip 后，末端跟随手柄的相对位置和相对姿态；
松开 Grip 后保持最后一个有效目标。

本次只控制 MuJoCo 仿真，不连接、不初始化也不发送任何 Kinova 实机指令。
控制器与执行环境之间建立明确的末端目标后端接口，为未来独立实现和验证
Kortex 实机后端保留边界。

## 2. 已确认的范围

- 输入设备固定为 PICO 左手柄。
- 控制量为完整 6DoF：三维位置和三维姿态。
- Grip 是唯一控制按键，只作为按住式安全离合。
- Grip 激活阈值为 `> 0.9`，释放阈值为 `< 0.8`，形成滞回以防止临界值抖动。
- Grip 激活期间使用相对位姿映射；激活瞬间目标不跳变。
- 松开 Grip 后保持目标；重新摆放手柄并再次按下时从当前末端位姿继续。
- 默认平移比例为 `0.5`；姿态采用 1:1 相对旋转。
- Trigger、摇杆、A/B/X/Y 和机械爪行为不在本次范围内。
- MuJoCo 模型固定使用 `kinova_gen3_mujoco/` 下的 Gen3 7DoF MJCF。
- `kinova/kinova.urdf` 实际描述 JACO2 J2S6S200，不用于本次 Gen3 控制。
- 主运行环境为 Windows 原生 Python/MuJoCo。
- 不依赖 WSL、XRoboToolkit PC Service 或 XRoboToolkit Python SDK。
- PICO 与 Windows 之间采用 L20 项目已验证的局域网 UDP 自动发现模式。
- 支持 ADB 自动启动 PICO 应用，也支持用户在头显中手动启动。
- 当前只实现 MuJoCo 后端；不创建可能被误认为可用的 Kortex 类骨架。

## 3. 已验证的开发环境

- PICO 型号：`A9210`，Android 14。
- USB ADB 序列号：`PA921BMGK7300311G`。
- ADB：`C:\adb\adb.exe`，Platform Tools `36.0.2-14143358`。
- Unity：`2022.3.62f3c1`。
- Unity XR Management：`4.4.0`。
- Unity OpenXR：`1.8.2`。
- PICO Unity OpenXR SDK：
  `3aa3e62bff41df618529eeb60ff02c29a515dafe`（release 1.4.0）。
- L20 项目的 PICO 协议、UDP 接收、文档和 Unity 桥契约测试共 35 项已通过。

固定 SDK 版本要求 PICO OS 5.13.0 或更高版本。版本固定以复用 L20
已验证组合，不在本次迁移中升级依赖。

## 4. 方案选择

### 4.1 采用方案

在 Kinova 项目中新建专用 PICO Unity 桥，复制并适配 L20 的 UDP
广播发现、READY 应答、数据发送、暂停恢复和健康检查结构。PICO 数据载荷
由 L20 的手部关节改为左手柄位置、四元数、tracking 状态和 Grip。

PICO 端、协议和 Windows 接收器属于 Kinova 项目，不修改
`D:\yezq\26.7.11_L20_controll`，也不与 L20 APK 共用包名或 UDP 端口。

### 4.2 不采用方案

- 不直接扩展 L20 APK：避免 Kinova 改动影响已稳定的 L20 项目。
- 不继续使用 XRoboToolkit 主链路：它引入 PC Service、原生 Python
  扩展和跨 Windows/WSL 配置，故障面比 L20 UDP 链路更大。
- 不采用 ADB Reverse 或自定义 USB TCP 作为主链路：用户已明确要求优先
  复用无已知故障的 L20 方法。
- 不同时维护 USB、XRoboToolkit 和 UDP 自动故障切换：多传输状态组合会
  增加未验证分支，与本次稳定性优先目标冲突。

旧输入代码暂时保留为显式兼容入口，README 和日常启动脚本只推荐新 UDP
链路。新链路通过真机验收后，再单独决定是否删除旧实现。

## 5. 总体架构

```text
PICO 左手柄
  -> Unity InputDevices + 固定版本 PICO OpenXR
  -> Kinova PICO UDP Bridge
  -> L20 式广播发现 / READY 应答
  -> Windows UDP 接收与协议校验
  -> ControllerSample
  -> Grip 相对离合 + 坐标转换 + 平滑 / 限幅
  -> 6DoF 末端目标
  -> EndEffectorTargetBackend
  -> MuJoCoBackend
  -> Gen3 Jacobian DLS IK + 七个位置执行器
```

边界约束：

1. PICO Unity 应用只采样左手柄和发送数据，不包含机器人控制逻辑。
2. UDP 层只负责发现、协议、顺序、时效和数值校验。
3. Grip 离合及坐标映射不依赖 UDP、Unity 或 MuJoCo。
4. 主控制器只生成末端目标，不直接访问 MuJoCo 关节数组。
5. 后端负责执行末端目标、保持、推进自身时钟和释放资源。

## 6. PICO Unity 应用

新 Unity 项目位于 `pico_unity_bridge/`，使用独立 Android 包名
`com.yezqin.kinovapicobridge`。

应用固定读取 `XRNode.LeftHand` 的以下 Unity XR 特征：

- `CommonUsages.isTracked`
- `CommonUsages.devicePosition`
- `CommonUsages.deviceRotation`
- `CommonUsages.grip`

XR API 只在 Unity 主线程的 `Update()` 中调用。任一必需特征不可用时发送
`tracked=false`，位置置零、四元数置为单位四元数、Grip 置零。

应用复用 L20 的非阻塞 UDP 结构：

- 启动后绑定一个临时本地 UDP 端口并允许广播。
- 未发现 Windows 时每秒广播一次发现消息。
- 收到 READY 后保存应答源地址，并向该地址持续单播手柄包。
- 发送错误、应用恢复或网络变化后清除目标并重新发现。
- 每帧限制处理的 READY 数据报数量，避免异常流量阻塞 Unity 主线程。
- `OnApplicationPause`、`OnDisable`、`OnDestroy` 和
  `OnApplicationQuit` 都安全停止网络资源。

头显内显示只读状态：发现状态、Windows 地址、tracking、Grip、序号和
发送频率。不增加交互式配置页面，也不在 APK 中保存固定 Windows IP。

## 7. UDP 自动发现与二进制协议

### 7.1 自动发现

- Windows/PICO 协议端口：UDP `15031`。
- 发现消息：ASCII `KINOVA_DISCOVER_V1`。
- 应答消息：ASCII `KINOVA_READY_V1`。
- Windows 接收器绑定 `0.0.0.0:15031`。
- PICO 向 `255.255.255.255:15031` 广播发现消息。
- Windows 把 READY 应答到发现消息的源地址和源端口。
- PICO 使用收到 READY 的源地址作为后续数据目标。

接收器锁定第一个产生有效数据的 PICO 源端点。活动源未超时时忽略其他源的
控制包，防止同一局域网中第二台设备抢占控制；活动源超时后允许新的发现会话
接管。发现消息始终可获得 READY 应答。

### 7.2 固定 56 字节控制包

所有多字节字段使用小端编码：

| 偏移 | 长度 | 类型 | 内容 |
|---:|---:|---|---|
| 0 | 8 | bytes | magic：`KINVPICO` |
| 8 | 1 | uint8 | version：`1` |
| 9 | 1 | uint8 | flags；bit 0 为 tracked |
| 10 | 2 | uint16 | reserved，必须为 `0` |
| 12 | 4 | uint32 | sequence |
| 16 | 8 | uint64 | PICO 单调时钟 `source_time_us` |
| 24 | 12 | 3×float32 | position `x,y,z`，单位米 |
| 36 | 16 | 4×float32 | quaternion `x,y,z,w` |
| 52 | 4 | float32 | Grip，范围 `[0,1]` |

发送端在每次 `Update()` 中递增 sequence，包括未跟踪帧。sequence
按 `uint32` 自然回绕。`source_time_us` 由 PICO 进程单调时钟产生，
不与 Windows 墙上时钟比较。

### 7.3 接收校验

接收器执行以下校验：

- 数据报长度必须恰好为 56 字节。
- magic、version、reserved 和未定义 flags 位必须匹配协议。
- position、quaternion 和 Grip 必须全部有限。
- 四元数范数必须大于最小阈值，接收后归一化。
- Grip 必须位于 `[0,1]`。
- sequence 允许向前跳跃并统计丢包；拒绝重复和倒序。
- 正确处理 `uint32` 从 `0xFFFFFFFF` 到 `0` 的回绕。
- 未跟踪包立即使输入无效。
- 使用 Windows 本地 `received_monotonic` 判断新鲜度。
- 超过 `0.2 s` 没有有效新包时输入失效。

错误包不会覆盖最后的协议统计，但会使当前控制输入失效。恢复有效数据后仍需
通过 Grip 释放门禁重新离合。

## 8. Grip 离合与 6DoF 映射

状态包括：

- `WAITING_FOR_RELEASE`：启动、失联或错误恢复后的安全状态。
- `READY`：已看到 Grip 释放，可接受下一次按下。
- `ACTIVE`：Grip 按住，持续产生相对末端目标。

主要转换：

```text
启动 / 恢复 -> WAITING_FOR_RELEASE
WAITING_FOR_RELEASE -- Grip 已释放 --> READY
READY -- Grip > 0.9 --> ACTIVE
ACTIVE -- Grip < 0.8 --> READY
ACTIVE -- stale / untracked / invalid --> WAITING_FOR_RELEASE
```

激活瞬间记录：

- PICO 左手柄参考位置和参考四元数；
- 后端报告的当前末端位置和当前末端四元数；
- 后端用于冗余关节零空间偏置的会话参考。

持续按住时：

```text
target_position =
    ee_reference_position
    + 0.5 * (mapped_controller_position - mapped_controller_reference)

controller_delta =
    mapped_controller_rotation * inverse(mapped_controller_reference_rotation)

target_rotation =
    controller_delta * ee_reference_rotation
```

沿用当前已测试的 `PICO_TO_WORLD` 基变换。四元数在每个边界归一化并统一
符号。目标继续使用现有参数：

- 低通时间常数：`0.05 s`
- 单周期最大目标平移：`0.02 m`
- 单周期最大目标旋转：`0.15 rad`

Grip 松开不会把机械臂移动回激活起点；它冻结最后一个有效目标。再次按下时
重新捕获两端参考位姿。

## 9. 末端目标后端

定义不依赖 MuJoCo 的后端协议：

```python
class EndEffectorTargetBackend(Protocol):
    def current_pose(self) -> Pose: ...
    def begin_control(self) -> None: ...
    def command_pose(self, target: Pose) -> BackendResult: ...
    def hold(self) -> None: ...
    def step(self) -> None: ...
    def close(self) -> None: ...
```

`BackendResult` 至少包含：

- `accepted`
- `converged`
- `position_error`
- `rotation_error`
- 可供日志显示的错误原因

`MuJoCoBackend` 封装：

- 已验证的 Gen3 模型契约；
- `pinch_site` 当前位姿；
- Grip 激活时的关节姿态参考；
- MuJoCo Jacobian 阻尼最小二乘 IK；
- 关节限位、连续关节邻近等效角、零空间姿态偏置；
- 七个位置执行器目标；
- 目标坐标架、仿真步进和 Viewer 生命周期。

IK 不收敛、目标不可达、误差或关节解非有限时，`command_pose` 返回拒绝，
不覆盖上一有效执行器目标。`hold()` 保持上一有效执行器目标。

本次不创建 `KortexBackend`。未来实机工作开始时，必须在独立设计与真机
验证中实现同一协议，并额外要求显式授权、工作空间限制、笛卡尔速度/加速度
限制、急停、网络看门狗、模式切换和 Kortex 错误恢复。

## 10. CLI 与运行流程

CLI 增加 `pico-udp` 输入，保留：

- `--dry-run`：确定性合成 6DoF 和 Grip 输入；
- `--input xrobotoolkit`：仅作为显式兼容入口；
- `--scale`：默认由新启动脚本传入 `0.5`；
- 有限样本输入检查模式；
- headless 和有限步数回归模式。

README 和日常脚本只推荐 `pico-udp`。

### 10.1 一次性构建/安装

`scripts/build_pico_udp_bridge.ps1`：

1. 验证固定 Unity 可执行文件存在。
2. 通过 Unity batchmode 解析固定 SDK 依赖。
3. 运行 Unity EditMode 测试。
4. 构建 Android ARM64/IL2CPP APK。
5. 输出 APK 和日志到被 Git 忽略的 `artifacts/`。
6. 可选通过已授权 ADB 安装 APK。

### 10.2 日常启动

`scripts/start_pico_udp_teleop.ps1`：

1. 启动或调用 Windows 原生 Python 输入健康检查。
2. 若检测到一台已授权 PICO USB，自动启动
   `com.yezqin.kinovapicobridge`。
3. 若 ADB 不可用，明确提示用户在头显中手动启动，并继续等待 UDP。
4. 要求至少 20 个 sequence 前进、tracked 且数值有效的左手柄样本。
5. 要求至少观察到一次 Grip 释放。
6. 健康门禁通过后启动 MuJoCo Viewer：

   ```powershell
   python -m kinova_teleop.main --input pico-udp --scale 0.5
   ```

7. Ctrl+C 或 Viewer 关闭后释放接收器和 MuJoCo 资源，不停止用户手动启动的
   PICO 应用。

## 11. 故障处理

- 未收到发现包：提示检查同一 Wi-Fi/VLAN、访客网络隔离、VPN 和 Windows
  防火墙。
- 收到发现但无有效位姿：提示唤醒并拿起左手柄。
- 启动时 Grip 已按住：持续等待释放，不打开运动离合。
- 数据包非法、倒序、未跟踪或陈旧：输入失效、离合释放、后端保持。
- 网络恢复：进入 `WAITING_FOR_RELEASE`，不会自动恢复旧控制会话。
- Unity 应用暂停或网络错误：PICO 关闭旧端点并重新发现。
- IK 不收敛或目标不可达：保持最后有效关节目标，并报告误差。
- Windows 端口被占用：启动失败并显示占用端口，不切换到随机端口。
- 默认不自动创建永久 Windows 防火墙规则；脚本只诊断并给出明确的管理员
  修复命令。
- 不把本机 IP、ADB 序列号、Unity 日志、APK 或运行状态文件提交到 Git。

## 12. 测试策略

### 12.1 Python 自动化

- 协议固定长度、字段布局、错误长度、magic/version/flags/reserved。
- 非有限位置、Grip、零四元数、Grip 越界。
- sequence 前进、间隙、重复、倒序和 `uint32` 回绕。
- 发现/READY、活动源锁定、活动源超时后切换。
- 未跟踪、无效包、断流和 `0.2 s` 看门狗。
- 启动先释放、按下无跳变、相对 6DoF、释放保持、恢复重新离合。
- 假后端验证 `current_pose`、`begin_control`、`command_pose`、`hold`、
  `step` 和 `close` 调用契约。
- MuJoCo 后端模型、IK、不可达保持和有限状态。
- CLI 输入选择、检查模式、dry-run 和资源清理。
- 全部现有测试继续通过。

### 12.2 Unity 自动化

- C# 编码结果与 Python 固定二进制 fixture 完全一致。
- sequence、单调时间戳、xyzw 四元数和 Grip 范围。
- tracked/untracked 编码。
- 发现间隔、READY 源锁定、发送失败后重新发现。
- 单帧 READY 处理上限及重复关闭安全。
- Unity batchmode EditMode 测试和 Android APK 构建。

### 12.3 PowerShell 自动化

- 固定 Unity 和 SDK 版本。
- ADB 自动启动与无 ADB 手动启动分支。
- MuJoCo 启动前必须通过 20 帧和 Grip 释放门禁。
- 脚本不调用 XRoboToolkit、WSL、ADB Reverse、`adb tcpip` 或
  `adb connect`。
- 构建和运行产物均位于忽略跟踪目录。

## 13. 真机与端到端验收

在 PICO A9210 上完成：

1. Unity 批量构建 APK 并通过 ADB 安装。
2. PICO 应用自动发现 Windows。
3. 连续接收至少 100 个有效包。
4. 人工移动和旋转左手柄，确认位置和四元数均变化。
5. 按压和释放 Grip，确认值跨越离合阈值并回落。
6. MuJoCo 中未按 Grip 时机械臂不移动。
7. 按住 Grip 后末端完整相对 6DoF 跟随。
8. 松开 Grip 后保持最后目标。
9. 暂停应用或中断网络后保持，不产生目标跳变。
10. 恢复后只有重新释放并按下 Grip 才恢复控制。
11. headless 长时间运行中 `qpos`、`qvel` 和 `ctrl` 始终有限。

需要用户物理移动手柄和按压 Grip 的验收步骤由脚本提供实时数值与明确提示，
不会用静态或合成输入冒充真机交互验证。

## 14. 完成标准

- L20 项目保持未修改。
- 新 PICO APK 使用固定、已验证的 Unity/PICO OpenXR 版本。
- PICO 与 Windows 通过自动发现 UDP 链路传输左手柄完整位姿和 Grip。
- Grip 释放门禁、相对离合、失联保持和恢复重离合均有自动化测试。
- MuJoCo Gen3 7DoF 末端可由左手柄完整 6DoF 控制。
- 后端接口不依赖 MuJoCo，当前只有经过测试的 MuJoCo 实现。
- 一次性构建/安装和日常启动均有 PowerShell 入口。
- 自动化测试、Unity APK 构建、PICO 真机数据检查和 MuJoCo 人工验收均有
  明确结果记录。
- 文档明确声明实体 Kinova 未实现、未验证且不会接收命令。

## 15. 参考

- PICO 官方支持示例索引：
  <https://github.com/picoxr/support>
- 固定的 PICO Unity OpenXR SDK：
  <https://github.com/Pico-Developer/PICO-Unity-OpenXR-SDK/tree/3aa3e62bff41df618529eeb60ff02c29a515dafe>
- 已验证的 L20 参考项目：
  `D:\yezq\26.7.11_L20_controll`
