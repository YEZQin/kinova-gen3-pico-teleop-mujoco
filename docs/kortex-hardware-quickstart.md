# Kortex 实体模式快速开始（门禁式，未真机验证）

> **安全声明：** 本分支的 Kortex 实体后端完成了设计与离线自动化测试，
> **尚未在真实机械臂上验证**。默认命令仍只启动 MuJoCo 仿真；只有具备
> 实体设备、安全条件，并接受下述验收责任的操作者，才可显式启用实体模式。
> 首次通电测试时：急停握在手中、机械臂周围留足净空、无人处于工作空间内。

## 1. 链路

```text
PICO 左手柄
  -> PICO Unity OpenXR bridge（协议 V2：位姿 + Grip + Trigger）
  -> UDP 15031 自动 DISCOVER/READY
  -> PicoUdpInput
  -> Grip 安全离合与相对 6DoF 映射
  -> KortexBackend（限速 Twist + 0.2 s 看门狗 + Stop 级联）
  -> Kinova Gen3 实体机械臂（TCP 10000）
```

操作语义与 MuJoCo 模式完全一致：先松开 Grip 到 `< 0.8`，再按到
`> 0.9` 才能运动；任何无效、陈旧、倒序输入都会解除离合并触发 Stop。

## 2. 前置条件

1. **Kortex Python wheel。** 从
   [Kinova 官方安装页](https://docs.kinovarobotics.com/getting_started/installation.html)
   或 [Kinova-kortex2_Gen3_G3L](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L)
   获取与**机械臂固件版本和本机 Python 版本匹配**的 `kortex_api-*.whl`，
   安装到本仓库使用的同一 `.venv`：

   ```powershell
   .\.venv\Scripts\python.exe -m pip install C:\path\to\kortex_api-<版本>.whl
   .\.venv\Scripts\python.exe -c "import kortex_api; print(kortex_api.__file__)"
   ```

   该 wheel 不是项目依赖；MuJoCo 模式不需要它。
2. **受信任的有线网络。** 控制主机与机械臂直连或经受信任交换机连接，
   确认 TCP `10000` 可达（默认地址 `192.168.1.10`）。
3. **Kinova Web App 只读检查。** 首次连接前先用浏览器登录机械臂
   Web App，确认固件版本、无故障、急停功能正常。此阶段不要发运动命令。
4. **PICO 链路已在 MuJoCo 模式验证。** 先完成
   [PICO UDP 快速开始](pico-udp-quickstart.md) 的全部步骤。

## 3. 只读连通性检查（不发运动命令）

```powershell
$env:KINOVA_PASSWORD = Read-Host 'Kortex password'
.\.venv\Scripts\python.exe -c @"
import os
from kinova_teleop.kortex_transport import KortexConfig, KortexConnection

connection = KortexConnection(
    KortexConfig('192.168.1.10', 'admin', os.environ['KINOVA_PASSWORD'])
).connect()
feedback = connection.base_cyclic.RefreshFeedback(
    options=connection.rpc_options()
).base
print('tool pose:',
      feedback.tool_pose_x, feedback.tool_pose_y, feedback.tool_pose_z)
print('stop confirmed on close:', connection.close())
"@
```

看到三个有限的位姿数值且 `stop confirmed on close: True` 即通过。

## 4. 启动实体遥操作

```powershell
$env:KINOVA_PASSWORD = Read-Host 'Kortex password'
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --backend kortex --enable-hardware `
  --robot-ip 192.168.1.10 --robot-user admin `
  --max-linear-speed 0.01 --max-angular-speed-deg 2
```

命令会要求在终端**精确输入 `MOVE`**；只有完全匹配才会导入 Kortex SDK、
建立连接并进入控制循环。`Ctrl+C` 停止；退出前后端总是先尝试 Stop。

首次测试建议保持上例中的低速限值（1 cm/s、2 deg/s），确认行为符合预期
后再逐步放宽到上限。

### 默认值与硬上限

| 参数 | 默认 | 上限 | 说明 |
| --- | --- | --- | --- |
| `--control-hz` | `25` | `40` | 每周期 2 次 RPC（反馈 + Twist）；实体模式强制按墙钟节拍 |
| `--scale` | `0.5` | `0.5` | 手柄平移到机械臂平移的比例 |
| `--stale-timeout` | `0.2` | `0.2` | 输入陈旧判定 |
| `--max-linear-speed` | `0.03` | `0.03` | m/s，代码内硬限幅 |
| `--max-angular-speed-deg` | `5` | `5` | deg/s，代码内硬限幅 |

### 内建安全行为

- **看门狗：** 最后一条非零 Twist 后 `0.2 s` 内没有新命令即自动 Stop，
  并阻塞后续反馈直到重新离合。UDP 断流、进程卡顿都会触发。
- **Stop 级联：** 任何 RPC 失败、超时、非法数值都会立即请求 Stop，
  未确认前拒绝恢复运动；清理失败时进程以非零码退出并打印
  `motion stop may be unconfirmed`。
- **启动期故障恢复：** 连接后若机械臂处于 fault 状态，会调用一次
  `ClearFaults` 并在 3 s 内等待 `SERVOING_READY`，失败则安全退出。
  **运行中出现的故障不会自动清除**，只会停机；需人工在 Web App 处理后
  重新启动。

## 5. 夹爪（可选，需要 V2 APK）

```powershell
.\.venv\Scripts\python.exe -m kinova_teleop.main `
  --backend kortex --enable-hardware --gripper
```

- `--gripper` 仅在 `--backend kortex` 下有效；离合按住时左手 **Trigger**
  按比例驱动夹爪开合（`GRIPPER_POSITION` 模式，2% 死区、最高 10 Hz）。
- Trigger 通道由 UDP 协议 V2（60 字节）携带。升级 PICO 应用：

  ```powershell
  $unityPath = 'C:\Program Files\Unity\Hub\Editor\2022.3.62f3c1\Editor\Unity.exe'
  .\scripts\build_pico_udp_bridge.ps1 -UnityPath $unityPath -Install
  ```

- 旧的 V1 APK（56 字节）仍可正常做无夹爪遥操作，trigger 按 `0` 处理。

## 6. 首次真机验收清单

- [ ] Kortex wheel 与固件/Python 匹配；`import kortex_api` 通过。
- [ ] 第 3 节只读检查通过，`stop confirmed on close: True`。
- [ ] Web App 中无故障；急停按钮实测有效。
- [ ] MuJoCo 模式下 PICO 链路（发现、离合、stale 门禁）已复验。
- [ ] 首次运动使用 `--max-linear-speed 0.01 --max-angular-speed-deg 2`。
- [ ] 验证：松开 Grip → 机械臂立即停住；拔掉 PICO Wi-Fi → 0.2 s 内停住；
      `Ctrl+C` → 进程退出且无 `unconfirmed` 报错。
- [ ] 以上全部通过后，才考虑放宽速度限值或启用 `--gripper`。

## 7. 排障

- `No module named 'kortex_api'`：wheel 未装进当前 `.venv`，用
  `.\.venv\Scripts\python.exe -m pip list | Select-String kortex` 核对。
- `KINOVA_PASSWORD must be set`：当前 PowerShell 会话未设置环境变量。
- 登录/会话失败：核对用户名密码、固件与 wheel 版本匹配、TCP `10000`
  连通（`Test-NetConnection 192.168.1.10 -Port 10000`）。
- `did not reach SERVOING_READY`：故障无法自动清除，登录 Web App 查看
  具体 fault 并人工处理。
- 运行中频繁 `stale=true`：优先检查 PICO Wi-Fi 链路质量；也可确认
  控制循环没有被杀毒软件或节能策略降速。
