# PICO 右手独立追踪器 → Kinova Gen3 XYZ PID 跟随

使用 XRoboToolkit 界面的 **PICO Motion Tracker / Object Tracking** 模式。
独立追踪器绑在右手，Python 按序列号读取其位置，不读右手柄 Grip，也不把全身骨骼的手节点当作独立追踪器。
目标只改变末端 XYZ，末端姿态保持启动锚点姿态。

## 已实现的控制方式

显式使用 `--enable-hardware` 启动后，程序检查追踪器新鲜帧与机器人只读状态，输入 Kortex 密码后自动锚定并跟随。此独立入口按用户要求不再等待 Grip 或 MOVE。

- 空格：暂停／恢复。暂停发送 Stop、清空积分；恢复读取当前末端与追踪器位置作为新起点。
- Esc 或 Ctrl+C：Stop 并退出。
- 追踪器丢失、时间戳回退或超过 0.2 秒未更新：停止并保持暂停；数据恢复后按空格重新锚定。
- 键盘按键由**前台运行终端**接收，不是全系统快捷键。程序阻塞时原 Kortex watchdog 仍负责停止请求。
- 夹爪不属于此入口的命令输出。

相对目标为 `p_target = p_robot_start + scale * R * (p_tracker - p_tracker_start)`。
机器人基座坐标下以实测位置误差计算速度：`v = Kp*error + integral - Kd*filtered_measured_velocity`。
积分限幅及饱和抑制防止长时间碰到边界后继续累积；微分在测量端计算以减少目标突变冲击。

默认 `Kp=3.3, Ki=0.15, Kd=0.02, scale=1, control_hz=40, max_linear_speed=0.05 m/s`。
10 mm 单轴阶跃、理想速度驱动模型下 T90 约 **0.675 s**；这是理想模型验证，不是实机响应测量。
0.7 s 的指标定义为末端达到阶跃目标位移 90% 所用的时间。50 mm 位移仅按速度上限估算就需要至少 1 s 才能全部走完；该指标不适用于任意幅度。

## XRoboToolkit 数据路径

PICO XRoboToolkit → PC Service → 原生 `xrobotoolkit_sdk` → MotionTrackerInput → TrackerFollower → CartesianPID → Kortex Cartesian Twist。

1. PC 和头显接入可互通的受信任网络，启动 PC Service。
2. 在头显打开 **XRoboToolkit**，连接 PC Service。
3. 打开 Object Tracking、确认 TrackerNum，打开 Send。不要选 Full Body；其输出是人体骨骼。
4. Python 工作目录下的 `PXREASetting.ini` 按 PC Service 实际地址配置；本机 Windows Service 通常使用 `127.0.0.1:60061`。这里不沿用旧自制 Bridge 的 UDP 15031 协议。
5. 使用同一个 Python 3.11 解释器安装 Kortex `2.8.0.post5`、protobuf `3.20.0` 和 Windows XRoboToolkit binding。

官方依据（2026-09-22 阅读）：

- [XRoboToolkit Unity 界面及 Object Tracking](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client)
- [独立追踪器 Python 示例](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/blob/main/examples/example_motion_tracker.py)
- [Pybind 实现](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind/blob/main/bindings/py_bindings.cpp)
- [共享硬件遥操循环](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python/blob/main/xrobotoolkit_teleop/common/base_hardware_teleop_controller.py)

实际使用 `num_motion_data_available`、`get_motion_tracker_serial_numbers`、`get_motion_tracker_pose` 和 `get_motion_timestamp_ns`。
独立运动时间戳不能替换为头显总时间戳，否则头显持续刷新会掩盖追踪器冻结。

### XRoboToolkit 1.1.1 无时间戳协议兼容

现场发现此版本的 `Motion` 消息仅包含 `joints` 和 `len`，无 `timeStampNs`。
Windows 输入已改用原生 SDK 的只读消息回调：只在真正收到有效 Motion 数据包时记录本机单调时间，终端显示 `time_source=host_packet_arrival`。
重复读取缓存、仅收到头显数据均不会刷新这个时间；超过 0.2 秒无追踪器包仍停止。
若消息含设备时间戳则保留它，并继续拒绝不推进的设备时间。缺字段与非法值区分处理，非法时间戳不自动降级。
到包时间只能证明链路在更新，无法独立证明传感器采样年龄，也不能用于声称测得传感器到机械臂的端到端延迟。
CSV 新增 `tracker_time_source`，明确记录时间来源。

现场只读验证：无时间戳包兼容后通过 100 帧输入检查；佩戴头显并平移追踪器时采得 449 个有效样本，XYZ 范围约为 347 / 89 / 141 mm。
未佩戴或头显位置跟踪暂停时，可能仍收到连续数据包和变化的旋转，但 XYZ 固定；请佩戴头显、保持应用前台后验证平移。
连续收包不能证明物理位置有效，静止时也不应通过虚构位移判断跟踪正常。

## 运行

### 当前电脑已准备的环境

在 `.worktrees/tracker-pid` 中已构建 Python 3.11 x64 原生绑定，文件在被 Git 忽略的 `.local-deps/tracker-binding`。
启动脚本自动发现此绑定，并在该工作区没有 `.venv-kortex` 时使用相邻 `kortex-merge` 的已验证 Python 环境；没有修改该旧环境。
Kortex、NumPy、XRoboToolkit 三者在同一个解释器下的导入已经检查，不调用 SDK init。
本工作区的本地 `PXREASetting.ini` 指向本机 PC Service `127.0.0.1:60061`。

```powershell
# 进入本工作区后，离线验证（不连接设备）
& .\scripts\start_gen3_tracker.ps1 -TrackerSerial offline-test

# 初始化 XR 连接、列出序列号；此命令不连接机械臂
$python = (Resolve-Path ..\kortex-merge\.venv-kortex\Scripts\python.exe).Path
$env:PYTHONPATH = (Get-Location).Path + ';' + (Resolve-Path .\.local-deps\tracker-binding\venv\Lib\site-packages).Path
& $python -m kinova_teleop.tracker_main --list-trackers
$serial = Read-Host '右手追踪器序列号'
& .\scripts\start_gen3_tracker.ps1 -TrackerSerial $serial -CheckInput
```

如需重建原生绑定，使用 `scripts/build_tracker_binding.ps1`；它要求 VS 2022 C++ x64 工具链，且只构建并导入，不启动服务或连接设备。异机使用时传 `-Python`、`-Source`、`-Sdk` 指定 Python 3.11、Pybind 源码和 Windows SDK x64 目录。

### 其他电脑

从此工作区根目录，使用装好依赖的 Python（以下 `$python` 为其绝对路径）：

```powershell
$python = (Resolve-Path .\.venv-kortex\Scripts\python.exe).Path
& $python -m kinova_teleop.tracker_main --dry-run
& $python -m kinova_teleop.tracker_main --list-trackers
$serial = Read-Host '右手追踪器序列号'
& $python -m kinova_teleop.tracker_main --check-input --tracker-serial $serial
```

先确认 X/Y/Z 随右手移动变化且时间戳递增。独立追踪器旋转不用于末端姿态控制。
默认 `R` 使用现有项目的 PICO 坐标变换：机器人 `X=-tracker Z, Y=-tracker X, Z=tracker Y`。
**该映射不等于任何站位下的操作者前后左右**：跟踪空间或站位改变时需要检查基座方向；可用 `-Rotation` 传入九个行优先的正交旋转矩阵数值。不要直接复制旧左手柄校准来假定新追踪空间相同。

在当前 TCP 位于已检查的绝对工作区内时启动：

```powershell
$robotHost = Read-Host 'Kinova IPv4'
[double[]]$minimum = @([double](Read-Host 'X min (m)'), [double](Read-Host 'Y min (m)'), [double](Read-Host 'Z min (m)'))
[double[]]$maximum = @([double](Read-Host 'X max (m)'), [double](Read-Host 'Y max (m)'), [double](Read-Host 'Z max (m)'))
& .\scripts\start_gen3_tracker.ps1 -TrackerSerial $serial -RobotHost $robotHost -WorkspaceMin $minimum -WorkspaceMax $maximum -PythonPath $python -EnableHardware
```

这条命令明确授权启动；初始化完成后自动开始。空格暂停／恢复、Esc 退出。
可附加 `-Scale 0.5 -MaxLinearSpeed 0.03 -Kp 3.3 -Ki 0.15 -Kd 0.02` 调整参数。
CSV 自动保存到 `sessions/tracker-*.csv`，记录主机单调时间、追踪器时间戳、目标与实测位置，供现场计算 T90、稳态误差及超调。两种时钟未同步，不能直接相减当作网络延迟。

## 验证边界

单元测试不连接设备；SDK 导入验证不调用 init。之后已完成实际 XR 数据只读采集，操作者于 2026-09-22 报告该版本可实机遥操。
需要现场验证对象追踪模式可用、三轴映射、暂停／退出停止、当前负载下 10 mm 阶跃的 T90 和超调，才能报告实机达到约 0.7 s。
本功能复用既有型号／固件检查、速度限幅、工作区投影和 watchdog。密码只临时读取使用，不修改账户或密码文件。

2026-09-22 离线结果：完整回归 837 passed、2 skipped（Windows 符号链接不可用），总覆盖率 87.90%；之后补充的 4 个退出／清理用例也通过。
既有 MuJoCo 2000 步检查 `finite_state=true`；两个新增 PowerShell 脚本语法检查通过。
上述计数属于开发阶段离线验证。后续实机成功为操作者报告；实际 0.7 s、停止距离及超调仍未测量，不作量化达标声明。

## 下次复现已成功的大工作区命令

保存的参数：X/Y 各 `[-10, 10] m`，Z `[0, 10] m`；`Scale=1`，最大线速度 `0.05 m/s`，`Kp=3.3, Ki=0.15, Kd=0.02`。
`-WideWorkspace` 是对这组成功参数的显式选择，单位是米；这是很大的软件目标盒，不表示机器人有 10 m 行程，也不是碰撞检测。仍保留 Z 非负校验、速度限制、断流停止和键盘停止。

同一台电脑：启动 PC Service，佩戴头显，在 XRoboToolkit 打开 Object Tracking 和 Send，然后从此工作区运行：

```powershell
$serial = Read-Host '右手追踪器序列号'
$robotHost = Read-Host 'Kinova IPv4'
& .\scripts\start_gen3_tracker.ps1 -TrackerSerial $serial -RobotHost $robotHost -WideWorkspace -EnableHardware
```

不再需要 `$minimum`、`$maximum`。输入密码后初始化完成即自动开始；空格暂停／恢复时重新锚定，Esc/Ctrl+C 停止退出，终端需保持前台。
重复运行相同命令会生成新的 CSV。可用 `-Scale`、`-MaxLinearSpeed`、`-Kp`、`-Ki`、`-Kd` 调整；手动边界与 `-WideWorkspace` 不能同时使用。

异机复现先克隆此分支，不能仅下载旧左手柄分支：

```powershell
git clone --branch codex/right-tracker-pid --single-branch https://github.com/YEZQin/kinova-gen3-pico-teleop-mujoco.git
Set-Location kinova-gen3-pico-teleop-mujoco
.\scripts\bootstrap_public_teleop.ps1
```

本分支使用官方 XRoboToolkit 应用，不要求构建旧自制 UDP Bridge APK。
安装 XRoboToolkit PC Service 和官方 PICO 客户端，按上文构建 Windows 原生绑定，指定 `-Python .\.venv-kortex\Scripts\python.exe -Source <Pybind源码目录> -Sdk <Windows SDK x64目录>`；构建脚本需要 VS 2022 C++ 工具。
在工作目录创建本地 `PXREASetting.ini`（它不含密码）：

```ini
[Client]
connectAddr=127.0.0.1
connectPort=60061
```

头显连接的是电脑 Wi-Fi 地址；上面 127.0.0.1 仅供电脑上的 Python 连接同机 Service。IP 改变时更新头显连接地址。
原生 DLL/PYD、APK、密码、追踪器序列号配置及原始现场 CSV 不随 Git 发布。

### 依赖审计记录

2026-09-22 对本机既有 Kortex 环境运行 pip-audit，报告 pip 24.0、setuptools 79.0.1 和 protobuf 3.20.0 存在已知公告；自定义 Kortex wheel 无法由 PyPI 审计覆盖。本次未修改刚完成实机复现的环境。
protobuf 3.20.0 是当前 Kortex 兼容检查固定的版本，不能将简单升级依赖等同于已验证兼容；升级需单独回归 SDK 和设备通讯。因此本次发布不宣称依赖审计无漏洞。
