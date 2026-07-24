# PICO 左手柄遥操 Kinova Gen3 MuJoCo 仿真设计

**日期：** 2026-07-24  
**状态：** 已完成口头设计评审，等待书面规格确认

## 1. 目标

在 WSL2 Ubuntu 22.04 中运行单机械臂 MuJoCo 仿真，通过 PICO 左手柄对
Kinova Gen3 7 自由度机械臂的末端执行器进行完整 6DoF 相对位姿遥操。

本项目仅控制机械臂本体，不添加或控制 Robotiq 夹爪。Windows 负责运行
XRoboToolkit PC Service 并接收 PICO 数据，WSL 通过 XRoboToolkit Python
SDK 获取左手柄状态。

## 2. 已确认的需求

- 运行环境：Windows 主机加 WSL2 Ubuntu 22.04。
- 输入设备：PICO 左手柄。
- 控制对象：单台 Kinova Gen3 7DoF 机械臂。
- 控制模式：末端位置和姿态完整 6DoF 跟随。
- 夹爪：不包含。
- MuJoCo 模型：`kinova_gen3_mujoco/scene.xml`。
- 末端控制点：MJCF 中的 `pinch_site`。
- 激活方式：按住左 Grip 时遥操，松开时保持。

## 3. 参考实现与版本

- 参考项目：
  `XR-Robotics/XRoboToolkit-Teleop-Sample-Python`
- 检查的参考提交：
  `79e5cb8a56e3455515ce1b476e993c764ec58739`
- XR Python 绑定：
  `XR-Robotics/XRoboToolkit-PC-Service-Pybind`
- 检查的绑定提交：
  `c64ccf6acd577a333e03b66fafe8efeeceb511b1`
- PC Service：
  `XR-Robotics/XRoboToolkit-PC-Service`
- 检查的服务提交：
  `85bac4dbc1fd5cef42c74a160d9c30aa3491f122`

参考项目采用 XR 输入、相对位姿映射、Placo IK 和 MuJoCo 位置执行器。本设计保留
XR 输入和相对位姿交互方式，将 Placo IK 替换为 MuJoCo 原生 Jacobian IK。

## 4. 方案比较与选择

### 4.1 完整复用 Placo IK

优点是与参考项目接近，并自带任务优先级和可操作度任务。缺点是需要维护一份与
MJCF 严格一致的 URDF，并引入 Placo、Pinocchio 和 Meshcat。对当前只有一台
7DoF 固定基座机械臂的目标而言，依赖和模型同步成本偏高。

### 4.2 MuJoCo 原生阻尼最小二乘 Jacobian IK

直接使用当前 MJCF 的关节、`pinch_site` 和位置执行器。它不依赖第二套机器人
描述，可避免 URDF/MJCF 关节和坐标不一致。7DoF 冗余通过零空间姿态回归处理。

这是本项目选定的方案。

### 4.3 操作空间动力学控制

该方案能获得更真实的力矩控制行为，但需要质量矩阵、重力补偿、接触稳定性和执行器
参数调优。它超出本次稳定遥操仿真的必要范围。

## 5. 系统架构

```text
Windows XRoboToolkit PC Service
            │ gRPC :60061
            ▼
WSL xrobotoolkit_sdk
            │ 左手柄位姿、Grip、时间戳
            ▼
XR 输入适配器
            │ 坐标转换、有效性与超时检查
            ▼
相对位姿映射器
            │ Grip 上升沿标定基准、生成 6DoF 末端目标
            ▼
MuJoCo Jacobian IK
            │ 阻尼最小二乘、零空间姿态回归、关节限位
            ▼
Gen3 位置执行器 joint_1～joint_7
            ▼
MuJoCo 仿真与 Viewer
```

## 6. 模块边界

### `kinova_teleop/xr_input.py`

封装 `xrobotoolkit_sdk`，输出统一的不可变手柄状态，包括：

- 位置 `(3,)`；
- 四元数 `(qx, qy, qz, qw)`；
- 左 Grip 浮点值；
- SDK 时间戳；
- 本机接收时刻；
- 数据有效标志。

该模块不包含机器人坐标转换或 IK。它提供真实 SDK 输入源和可由测试、回放及
`--dry-run` 使用的输入接口。

### `kinova_teleop/pose_mapping.py`

负责：

- PICO 坐标系到 MuJoCo 世界坐标系的基变换；
- 四元数格式转换、归一化和有效性检查；
- Grip 上升沿/下降沿检测；
- 手柄相对位姿到末端相对位姿的映射；
- 目标位置和姿态的一阶低通滤波；
- 单周期目标变化限制。

该模块不读取 SDK，也不访问 MuJoCo。

### `kinova_teleop/ik_solver.py`

负责：

- 读取 `pinch_site` 当前位姿与 6×7 Jacobian；
- 计算位置和旋转误差；
- 使用自适应阻尼最小二乘求解关节增量；
- 使用零空间项回归 Grip 激活时的关节姿态；
- 处理关节限位、连续关节邻近等效角和单周期关节变化限制；
- 在失败时返回上一帧有效关节目标。

该模块只依赖 MuJoCo 和 NumPy，不依赖 XR SDK。

### `kinova_teleop/teleop_controller.py`

负责主控制循环：

1. 读取并验证手柄状态；
2. 更新离合状态；
3. 生成并过滤末端目标；
4. 求解 IK；
5. 更新 `joint_1`～`joint_7` 的位置执行器控制量；
6. 推进 MuJoCo 仿真；
7. 同步 Viewer；
8. 在退出时关闭 SDK。

### `kinova_teleop/main.py`

提供命令行入口、路径解析、配置检查和运行模式选择。

### `tests/`

使用模拟 XR 输入和真实 MuJoCo 模型验证控制行为，不要求连接 PICO。

## 7. WSL 与 Windows 通信

PC Service 的 gRPC 服务默认读取工作目录中的 `setting.ini`，默认仅监听
`127.0.0.1:60061`。为了让 WSL2 客户端访问，Windows 端配置为：

```ini
[Service]
listenAddr=0.0.0.0
listenPort=60061
```

Windows 防火墙仅在专用网络配置文件中允许 TCP 60061 入站。

XR Python SDK 从当前工作目录的 `PXREASetting.ini` 读取客户端地址。WSL 端配置为：

```ini
[Client]
connectAddr=<Windows 主机在 WSL 中的网关地址>
connectPort=60061
```

Windows 主机地址通过 WSL 默认路由获取：

```bash
ip route show default | awk '{print $3}'
```

README 必须同时说明 WSL 镜像网络模式下可使用 `127.0.0.1` 的情况，以及连接失败时
如何使用上述网关地址和 `nc -vz <地址> 60061` 排查。

## 8. 控制行为

### 8.1 初始状态

- MuJoCo 从 `home` 关键帧启动。
- 位置执行器控制量初始化为当前关节位置。
- 左 Grip 未按下时，手柄运动不改变末端目标。

### 8.2 离合与相对映射

左 Grip 激活阈值为 `0.9`。

Grip 上升沿同时记录：

- 左手柄基准位置和姿态；
- `pinch_site` 基准位置和姿态；
- 当前 7 维关节位置，作为本次激活期间的零空间参考。

持续按住 Grip 时：

```text
目标位置 = 末端基准位置
         + 平移比例 × 基变换 × (当前手柄位置 - 手柄基准位置)

目标姿态 = 手柄世界坐标相对旋转 × 末端基准姿态
```

Grip 下降沿冻结最后一个有效末端目标，并清除手柄与末端基准。下次按下时重新标定，
因此启用瞬间不会发生绝对位姿跳变，也允许用户通过多次离合扩展操作范围。

### 8.3 坐标变换

默认使用参考项目的 PICO/头显到机器人世界坐标基变换：

```text
[[ 0,  0, -1],
 [-1,  0,  0],
 [ 0,  1,  0]]
```

平移直接左乘该矩阵。旋转采用基变换共轭：

```text
R_world_controller = B × R_xr_controller × Bᵀ
```

四元数在 SDK 的 `(qx, qy, qz, qw)` 和 MuJoCo 的 `(qw, qx, qy, qz)` 之间显式转换。

## 9. IK 算法

对位置误差 `e_p` 和旋转误差 `e_r` 组成 6 维误差：

```text
e = [k_p e_p, k_r e_r]
```

使用 `mj_jacSite` 获得 `pinch_site` 的位置与旋转 Jacobian，并求解：

```text
dq_task = Jᵀ (J Jᵀ + λ² I)⁻¹ e
```

阻尼 `λ` 根据最小奇异值自适应调整。零空间项为：

```text
dq_null = (I - J⁺J) k_null (q_reference - q)
```

最终关节增量为两项之和，并受单周期关节增量限制。求解在独立的 MuJoCo
`MjData` 中迭代，避免修改正在仿真的状态；迭代完成后的目标关节位置写入位置执行器。

有限关节使用 MJCF `jnt_range` 裁剪。连续关节选择最接近当前关节位置的等效角，
避免跨越 `±π` 时产生大跳变。

## 10. 安全与错误处理

- SDK 位姿必须为 7 个有限值。
- 四元数范数必须大于最小阈值并在使用前归一化。
- SDK 时间戳必须递增；超过 200 ms 没有新数据时自动解除 Grip 激活并保持。
- 目标位置和旋转使用默认约 50 ms 时间常数的一阶低通滤波。
- 限制每周期末端平移、旋转和关节变化量。
- IK 误差、关节目标和执行器控制量必须全部为有限值。
- IK 未收敛或产生非有限值时沿用上一帧有效控制量。
- 模型缺少 `pinch_site`、7 个指定关节、匹配执行器或 `home` 关键帧时，启动失败并给出明确错误。
- `Ctrl+C`、Viewer 关闭和未处理异常都必须通过 `finally` 关闭 SDK。

## 11. 命令行接口

正常运行：

```bash
python -m kinova_teleop.main \
  --model kinova_gen3_mujoco/scene.xml \
  --controller left \
  --scale 1.0
```

XR 链路检查：

```bash
python -m kinova_teleop.main --check-xr --controller left
```

该模式持续打印左手柄 7 维位姿、Grip 和时间戳，不加载 MuJoCo。

无 PICO 自检：

```bash
python -m kinova_teleop.main \
  --model kinova_gen3_mujoco/scene.xml \
  --dry-run
```

该模式使用确定性的内置小幅 6DoF 轨迹驱动目标，用于验证模型、IK、执行器和仿真循环。

主要可配置项：

- `--model`：MuJoCo 场景路径；
- `--controller`：固定支持 `left`，保留接口一致性；
- `--scale`：平移比例，默认 `1.0`；
- `--control-hz`：控制频率，默认 `100`；
- `--stale-timeout`：XR 超时秒数，默认 `0.2`；
- `--dry-run`：使用确定性模拟输入；
- `--check-xr`：只检查 XR 输入链路；
- `--headless`：不启动 Viewer，用于自动化验证。

## 12. 测试策略

### 单元测试

- PICO 到 MuJoCo 平移坐标变换。
- 四元数格式转换、归一化和相对旋转。
- Grip 上升沿时目标无跳变。
- Grip 持续按下时相对 6DoF 映射正确。
- Grip 松开后目标保持。
- 无效位姿、非递增时间戳和 200 ms 超时会解除激活。
- 目标滤波和单周期变化限制。
- 有限关节裁剪和连续关节邻近等效角。

### MuJoCo 集成测试

- `scene.xml`、`pinch_site`、7 个关节、7 个位置执行器和 `home` 关键帧完整。
- IK 对多个可达位置与姿态目标收敛。
- 近奇异姿态下输出保持有限。
- `--dry-run --headless` 运行数千仿真步后，`qpos`、`qvel` 和 `ctrl` 均为有限值。
- Grip 释放或输入超时后，执行器目标不继续跟随模拟手柄运动。

### 真实设备验收

`--check-xr` 必须连续收到：

- 长度为 7 的左手柄位姿；
- 有效归一化四元数；
- 可随左 Grip 改变的 Grip 值；
- 单调递增的 SDK 时间戳。

正常模式下：

- 按下 Grip 时无初始跳变；
- 平移和旋转方向与手柄一致；
- 松开 Grip 后机械臂保持；
- 重新按下 Grip 后从当前机械臂位姿继续控制；
- 关闭 Viewer 或按 `Ctrl+C` 后进程正常退出。

## 13. README 内容

根目录 `README.md` 必须包含：

1. 系统架构和支持范围；
2. PICO 与 XRoboToolkit PC Service 前置条件；
3. Windows `setting.ini` 和防火墙配置；
4. WSL2 Ubuntu 22.04 安装；
5. XR Python SDK 编译和安装；
6. `PXREASetting.ini` 与 Windows 主机 IP 获取；
7. MuJoCo、测试依赖和项目安装；
8. `--check-xr`、`--dry-run` 和正常遥操的启动顺序；
9. 左 Grip 离合行为与坐标映射；
10. 常见连接、SDK、Viewer、IK 和性能问题排查；
11. 自动化测试命令和预期结果；
12. 当前限制：单臂、左手柄、无夹爪、仅 MuJoCo 仿真。

## 14. 非目标

- 控制真实 Kinova Gen3 硬件；
- 添加或控制夹爪；
- 双臂遥操；
- 数据采集或 LeRobot 数据集导出；
- 手部追踪、头显追踪或身体追踪；
- 力反馈；
- 操作空间力矩控制；
- WSL 之外的平台打包。

## 15. 完成标准

只有在以下条件全部满足时才视为完成：

- 所有新函数均有先失败后通过的自动化测试；
- 全部单元和集成测试通过；
- `--dry-run --headless` 使用真实 Gen3 MJCF 成功运行；
- MuJoCo 状态与控制量保持有限；
- `--check-xr` 在缺少 SDK 或服务时输出可操作的错误信息；
- README 覆盖第 13 节全部内容；
- 无 PICO 环境可完成除真实设备验收外的全部验证；
- 真实 PICO 验收步骤明确标记为需要用户硬件执行。
