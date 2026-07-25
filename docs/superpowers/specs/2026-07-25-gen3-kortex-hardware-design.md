# Kinova Gen3 Ultra Kortex 实机遥操作设计

日期：2026-07-25
状态：设计已批准；本分支只交付设计，不包含实体运动代码，也未宣称实机验证通过。

## 1. 目标与边界

在现有 PICO 左手柄遥操作项目上增加 Kinova Gen3 Ultra（7DoF、无夹爪）实体后端：

- 固定使用左手柄，按住 Grip 时进行完整 6DoF 末端控制。
- 手柄平移控制末端位置，手柄旋转控制末端姿态。
- 默认仍启动 MuJoCo；必须显式选择和确认实体后端。
- 当前只完成可实施设计和离线测试设计。未来由具备实体设备和安全条件的操作者验收。
- 不自动执行 Home，不控制夹爪，不采用 1 kHz Python UDP 低层伺服。
- 保留 MuJoCo 行为和既有 URDF 实验回溯点，不恢复或合并 URDF 实验。

## 2. 参考基线

设计参考 Kinova 官方仓库
[`Kinova-kortex2_Gen3_G3L`](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L)，固定参考提交：
`9782b6baca1a69a1a5204f9de8924d2dec7159aa`。

采用的官方接口：

- Python 高层接口使用 TCP transport、Router、SessionManager、BaseClient 和
  BaseCyclicClient；默认 TCP 端口为 `10000`。
- `BaseCyclicClient.RefreshFeedback()` 读取当前末端位置和 XYZ 欧拉角。
- `BaseClient.SendTwistCommand()` 发送末端 Twist，`BaseClient.Stop()` 停止运动。
- Kortex Pose 姿态为固定轴 X-Y-Z 外旋，角度单位为度。
- `TwistCommand.duration` 在参考文档中标为尚未实现，因此不把它作为安全停止机制。

Kortex Python wheel 必须与目标机械臂固件及受支持的 Python 版本匹配。项目不提交
SDK 二进制，也不把 Kortex SDK 加入 MuJoCo 的必装依赖；实体后端采用延迟导入。

## 3. 总体结构

### 3.1 后端协议

新增 `kinova_teleop/backend.py`：

```python
class EndEffectorTargetBackend(Protocol):
    def current_pose(self) -> Pose: ...
    def begin_control(self) -> None: ...
    def command_pose(self, target: Pose) -> BackendResult: ...
    def hold(self) -> None: ...
    def step(self) -> None: ...
    def close(self) -> None: ...
```

- `MuJoCoBackend`：从当前 `TeleopController` 中原样抽取模型、IK 和仿真步进逻辑，
  不改变既有仿真行为。
- `KortexBackend`：把末端目标姿态转换为限速的高层 Cartesian Twist。
- `TeleopController`：只负责读取 XR、相对位姿映射和调用后端，不再直接依赖
  MuJoCo。

### 3.2 Kortex transport

新增 `kinova_teleop/kortex_transport.py`：

- 仅在实体模式通过全部安全门控后导入 `kortex_api`。
- 使用 TCP `TCPTransport`、`RouterClient`、`SessionManager` 建立会话。
- 暴露 `BaseClient`、`BaseCyclicClient`，并保证会话和 transport 按逆序关闭。
- 客户端和时钟可注入，以便不连接实体完成单元测试。
- 异常文本和日志不得包含密码。

### 3.3 Kortex 后端

新增 `kinova_teleop/kortex_backend.py`：

- 连接后设置 `SINGLE_LEVEL_SERVOING`。
- `current_pose()` 读取 tool pose，并把 Kortex 固定轴 XYZ 欧拉角（度）转换为项目
  使用的 wxyz 四元数。
- Twist 使用 `CARTESIAN_REFERENCE_FRAME_BASE`。位置误差和旋转误差都在机械臂
  base 坐标系中计算。
- 不调用预存动作、不自动回零、不在连接时产生运动。
- `close()` 幂等，先 Stop，再关闭会话和 transport。

## 4. 6DoF 控制算法

第一次检测到新鲜的左 Grip 按下沿时：

1. 读取当前实体末端姿态作为机械臂锚点。
2. 读取当前左手柄姿态作为手柄锚点。
3. 后续目标为“机械臂锚点 + 手柄相对锚点变化”，沿用现有
   `RelativePoseMapper` 的坐标变换。

每个控制周期：

```text
position_error = target_position - measured_position
rotation_error = quaternion_log(target_orientation * inverse(measured_orientation))
linear_velocity = clip_norm(kp_linear * position_error, linear_limit)
angular_velocity = clip_norm(kp_angular * rotation_error, angular_limit)
```

- 旋转误差采用最短四元数路径，避免欧拉角跨 ±180° 跳变。
- Kortex 角速度字段使用度/秒，因此发送前从弧度/秒转换。
- 固定安全上限：
  - 平移速度向量范数不超过 `0.03 m/s`。
  - 旋转速度向量范数不超过 `5 deg/s`。
- CLI 只能把上限调低；超过固定上限直接拒绝启动，不静默截断。
- 比例增益和小误差死区是实现参数，首版采用保守值并由离线测试覆盖；不得绕过
  上述硬上限。
- 实体模式默认手柄平移比例为 `0.5`，仍允许操作者调低。

## 5. 状态与停止语义

```text
DISCONNECTED
  -> CONNECTED_STOPPED：通过实体门控并建立会话，保持静止
  -> ACTIVE：收到新鲜的左 Grip 按下，建立双锚点
  -> CONNECTED_STOPPED：Grip 松开、输入失效、异常或看门狗超时
  -> CLOSED：退出；先 Stop，再断开
```

停止规则：

- Grip 松开立即 Stop。
- XR 数据超过 `200 ms` 未更新立即 Stop。
- 非有限数、无效四元数、Kortex 异常或程序退出立即 Stop。
- `hold()` 仅在 ACTIVE 到停止态的转换中发送一次 Stop，避免每个循环重复调用。
- 所有清理路径均幂等；`KeyboardInterrupt` 和未处理异常走同一 Stop/close 路径。

### 5.1 独立软件看门狗

- 使用单独工作线程和 monotonic clock，不依赖主控制循环继续运行。
- 第一次成功发送非零 Twist 后才武装。
- 最近一次成功发送新鲜命令超过 `200 ms` 时，发送一次 Stop 并锁存停止态。
- 发送命令、更新心跳和 Stop 共用锁，防止超时 Stop 后仍有并发命令发出。
- 重新按 Grip 必须重新建立锚点后才能恢复 ACTIVE。
- 不依赖 `TwistCommand.duration`。

软件看门狗不能替代机械臂实体急停、限速配置和清空工作空间。

## 6. 启动门控与凭据

默认命令仍启动 MuJoCo。实体模式必须同时满足：

```powershell
$env:KINOVA_PASSWORD = "<由操作者在本机设置>"
python -m kinova_teleop.main `
  --backend kortex `
  --robot-ip 192.168.1.10 `
  --robot-user admin `
  --enable-hardware
```

随后终端要求操作者精确输入 `MOVE`。只有输入正确后才允许创建 Kortex 连接。

新增 CLI：

- `--backend {mujoco,kortex}`，默认 `mujoco`。
- `--enable-hardware`，实体模式必需。
- `--robot-ip`，默认官方常用地址 `192.168.1.10`。
- `--robot-user`，默认 `admin`。
- `--max-linear-speed`，默认且最高 `0.03`。
- `--max-angular-speed-deg`，默认且最高 `5`。

约束：

- 密码只从 `KINOVA_PASSWORD` 获取，不接受命令行密码，不写入日志或配置。
- 缺少开关、密码、错误确认、非法限速或互斥参数时，在导入 SDK和连接网络前失败。
- `--dry-run` 不能与真实 Kortex 连接组合；离线测试使用 fake client。
- MuJoCo 模式不读取实体凭据，也不要求安装 Kortex wheel。

## 7. 离线验证

在没有实体机械臂时必须完成：

1. MuJoCo 回归：现有启动、Grip 锚定、6DoF 映射、释放停止和 headless 测试保持通过。
2. 门控测试：缺少 `--enable-hardware`、密码、`MOVE` 或限速超限时，不导入 SDK、
   不连接网络。
3. transport 测试：会话创建/关闭顺序正确，断开前 Stop，错误中无密码。
4. 姿态测试：固定轴 XYZ 外旋的手算用例、单位转换和 ±180° 跨界。
5. 控制测试：六轴误差方向、向量范数限速、弧度到度转换。
6. 停止测试：Grip 释放、陈旧/非法输入、发送异常、KeyboardInterrupt 和重复 close。
7. 看门狗测试：使用 fake clock/同步原语，不真实 sleep；超过 200 ms 只 Stop 一次，
   新鲜命令重置计时，并验证不存在 Stop 后发送竞态。

## 8. 未来实体验收

此清单只能由具备设备和安全责任的操作者执行：

1. 固定机械臂，清空工作空间，确认实体急停可用，在 Kinova Web App 中设置低速并
   手动放到安全姿态。
2. 核对固件版本并安装匹配的 Kortex Python wheel；确认有线网络和 TCP 10000。
3. 先运行只读诊断：确认连接的是 7 轴 Gen3，仅读反馈，不发运动命令。
4. 首次以 `0.01 m/s`、`2 deg/s` 上限运行。
5. 确认连接后和 Grip 未按下时没有自动运动，也没有自动 Home。
6. 分别做六个方向的小幅动作，验证方向、锚点和响应。
7. 依次验证 Grip 松开、XR 断流、程序退出和网络异常时的停止行为。
8. 全部通过后，才允许提高到设计上限。

验收记录必须区分“离线测试通过”和“实体测试通过”；未完成本节不得标记实机功能
已验证。

## 9. 实施顺序与回退

1. 抽取后端协议和 MuJoCo 后端，先确保仿真回归。
2. 实现可注入的 Kortex transport 和 fake 测试。
3. 实现 pose-to-twist、限速、状态机与看门狗。
4. 加入 CLI 门控、凭据处理和文档。
5. 离线审查通过后另开待实机验证分支；完成第 8 节前不合并到公开稳定主线。

回退只需选择默认 `--backend mujoco`，或回到本设计之前的稳定提交；实体后端不会
改变已有模型文件和 URDF 实验回溯点。
