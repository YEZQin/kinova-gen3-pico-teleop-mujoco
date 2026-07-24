# PICO 左手柄遥操 Kinova Gen3（MuJoCo）

本项目通过 PICO 左手柄的相对位置和姿态，实时控制 MuJoCo 中单台 Kinova Gen3 7DoF 机械臂的末端 `pinch_site`。按住左手柄 Grip 后进行完整 6DoF 跟随，松开 Grip 则保持当前机械臂目标并允许重新离合。

数据链路如下：

```text
PICO 左手柄 → Windows XRoboToolkit PC Service → WSL xrobotoolkit_sdk
             → 相对 6DoF 映射/Grip 离合 → MuJoCo Jacobian DLS IK
             → Gen3 七个位置执行器 + MuJoCo Viewer
```

## 功能与边界

- 只使用 PICO **左手柄**。
- 跟随位置和姿态（6DoF），采用相对映射，按下 Grip 时不会跳变。
- Grip 值大于 `0.9` 时启用；松开、无效位姿或时间戳超过 `0.2 s` 未更新时保持上一有效关节目标。
- MuJoCo 原生 Jacobian 阻尼最小二乘 IK，包含关节限位、连续关节角度环绕、奇异位形阻尼和有限值检查。
- 红/绿/蓝坐标架表示当前末端目标。
- 只控制仿真中的单机械臂，不控制真实 Kinova 硬件。
- 模型不带夹爪，因此没有夹爪操作。

工作区原有 `kinova/kinova.urdf` 是 JACO2 J2S6S200，不是 Gen3；本项目使用已验证的 `kinova_gen3_mujoco/`。该模型对应 Kinova 官方产品名称中的 **Gen3 Ultra lightweight robot（7DoF）**。

## 目录

```text
kinova_teleop/             遥操、XR 输入、位姿映射、IK 和 CLI
kinova_gen3_mujoco/        可直接加载的 Gen3 MJCF、STL 与遥操场景
tests/                     单元测试和 MuJoCo 集成测试
docs/superpowers/          设计说明与实施计划
```

## 1. 安装 WSL2 与 Python 环境

在管理员 PowerShell 中安装 Ubuntu 22.04（已经安装则跳过）：

```powershell
wsl --install -d Ubuntu-22.04
```

打开 Ubuntu-22.04，安装依赖：

```bash
sudo apt update
sudo apt install -y python3-venv python3-dev build-essential cmake git \
  libgl1-mesa-dev libglfw3 netcat-openbsd
```

若使用当前工作区：

```bash
cd /mnt/d/yezq/26.7.24_Kinova_controll
python3 -m venv .venv-wsl
source .venv-wsl/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

若从 GitHub 克隆，把上面的 `cd` 替换为实际克隆目录。

> 性能提示：某些 WSL2 版本在 `/mnt/d` 中创建 venv 会因跨文件系统小文件 I/O 耗时很久。若安装明显缓慢，推荐把 venv 放在 WSL 的 Linux 文件系统中；后续命令中的 `source .venv-wsl/bin/activate` 相应替换为下面的路径：

```bash
mkdir -p ~/.venvs
python3 -m venv ~/.venvs/kinova-pico-teleop
source ~/.venvs/kinova-pico-teleop/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[dev]"
```

先验证 MuJoCo、模型、IK 和无头控制循环：

```bash
python -m pytest -q
python -m kinova_teleop.main --dry-run --headless --steps 2000
```

成功时最后一行应包含：

```text
completed steps=2000 finite_state=true
```

## 2. Windows 端：PICO 与 PC Service

1. 按 [XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service) 的说明在 Windows 安装或构建 PC Service。
2. 按 XRoboToolkit/PICO 端说明连接头显，使 PC Service 能看到设备及左右控制器数据。
3. 完全退出 PC Service。
4. 找到与 PC Service 可执行文件配套的 `setting.ini`，修改为：

```ini
[Service]
listenAddr=0.0.0.0
listenPort=60061
```

5. 重新启动 PC Service，并在 Windows 防火墙中只对受信任的专用网络/WSL 网段允许该程序或 TCP `60061` 入站。

WSL2 默认使用 NAT，WSL 内的 `127.0.0.1` 通常不是 Windows PC Service。这里让服务监听 Windows 网卡，再由 WSL 使用默认网关地址连接。`0.0.0.0` 会监听多个接口，不应在不受信任的网络上开放端口。

## 3. WSL 端：构建 XRoboToolkit Python SDK

项目运行时会延迟导入 `xrobotoolkit_sdk`；dry-run 不需要它，连接真实 PICO 时需要。以下固定使用本项目验证过的 Pybind 提交：

```bash
cd /mnt/d/yezq/26.7.24_Kinova_controll
source .venv-wsl/bin/activate
mkdir -p .local-deps
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git \
  .local-deps/XRoboToolkit-PC-Service-Pybind
cd .local-deps/XRoboToolkit-PC-Service-Pybind
git checkout c64ccf6acd577a333e03b66fafe8efeeceb511b1
```

该提交的 `setup_ubuntu.sh` 在非 Conda 环境中把 `-y` 误传给 `pip install`。在 Python venv 中先修正这一行，再运行原脚本：

```bash
sed -i 's/pip install pybind11 -y/pip install pybind11/' setup_ubuntu.sh
bash setup_ubuntu.sh
cd /mnt/d/yezq/26.7.24_Kinova_controll
```

每次新开 WSL 终端，激活环境后加入 SDK 动态库目录：

```bash
source .venv-wsl/bin/activate
export LD_LIBRARY_PATH="$PWD/.local-deps/XRoboToolkit-PC-Service-Pybind/lib:${LD_LIBRARY_PATH:-}"
python -c "import xrobotoolkit_sdk; print('xrobotoolkit_sdk import OK')"
```

## 4. 配置 WSL 到 Windows 的连接

确保 Windows PC Service 已运行，然后在**项目根目录**生成 `PXREASetting.ini`。SDK 从当前工作目录读取这个文件：

```bash
WINDOWS_HOST_IP="$(ip route show default | awk '{print $3}')"
cat > PXREASetting.ini <<EOF
[Client]
connectAddr=${WINDOWS_HOST_IP}
connectPort=60061
EOF
nc -vz "${WINDOWS_HOST_IP}" 60061
```

`nc` 成功表示 TCP 端口可达；它不代表 PICO 数据已经正常，需要继续执行输入检查。

## 5. 推荐启动顺序

每次运行按这个顺序操作：

1. 启动 PICO 端 XRoboToolkit 应用并连接设备/控制器。
2. 启动 Windows XRoboToolkit PC Service，确认设备在线。
3. 打开 WSL，进入项目目录，激活 venv 并设置 `LD_LIBRARY_PATH`。
4. 检查左手柄数据：

```bash
python -m kinova_teleop.main --check-xr --samples 100
```

输出应连续包含 `left position`、`quat_xyzw`、`grip` 和递增的 `timestamp_ns`。移动左手柄、按压 Grip 时，这些值应变化。

5. 首次使用或改动代码后先跑无头自检：

```bash
python -m kinova_teleop.main --dry-run --headless --steps 2000
```

6. 启动真实遥操和 Viewer：

```bash
python -m kinova_teleop.main --model kinova_gen3_mujoco/teleop_scene.xml
```

若手柄移动对应的机械臂平移过大，先降低比例：

```bash
python -m kinova_teleop.main --scale 0.5
```

WSLg 正常时 Viewer 会直接显示在 Windows 桌面。关闭 Viewer 或按 `Ctrl+C` 退出。

## 6. 操作方法

1. 左手柄移动到舒适位置，暂时不要按 Grip。
2. **按住左手柄 Grip（left Grip）**，其值超过 `0.9` 时捕获当前手柄和末端位姿作为参考。
3. 保持 Grip，移动和旋转左手柄；机械臂末端跟随完整相对 6DoF 变化。
4. 松开 Grip：机械臂保持最后有效关节目标。
5. 把左手移动到新的舒适位置，再次按住 Grip 继续，不会因手柄绝对位置产生目标跳变。

红/绿/蓝坐标架是期望末端位姿。接近不可达区域时 IK 可能无法完全收敛；程序会保持上一帧已收敛的有效关节目标。关节目标同时受限位、单步变化和有限值检查保护。

## 7. 命令行参数

```bash
python -m kinova_teleop.main --help
```

- `--controller left`：仅允许左手柄。
- `--model PATH`：MJCF 场景，默认 `kinova_gen3_mujoco/teleop_scene.xml`。
- `--scale FLOAT`：手柄平移到机械臂平移的比例，默认 `1.0`。
- `--control-hz FLOAT`：控制频率，默认 `100` Hz。
- `--stale-timeout FLOAT`：时间戳不更新后的释放时间，默认 `0.2` s。
- `--check-xr [--samples N]`：只检查 XR 链路，不加载 MuJoCo。
- `--dry-run`：使用确定性的内置小幅 6DoF 轨迹。
- `--headless --steps N`：不打开 Viewer，运行固定步数后退出。

安装后也可使用入口命令 `kinova-pico-teleop`，参数相同。

## 8. 故障排查

### `nc` 连接失败

- 确认 Windows PC Service 正在运行，`setting.ini` 修改后已重启。
- 确认 `listenAddr=0.0.0.0`、端口两端均为 `60061`。
- 重新计算 `WINDOWS_HOST_IP`，WSL 重启后默认网关可能改变。
- 检查 Windows 防火墙入站规则和当前网络配置文件。

### `xrobotoolkit_sdk is unavailable` 或共享库错误

- 确认当前终端激活的是 `.venv-wsl`。
- 重新运行 `python -c "import xrobotoolkit_sdk"` 查看底层错误。
- 确认 `LD_LIBRARY_PATH` 包含 Pybind 仓库的 `lib/`。
- 用 `ldd "$(python -c 'import xrobotoolkit_sdk; print(xrobotoolkit_sdk.__file__)')"` 检查缺失的 `.so`。
- 若 ABI/编译失败，在 Ubuntu 22.04、Python 3.10 环境按第 3 节重新构建。

### `--check-xr` 能连接但时间戳不变

- 确认 PICO 端应用正在发送跟踪数据，设备在 PC Service 中在线。
- 唤醒头显和左手柄；移动手柄并按 Grip 观察输出。
- 时间戳持续不变超过 `0.2 s` 时，本程序会自动解除遥操并保持机械臂。

### Viewer 不显示

```bash
echo "$DISPLAY"
python -m mujoco.viewer --mjcf kinova_gen3_mujoco/teleop_scene.xml
```

若 `DISPLAY` 为空或 Viewer 报 OpenGL/GLFW 错误，先更新 WSL/WSLg：

```powershell
wsl --update
wsl --shutdown
```

重新打开 Ubuntu 后再试。自动化验证可始终使用 `--headless`。

### 运动方向或幅度不符合预期

- 先确认只使用左手柄且 `--check-xr` 的位置/四元数变化合理。
- 本项目已将 PICO 坐标转换到 MuJoCo 世界坐标；不要同时在外部再次变换。
- 用 `--scale 0.5` 或更小值降低平移幅度。
- 松开 Grip、换一个舒适位姿后重新按下，以重设相对参考。

## 9. 开发验证

```bash
python -m pip install -e ".[dev]"
python -m pytest -q
python -m kinova_teleop.main --dry-run --headless --steps 5000
```

核心模型契约为：7 个关节 `joint_1`～`joint_7`、7 个位置执行器、`home` 关键帧及 `pinch_site` 末端站点。缺少其中任一项都会在启动时明确报错。

## 来源与许可

- 遥操接口设计参考 [XR-Robotics/XRoboToolkit-Teleop-Sample-Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python)。
- XR Python 绑定来自 [XRoboToolkit-PC-Service-Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind)。
- Gen3 MJCF 来自 [MuJoCo Menagerie 的 Kinova Gen3](https://github.com/google-deepmind/mujoco_menagerie/tree/main/kinova_gen3)。
- 本项目 Python 代码采用根目录 MIT License；`kinova_gen3_mujoco/` 内模型资产保留其 BSD-3-Clause License。
