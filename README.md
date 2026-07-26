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

进入克隆后的项目目录，并把 venv 放在 WSL 的 Linux 文件系统中：

```bash
cd ~/kinova-gen3-pico-teleop-mujoco
mkdir -p ~/.venvs
python3 -m venv ~/.venvs/kinova-pico-teleop
source ~/.venvs/kinova-pico-teleop/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install -e ".[dev]"
```

如果项目位于其他位置，只替换 `cd` 路径。不要在 `/mnt/c`、`/mnt/d`
等 Windows 挂载盘中创建 venv；跨文件系统小文件 I/O 通常更慢。

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
cd ~/kinova-gen3-pico-teleop-mujoco
source ~/.venvs/kinova-pico-teleop/bin/activate
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
cd ~/kinova-gen3-pico-teleop-mujoco
```

每次新开 WSL 终端，激活环境后加入 SDK 动态库目录：

```bash
cd ~/kinova-gen3-pico-teleop-mujoco
source ~/.venvs/kinova-pico-teleop/bin/activate
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
- `--scale FLOAT`：手柄平移到机械臂平移的比例；MuJoCo 默认 `1.0`，Kortex 默认 `0.5`，Kortex 允许调低但不允许高于 `0.5`。
- `--control-hz FLOAT`：控制频率，默认 `100` Hz。
- `--stale-timeout FLOAT`：时间戳不更新后的释放时间，默认 `0.2` s。
- `--check-xr [--samples N]`：只检查 XR 链路，不加载 MuJoCo。
- `--dry-run`：使用确定性的内置小幅 6DoF 轨迹。
- `--headless --steps N`：不打开 Viewer，运行固定步数后退出。

安装后也可使用入口命令 `kinova-pico-teleop`，参数相同。

## 8. Kinova Gen3 实体模式（已设计并离线测试，尚未完成实机验证）

> **安全状态：**本分支的 Kortex 实体后端只完成了设计与离线自动化测试，**未在真实机械臂上验证**。默认命令仍只启动 MuJoCo；只有具备实体设备、安全条件并完成下列验收的操作者，才可选择实体模式。

实体模式需要从 Kinova 获取与目标机械臂**固件版本和受支持 Python 版本相匹配**的 Kortex Python wheel。该 wheel 不随本项目分发，也不是项目依赖；MuJoCo 模式不需要安装它。机械臂与控制主机应使用受信任的**有线网络**连接，并确认 Kortex TCP 端口 `10000` 可达。

### 8.1 推荐部署拓扑

本项目只把下面这套环境作为从零部署的主流程：

```text
PICO 左手柄
  └─ PICO 端 XRoboToolkit 应用
       └─ Windows XRoboToolkit PC Service（TCP 60061）
            └─ WSL2 Ubuntu 22.04 / Python 3.10 / 同一个 venv
                 ├─ xrobotoolkit_sdk
                 ├─ 本项目 kinova_teleop
                 └─ 与机器人固件匹配的 kortex_api wheel
                      └─ Windows 有线网卡 → Gen3 TCP 10000
```

PICO PC Service 和机械臂网卡位于 Windows；Python 程序、XR binding 与
Kortex API 位于同一个 WSL venv。不要把 Kortex SDK 装在 Windows Python、
却从 WSL 运行本项目；两个 Python 环境不能共享已安装模块。

### 8.2 第一步：从空白 Windows 安装 WSL

在管理员 PowerShell 中执行：

```powershell
wsl --install -d Ubuntu-22.04
wsl --update
```

首次打开 Ubuntu-22.04并创建 Linux 用户后，在 WSL 中执行：

```bash
sudo apt update
sudo apt install -y python3.10 python3.10-venv python3-dev \
  build-essential cmake git libgl1-mesa-dev libglfw3 netcat-openbsd
```

### 8.3 第二步：下载实机分支并创建项目环境

本功能尚未合并到 `main`。在 WSL 中克隆公开的实机分支：

```bash
cd ~
git clone --branch codex/gen3-hardware-design --single-branch \
  https://github.com/YEZQin/kinova-gen3-pico-teleop-mujoco.git
cd ~/kinova-gen3-pico-teleop-mujoco

mkdir -p ~/.venvs
python3.10 -m venv ~/.venvs/kinova-pico-teleop
source ~/.venvs/kinova-pico-teleop/bin/activate
python -m pip install --upgrade pip setuptools wheel

cd ~/kinova-gen3-pico-teleop-mujoco
python -m pip install -e ".[dev]"
```

也可以从
[Draft PR #2](https://github.com/YEZQin/kinova-gen3-pico-teleop-mujoco/pull/2)
查看改动，或下载
[该分支 ZIP](https://github.com/YEZQin/kinova-gen3-pico-teleop-mujoco/archive/refs/heads/codex/gen3-hardware-design.zip)。
ZIP 解压后，把后续命令中的 `~/kinova-gen3-pico-teleop-mujoco` 替换为实际解压目录。

如果项目实际克隆到了其他目录，把最后的 `cd` 改为该目录。每次新开 WSL
终端都必须重新激活同一个 venv：

```bash
source ~/.venvs/kinova-pico-teleop/bin/activate
```

先完成与实体无关的检查：

```bash
python --version
python -m pytest -q
python -m kinova_teleop.main --dry-run --headless --steps 2000
```

最后一条必须输出 `finite_state=true`。失败时先修复 Python、项目或 MuJoCo
环境，不要继续连接机械臂。

### 8.4 第三步：安装和验证 PICO/XR 链路

严格按第 2～4 节完成以下工作：

1. 在 Windows 安装并启动 XRoboToolkit PC Service，设置
   `listenAddr=0.0.0.0`、`listenPort=60061`。
2. 在 PICO 中启动 XRoboToolkit 应用并确认左手柄在线。
3. 在当前 **同一个 venv** 中构建
   `XRoboToolkit-PC-Service-Pybind`，不要切换到另一个 Python。
4. 在项目根目录创建 `PXREASetting.ini`，指向 Windows WSL 网关地址。
5. 每次运行前设置 native library 路径：

```bash
cd ~/kinova-gen3-pico-teleop-mujoco
source ~/.venvs/kinova-pico-teleop/bin/activate
export LD_LIBRARY_PATH="$PWD/.local-deps/XRoboToolkit-PC-Service-Pybind/lib:${LD_LIBRARY_PATH:-}"

python -c "import xrobotoolkit_sdk; print('xrobotoolkit_sdk import OK')"
python -m kinova_teleop.main --check-xr --samples 100
```

移动左手柄、旋转手柄并按 Grip，确认位置、四元数、Grip 和递增时间戳都在变化。
时间戳不递增时不得进入实体模式。

### 8.5 第四步：获取并安装 Kinova Kortex Python SDK

1. 打开目标机械臂的 Kinova Web App，记录**完整固件版本**和机器人型号
   （Gen3、7DoF）。不要仅凭仓库示例猜 SDK 版本。
2. 从 Kinova 官方
   [Kortex API 安装页](https://docs.kinovarobotics.com/getting_started/installation.html)
   或 Kinova 支持渠道下载与该固件、操作系统和 Python 版本匹配的
   `kortex_api-*.whl`。
3. 官方
   [Kinova-kortex2_Gen3_G3L](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L)
   仓库主要提供示例和 API 文档，不等同于已经安装 SDK。其
   [Python 示例说明](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_python/examples/readme.md)
   展示了 wheel 安装方法，但其中固定版本链接不应覆盖现场固件匹配要求。
4. 把下载的 wheel 放到 WSL 可读路径，然后在项目使用的同一个 venv 中安装。
   例如 wheel 位于 Windows 下载目录时：

```bash
source ~/.venvs/kinova-pico-teleop/bin/activate
python -m pip install \
  "/mnt/c/Users/<Windows用户名>/Downloads/<实际Kortex-wheel文件名>.whl"
python -m pip check
```

尖括号内容必须替换为真实用户名和文件名。不要复制示例占位符原样执行。

验证本项目需要的 SDK 接口：

```bash
python - <<'PY'
import kortex_api
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2

print("kortex_api import OK:", kortex_api.__file__)
print("required Kortex TCP interfaces OK")
PY
```

如果 pip 报 `not a supported wheel on this platform`，或上述 import 失败：

- 核对 wheel 是否为 Linux/Python wheel，而不是其他平台的包。
- 核对 Kinova 发布说明中该 wheel 支持的 Python 版本。
- 本项目要求 Python `>=3.10`。如果与现场固件匹配的 wheel 不支持 Python
  3.10 或其他项目支持的 `>=3.10` 版本，则当前组合**不受支持**：停止部署并
  联系 Kinova 或项目维护者，不要退回 Python 3.9 以下强装本项目。
- 如果改用 wheel 明确支持的另一个 Python `>=3.10` 版本，必须重新创建干净
  venv，并在其中重新安装本项目、重新构建 XRoboToolkit binding；不要在两个
  Python 环境之间复制 `.so` 或 `site-packages`。
- 如果固件与可下载 wheel 的对应关系不明确，先联系 Kinova 支持，不要用
  “能 import”的其他版本直接控制机械臂。

### 8.6 第五步：配置 Gen3 有线网络

Kinova 官方 Python 示例假定机器人默认地址为 `192.168.1.10`，并建议主机有线
接口使用同网段静态地址，例如 `192.168.1.11/24`。如果现场已修改机器人地址，
下面所有地址都要相应替换。

1. 使用独立网线连接电脑与机械臂控制器。
2. 在 Windows“设置 → 网络和 Internet → 以太网 → IP 分配”中手动配置该
   **有线网卡**：
   - IPv4：`192.168.1.11`
   - 子网前缀长度：`24`
   - 网关和 DNS：直连场景通常留空
3. 不要修改 PICO 所用 Wi-Fi 网卡，也不要让两个 Windows 网卡使用相同静态 IP。
4. 先在 Windows 浏览器打开机械臂地址进入 Web App；只能确认页面可达和状态，
   不执行运动动作。
5. 在 WSL 中检查路由和 Kortex TCP 端口：

```bash
ip route get 192.168.1.10
nc -vz -w 2 192.168.1.10 10000
```

`nc` 必须显示端口连接成功。如果 Windows 能打开 Web App、但 WSL 无法访问：

```powershell
wsl --update
wsl --shutdown
```

重新进入 WSL 后再检查。仍失败时检查 Windows 防火墙、VPN、虚拟网卡优先级和
WSL 网络模式；在 TCP `10000` 可达前不要运行实体命令。

### 8.7 第六步：按顺序完成上机前分层检查

每次首次部署或更换电脑、Python、固件、wheel 后，严格按顺序执行：

```bash
cd ~/kinova-gen3-pico-teleop-mujoco
source ~/.venvs/kinova-pico-teleop/bin/activate
export LD_LIBRARY_PATH="$PWD/.local-deps/XRoboToolkit-PC-Service-Pybind/lib:${LD_LIBRARY_PATH:-}"

# 1. 项目和 MuJoCo
python -m pytest -q
python -m kinova_teleop.main --dry-run --headless --steps 2000

# 2. PICO 左手柄
python -m kinova_teleop.main --check-xr --samples 100

# 3. Kortex SDK
python -c "import kortex_api; print(kortex_api.__file__)"

# 4. 机器人 TCP
nc -vz -w 2 192.168.1.10 10000
```

上述四层任一失败都必须停止排障，不能输入 `MOVE`。这些检查只证明软件、输入和
端口基本可用，不证明机械臂已经安全或程序已通过实机验证。

### 8.8 第七步：上机前安全检查

1. 固定机械臂并清空工作空间；人员离开危险区域，实体急停保持可触及且已验证可用。
2. 先在 Kinova Web App 中将机械臂设为安全姿态、配置低速和工作空间限制；不要依赖本程序替代实体急停或机械臂安全配置。
3. 核对固件、Python 与 Kortex wheel 匹配，确认有线网络和 TCP `10000`。先只做 Web App/网络的**只读检查**，不要在此阶段发起运动命令。
4. 只通过环境变量提供密码；不要把密码写入命令历史、脚本、配置文件或本仓库。可在当前终端交互设置：

```bash
read -rsp "Kortex password: " KINOVA_PASSWORD; echo
export KINOVA_PASSWORD
```

### 8.9 第八步：首次低速启动和操作

首次上机从低速开始：线速度 `0.01 m/s`、角速度 `2 deg/s`。以下命令中的 IP 和用户必须替换为目标机械臂的实际值：

```bash
cd ~/kinova-gen3-pico-teleop-mujoco
source ~/.venvs/kinova-pico-teleop/bin/activate
export LD_LIBRARY_PATH="$PWD/.local-deps/XRoboToolkit-PC-Service-Pybind/lib:${LD_LIBRARY_PATH:-}"

python -m kinova_teleop.main \
  --backend kortex \
  --enable-hardware \
  --robot-ip 192.168.1.10 \
  --robot-user admin \
  --max-linear-speed 0.01 \
  --max-angular-speed-deg 2
```

命令会要求在终端中**精确输入 `MOVE`**；只有输入完全匹配后才会建立 Kortex
连接。输入 `MOVE` 前必须完全松开左 Grip，并确认手柄没有被挤压或误触。程序
没有自动 Home：连接后且 Grip 未按下时，机械臂应保持静止。程序首次收到
`Grip > 0.9` 的新鲜有效输入时，会读取当前实体末端位姿作为机械臂锚点，并以
当时手柄位姿建立相对 6DoF 映射；因此先松开 Grip，把手柄移动到舒适位置，
再按住 Grip。

实体模式不会打开 MuJoCo Viewer；终端保持运行是正常现象。操作顺序：

1. 不按 Grip，确认机械臂保持静止。
2. 左手柄放到舒适位置后按住 Grip，只做毫米级平移和小角度旋转。
3. 确认六个方向与预期一致；方向或姿态不正确时立即松开 Grip。
4. 松开 Grip 后确认实体停止；不要靠持续按 Grip 调整异常方向。
5. 按 `Ctrl+C` 退出并观察 Stop/cleanup 结果。出现
   `Stop unconfirmed` 或 cleanup failure 时，使用实体急停/Web App 确认安全，
   不要直接重启程序继续运动。
6. 每次试验结束后可以关闭程序；下一次重新执行密码设置和启动命令即可。

Grip 松开、XR 输入陈旧/无效、发送错误或程序退出时，软件会请求 Stop。`200 ms` 仅是主机侧的 stop-request/admission guarantee：它限制主机何时接受或继续发送控制，不足以证明机械臂在 200 ms 内完成物理停止。实体急停、Web App 限速和清空工作空间仍是必需的独立安全层。

### 8.10 首次实体验收清单

- [ ] 已记录本次为“实体测试”，且未把离线测试结果误记为实机验证。
- [ ] 实体急停、固定安装、清空工作空间和 Kinova Web App 的安全姿态/低速设置均已确认。
- [ ] Kortex wheel 与固件/Python 匹配；有线网络与 TCP `10000` 已通过只读检查。
- [ ] 密码仅通过 `KINOVA_PASSWORD` 环境变量提供，未出现在文件或命令历史中。
- [ ] 使用上述 `0.01 m/s`、`2 deg/s` 命令启动，并仅在提示时精确输入 `MOVE`。
- [ ] 连接后、未按 Grip 时无运动，且程序未执行自动 Home。
- [ ] 输入 `MOVE` 前 Grip 已完全松开；首次收到 `Grip > 0.9` 的有效输入时仅
      建立手柄—当前实体末端锚点，无初始跳变。
- [ ] 分别验证 Grip 松开、XR 断流、网络异常和程序退出时的 Stop 行为；记录观察到的实体停止结果。

### 8.11 一次完整启动的最短命令清单

环境已经按本节配置完成后，每次开机只需：

```bash
cd ~/kinova-gen3-pico-teleop-mujoco
source ~/.venvs/kinova-pico-teleop/bin/activate
export LD_LIBRARY_PATH="$PWD/.local-deps/XRoboToolkit-PC-Service-Pybind/lib:${LD_LIBRARY_PATH:-}"

python -m kinova_teleop.main --check-xr --samples 20
nc -vz -w 2 192.168.1.10 10000

read -rsp "Kortex password: " KINOVA_PASSWORD; echo
export KINOVA_PASSWORD

python -m kinova_teleop.main \
  --backend kortex \
  --enable-hardware \
  --robot-ip 192.168.1.10 \
  --robot-user admin \
  --max-linear-speed 0.01 \
  --max-angular-speed-deg 2
```

最后一条命令提示后，由现场操作者确认安全条件，再精确输入 `MOVE`。

## 9. 故障排查

### `No module named 'kortex_api'`

- 确认已激活安装 wheel 时使用的同一个 venv：
  `which python` 和 `python -m pip --version` 应指向
  `~/.venvs/kinova-pico-teleop/`。
- 用 `python -m pip list | grep -i kortex` 和
  `python -c "import kortex_api; print(kortex_api.__file__)"`
  检查实际安装位置。
- 不要只克隆 Kinova examples 仓库；必须按第 8.5 节安装官方 wheel。

### Kortex wheel 无法安装或 import protobuf 失败

- 重新核对机器人固件、wheel 版本和 wheel 支持的 Python 版本。
- 使用 `python -m pip check` 查看依赖冲突。
- 不要单独升级/downgrade `protobuf` 来强行消除错误；优先重新创建干净 venv，
  再安装匹配 wheel 和本项目。
- 如果匹配 wheel 不支持任何 Python `>=3.10` 版本，则当前项目组合不受支持，
  停止部署；不要退回 Python 3.9 以下强装。
- 如果改用另一个受支持的 Python `>=3.10` 版本，XR binding 也必须在那个
  venv 中重新构建。

### TCP `10000` 超时或拒绝连接

- 确认网线连接的是机械臂控制器和配置了静态地址的 Windows 有线网卡。
- 确认主机与机器人位于同一子网，且没有给 Wi-Fi 和有线网卡配置相同 IP。
- 先确认 Windows 能访问 Web App，再在 WSL 运行
  `ip route get <机器人IP>` 和 `nc -vz -w 2 <机器人IP> 10000`。
- 暂停会改写路由的 VPN；检查 Windows 防火墙和虚拟网卡优先级。
- 端口不通时不要反复输入 `MOVE`。

### Kortex 登录或 session 创建失败

- 核对 `--robot-user` 和 Web App 中可用的账户；不要假定现场密码仍是默认值。
- 重新用 `read -rsp` 设置 `KINOVA_PASSWORD`，不要把密码直接放在命令行。
- 核对 wheel 与固件；API 不匹配也可能表现为 session/RPC 错误。
- 本项目不会打印密码。排障日志如来自第三方 SDK，分享前仍应人工检查并脱敏。

### `Stop unconfirmed`、cleanup failure 或机器人状态异常

- 立即停止试验，松开 Grip；必要时按实体急停。
- 在 Web App 中确认机械臂实际状态、故障码和伺服状态。
- 不要通过提高 timeout、删除 watchdog、跳过 `MOVE` 或注释 Stop 逻辑来继续。
- 只有现场安全负责人确认机器人已静止并处理故障后，才能关闭程序重新开始。

### `nc` 连接失败

- 确认 Windows PC Service 正在运行，`setting.ini` 修改后已重启。
- 确认 `listenAddr=0.0.0.0`、端口两端均为 `60061`。
- 重新计算 `WINDOWS_HOST_IP`，WSL 重启后默认网关可能改变。
- 检查 Windows 防火墙入站规则和当前网络配置文件。

### `xrobotoolkit_sdk is unavailable` 或共享库错误

- 确认当前终端激活的是 `~/.venvs/kinova-pico-teleop`。
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

## 10. 开发验证

```bash
python -m pip install -e ".[dev]"
python -m pytest -q
python -m kinova_teleop.main --dry-run --headless --steps 5000
```

核心模型契约为：7 个关节 `joint_1`～`joint_7`、7 个位置执行器、`home` 关键帧及 `pinch_site` 末端站点。缺少其中任一项都会在启动时明确报错。

## 11. 本地已有环境的启动方式

当前本地环境可继续按既有 MuJoCo/PICO 流程启动；默认后端就是 MuJoCo：

```bash
cd ~/kinova-gen3-pico-teleop-mujoco
source ~/.venvs/kinova-pico-teleop/bin/activate
export LD_LIBRARY_PATH="$PWD/.local-deps/XRoboToolkit-PC-Service-Pybind/lib:${LD_LIBRARY_PATH:-}"
python -m kinova_teleop.main --model kinova_gen3_mujoco/teleop_scene.xml
```

不接 PICO 或仅验证本地环境时，使用离线 MuJoCo dry-run：

```bash
cd ~/kinova-gen3-pico-teleop-mujoco
source ~/.venvs/kinova-pico-teleop/bin/activate
python -m kinova_teleop.main --dry-run --headless --steps 2000
```

未来满足第 8 节全部实体安全条件、已安装匹配 Kortex wheel 且明确接受实体验收责任后，才使用第 8 节的 `--backend kortex --enable-hardware` 命令启动实体模式；它不会替代 MuJoCo 命令，也不会自动选择实体后端。

## 来源与许可

- 遥操接口设计参考 [XR-Robotics/XRoboToolkit-Teleop-Sample-Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python)。
- XR Python 绑定来自 [XRoboToolkit-PC-Service-Pybind](https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind)。
- Kortex wheel 安装方式参考
  [Kinova 官方 Kortex API 安装页](https://docs.kinovarobotics.com/getting_started/installation.html)
  和
  [官方 Python examples 说明](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_python/examples/readme.md)。
- Gen3 MJCF 来自 [MuJoCo Menagerie 的 Kinova Gen3](https://github.com/google-deepmind/mujoco_menagerie/tree/main/kinova_gen3)。
- 本项目 Python 代码采用根目录 MIT License；`kinova_gen3_mujoco/` 内模型资产保留其 BSD-3-Clause License。
