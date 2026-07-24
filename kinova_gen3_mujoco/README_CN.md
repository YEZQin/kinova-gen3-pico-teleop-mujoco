# Kinova Gen3 Ultra（7 DoF）MuJoCo 模型

## 结论

本目录是可由 MuJoCo 直接加载的 Kinova Gen3 7 自由度机械臂模型。Kinova 的正式产品名称是
“KINOVA Gen3 Ultra lightweight robot”；MuJoCo Menagerie 将其列为 “Kinova Gen3”。

工作区原有的 `../kinova/kinova.urdf` 不是 Gen3。其链接和关节均以 `j2s6s200` 命名，
对应 JACO2 6 自由度、2 指夹爪型号。

## 来源

- 上游仓库：<https://github.com/google-deepmind/mujoco_menagerie/tree/main/kinova_gen3>
- 下载时的上游提交：`71f066ad0be9cd271f7ed58c030243ef157af9f4`
- 上游说明：该 MJCF 从 Kinova 官方 `ros_kortex` 的 Gen3 URDF/Xacro 转换而来
- 许可证：BSD-3-Clause，见 `LICENSE`

## 文件

- `gen3.xml`：机械臂本体、惯性、碰撞/视觉网格、7 个位置执行器和关键帧
- `scene.xml`：包含机械臂、地面、灯光和天空盒的可视化场景
- `assets/`：9 个 STL 网格；模型默认使用带腕部视觉模块的 8 个网格
- `local_render.png`：使用本机 MuJoCo 3.10.0 从 `scene.xml` 离屏渲染的验证图

该模型包含 Gen3 7 轴机械臂和腕部视觉模块，不包含可选的 Robotiq 2F-85/2F-140
夹爪。夹爪不是 Gen3 本体的固定组成部分，Kinova 官方描述也把夹爪作为可选配置。

## 本机打开

在工作区根目录执行：

```powershell
.\.venv\Scripts\python.exe -m mujoco.viewer --mjcf .\kinova_gen3_mujoco\scene.xml
```

MuJoCo Viewer 中可用鼠标旋转、平移和缩放视角，并在右侧控制面板调整 `joint_1`
到 `joint_7`。

## 已执行的验证

- MuJoCo 版本：3.10.0
- `gen3.xml`：成功编译，`nq=7`、`nv=7`、`nu=7`、`nbody=9`、`ngeom=16`、`nmesh=8`
- `scene.xml`：成功编译，`nq=7`、`nv=7`、`nu=7`、`nbody=9`、`ngeom=17`、`nmesh=8`
- 从 `home` 关键帧连续仿真 2000 步（4 秒），位置和速度均保持有限值
- 640×480 离屏 OpenGL 渲染成功，输出为 `local_render.png`

## 原 URDF 的 MuJoCo 兼容性

原 `../kinova/kinova.urdf` 引用 Collada `.dae` 网格。MuJoCo 3.10.0 编译时返回
`no decoder found for mesh file`，因此不能直接在当前 MuJoCo 中检视。即使转换网格，
它仍然是 JACO2 J2S6S200，而不是 Gen3 Ultra。
