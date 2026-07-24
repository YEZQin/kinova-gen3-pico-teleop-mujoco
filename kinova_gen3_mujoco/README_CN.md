# Kinova Gen3 Ultra（7DoF）MuJoCo 模型

## 结论

本目录是可由 MuJoCo 直接加载的 Kinova Gen3 7 自由度机械臂模型。Kinova 的正式产品名称为 “KINOVA Gen3 Ultra lightweight robot”，MuJoCo Menagerie 将该模型列为 “Kinova Gen3”。

工作区旧的 `../kinova/kinova.urdf` 不是 Gen3：其链接和关节使用 `j2s6s200` 命名，对应 JACO2 6DoF。该旧 URDF 还引用 MuJoCo 不能直接解码的 Collada `.dae` 网格，因此本项目不使用它。

## 来源与许可

- 上游：[google-deepmind/mujoco_menagerie/kinova_gen3](https://github.com/google-deepmind/mujoco_menagerie/tree/main/kinova_gen3)
- 下载时的上游提交：`71f066ad0be9cd271f7ed58c030243ef157af9f4`
- 上游说明：MJCF 由 Kinova 官方 `ros_kortex` 的 Gen3 URDF/Xacro 转换而来。
- 模型许可：BSD-3-Clause，见本目录 `LICENSE`。

## 文件

- `gen3.xml`：机器人本体、惯性、碰撞/视觉网格、7 个位置执行器和 `home` 关键帧。
- `scene.xml`：机器人、地面、灯光和天空盒。
- `teleop_scene.xml`：在 `scene.xml` 基础上增加遥操目标坐标架。
- `assets/`：8 个 STL 网格。
- `local_render.png`：本机 MuJoCo 离屏渲染验证图。

模型包含带腕部视觉模块的 Gen3 本体，不包含可选 Robotiq 夹爪；遥操程序也没有夹爪行为。

## 加载与检视

在项目根目录运行：

```bash
python -m mujoco.viewer --mjcf kinova_gen3_mujoco/scene.xml
```

遥操场景：

```bash
python -m mujoco.viewer --mjcf kinova_gen3_mujoco/teleop_scene.xml
```

## 已验证的模型契约

- `nq=7`、`nv=7`、`nu=7`
- 关节：`joint_1`～`joint_7`
- 每个关节恰好对应一个位置执行器
- 末端站点：`pinch_site`
- 初始关键帧：`home`
- `scene.xml` 和 `teleop_scene.xml` 均可由 MuJoCo 3.10.0 编译
- 从 `home` 状态进行无头仿真后，`qpos`、`qvel` 和 `ctrl` 保持有限值

项目自动化测试会在每次运行时重新检查上述契约。
