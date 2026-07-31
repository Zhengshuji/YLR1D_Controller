# ylr1d_description — 模型资产（单一来源）

YLR1D 机器人的模型资产包，是 xacro / meshes / 模型配置 / rviz / world 的**唯一来源**。
`ylr1d_plant` 与各类展示 launch 均从这里取资产。

---

## 一、包介绍

| 目录/文件 | 内容 |
|-----------|------|
| `urdf/ylr1d.xacro` | 模型宏定义，通过 `${links.X.Y}` 等占位符引用 `config/*.yaml` 的配置（launch 运行时解析） |
| `urdf/ylr1d.urdf` | **静态自包含 URDF**：模型配置已内联为数值，不引用任何外部 yaml，可随工作空间移动 |
| `config/` | 模型参数：`links`（质量/惯量/原点）、`colors`、`limits`、`scale`、`calibration`、`dynamics`、`sensors/`（传感器配置）、`controllers.yaml`（minimal 展示版） |
| `meshes/` | STL 网格（`Link_*.STL`） |
| `rviz/display.rviz` | RViz 显示配置 |
| `worlds/empty.world` | 空世界（展示用） |
| `launch/xacro_display.launch.py` | 动态生成 URDF 展示（推荐） |
| `launch/urdf_display.launch.py` | 静态 URDF 展示 |

> 模型参数以 `config/*.yaml` 为准；`ylr1d.xacro` 是模板，`ylr1d.urdf` 是解析后的产物。

---

## 二、使用方法

### 1. 纯展示（RViz + Gazebo，无控制器）

```bash
# 动态生成（推荐，始终拿到最新模型参数）
ros2 launch ylr1d_description xacro_display.launch.py

# 静态自包含 URDF（不依赖外部 yaml）
ros2 launch ylr1d_description urdf_display.launch.py
```

两个 launch 都会启动 robot_state_publisher + joint_state_publisher_gui + gzserver + gzclient + RViz，
可在 RViz 拖动滑杆预览关节运动。

### 2. 供其他包使用

- `ylr1d_plant` 的 `gazebo.launch.py` / `gazebo_effort.launch.py` 从本包取 xacro、模型 config、rviz、world
- 命令行查看 URDF：

```bash
# 通过 xacro 动态生成（结果输出到 stdout）
ros2 run xacro xacro src/ylr1d_description/urdf/ylr1d.xacro
```

---

## 三、详细说明

### 模型配置（config/*.yaml）

| 文件 | 内容 |
|------|------|
| `links.yaml` | 各 link 的 mass / inertia / inertial origin |
| `colors.yaml` | 各部件材质颜色 |
| `limits.yaml` | 关节限位（effort/lower/upper/velocity） |
| `scale.yaml` | 模型缩放 |
| `calibration.yaml` | 关节校准（falling/rising） |
| `dynamics.yaml` | 关节阻尼/摩擦 |
| `sensors/*.yaml` | 相机/雷达/IMU/超声波传感器参数 |

launch 通过 `_resolve_yaml_refs`（正则预处理器）在 xacro 处理前把 `${links.X.Y}` 等占位符
替换为 yaml 中的数值，再交给 xacro 展开。

### 静态 URDF（ylr1d.urdf）

- 是 `ylr1d.xacro` + `config/*.yaml` 解析后的产物，模型配置已内联为数值
- 不包含 `<parameters>`（controllers.yaml）引用，**完全自包含**，不受工作空间位置影响
- 展示 launch 无 controller spawner，因此不需要 controllers.yaml

### GAZEBO_MODEL_PATH

Gazebo 会把 `package://ylr1d_description/meshes/*.STL` 重写为 `model://ylr1d_description/...`。
launch 已自动把 meshes 目录与包 share 目录加入 `GAZEBO_MODEL_PATH`，一般无需手动设置。

---

## 四、补充说明

- `config/controllers.yaml` 是 **minimal 展示版**（供 desc 展示 launch 用），与 `ylr1d_plant` 的
  `config/controllers.yaml` 相互独立、各归其位
- 修改模型参数：改 `config/*.yaml` → 用 `xacro_display.launch.py` 预览 → 如需静态版则重新生成
  `ylr1d.urdf`（沿用 `_resolve_yaml_refs` + xacro 流程）

---

## 五、问题解决

### WSL 下 Gazebo 显示问题
- GPU 受限：launch 已设 `LIBGL_ALWAYS_SOFTWARE=1`
- 控制器/模型加载慢：WSL 下耐心等待（30-60s），不要抢跑

### meshes 找不到
- 确认 `GAZEBO_MODEL_PATH` 包含 meshes 目录与包 share 目录（launch 已自动处理）
- 若手动启动 gzserver，需自行 export
