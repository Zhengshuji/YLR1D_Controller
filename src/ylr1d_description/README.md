# ylr1d_description — 模型资产（单一来源）

YLR1D 机器人的模型资产包，是 xacro / meshes / 模型配置 / rviz / world 的**唯一来源**。
`ylr1d_plant` 与各类展示 launch 均从这里取资产。

---

## 一、包介绍

| 目录/文件 | 内容 |
|-----------|------|
| `urdf/ylr1d.xacro` | 模型宏定义，通过 `${links.X.Y}` 等占位符引用 `config/*.yaml` 的配置（launch 公共模块 `xacro_utils` 运行时解析） |
| `urdf/ylr1d.urdf` | **静态自包含 URDF**：模型配置已内联为数值，不引用任何外部 yaml，可随工作空间移动 |
| `config/` | 模型参数：`links`（质量/惯量）、`colors`、`limits`、`calibration`、`dynamics`；`scale`（密度）与 `sensors/` 为**参考/未消费**配置（待接入 xacro）；`controllers.yaml`（minimal 展示版） |
| `meshes/` | STL 网格（`Link_*.STL`） |
| `rviz/display.rviz` | RViz 显示配置 |
| `worlds/empty.world` | 空世界（仅地面+太阳，展示用，默认） |
| `worlds/sensors_test.world` | 传感器测试世界（8 个障碍物分布在 0.5~3 m，供相机/雷达/超声波有物可测） |
| `model.config` | Gazebo model 数据库残留（指向不存在的 `model.sdf`），未被任何流程使用 |
| `launch/xacro_display.launch.py` | 动态生成 URDF 展示（推荐） |
| `launch/urdf_display.launch.py` | 静态 URDF 展示 |
| `launch/python_utils/xacro_utils.py` | **xacro → URDF 公共导入逻辑（单一来源）**：`resolve_yaml_refs`（yaml 预替换）+ `process_xacro_to_urdf`（完整管道）；本包两个展示 launch 与 `ylr1d_plant` 的两个 gazebo launch 均从这里导入 |
| `urdf/ylr1d_sensor_vis.xacro` | **传感器可视化诊断副本**：由 `ylr1d.xacro` 复制，给每个传感器加 `<visualize>true</visualize>`（ray 画扫描射线、camera 画视锥），并修正了部分传感器安装朝向（详见下方"传感器方向修正记录"） |
| `launch/sensor_vis.launch.py` | 传感器可视化测试 launch：固定加载 `ylr1d_sensor_vis.xacro` + `sensors_test.world`，spawn 在 (0,0,0.3)，不起控制器 |

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

- `ylr1d_plant` 的 `gazebo.launch.py` / `gazebo_effort.launch.py` 从本包取 xacro、模型 config、rviz、world，并注入**它们自己的** `controllers.yaml`；xacro → URDF 统一走本包公共模块 `launch/python_utils/xacro_utils.py` 的 `process_xacro_to_urdf`
- ⚠️ 命令行直接跑 xacro **会失败**：`${links.X.Y}` 等占位符必须由 `xacro_utils.resolve_yaml_refs` 预替换（xacro 本身不加载 yaml），实测报错 `name 'links' is not defined`：

```bash
# ❌ 失败：占位符未预替换
ros2 run xacro xacro src/ylr1d_description/urdf/ylr1d.xacro
# ✅ 正确：用 xacro_display launch 预览，或 `from xacro_utils import process_xacro_to_urdf` 处理后生成 URDF
```

---

## 三、详细说明

### 模型配置（config/*.yaml）

| 文件 | 内容 | 实际生效状态 |
|------|------|--------------|
| [`links.yaml`](config/links.yaml) | 各 link 的 mass / inertia | ✅ 被 xacro 引用（inertial `origin` 硬编码在 xacro 里） |
| [`colors.yaml`](config/colors.yaml) | 各部件材质颜色 | ✅ 被 xacro 引用 |
| [`limits.yaml`](config/limits.yaml) | 关节限位（effort/lower/upper/velocity） | ✅ 被 xacro 引用 |
| [`calibration.yaml`](config/calibration.yaml) | 关节校准（falling/rising） | ✅ 被 xacro 引用（当前全 0） |
| [`dynamics.yaml`](config/dynamics.yaml) | 关节阻尼/摩擦 | ✅ 被 xacro 引用（当前全 0） |
| [`scale.yaml`](config/scale.yaml) | **密度**（density，非"缩放"） | ⚠️ xacro 中无 `${scale.*}` 引用，改它**不生效**（待接入 xacro） |
| `sensors/` | 传感器参数（见下表） | ⚠️ 传感器实际硬编码在 `ylr1d.xacro` 内，改它们**不生效**（待接入 xacro） |

`sensors/` 目录下的具体文件：

| 文件 | 内容 |
|------|------|
| [`rgb_camera.yaml`](config/sensors/rgb_camera.yaml) | RGB 相机 |
| [`depth_camera.yaml`](config/sensors/depth_camera.yaml) | 深度相机 |
| [`infrared_camera.yaml`](config/sensors/infrared_camera.yaml) | 红外相机 |
| [`radar_sensor.yaml`](config/sensors/radar_sensor.yaml) | 雷达 |
| [`imu_sensor.yaml`](config/sensors/imu_sensor.yaml) | IMU |
| [`lf_ultrasonic_sensor.yaml`](config/sensors/lf_ultrasonic_sensor.yaml) | 左前超声波 |
| [`rf_ultrasonic_sensor.yaml`](config/sensors/rf_ultrasonic_sensor.yaml) | 右前超声波 |
| [`lb_ultrasonic_sensor.yaml`](config/sensors/lb_ultrasonic_sensor.yaml) | 左后超声波 |
| [`rb_ultrasonic_sensor.yaml`](config/sensors/rb_ultrasonic_sensor.yaml) | 右后超声波 |

launch 通过公共模块 `launch/python_utils/xacro_utils.py` 的 `resolve_yaml_refs`（正则预处理器）
在 xacro 处理前把 `${links.X.Y}` 等占位符替换为 yaml 中的数值，再交给 xacro 展开。

### 静态 URDF（ylr1d.urdf）

- 是 `ylr1d.xacro` + `config/*.yaml` 解析后的产物，模型配置已内联为数值
- 不包含 `<parameters>`（controllers.yaml）引用，**完全自包含**，不受工作空间位置影响
- 展示 launch 无 controller spawner，因此不需要 controllers.yaml
- 注意：静态 urdf 内仍保留 `<ros2_control>` / `gazebo_ros2_control`（GazeboSystem）块，只是没有 controllers.yaml —— 属于"纯展示、无控制器"模式

### GAZEBO_MODEL_PATH

Gazebo 会把 `package://ylr1d_description/meshes/*.STL` 重写为 `model://ylr1d_description/...`。
launch 已自动把 meshes 目录与包 share 目录加入 `GAZEBO_MODEL_PATH`，一般无需手动设置。

---

## 四、补充说明

- `config/controllers.yaml` 是 **minimal 展示版**（供 desc 展示 launch 用），与 `ylr1d_plant` 的
  `config/controllers.yaml` 相互独立、各归其位
- 修改模型参数：改 `config/*.yaml` → 用 `xacro_display.launch.py` 预览。⚠️ 包内**没有**重新生成
  `ylr1d.urdf` 的现成命令（launch 只生成到 `/tmp` 临时文件）；如需落盘可调用
  `launch/python_utils/xacro_utils.py` 的 `process_xacro_to_urdf` 自行生成
- xacro 导入逻辑统一在 `launch/python_utils/xacro_utils.py`（单一来源）：`resolve_yaml_refs` 加载
  `config/*.yaml` + `config/sensors/*.yaml`（超集），`process_xacro_to_urdf(xacro_path, config_dir,
  controllers_yaml, transforms=())` 通过 `transforms` 承接 effort 版注入 `<command_interface effort/>`
- xacro 中的 `base_footprint` 虚拟连接被注释掉，TF 根帧为 `Link_Base`（rviz 固定帧同）

### 模型资产细节（从代码可直接看到，README 概览未覆盖）

- **传感器共 15 个，全部硬编码在 `ylr1d.xacro`**（`<gazebo reference=...>` 内）：
  `Link_GlobalCameraSensor` / `Link_LeftCameraSensor` / `Link_RightCameraSensor` 各含 rgb/depth/infrared 相机，
  外加雷达、IMU、4 个超声波（LF/RF/LB/RB）。
  主题：`/{global,left,right}_camera/{rgb,depth,infrared}/image_raw`、`/radar/scan`、`/imu_data`、
  `/{lf,rf,lb,rb}_ultrasonic/range`
- **`ros2_control` 接口划分（30 个可动关节，全走 `gazebo_ros2_control/GazeboSystem`）**：
  转向 4 × position、车轮 4 × velocity、躯干 4 × position、双臂 7+7 × position、手指 4 × position
- `ylr1d_plant` 的 launch 支持 `world:=` 参数切换世界：
  `ros2 launch ylr1d_plant gazebo.launch.py world:=sensors_test.world`

### 传感器方向修正记录（ylr1d.xacro → ylr1d_sensor_vis.xacro）

`ylr1d_sensor_vis.xacro` 是从 `ylr1d.xacro` 复制的诊断模型，除给每个传感器加
`<visualize>true</visualize>`（ray 画扫描射线、camera 画视锥）外，还修正了部分传感器的
**安装朝向**（joint `origin` 的 `rpy`）。所有传感器 joint 的 `xyz` 均未改动，只改 `rpy`。
逐传感器对比（原始值 → 修正值）：

| 传感器 joint | 原始 rpy（`ylr1d.xacro`） | 修正 rpy（sensor_vis） | 修改说明 |
|---|---|---|---|
| `Joint_Body4_to_GlobalCameraSensor`（主相机） | `0.000125 1.56906 0.000125` | `1.56906 0 0.000125` | roll/pitch 交换（≈90° 由 pitch 移到 roll）：光轴（相机本地 -Z）由侧向拨正朝前 |
| `Joint_LeftArm7_to_LeftCameraSensor`（左臂相机） | `0 0 -1.5769` | `3.14 -1.57687 -1.5769` | 增加 roll≈180° + pitch≈-90°：光轴拨正朝前 |
| `Joint_RightArm7_to_RightCameraSensor`（右臂相机） | `0 0 -1.57687` | `0 -1.57687 -1.57687` | 增加 pitch≈-90°：光轴拨正朝前 |
| `Joint_Base_to_RadarSensor`（雷达） | `1.5708 0 -2.9674` | `0 0 -2.9674` | 去掉 roll=90°：原扫描圆面"立起来"，去掉后放平到水平面。⚠️ 其 `<visualize>` 已注释，暂不显示 |
| `Joint_Base_to_LFUltrasonicSensor`（左前超声波） | `1.5708 0 -0.7854` | `1.5708 0 -2.3562` | yaw 减 90°：修正俯视下整体逆时针偏 90° |
| `Joint_Base_to_RFUltrasonicSensor`（右前超声波） | `1.5708 0 -2.3562` | `1.5708 0 2.3562` | 同上（yaw 减 90° 等价） |
| `Joint_Base_to_LBUltrasonicSensor`（左后超声波） | `1.5708 0 0.7854` | `1.5708 0 -0.7854` | 同上 |
| `Joint_Base_to_RBUltrasonicSensor`（右后超声波） | `1.5708 0 2.3562` | `1.5708 0 0.7854` | 同上 |

**结论**：相机光轴统一拨正朝前；雷达去掉 roll 使扫描平面放平；4 个超声波 yaw 统一回拨 90°。
这些修正目前只落在诊断模型 `ylr1d_sensor_vis.xacro` 中，**正式模型 `ylr1d.xacro` 尚未同步**——
若确认方向正确，需将上述 rpy 回填到正式 xacro。

### launch 运行时行为（Gazebo 相关）

- `gzserver` 以无界面模式启动（`-s libgazebo_ros_init.so -s libgazebo_ros_factory.so`），`gzclient` 延迟 5 s 启动
- 环境注入：`LIBGL_ALWAYS_SOFTWARE=1`、`GAZEBO_MODEL_DATABASE_URI=""`、`LD_LIBRARY_PATH`（追加 `/opt/ros/humble/lib`）、
  `GAZEBO_MODEL_PATH`（追加 meshes 目录与包 share 目录）

---

## 五、问题解决

### WSL 下 Gazebo 显示问题
- GPU 受限：launch 已设 `LIBGL_ALWAYS_SOFTWARE=1`
- 控制器/模型加载慢：WSL 下耐心等待（30-60s），不要抢跑

### meshes 找不到
- 确认 `GAZEBO_MODEL_PATH` 包含 meshes 目录与包 share 目录（launch 已自动处理）
- 若手动启动 gzserver，需自行 export
