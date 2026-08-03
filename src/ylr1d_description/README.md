# ylr1d_description — 模型资产（单一来源）

YLR1D 机器人的模型资产包，是 xacro / meshes / 模型配置 yaml / rviz / world 的**唯一来源**。`ylr1d_plant` 与各类展示 launch 均从这里取资产。

---

## 一、功能定位

- **架构位置**：最底层，纯资产提供方，不参与运行期控制。
- **职责**：统一维护模型定义（xacro 模板 + config 参数），让物理层与展示层拿到同一份模型。
- **不做什么**：不启动控制器、无运行期节点、无 topic 收发。

---

## 二、包结构

| 目录/文件 | 内容 |
|-----------|------|
| `urdf/ylr1d.xacro` | 模型宏定义，通过 `${links.X.Y}` 等占位符引用 `config/*.yaml`（由 launch 公共模块运行时解析） |
| `urdf/ylr1d.urdf` | **静态自包含 URDF**：配置已内联为数值，不引用外部 yaml，可随工作空间移动 |
| `config/` | 模型参数 yaml（links / colors / limits / calibration / dynamics / scale）+ `controllers.yaml`（minimal 展示版） |
| `meshes/` | STL 网格（`Link_*.STL`） |
| `rviz/display.rviz` | RViz 显示配置 |
| `worlds/` | `empty.world`（默认）、`sensors_test.world`（传感器测试用） |
| `launch/xacro_display.launch.py` | 动态生成 URDF 展示（推荐） |
| `launch/urdf_display.launch.py` | 静态 URDF 展示 |
| `launch/sensor_vis.launch.py` | 传感器可视化诊断（固定加载 `ylr1d_sensor_vis.xacro` + `sensors_test.world`，不起控制器） |
| `urdf/ylr1d_sensor_vis.xacro` | 传感器可视化诊断副本（加了 `<visualize>true</visualize>`，并修正了部分传感器安装朝向，见[关键机制](#六关键机制选读)） |
| `launch/python_utils/xacro_utils.py` | **xacro → URDF 公共导入逻辑（单一来源）**，本包与 `ylr1d_plant` 的 launch 共用 |

> 模型参数以 `config/*.yaml` 为准；`ylr1d.xacro` 是模板，`ylr1d.urdf` 是解析后的产物。

---

## 三、使用方法

### 1. 纯展示（RViz + Gazebo，无控制器）

```bash
# 动态生成（推荐，始终拿到最新模型参数）
ros2 launch ylr1d_description xacro_display.launch.py

# 静态自包含 URDF（不依赖外部 yaml）
ros2 launch ylr1d_description urdf_display.launch.py

# 传感器可视化诊断
ros2 launch ylr1d_description sensor_vis.launch.py
```

展示 launch 都会启动 robot_state_publisher + joint_state_publisher_gui + gzserver + gzclient + RViz，可在 RViz 拖动滑杆预览关节运动。

### 2. 供其他包使用

`ylr1d_plant` 的 `gazebo.launch.py` / `gazebo_effort.launch.py` 从本包取 xacro、模型 config、rviz、world，并注入它们自己的 `controllers.yaml`；xacro → URDF 统一走本包公共模块 `xacro_utils`。

> ⚠️ 命令行直接跑 xacro **会失败**：`${links.X.Y}` 等占位符必须由 `xacro_utils.resolve_yaml_refs` 预替换（xacro 本身不加载 yaml），实测报错 `name 'links' is not defined`。预览请用展示 launch。

---

## 四、接口

- 本包**无运行期 topic / action / 服务**，属于纯资产提供方。
- 对外以参数方式提供 `robot_description`（经 robot_state_publisher 从本包 URDF 加载）。
- 对外提供 3 个展示 launch 与公共模块 `xacro_utils.py`（供 `ylr1d_plant` 复用）。

---

## 五、配置

### 模型配置（config/*.yaml）

| 文件 | 内容 | 实际生效状态 |
|------|------|--------------|
| `links.yaml` | 各 link 的 mass / inertia | ✅ 被 xacro 引用（inertial `origin` 硬编码在 xacro 里） |
| `colors.yaml` | 各部件材质颜色 | ✅ 被 xacro 引用 |
| `limits.yaml` | 关节限位（effort/lower/upper/velocity） | ✅ 被 xacro 引用 |
| `calibration.yaml` | 关节校准（falling/rising） | ✅ 被 xacro 引用（当前全 0） |
| `dynamics.yaml` | 关节阻尼/摩擦 | ✅ 被 xacro 引用（当前全 0） |
| `scale.yaml` | **密度**（density，非"缩放"） | ⚠️ xacro 中无 `${scale.*}` 引用，改它**不生效**（待接入 xacro） |
| `sensors/` | 传感器参数（9 个 yaml） | ⚠️ 传感器实际硬编码在 `ylr1d.xacro` 内，改它们**不生效**（待接入 xacro） |

`config/sensors/` 目录：`rgb_camera` / `depth_camera` / `infrared_camera` / `radar_sensor` / `imu_sensor` / `lf|rf|lb|rb_ultrasonic_sensor` 共 9 个 yaml。

> launch 通过公共模块 `xacro_utils.resolve_yaml_refs`（正则预处理器）在 xacro 处理前把 `${links.X.Y}` 等占位符替换为 yaml 数值，再交给 xacro 展开。

### 静态 URDF（ylr1d.urdf）

- 是 `ylr1d.xacro` + `config/*.yaml` 解析后的产物，配置已内联为数值，**完全自包含**，不受工作空间位置影响。
- 仍保留 `<ros2_control>` / `gazebo_ros2_control`（GazeboSystem）块，只是没有 controllers.yaml——"纯展示、无控制器"模式。

### GAZEBO_MODEL_PATH

Gazebo 会把 `package://ylr1d_description/meshes/*.STL` 重写为 `model://ylr1d_description/...`。launch 已自动把 meshes 目录与包 share 目录加入 `GAZEBO_MODEL_PATH`，一般无需手动设置。

---

## 六、关键机制（选读）

### xacro → URDF 公共管道

`launch/python_utils/xacro_utils.py` 是唯一入口：`resolve_yaml_refs`（加载 `config/*.yaml` + `config/sensors/*.yaml` 超集，预替换占位符）+ `process_xacro_to_urdf(xacro_path, config_dir, controllers_yaml, transforms=())`（`transforms` 承接 effort 版注入 `<command_interface effort/>`）。

### 模型资产细节

- **传感器共 15 个，全部硬编码在 `ylr1d.xacro`**（`<gazebo reference=...>` 内）：`Link_GlobalCameraSensor` / `Link_LeftCameraSensor` / `Link_RightCameraSensor` 各含 rgb/depth/infrared 相机，外加雷达、IMU、4 个超声波（LF/RF/LB/RB）。主题：`/{global,left,right}_camera/{rgb,depth,infrared}/image_raw`、`/radar/scan`、`/imu_data`、`/{lf,rf,lb,rb}_ultrasonic/range`
- **`ros2_control` 接口划分（30 个可动关节，全走 `gazebo_ros2_control/GazeboSystem`）**：转向 4 × position、车轮 4 × velocity、躯干 4 × position、双臂 7+7 × position、手指 4 × position
- `ylr1d_plant` 的 launch 支持 `world:=` 参数切换世界：`ros2 launch ylr1d_plant gazebo.launch.py world:=sensors_test.world`

### 传感器方向修正记录（ylr1d.xacro → ylr1d_sensor_vis.xacro）

`ylr1d_sensor_vis.xacro` 是从 `ylr1d.xacro` 复制的诊断模型，除给每个传感器加 `<visualize>true</visualize>` 外，还修正了部分传感器**安装朝向**（只改 joint `origin` 的 `rpy`，`xyz` 不动）。逐传感器对比（原始值 → 修正值）：

| 传感器 joint | 修改说明 |
|---|---|
| `Joint_Body4_to_GlobalCameraSensor`（主相机） | roll/pitch 交换，光轴由侧向拨正朝前 |
| `Joint_LeftArm7_to_LeftCameraSensor`（左臂相机） | 增 roll≈180° + pitch≈-90°，光轴拨正朝前 |
| `Joint_RightArm7_to_RightCameraSensor`（右臂相机） | 增 pitch≈-90°，光轴拨正朝前 |
| `Joint_Base_to_RadarSensor`（雷达） | 去掉 roll=90°，扫描圆面放平到水平面（⚠️ 其 `<visualize>` 已注释，暂不显示） |
| 4 个超声波（LF/RF/LB/RB） | yaw 统一回拨 90°，修正俯视下整体逆时针偏 90° |

**结论**：相机光轴统一拨正朝前；雷达去掉 roll 使扫描平面放平；超声波 yaw 统一回拨 90°。这些修正目前只落在诊断模型 `ylr1d_sensor_vis.xacro` 中，**正式模型 `ylr1d.xacro` 尚未同步**——若确认方向正确，需将上述 rpy 回填到正式 xacro。

### launch 运行时行为（Gazebo 相关）

- `gzserver` 无界面启动（`-s libgazebo_ros_init.so -s libgazebo_ros_factory.so`），`gzclient` 延迟 5 s 启动
- 环境注入：`LIBGL_ALWAYS_SOFTWARE=1`、`GAZEBO_MODEL_DATABASE_URI=""`、`LD_LIBRARY_PATH`（追加 `/opt/ros/humble/lib`）、`GAZEBO_MODEL_PATH`（追加 meshes 目录与包 share 目录）

---

## 七、已知限制与注意事项

- **修改模型参数流程**：改 `config/*.yaml` → 用 `xacro_display.launch.py` 预览。⚠️ 包内**没有**重新生成 `ylr1d.urdf` 的现成命令（launch 只生成到 `/tmp` 临时文件）；如需落盘可调用 `xacro_utils.process_xacro_to_urdf` 自行生成
- `config/controllers.yaml` 是 **minimal 展示版**（供展示 launch 用），与 `ylr1d_plant` 的 `config/controllers.yaml` 相互独立、各归其位
- xacro 中的 `base_footprint` 虚拟连接被注释掉，TF 根帧为 `Link_Base`（rviz 固定帧同）
- 展示 launch 无 controller spawner，因此不需要 controllers.yaml
- WSL 下 Gazebo 显示问题：launch 已设 `LIBGL_ALWAYS_SOFTWARE=1`；控制器/模型加载慢需耐心等待（30-60s）
