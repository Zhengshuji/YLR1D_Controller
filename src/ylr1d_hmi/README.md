# ylr1d_hmi

基于 Qt5 的人机交互界面，用于 YLR1D 机器人调试。提供关节状态观测 + 关节控制器。

提供两个变体：

| 变体 | 可执行文件 | Launch 文件 | 状态 |
|------|-----------|-------------|------|
| **Lite** | `ylr1d_hmi` | `hmi.launch.py` | ✅ 正常工作 |
| **RViz2** | `ylr1d_hmi_rviz` | `hmi_rviz.launch.py` | ❌ 存在构建/运行时问题 |

---

## Lite 版 — 界面布局

```
┌───────────────────────────────────────┐
│  Joint Observer                       │
│  ├─ Chassis (8)                       │
│  │   ├─ RF_Steer    0.0000  0.0000   │
│  │   ├─ LF_Steer    0.0000  0.0000   │
│  │   ├─ ...                          │
│  ├─ Torso (4)                        │
│  ├─ Left Arm (9)                     │
│  └─ Right Arm (9)                    │
├───────────────────────────────────────┤
│  Joint Controller                     │
│  ┌─────────┐ ┌─────────┐ ┌────────┐ ┌─────────┐
│  │ Chassis │ │  Torso  │ │LeftArm │ │RightArm  │
│  │ RF_Steer│ │  Lift   │ │Shdr1   │ │Shdr1    │
│  │ [====]  │ │ [====]  │ │ [====] │ │ [====]  │
│  │ ...     │ │ ...     │ │ ...    │ │ ...     │
│  └─────────┘ └─────────┘ └────────┘ └─────────┘
└───────────────────────────────────────┘
```

### 1. 关节观测器（上半部分）
- 四组树形列表：底盘(8)、躯干(4)、左臂(9)、右臂(9)
- 实时显示每个关节的 **位置 (rad)** 和 **速度 (rad/s)**
- 50Hz 更新（与 ROS spin 定时器同步）
- 订阅 `/joint_states`

### 2. 关节控制器（下半部分）
- 四组横向排列的 GroupBox
- 每个关节一行：`[名称] [Slider] [SpinBox]`
- **位置关节**（转向、躯干、臂）：Slider 范围对应关节限位，SpinBox 显示 3 位小数
- **速度关节**（轮子）：Slider ±5.0，单位 rad/s
- Slider / SpinBox 双向同步
- 发布到 `/desired_joint_states`

---

## RViz2 版（未正常工作）

嵌入 RViz2 3D 渲染窗口到界面中的版本。布局为：左侧 RViz2 渲染 + 右侧 Observer 横向分割，底部 Controller。

**已知问题：**
- 编译需要安装 `rviz_common`、`rviz_rendering`、`rviz_default_plugins`（已列在 package.xml 中）
- WSL 下 Ogre 渲染初始化容易失败（`RenderSystem::get()` 崩溃）
- X11 宏污染 Qt 枚举，通过头文件包含顺序规避（`hmi_window_rviz.cpp` 的第 3 行注释说明了当前 workaround）
- 当前未做充分测试，不推荐使用

如果后续需要修复，可参考的排查方向：
1. `rviz_rendering::RenderSystem::get()` 是否抛出 `std::runtime_error`
2. WSL 下 `LIBGL_ALWAYS_SOFTWARE=1` 是否影响 Ogre 的 GL 选择
3. VisualizationManager 的 `initialize()` 是否在 render panel 可见后才能调用

---

## 关节限位

| 分组 | 关节数 | 位置限位 | 说明 |
|------|--------|----------|------|
| 底盘-转向 | 4 | ±3.14 rad | 全向转向 |
| 底盘-轮子 | 4 | ±5.0 rad/s | 速度控制，连续旋转 |
| 躯干 | 4 | Lift: ±0.30 m, Yaw: ±3.14 rad, Pitch1/2: ±1.57 rad | |
| 左臂 | 9 | 各关节参考 `hmi_window.cpp` | Finger1/2: [-0.014, 0] m |
| 右臂 | 9 | 各关节参考 `hmi_window.cpp` | Finger1/2: [0, 0.014] m |

> 夹爪（棱柱关节）限位：左指负值张开、正值闭合；右指正值张开、负值闭合。

---

## 话题

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `/joint_states` | `sensor_msgs/JointState` | 关节反馈（来自 Gazebo） |
| 发布 | `/desired_joint_states` | `sensor_msgs/JointState` | 用户设定的期望值 |

### `/desired_joint_states` 发布规则

- **转向关节**（`Joint_Base_to_*WheelF` ×4）：`position` 字段
- **轮子关节**（`Joint_*WheelF_to_*Wheel` ×4）：`velocity` 字段
- **躯干/臂关节**（其余 22 关节）：`position` 字段
- 发布频率：50Hz（随 GUI 操作触发）

---

## 使用

```bash
# 终端 1: 启动 Gazebo + 控制器
ros2 launch ylr1d_mid_control gazebo_effort.launch.py

# 终端 2: 启动 HMI（Lite 版）
ros2 launch ylr1d_hmi hmi.launch.py
```

> WSL 下需 X server（WSLg 或 VcXsrv）支持 GUI 显示。
> 环境变量 `LIBGL_ALWAYS_SOFTWARE=1` 已在 launch 文件中自动设置。

**注意：** RViz2 版使用 `hmi_rviz.launch.py`，但当前未正常工作，详见上方说明。

---

## 依赖

### Lite 版
- **ROS2**: rclcpp, sensor_msgs, std_msgs
- **Qt5**: Widgets, Core
- 系统: qtbase5-dev, libqt5widgets5

### RViz2 版（额外）
- rviz_common, rviz_rendering, rviz_default_plugins

---

## 构建

```bash
source /opt/ros/humble/setup.bash
cd <workspace>
colcon build --packages-select ylr1d_hmi
source install/setup.bash
```

---

## 关键文件

```
ylr1d_hmi/
├── include/ylr1d_hmi/
│   ├── hmi_window.hpp              # 基础 HmiWindow 类定义（共享）
│   └── hmi_window_rviz.hpp         # HmiWindowRviz 子类定义
├── src/
│   ├── hmi_window.cpp              # 核心实现：Observer + Controller + ROS 通信
│   ├── main.cpp                    # Lite 版入口
│   ├── main_rviz.cpp               # RViz2 版入口（额外初始化 Ogre）
│   └── hmi_window_rviz.cpp         # RViz2 布局构建
├── config/
│   └── hmi.rviz                    # RViz2 显示配置
├── launch/
│   ├── hmi.launch.py               # Lite 版 launch
│   └── hmi_rviz.launch.py          # RViz2 版 launch
├── CMakeLists.txt                  # 同时构建两个可执行文件
├── package.xml
└── README.md
```

## 已知限制

- **单线程设计**：Qt 主线程同时处理 GUI 和 ROS spin（`rclcpp::spin_some` 在 QTimer 中调用），重负载下可能卡顿
- **未做代码层硬限幅**：Slider/SpinBox 的 range 约束了输入范围，但代码层面未对发布值做二次限幅
- **WSL 渲染性能**：WSL 下 GUI 响应较慢（软件渲染）
- **RViz2 版未稳定**：详见上方 "RViz2 版" 章节
