# ylr1d_mid_control — 中间层控制器（Gazebo 仿真测试）

> **测试专用包，非生产代码。** 用于在 Gazebo Classic + ros2_control 环境中验证 YLR1D 机器人的关节控制方案。

---

## 三种控制方案

本包提供三种可选的关节控制方案，通过不同的 launch 文件启动：

| 方案 | 接口类型 | 控制器后缀 | Launch 文件 | 配置文件 | 状态 |
|------|----------|------------|-------------|----------|------|
| **原始（位置/速度）** | 转向`position` + 车轮`velocity` + 其余`position` | `_controller` | `gazebo.launch.py` | `controllers.yaml` | ✅ 已验证 |
| **力控制**（推荐） | `effort` | `_effort_controller` | `gazebo_effort.launch.py` | `effort_controllers.yaml` | ✅ 已验证 |
| **速度控制**（新增） | `velocity` | `_velocity_controller` | `gazebo_velocity.launch.py` | `velocity_controllers.yaml` | ✅ 已验证 |
| ~~加速度控制~~ | ~~`acceleration`~~ | ~~`_accel_controller`~~ | ~~`gazebo_accel.launch.py`~~ | ~~`acceleration_controllers.yaml`~~ | ❌ 已弃用 |

> **加速度方案弃用原因**：`gazebo_ros2_control/GazeboSystem` 的 `ControlMethod` 枚举（NONE, POSITION, VELOCITY, EFFORT, VELOCITY_PID, POSITION_PID）不包含 `acceleration`，注册命令接口时静默丢弃，导致 controller_manager 报 `"Not acceptable command interfaces combination"` 错误。

---

## 方案详解

### 1. 原始位置/速度控制

不修改 xacro，使用 `ylr1d_mid.xacro` 中硬编码的接口定义。

| 控制器名 | 类型 | 关节数 | 接口 | 话题 |
|----------|------|--------|------|------|
| `joint_state_broadcaster` | `JointStateBroadcaster` | — | state | `/joint_states` |
| `chassis_steering_controller` | `ForwardCommandController` | 4 | position | `/chassis_steering_controller/commands` |
| `chassis_wheels_controller` | `ForwardCommandController` | 4 | velocity | `/chassis_wheels_controller/commands` |
| `torso_controller` | `ForwardCommandController` | 4 | position | `/torso_controller/commands` |
| `left_arm_controller` | `ForwardCommandController` | 9 | position | `/left_arm_controller/commands` |
| `right_arm_controller` | `ForwardCommandController` | 9 | position | `/right_arm_controller/commands` |

**关节顺序：**
- `chassis_steering_controller`: `[RFWheelF, LFWheelF, RBWheelF, LBWheelF]`
- `chassis_wheels_controller`: `[RFWheel, LFWheel, RBWheel, LBWheel]`
- `torso_controller`: `[Base_to_Body1, Body1_to_Body2, Body2_to_Body3, Body3_to_Body4]`
- `left_arm_controller`: `[Body2_to_LeftArm1..7, LeftFinger1, LeftFinger2]`
- `right_arm_controller`: `[Body2_RightArm1..7, RightFinger1, RightFinger2]`

> **夹爪说明：** 左右臂各附带 2 个夹爪（棱柱关节），范围 ±0.05m，单位**米**。data 数组最后 2 个值对应夹爪，前 7 个为旋转关节（弧度）。

```bash
# 启动
ros2 launch ylr1d_mid_control gazebo.launch.py

# ── 转向轮偏角（弧度 position） ──
ros2 topic pub /chassis_steering_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.3, -0.3, 0.3, -0.3]}" --once

# ── 车轮滚动（rad/s velocity） ──
ros2 topic pub /chassis_wheels_controller/commands \
  std_msgs/Float64MultiArray "{data: [2.0, 2.0, 2.0, 2.0]}" --once

# ── 躯干各关节（弧度 position） ──
ros2 topic pub /torso_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.1, 0.0, 0.0, 0.0]}" --once

# ── 左臂各关节（弧度 position） ──
ros2 topic pub /left_arm_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once

# ── 右臂各关节（弧度 position） ──
ros2 topic pub /right_arm_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once

# ── 以 10Hz 持续发送（适用速度/力控制） ──
ros2 topic pub --rate 10 /chassis_wheels_controller/commands \
  std_msgs/Float64MultiArray "{data: [2.0, 2.0, 2.0, 2.0]}"
```

---

### 2. 力控制（推荐）

在预处理阶段向 `ylr1d_mid.xacro` 中每个关节的 ros2_control 块注入 `<command_interface name="effort"/>`。GazeboSystem 原生支持 `effort` 接口，底层调用 `joint->SetForce()`。

| 控制器名 | 接口 | 关节数 | 话题 |
|----------|------|--------|------|
| `chassis_steering_effort_controller` | effort | 4 | `/chassis_steering_effort_controller/commands` |
| `chassis_wheels_effort_controller` | effort | 4 | `/chassis_wheels_effort_controller/commands` |
| `torso_effort_controller` | effort | 4 | `/torso_effort_controller/commands` |
| `left_arm_effort_controller` | effort | 9 | `/left_arm_effort_controller/commands` |
| `right_arm_effort_controller` | effort | 9 | `/right_arm_effort_controller/commands` |

关节顺序与原始方案一致。

> **夹爪说明：** 同上，data 数组最后 2 个值为夹爪（棱柱关节，单位 m），范围 ±0.05m。

```bash

# ── 转向轮偏角力矩（Nm） ──
ros2 topic pub /chassis_steering_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.5, -0.5, 0.5, -0.5]}" --once

# ── 车轮驱动力矩（Nm） ──
ros2 topic pub /chassis_wheels_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [1.0, 1.0, 1.0, 1.0]}" --once

# ── 躯干升降关节施加 20N（棱柱关节为力），其余关节 0 ──
ros2 topic pub /torso_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [20.0, 0.0, 0.0, 0.0]}" --once

# ── 左臂各关节施加 0.5Nm ──
ros2 topic pub /left_arm_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.0]}" --once

# ── 右臂各关节施加 -0.3Nm ──
ros2 topic pub /right_arm_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [-0.3, -0.3, -0.3, -0.3, -0.3, -0.3, -0.3, 0.0, 0.0]}" --once

# ── 以 10Hz 持续发送 ──
ros2 topic pub --rate 10 /chassis_wheels_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [1.0, 1.0, 1.0, 1.0]}"
```

---

### 3. 速度控制（新增）

在预处理阶段向 `ylr1d_mid.xacro` 中仅有 `position` 命令接口的关节注入 `<command_interface name="velocity"/>`（车轮关节已有 `velocity`，不重复注入）。所有控制器使用 `velocity` 接口，适合上层做速度前馈控制。

底层使用 `gazebo_ros2_control/GazeboSystem` 的 `VELOCITY` 控制方式（GazeboSystem 原生支持），底层调用 `joint->SetVelocity()`。

| 控制器名 | 接口 | 关节数 | 话题 |
|----------|------|--------|------|
| `chassis_steering_velocity_controller` | velocity | 4 | `/chassis_steering_velocity_controller/commands` |
| `chassis_wheels_velocity_controller` | velocity | 4 | `/chassis_wheels_velocity_controller/commands` |
| `torso_velocity_controller` | velocity | 4 | `/torso_velocity_controller/commands` |
| `left_arm_velocity_controller` | velocity | 9 | `/left_arm_velocity_controller/commands` |
| `right_arm_velocity_controller` | velocity | 9 | `/right_arm_velocity_controller/commands` |

关节顺序与原始方案一致。

```bash
# 启动
ros2 launch ylr1d_mid_control gazebo_velocity.launch.py

# ── 转向轮偏转角速度（rad/s） ──
ros2 topic pub /chassis_steering_velocity_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.2, -0.2, 0.2, -0.2]}" --once

# ── 车轮滚动速度（rad/s） ──
ros2 topic pub /chassis_wheels_velocity_controller/commands \
  std_msgs/Float64MultiArray "{data: [3.0, 3.0, 3.0, 3.0]}" --once

# ── 躯干各关节速度（rad/s，棱柱关节为 m/s） ──
ros2 topic pub /torso_velocity_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.05, 0.0, 0.0, 0.0]}" --once

# ── 左臂各关节速度（rad/s） ──
ros2 topic pub /left_arm_velocity_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0, 0.0]}" --once

# ── 右臂各关节速度（rad/s） ──
ros2 topic pub /right_arm_velocity_controller/commands \
  std_msgs/Float64MultiArray "{data: [-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.0, 0.0]}" --once

# ── 以 10Hz 持续发送 ──
ros2 topic pub --rate 10 /chassis_wheels_velocity_controller/commands \
  std_msgs/Float64MultiArray "{data: [3.0, 3.0, 3.0, 3.0]}"
```

---

## 构建

```bash
source /opt/ros/humble/setup.bash
cd <workspace>
colcon build --packages-select ylr1d_mid_control
source install/setup.bash
```

---

## 监控

```bash
# 查看控制器状态
ros2 control list_controllers

# 查看关节状态
ros2 topic echo /joint_states

# 查看控制器管理器参数
ros2 param get /controller_manager controller_components
```

---

## 关键文件

```
ylr1d_mid_control/
├── config/
│   ├── controllers.yaml                 # 原始方案：控制器类型 + 参数
│   ├── effort_controllers.yaml          # 力控制方案
│   ├── velocity_controllers.yaml        # 速度控制方案 [NEW]
│   ├── sensors/                         # 传感器参数配置 [NEW]
│   │   ├── rgb_camera.yaml              #   RGB 相机参数
│   │   ├── depth_camera.yaml            #   深度相机参数
│   │   ├── infrared_camera.yaml         #   红外相机参数
│   │   ├── imu_sensor.yaml              #   IMU 参数
│   │   ├── radar_sensor.yaml            #   雷达参数
│   │   ├── lf_ultrasonic_sensor.yaml    #   前左超声波参数
│   │   ├── rf_ultrasonic_sensor.yaml    #   前右超声波参数
│   │   ├── lb_ultrasonic_sensor.yaml    #   后左超声波参数
│   │   └── rb_ultrasonic_sensor.yaml    #   后右超声波参数
│   ├── colors.yaml                      # 颜色配置
│   ├── calibration.yaml                 # 校准参数
│   ├── dynamics.yaml                    # 动力学参数
│   ├── limits.yaml                      # 关节限位
│   ├── links.yaml                       # 连杆参数
│   └── scale.yaml                       # 缩放参数
├── launch/
│   ├── gazebo.launch.py                 # 原始方案启动
│   ├── gazebo_effort.launch.py          # 力控制启动
│   └── gazebo_velocity.launch.py        # 速度控制启动 [NEW]
├── urdf/
│   ├── ylr1d_mid.xacro                  # 基础 URDF + ros2_control 硬件接口定义
│   ├── ylr1d_mid_effort.xacro           # 力控制 xacro 入口（include 基础文件）
├── rviz/
│   └── display.rviz                     # RViz 显示配置
├── verify_urdf.py                       # URDF 验证脚本
└── README.md
```

> **注意**：`ylr1d_mid_effort.xacro` 和 `ylr1d_mid_accel.xacro` 只是 `include` 基础 `ylr1d_mid.xacro` 的入口文件，实际接口注入逻辑在各自的 launch 文件中完成，而非在 xacro 层处理。

---

## 测试步骤

```bash
# 1. 构建
colcon build --packages-select ylr1d_mid_control

# 2. 启动（以速度控制为例）
ros2 launch ylr1d_mid_control gazebo_velocity.launch.py

# 3. 等待 Gazebo 启动 + 控制器加载（约 10-15s，WSL 下可能需要更久）

# 4. 确认所有控制器已激活
ros2 control list_controllers
# 期望输出：全部显示 "active"

# 5. 发送指令验证关节响应
ros2 topic pub /chassis_wheels_velocity_controller/commands \
  std_msgs/Float64MultiArray "{data: [2.0, 2.0, 2.0, 2.0]}"

# 6. 检查关节状态反馈
ros2 topic echo /joint_states --once
```

---

## 已知问题

- **Gazebo Classic** 已于 2025 年 1 月 EOL，建议后续迁移至 Gazebo Ignition / Harmonic
- **传感器噪声** `[Err] [Sensor.cc:510]` 为非关键告警，不影响控制
- **WSL 环境** 下 GPU 渲染受限（需设置 `LIBGL_ALWAYS_SOFTWARE=1`），但不影响物理仿真
- **RViz 网格加载** `Error retrieving file ... .STL` 为文件名大小写问题，不影响仿真与控制
- **root link 惯性** `[WARN] [kdl_parser]` KDL 不支持 root link 带惯性，不影响仿真
- **pose/info 队列告警** `[Wrn] [Publisher.cc:135] Queue limit reached` 为正常节流，不报错

---

## 解决过的问题

### 1. gazebo_ros2_control 插件 CLI 参数解析失败

**症状**: Plugin 解析 URDF 时显示 `--param robot_description:=<?xml...` parser error。

**根因**: `gazebo_ros2_control_plugin.cpp` 构造 CLI 参数时将原始 XML 作为 `--param` 值传入，XML 中包含特殊字符。

**修复**: 修改插件源码，将 `robot_description` 直接设置为节点参数而非通过 CLI 参数传递。

### 2. gazebo_ros2_control 插件找不到

**症状**: Gazebo 启动后 plugin 未加载。

**根因**: Gazebo Classic 未在 `LD_LIBRARY_PATH` 中找到 `/opt/ros/humble/lib`。

**修复**: 在 launch 文件中通过 `env` 参数注入 `LD_LIBRARY_PATH`。

### 3. 控制器 type 参数未定义

**症状**: `spawner` 报 `"The 'type' param was not defined for 'X'"`。

**根因**: controllers.yaml 中控制器定义不在 `controller_manager.ros__parameters` 下。

**修复**: 确保控制器定义嵌套在正确的 YAML 路径下。

### 4. 控制器配置失败 — joints 参数为空

**症状**: ForwardCommandController 加载后显示 `unconfigured`。

**根因**: `joints` 和 `interface_name` 参数嵌套在 `controller_manager.ros__parameters` 下，控制器节点无法读到。

**修复**: 为每个控制器增加顶级配置段。

### 5. 加速度控制方案不可行

**症状**: 控制器报 `"Not acceptable command interfaces combination"`，`acceleration` 命令接口不存在。

**根因**: `gazebo_ros2_control/GazeboSystem` 的 `ControlMethod` 枚举（定义在 `gazebo_system_interface.hpp`）仅支持 NONE, POSITION, VELOCITY, EFFORT, VELOCITY_PID, POSITION_PID，不包含 `acceleration`。`registerJoints()` 静默丢弃不支持的接口类型。

**修复**: 改为速度控制方案（velocity），使用原生支持的 `VELOCITY` 控制方式。

### 6. Gazebo 找不到网格文件

**症状**: Gazebo 启动后卡住或报 `[Err] [Model] [link] has no visual geometry`。

**根因**: URDF 中使用 `package://ylr1d_description/` URI 路径，Gazebo Classic 不支持解析。

**修复**: 在 launch 文件的预处理阶段将 `package://ylr1d_description` 替换为绝对路径（通过 `ament_index_python` 获取）。

另外，也可以修改环境变量 `GAZEBO_MODEL_PATH` ，保证功能包能够被找到。

### 7. WSL 下 Gazebo 进程冲突

**症状**: 第二次运行 launch 时 `spawn_entity` 报 `"Entity already exists"`，Gazebo 进程崩溃。

**根因**: 前次运行未正常清理，Gazebo 服务器仍在后台运行。

**修复**: 每次新启动前运行 `killall -q gzserver gzclient gazebo 2>/dev/null` 清理遗留进程。
