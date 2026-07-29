# ylr1d_mid_control — 中间层控制器（测试用）

> **⚠️ 测试专用包，非生产代码。**
> 用于在 Gazebo Classic + ros2_control 环境中验证 YLR1D 机器人的关节控制方案。

---

## 控制方案

| 方案 | 接口 | 控制器后缀 | 状态 |
|------|------|------------|------|
| **位置/速度**（原始） | `position` / `velocity` | `_controller` | ✅ 已验证 |
| **力控制**（推荐） | `effort` | `_effort_controller` | ✅ 已验证 |
| ~~加速度~~ | ~~`acceleration`~~ | ~~`_accel_controller`~~ | ❌ 已弃用，由力控制替代 |

### 力控制（推荐）

- 不修改原始 `ylr1d_mid.xacro`
- `gazebo_effort.launch.py` 在预处理阶段注入 `<command_interface name="effort"/>` 到每个关节
- `gazebo_ros2_control/GazeboSystem` 原生支持 `effort`，底层调用 `joint->SetForce()`
- 共 **30 个关节** 获得 effort 命令接口

### 原始位置/速度控制

控制方案保持不变，使用 `gazebo.launch.py` + `controllers.yaml`。

---

## 已验证的能力

| 验证项 | 状态 |
|--------|------|
| 30 个可控关节在 Gazebo 中正确加载 | ✅ |
| `gazebo_ros2_control` 插件加载 URDF + 参数 | ✅ |
| `controller_manager` 创建并管理 6 个控制器 | ✅ |
| `joint_state_broadcaster` 发布 `/joint_states` | ✅ |
| `ForwardCommandController`（position 接口）配置并激活 | ✅ |
| `ForwardCommandController`（velocity 接口）配置并激活 | ✅ |
| `ForwardCommandController`（effort 接口）配置并激活 | ✅ |
| 通过 Topic 发送 `Float64MultiArray` 控制关节 | ✅ |
| Effort 命令注入 30 个关节 | ✅ |

---

## 控制器清单

### 原始控制器（位置/速度）

| 控制器名 | 类型 | 关节数 | 接口 | Topic |
|----------|------|--------|------|-------|
| `joint_state_broadcaster` | `JointStateBroadcaster` | — | state | `/joint_states` |
| `chassis_steering_controller` | `ForwardCommandController` | 4 | position | `/chassis_steering_controller/commands` |
| `chassis_wheels_controller` | `ForwardCommandController` | 4 | velocity | `/chassis_wheels_controller/commands` |
| `torso_controller` | `ForwardCommandController` | 4 | position | `/torso_controller/commands` |
| `left_arm_controller` | `ForwardCommandController` | 9 | position | `/left_arm_controller/commands` |
| `right_arm_controller` | `ForwardCommandController` | 9 | position | `/right_arm_controller/commands` |

**关节顺序**（重要 — 发送命令时需按此顺序排列 data 数组）：

- `chassis_steering_controller`: `[RFWheelF, LFWheelF, RBWheelF, LBWheelF]`
- `chassis_wheels_controller`: `[RFWheel, LFWheel, RBWheel, LBWheel]`
- `torso_controller`: `[Base_to_Body1, Body1_to_Body2, Body2_to_Body3, Body3_to_Body4]`
- `left_arm_controller`: `[Body2_to_LeftArm1..7, LeftFinger1, LeftFinger2]`
- `right_arm_controller`: `[Body2_RightArm1..7, RightFinger1, RightFinger2]`

### 力控制器（推荐）

| 控制器 | 接口 | 关节数 | 话题 |
|--------|------|--------|------|
| `chassis_steering_effort_controller` | effort | 4 | `/chassis_steering_effort_controller/commands` |
| `chassis_wheels_effort_controller` | effort | 4 | `/chassis_wheels_effort_controller/commands` |
| `torso_effort_controller` | effort | 4 | `/torso_effort_controller/commands` |
| `left_arm_effort_controller` | effort | 9 | `/left_arm_effort_controller/commands` |
| `right_arm_effort_controller` | effort | 9 | `/right_arm_effort_controller/commands` |

力控制器的关节顺序与原始控制器一致。

---

## 关键文件

```
ylr1d_mid_control/
├── config/
│   ├── controllers.yaml              # 原始控制器类型 + 参数定义
│   ├── effort_controllers.yaml       # 力控制器配置 [NEW]
│   └── acceleration_controllers.yaml # 加速度控制器配置（已弃用）
├── launch/
│   ├── gazebo.launch.py              # 原始启动 Gazebo + 加载控制器
│   └── gazebo_effort.launch.py       # 力控制启动 [NEW]
├── urdf/
│   ├── ylr1d_mid.xacro               # 原始 ros2_control 硬件接口定义
│   ├── ylr1d_mid_effort.xacro        # 力控制 xacro 入口 [NEW]
│   └── ylr1d_mid_accel.xacro         # 加速度 xacro（已弃用）
└── rviz/display.rviz                 # RViz 显示配置
```

---

## 使用

### 构建

```bash
source /opt/ros/humble/setup.bash
cd <workspace>
colcon build --packages-select ylr1d_mid_control
source install/setup.bash
```

### 启用力控制

```bash
ros2 launch ylr1d_mid_control gazebo_effort.launch.py
```

### 发送力指令

```bash
# 躯干升降关节施加 20N
ros2 topic pub /torso_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [20.0, 0.0, 0.0, 0.0]}"

# 左臂全部关节施加 0.5Nm
ros2 topic pub /left_arm_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.0, 0.0]}"
```

### 启动原始位置/速度控制

```bash
ros2 launch ylr1d_mid_control gazebo.launch.py
```

### 发送位置/速度指令

```bash
# 躯干各关节转到指定弧度
ros2 topic pub /torso_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.1, 0.0, 0.0, 0.0]}"

# 车轮滚动
ros2 topic pub /chassis_wheels_controller/commands \
  std_msgs/Float64MultiArray "{data: [2.0, 2.0, 2.0, 2.0]}"
```

### 监控

```bash
ros2 control list_controllers   # 查看控制器状态
ros2 topic echo /joint_states   # 查看关节状态
```

---

## 测试步骤

1. 构建：`colcon build --packages-select ylr1d_mid_control`
2. 启动：`ros2 launch ylr1d_mid_control gazebo_effort.launch.py`
3. 等待控制器加载（约 10s）
4. 确认：`ros2 control list_controllers` → 全部 `active`
5. 发指令验证关节响应
6. 检查：`ros2 topic echo /joint_states`

---

## 已知问题

- **Gazebo Classic 已 EOL**（2025年1月），建议后续迁移至 Gazebo Ignition
- **传感器噪声告警** `[Err] [Sensor.cc:510]` 为非关键告警，不影响控制
- **WSL 环境**下渲染受限，但不影响物理仿真
- **root link 惯性** `[WARN] [kdl_parser]` KDL 不支持 root link 惯性，不影响仿真

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
