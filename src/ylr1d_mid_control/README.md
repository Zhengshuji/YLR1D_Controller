# ylr1d_mid_control — 中层控制

## 定位

本包是 YLR1D 双机械臂轮式机器人 **三层控制架构** 的中间层，负责：

- 将上层（运动学/规划层）的抽象指令转发为底层硬件的关节指令
- 使用 `ros2_control` 框架的 `ForwardCommandController`，通过 ROS 2 Topic 接收 `Float64MultiArray` 命令
- 桥接 Gazebo Classic 仿真（通过 `gazebo_ros2_control` 插件）

## 已验证的能力

| 验证项 | 状态 |
|--------|------|
| 30 个可控关节在 Gazebo 中正确加载 | ✅ |
| `gazebo_ros2_control` 插件加载 URDF + 参数 | ✅ |
| `controller_manager` 创建并管理 6 个控制器 | ✅ |
| `joint_state_broadcaster` 发布 `/joint_states` | ✅ |
| `ForwardCommandController` 配置并激活 | ✅ |
| 通过 Topic 发送 `Float64MultiArray` 控制关节 | ✅ |

## 控制器清单

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

## 关键文件

```
ylr1d_mid_control/
├── config/controllers.yaml   # 控制器类型 + 参数定义
├── launch/gazebo.launch.py   # 启动 Gazebo + 加载控制器
├── urdf/ylr1d_mid.xacro      # ros2_control 硬件接口定义
└── rviz/display.rviz         # RViz 显示配置
```

## 解决过的问题

### 1. gazebo_ros2_control 插件 CLI 参数解析失败

**症状**: Plugin 解析 URDF 时显示 `--param robot_description:=<?xml...` parser error。

**根因**: `gazebo_ros2_control_plugin.cpp` 构造 CLI 参数时将原始 XML 作为 `--param` 值传入，XML 中包含 `<?`, `<`, `>`, `"` 等特殊字符，rcl 的 YAML 参数解析器无法处理。

**修复**: 修改插件源码，将 `robot_description` 直接设置为节点参数而非通过 CLI 参数传递。详见 `others/` 目录下的补丁。

### 2. gazebo_ros2_control 插件找不到

**症状**: Gazebo 启动后 plugin 未加载，日志无任何 ros2_control 相关输出。

**根因**: Gazebo Classic 未在 `LD_LIBRARY_PATH` 中找到 `/opt/ros/humble/lib`（ROS2 Humble 的 `setup.bash` 不设置该变量）。

**修复**: 在 `gazebo.launch.py` 中通过 `env` 参数注入 `LD_LIBRARY_PATH`。

### 3. 控制器 type 参数未定义

**症状**: `spawner` 报 `"The 'type' param was not defined for 'X'"`。

**根因**: controllers.yaml 中控制器定义不在 `controller_manager.ros__parameters` 下。

**修复**: 确保控制器定义嵌套在正确的 YAML 路径下。

### 4. 控制器配置失败 — joints 参数为空

**症状**: 所有 ForwardCommandController 加载后显示 `unconfigured`，只有 `joint_state_broadcaster` 正常。

**根因**: controllers.yaml 中 `joints` 和 `interface_name` 参数嵌套在 `controller_manager.ros__parameters` 下，控制器节点无法从该路径读到自己的参数。

**修复**: YAML 文件为每个控制器增加顶级配置段（`torso_controller.ros__parameters.joints`），使控制器节点能通过 `automatically_declare_parameters_from_overrides` 机制找到参数。

---

## 三层控制架构总览

```
┌──────────────────────────────────────────────────┐
│  上层 — 运动学 / 规划层 (ylr1d_upper_control)    │
│                                                   │
│  • 底盘运动学: 麦克纳姆轮正逆解                    │
│  • 双臂运动学: IK/ID 求解器                       │
│  • 躯干运动学: 升降 + 腰部耦合                     │
│  • Cartestian 空间规划                             │
│  • 输入: 速度指令 / 末端目标位姿                    │
│  • 输出: Float64MultiArray → 6 个 controller topic │
└──────────────────────┬───────────────────────────┘
                       │ 6 × /<controller>/commands
                       ▼
┌──────────────────────────────────────────────────┐
│  中層 — 指令转发层 (ylr1d_mid_control)  ← 本包    │
│                                                   │
│  ros2_control + ForwardCommandController           │
│  6 个控制器, 30 个可控关节                         │
│  Gazebo Classic 仿真验证通过                       │
└──────────────────────┬───────────────────────────┘
                       │ 30 × command_interface
                       ▼
┌──────────────────────────────────────────────────┐
│  下层 — 关节驱动层 (待实现)                        │
│                                                   │
│  • PD 控制器 + 重力补偿                            │
│  • 关节限位保护                                    │
│  • 力矩/电流环                                     │
│  • 输入: 期望位置/速度/力矩                         │
│  • 输出: 电机 PWM / 力矩指令                        │
└──────────────────────────────────────────────────┘
```

### 下层（待实现）

下层负责 **关节级别的实时控制**，需要实现：

1. **PD 位置/速度控制器** — 每个关节一个闭环
2. **重力补偿** — 双臂和躯干的重力矩前馈
3. **关节限位保护** — 软限位 + 缓冲
4. **力矩/电流环** — 如果需要力控

在 Gazebo 中下层可通过 `gazebo_ros2_control` 的 `GazeboSystem` 硬件接口内部 PID 实现，实际硬件上可能需要独立的微控制器或 RT Linux 进程。

### 上层（待实现）

上层负责 **运动学和规划**，可能包括：

1. **底盘运动学** — 麦克纳姆轮正逆解（底盘速度 → 4 轮速度 + 4 舵角）
2. **双臂运动学** — 使用 `moveit2` 或自建 IK 求解器
3. **躯干运动学** — 升降 + 偏航 + 俯仰的耦合分析
4. **空间规划** — 末端执行器的 Cartestian 路径规划
5. **整身运动学** — 协调底盘 + 躯干 + 双臂的运动

### 依赖关系

```
ylr1d_description (共享包)
   ├── ylr1d_mid_control (本包)
   ├── ylr1d_upper_control (待建)
   └── ylr1d_lower_control (待建)
```

`ylr1d_description` 提供机器人 URDF 模型、STL 网格文件、颜色和动力学参数等，所有上层包都依赖它。
