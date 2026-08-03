# ylr1d_plant — 物理层（中控）

管理 **Gazebo + ros2_control** 的配置与启动，是机器人的"物理层"。模型资产统一从 `ylr1d_description` 获取，本包只保留自己的控制器配置。

---

## 一、功能定位

- **架构位置**：物理层/中控，承接 `ylr1d_control_sim` 下发的命令，模拟真实机器人。
- **职责**：spawn 机器人、启动 6 个 ros2_control 控制器、把命令落到 Gazebo、发布关节反馈。
- **提供两套物理接口**：**position 方案**直接下发目标角度；**effort 力控方案**下发力矩（更贴近真实执行器）。
- **不做什么**：不涉及上层控制逻辑（软仿真 / 转译 / HMI），只负责"执行命令 + 给反馈"。

---

## 二、包结构

| 内容 | 说明 |
|------|------|
| `launch/gazebo.launch.py` | position 接口方案：6 个控制器，命令话题为 `*_controller/commands`（位置值），`gzserver` 无头启动（无 GUI） |
| `launch/gazebo_effort.launch.py` | effort 力控方案：6 个控制器，命令话题为 `*_effort_controller/commands`（力矩值），内置 `joint_state_filter`，`gazebo` 启动（带 GUI） |
| `config/controllers.yaml` | position 方案控制器定义 |
| `config/effort_controllers.yaml` | effort 方案控制器定义 |
| `src/joint_state_filter.cpp` | 棱柱关节 NaN → 0.0 过滤器，发布 `/joint_states_filtered` |

---

## 三、使用方法

单独启动物理层：

```bash
# position 接口方案（无头 gzserver，无 GUI，适合无显示/远程环境）
ros2 launch ylr1d_plant gazebo.launch.py

# effort 力控方案（带 GUI 的 gazebo 窗口）
ros2 launch ylr1d_plant gazebo_effort.launch.py

# 可选：指定 world（默认 empty.world，可换 ylr1d_description/worlds/ 下的其他 world）
ros2 launch ylr1d_plant gazebo.launch.py world:=sensors_test.world
```

与 `ylr1d_control_sim` + `ylr1d_hmi` 组成完整闭环时，用 `ylr1d_bringup` 一键启动：

```bash
ros2 launch ylr1d_bringup bringup.launch.py
```

WSL 下 Gazebo 启动很慢（30-60s 控制器才加载完），务必等待全部控制器 active：

```bash
ros2 control list_controllers
# 期望全部 active —— position 方案：
#   chassis_steering_controller  chassis_wheels_controller  torso_controller
#   left_arm_controller         right_arm_controller       joint_state_broadcaster
# —— effort 方案（同名加 `_effort` 后缀）
```

### 查看关节反馈

```bash
ros2 topic echo /joint_states --once        # 30 关节 position/velocity/effort
ros2 topic echo /joint_states_filtered --once  # NaN → 0.0 过滤后（仅 effort 方案存在）
```

### 发送命令示例

ForwardCommandController 保持最后值，建议 `--rate 20` 持续发送。命令数组顺序见[接口](#四接口)：

```bash
# effort 方案：力矩（Nm），如转向 / 车轮
ros2 topic pub --rate 20 /chassis_steering_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [30.0, 30.0, 30.0, 30.0]}"
ros2 topic pub --rate 20 /chassis_wheels_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [20.0, 20.0, 20.0, 20.0]}"

# position 方案：目标角度（rad），话题名去掉 `_effort` 后缀
ros2 topic pub --rate 20 /chassis_steering_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.2, -0.2, 0.2, -0.2]}"
```

> ⚠️ 躯干 `Joint_Base_to_Body1` 限位 ±0.3 rad，初始位置就在 0.3（上限），正向力矩推不动，先负向离开限位。

---

## 四、接口

### 话题

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `/joint_states` | `sensor_msgs/JointState` | 30 关节全量反馈（position / velocity / effort） |
| 发布 | `/joint_states_filtered` | `sensor_msgs/JointState` | NaN → 0.0 过滤后（仅 effort 方案） |
| 接收 | `*_controller/commands` × 5 | `std_msgs/Float64MultiArray` | position 方案命令（位置值） |
| 接收 | `*_effort_controller/commands` × 5 | `std_msgs/Float64MultiArray` | effort 方案命令（力矩值） |

### 命令数组顺序（与控制器定义一致）

| 控制器 | 关节数 | 关节 |
|--------|--------|------|
| `chassis_steering` | 4 | `Joint_Base_to_{RF,LF,RB,LB}WheelF` |
| `chassis_wheels` | 4 | `Joint_{RF,LF,RB,LB}WheelF_to_{RF,LF,RB,LB}Wheel` |
| `torso` | 4 | `Joint_Base_to_Body1`、`Joint_Body1_to_Body2`、`Joint_Body2_to_Body3`、`Joint_Body3_to_Body4` |
| `left_arm` | 9 | 左臂 7-DOF + 2 夹爪指 |
| `right_arm` | 9 | 右臂 7-DOF + 2 夹爪指 |

---

## 五、配置

| 文件 | 方案 | 说明 |
|------|------|------|
| `config/controllers.yaml` | position | 控制器定义（`*_controller`） |
| `config/effort_controllers.yaml` | effort | 控制器定义（`*_effort_controller`） |

- 两套方案基于**同一份 xacro 资产**（来自 `ylr1d_description`），差异见[关键机制](#六关键机制选读)。
- `${controllers_yaml_path}` 占位符由本包 launch 注入：position → `config/controllers.yaml`，effort → `config/effort_controllers.yaml`。

---

## 六、关键机制（选读）

### 两套方案差异

| 维度 | position 方案 | effort 方案 |
|------|--------------|-------------|
| 命令接口 | position/velocity 接口，给目标角度 | launch 正则向 xacro 注入 `<command_interface name="effort"/>`，给力矩 |
| 启动器 | `gzserver`（无头） | `gazebo`（带 GUI） |
| 数据流 | 无过滤器，rsp 直接订阅原始 `/joint_states` | 启动 `joint_state_filter`，rsp 订阅过滤后的 `/joint_states_filtered` |

### 控制器加载时序

launch 使用 `TimerAction(period=8.0)` 延迟 spawner 启动，避免 Gazebo 未就绪导致连不上 controller_manager 服务。

### joint_state_filter（NaN → 0.0）

Gazebo 将 prismatic joint 位置初始化为 NaN，robot_state_publisher 持续报 `TF_NAN`。本包内 `joint_state_filter` 节点将 NaN → 0.0 并发布 `/joint_states_filtered`，已集成到 `gazebo_effort.launch.py`（该 launch 把 rsp 的 `joint_states` 话题 remap 到 `/joint_states_filtered`）。注意：position 方案（`gazebo.launch.py`）没有 filter，因此只有 effort 方案规避了 TF_NAN。

### 资产来源

- xacro / 模型 config / meshes / rviz / world：`ylr1d_description`（经其公共模块 `xacro_utils` 处理）

---

## 七、已知限制与注意事项

- **TF_NAN 仅 effort 方案规避**：position 方案无 filter，rsp 直接订阅原始 `/joint_states`，可能持续报 TF_NAN
- **Gazebo 残留进程**：`ros2 launch` 退出后 `gzserver` 仍在后台，再次启动报 `Entity already exists`：
  ```bash
  pkill -f gzserver; pkill -f gzclient
  ```
- **控制器 active 慢**：WSL 下启动到 active 需 30-60s，用 `ros2 control list_controllers` 轮询，不能抢跑
- **`Joint_Base_to_Body1` 限位**：限位 [-0.3, 0.3] rad，初始位置就是 0.3（上限），正向力矩推不动，先负向离开
- **effort 命令需持续发送**：单次 `--once` 在重力/碰撞下可能不足，用 `--rate 20` 持续发送
