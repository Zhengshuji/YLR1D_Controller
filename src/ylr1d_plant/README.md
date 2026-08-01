# ylr1d_plant — 物理层（中控）

管理 **Gazebo + ros2_control** 的配置与启动，是机器人的"物理层"。模型资产（xacro / meshes /
模型配置 yaml / rviz / world）统一从 `ylr1d_description` 获取，本包只保留自己的控制器
配置（`config/controllers.yaml`、`config/effort_controllers.yaml`）。

---

## 一、包介绍

| 内容 | 说明 |
|------|------|
| `launch/gazebo.launch.py` | position 接口方案：6 个控制器，命令话题为 `*_controller/commands`（std_msgs/Float64MultiArray 位置值）。用 `gzserver` 无头启动（无 GUI 窗口） |
| `launch/gazebo_effort.launch.py` | effort 力控方案：同一套 6 个控制器，命令话题为 `*_effort_controller/commands`（力矩值），内置 `joint_state_filter` 节点。用 `gazebo` 启动（带 GUI 窗口） |
| `config/controllers.yaml` | position 方案控制器定义 |
| `config/effort_controllers.yaml` | effort 方案控制器定义 |
| `src/joint_state_filter.cpp` | 棱柱关节 NaN → 0.0 过滤器，发布 `/joint_states_filtered` |

> 两套方案基于同一份 xacro 资产（来自 `ylr1d_description`），但最终 URDF 与行为有差异：
> - 命令接口：position 用 position/velocity 接口，直接给目标角度；effort 方案由 launch 正则向
>   xacro 注入 `<command_interface name="effort"/>`，给的是力矩；
> - 启动器：position 用 `gzserver`（无头），effort 用 `gazebo`（带 GUI）；
> - 数据流：effort 方案启动 `joint_state_filter` 并让 robot_state_publisher 订阅过滤后的
>   `/joint_states_filtered`；position 方案无过滤器、直接订阅原始 `/joint_states`。

---

## 二、使用方法

单独启动物理层：

```bash
# position 接口方案（无头 gzserver，无 GUI，适合无显示/远程环境）
ros2 launch ylr1d_plant gazebo.launch.py

# effort 力控方案（带 GUI 的 gazebo 窗口）
ros2 launch ylr1d_plant gazebo_effort.launch.py

# 可选：指定 world（默认 empty.world，可换 ylr1d_description/worlds/ 下的其他 world）
ros2 launch ylr1d_plant gazebo.launch.py world:=sensors_test.world
```

与 `ylr1d_control_sim` + `ylr1d_hmi` 组成完整闭环时，可用 `ylr1d_bringup` 一键启动：

```bash
ros2 launch ylr1d_bringup bringup.launch.py   # 聚合 gazebo.launch.py + position_simulate + hmi
```

WSL 下 Gazebo 启动很慢（30-60s 控制器才加载完），务必等待全部控制器 active：

```bash
ros2 control list_controllers
# 期望全部 active —— effort 方案：
#   chassis_steering_effort_controller   chassis_wheels_effort_controller
#   torso_effort_controller              left_arm_effort_controller
#   right_arm_effort_controller          joint_state_broadcaster
# —— position 方案（同名去掉 `_effort`）：
#   chassis_steering_controller          chassis_wheels_controller
#   torso_controller                     left_arm_controller
#   right_arm_controller                 joint_state_broadcaster
```

---

## 三、详细控制方法

### 1. 查看关节反馈

```bash
ros2 topic echo /joint_states --once        # 30 关节 position/velocity/effort
# 注意：/joint_states_filtered 仅 effort 方案存在（position 方案未启动 filter 节点）
ros2 topic echo /joint_states_filtered --once  # NaN → 0.0 过滤后
```

### 2. 力控命令（effort 方案，力矩单位 Nm）

ForwardCommandController 保持最后值，建议 `--rate 20` 持续发送：

```bash
# 转向（4 关节）
ros2 topic pub --rate 20 /chassis_steering_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [30.0, 30.0, 30.0, 30.0]}"

# 车轮（4 关节）
ros2 topic pub --rate 20 /chassis_wheels_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [20.0, 20.0, 20.0, 20.0]}"

# 躯干（注意 Joint_Base_to_Body1 上限 0.3 rad，初始就在上限，先负向离开）
ros2 topic pub --rate 20 /torso_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [-100.0, 0.0, 0.0, 0.0]}"

# 左臂（9 关节，最后 2 个是夹爪）
ros2 topic pub --rate 20 /left_arm_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0]}"

# 右臂
ros2 topic pub --rate 20 /right_arm_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0]}"
```

### 3. 位置命令（position 方案，单位 rad）

话题名去掉 `_effort` 后缀，值语义为目标位置：

```bash
# 转向（4 关节，位置 rad）
ros2 topic pub --rate 20 /chassis_steering_controller/commands \
  std_msgs/Float64MultiArray "{data: [0.2, -0.2, 0.2, -0.2]}"

# 躯干（Joint_Base_to_Body1 上限 0.3 rad，注意别越限）
ros2 topic pub --rate 20 /torso_controller/commands \
  std_msgs/Float64MultiArray "{data: [-0.1, 0.0, 0.0, 0.0]}"
```

---

## 四、补充说明

### 关节分组与命令数组顺序

命令数组长度固定，顺序与控制器定义一致：

| 控制器 | 关节数 | 关节 |
|--------|--------|------|
| chassis_steering | 4 | `Joint_Base_to_{RF,LF,RB,LB}WheelF` |
| chassis_wheels | 4 | `Joint_{RF,LF,RB,LB}WheelF_to_{RF,LF,RB,LB}Wheel` |
| torso | 4 | `Joint_Base_to_Body1`、`Joint_Body1_to_Body2`、`Joint_Body2_to_Body3`、`Joint_Body3_to_Body4` |
| left_arm | 9 | 左臂 7-DOF + 2 夹爪指 |
| right_arm | 9 | 右臂 7-DOF + 2 夹爪指 |

### 控制器加载时序

launch 中使用 `TimerAction(period=8.0)` 延迟 spawner 启动，避免 Gazebo 未就绪导致连不上
controller_manager 服务。

### 资产来源

- xacro / 模型 config / meshes / rviz / world：`ylr1d_description`
- `${controllers_yaml_path}` 占位符由本包 launch 注入：position 方案 → `config/controllers.yaml`，
  effort 方案 → `config/effort_controllers.yaml`

---

## 五、问题解决

### TF_NAN（棱柱关节 NaN）
Gazebo 将 prismatic joint 位置初始化为 NaN，robot_state_publisher 持续报 `TF_NAN`。
`joint_state_filter` 节点（本包内）将 NaN → 0.0，过滤后数据在 `/joint_states_filtered`。
已集成到 `gazebo_effort.launch.py`：该 launch 启动 filter 节点，并把 robot_state_publisher
的 `joint_states` 话题 remap 到 `/joint_states_filtered`，从而消除 TF_NAN。
注意：position 方案（`gazebo.launch.py`）**没有** filter，robot_state_publisher 直接订阅
原始 `/joint_states`，因此只有 effort 方案规避了该问题。

### Gazebo 残留进程
`ros2 launch` 退出后 `gzserver` 仍在后台，再次启动报 `Entity already exists`：
```bash
pkill -f gzserver; pkill -f gzclient
```

### 控制器 active 慢
WSL 下启动到 active 需 30-60s，用 `ros2 control list_controllers` 轮询，不能抢跑。

### Joint_Base_to_Body1 限位
限位 [-0.3, 0.3] rad，初始位置就是 0.3（上限），正向力矩推不动。先用负向力矩离开限位。

### effort 命令需持续发送
单次 `--once` 在重力/碰撞下可能不足，用 `--rate 20` 持续发送。
