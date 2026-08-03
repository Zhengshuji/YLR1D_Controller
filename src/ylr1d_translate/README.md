# ylr1d_translate

转译层（translate）：位于上层（规划层 / 上层 HMI）与控制层 `ylr1d_control_sim` 之间。
基于 action 接收上层的高级指令，解算后以 `/desired_joint_states` 下发到控制层。

- **底盘**：把"运动方式 + 方向 + 速度"解算为 4 转向角 + 4 轮速（先转向后移动）
- **机械臂 / 躯干**：把"部位名 + 关节坐标"映射为对应关节组的期望位置
- **夹爪**：把布尔开合映射为两个夹指的目标位置

## 架构总览

```
规划层 / translate HMI
       │  action goal（/chassis_move /arm_move /gripper_move）
       ▼
┌───────────────────────────────┐
│  translate_server             │  （20Hz 定时推进三个目标）
│   ├─ 订阅 /joint_states 反馈  │
│   └─ 发布 /desired_joint_states│
└───────────────────────────────┘
       │  /desired_joint_states (sensor_msgs/JointState)
       ▼
  ylr1d_control_sim ──5组命令──▶ plant (Gazebo)
```

数据流：上层 → action goal → translate_server → `/desired_joint_states` → control_sim
→ 5 组 ForwardCommandController 命令 → plant(Gazebo)；`/joint_states` 反馈回路用于
转向到位 / 目标到位检测。

> 下游依赖：`/desired_joint_states` 需由 `ylr1d_control_sim` 订阅，`/joint_states`
> 需由 plant 发布。translate 自身不接触 Gazebo。

## Action 接口

三个 action server 由 `rosidl_generate_interfaces` 生成，goal / result / feedback
定义见 `action/*.action`，语义常量见 `constants.hpp`。

| Action | 服务名 | Goal | Result | Feedback |
|--------|--------|------|--------|----------|
| ChassisMove | `/chassis_move` | mode, direction, speed, duration | success, message | phase |
| ArmMove | `/arm_move` | part, positions[] | success, message | progress |
| GripperMove | `/gripper_move` | part, open | success, message | progress |

三个目标各自独立推进（互不阻塞），可同时存在活跃的底盘 / 机械臂 / 夹爪目标。
新 goal 到达会 `abort` 上一目标（message = "superseded by new goal"）。

### ChassisMove（/chassis_move）

| 字段 | 类型 | 说明 |
|------|------|------|
| mode | int8 | 0 平移 / 1 原地旋转 / 2 停车 |
| direction | float64 | 平移时的运动方向角（rad，绕 Z，底盘坐标系），4 轮转向角 = direction；旋转 / 停车忽略 |
| speed | float64 | 平移=轮速（rad/s）；旋转=车体角速度（rad/s），轮速 = speed / 3.7538；停车忽略 |
| duration | float64 | 移动时长（s）；0 = 持续执行直到被替换 / 取消 |

解算结果：

| mode | 转向角 | 轮速 |
|------|--------|------|
| 平移 (0) | 4 轮均 = direction | 4 轮均 = speed |
| 旋转 (1) | 固定 `[2.2150, 0.9265, -2.2150, -0.9265]`（RF/LF/RB/LB） | speed / 3.7538（ω = 3.7538·v） |
| 停车 (2) | 固定 `[-0.6443, 0.6443, -2.4973, 2.4973]`（RF/LF/RB/LB） | 0 |

**先转向后移动**：平移 / 旋转模式在转向阶段轮速钳制为 0，直到转向角到位
（容差 `TURN_TOL` = 0.03 rad）才进入移动阶段并开始 `duration` 计时。反馈 `phase`
指示 `steering` / `moving` / `stopped`。停车模式转向到位即完成。

取消目标：`abort`（message = "canceled"）并停止发布底盘命令——控制层保持最后值，
**停车请用 mode=2，而非 cancel**。

### ArmMove（/arm_move）

| 字段 | 类型 | 说明 |
|------|------|------|
| part | int8 | 0 躯干（4 关节）/ 1 左臂（7 关节）/ 2 右臂（7 关节） |
| positions | float64[] | 该部位全部关节的期望位置，顺序与关节表一致（见下文） |

> 机械臂与夹爪解耦：夹爪两指由 `GripperMove` 控制，不在 `ArmMove` 的关节表中。

`positions` 数量必须与部件关节数一致，否则立即 `abort`（"positions size mismatch"）。
反馈 `progress` = 已到位关节占比；全部关节到达目标（容差 `ARM_TOL` = 0.02）即
`succeed`，超时 `ARM_TIMEOUT` = 30 s 则 `abort`（"timeout"）。

### GripperMove（/gripper_move）

| 字段 | 类型 | 说明 |
|------|------|------|
| part | int8 | 0 左夹爪 / 1 右夹爪 |
| open | bool | true 开（张到限位）/ false 关（并拢） |

开合映射到两个夹指关节（棱柱关节，单位 m）：

| 夹爪 | 开 | 关 |
|------|----|----|
| 左 | -0.014 | 0.0 |
| 右 | 0.0 | +0.014 |

> 左右不对称：左指限位 `[-0.014, 0]`，右指限位 `[0, +0.014]`（与 control_sim 夹爪说明一致）。
> 到位容差用独立的 `GRIPPER_TOL` = 0.002 m（夹指行程仅 0.014 m，远小于机械臂容差 `ARM_TOL`
> 0.02；复用 ARM_TOL 会导致夹爪未动作就被误判到位）。

## 话题

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 发布 | `/desired_joint_states` | sensor_msgs/JointState | 活跃目标对应关节的期望值 |
| 订阅 | `/joint_states` | sensor_msgs/JointState | 反馈，用于转向 / 到位检测（NaN 忽略） |

`/desired_joint_states` 发布规则（JointState 三数组等长、按索引对齐，未用字段填 0）：
- **转向关节 ×4**：`position` = 转向角
- **轮子关节 ×4**：`velocity` = 轮速（`position` 填 0）
- **躯干 / 臂 / 夹爪**：`position` = 目标位置（`velocity` 填 0）

仅发布当前活跃目标涉及的关节；无活跃目标时不发布。

## 关节表

关节顺序与 `ylr1d_control_sim`（及 `/desired_joint_states` 约定）完全一致，
定义在 `joint_names.hpp`：

| 组 | 数量 | 关节名（顺序） |
|----|------|----------------|
| 转向 kSteering | 4 | `Joint_Base_to_RFWheelF`, `LFWheelF`, `RBWheelF`, `LBWheelF` |
| 轮子 kWheels | 4 | `Joint_RFWheelF_to_RFWheel`, `LFWheelF_to_LFWheel`, `RBWheelF_to_RBWheel`, `LBWheelF_to_LBWheel` |
| 躯干 kTorso | 4 | `Joint_Base_to_Body1`, `Body1_to_Body2`, `Body2_to_Body3`, `Body3_to_Body4` |
| 左臂 kLeftArm | 7 | `Joint_Body2_to_LeftArm1` … `LeftArm7` |
| 右臂 kRightArm | 7 | `Joint_Body2_to_RightArm1` … `RightArm7` |
| 左指 kLeftFingers | 2 | `Joint_LeftArm7_to_LeftFinger1`, `LeftFinger2` |
| 右指 kRightFingers | 2 | `Joint_RightArm7_to_RightFinger1`, `RightFinger2` |

> 左 / 右臂仅含 7 个臂关节；夹爪两指在 `kLeftFingers` / `kRightFingers` 中，由 `GripperMove`
> 单独控制（机械臂与夹爪解耦）。`kPartJoints()` 按 `part` 枚举
> （0 躯干 / 1 左臂 / 2 右臂）聚合，供 ArmMove 使用。

## 使用

```bash
# 完整链路（转译层栈，推荐）
ros2 launch ylr1d_bringup bringup_translate.launch.py

# 或分步：
# 终端 1: 物理层
ros2 launch ylr1d_plant gazebo.launch.py
# 终端 2: 控制层软仿真
ros2 launch ylr1d_control_sim position_simulate.launch.py
# 终端 3: 转译层
ros2 launch ylr1d_translate translate.launch.py
# 终端 4: 上层客户端（转译层 HMI，发 action goal）
ros2 launch ylr1d_hmi hmi_translate.launch.py
```

验证 action server 已就绪：

```bash
ros2 action list -t
# 期望出现 /chassis_move /arm_move /gripper_move（server = translate_server）
```

命令行直接发送示例：

```bash
ros2 action send_goal /chassis_move ylr1d_translate/action/ChassisMove \
  "{mode: 0, direction: 0.0, speed: 1.0, duration: 3.0}" --feedback
ros2 action send_goal /gripper_move ylr1d_translate/action/GripperMove \
  "{part: 0, open: true}" --feedback
```

## 依赖

- **ROS2**: rclcpp, rclcpp_action, std_msgs, sensor_msgs, rosidl_default_generators
- 与 `ylr1d_control_sim` / `ylr1d_hmi` 仅通过话题 / action 接口耦合，无构建期依赖

## 构建

```bash
source /opt/ros/humble/setup.bash
cd <workspace>
colcon build --packages-select ylr1d_translate
source install/setup.bash
```

## 关键文件

```
ylr1d_translate/
├── action/
│   ├── ChassisMove.action    # 底盘：mode + direction + speed + duration
│   ├── ArmMove.action        # 机械臂 / 躯干：part + positions[]
│   └── GripperMove.action    # 夹爪：part + open
├── include/ylr1d_translate/
│   ├── constants.hpp         # 模式 / 部位 / 夹爪常量、转向角表、容差 / 超时 / 周期
│   ├── joint_names.hpp       # 关节名表（与控制层顺序一致）
│   └── translate_node.hpp    # TranslateNode：3 个 action server + 20Hz 定时推进
├── src/
│   ├── translate_node.cpp    # 目标解算 / 状态机 / 发布 desired_joint_states
│   └── main.cpp              # 入口
├── launch/translate.launch.py
├── CMakeLists.txt            # action 生成 + translate_server 目标
└── package.xml
```

## 已知限制 / 注意

- **必须下层已运行**：translate 的解算结果走 `/desired_joint_states` → control_sim，
  转向 / 到位判定依赖 `/joint_states` 反馈。下层未启动时目标会一直停在 `steering` 阶段
- **cancel 不停车**：取消底盘目标只是 `abort` 并停止发布，控制层保持最后轮速；
  需要停车应发送 `mode=2`（MODE_STOP）
- **勿与控制层 HMI 混用**：完整链路用 `ylr1d_bringup bringup_translate.launch.py`；
  与控制层 HMI（`hmi.launch.py`）同时使用会竞争同一批关节
- **JointState 对齐约定**：`/desired_joint_states` 的 name / position / velocity 三数组
  等长按索引对齐，未用字段填 0（控制层按关节名取期望值）
- **20Hz 推进**：定时器周期 `LOOP_DT` = 0.05 s；到位判定基于最后一次 `/joint_states`
  消息，反馈有延迟时判定会滞后
