# ylr1d_control — 控制层（采样保持器 + 通信节点）

ROS2 控制节点，接收上层期望值（`/desired_joint_states`）与算法层仿真输出（`/ctrl/<组>/output`），**采样保持后**回发反馈并下发 5 组 ForwardCommandController 命令。**不做任何算法计算**——控制计算与软仿真全部委托算法层（`ylr1d_algorithm_sim`）仿真控制器节点，两层经 topic 通信解耦。

---

## 一、功能定位

- **架构位置**：控制层，位于 `ylr1d_hmi` / `ylr1d_translate` 与 `ylr1d_plant` 之间。控制层与算法层**并列**且经 topic 通信解耦（`/ctrl/<组>/*`）。
- **职责**：只管**消息的进出、采样保持与组装**——
  - 组期望：订阅 `/desired_joint_states`，采样保持后发布 `/ctrl/<组>/desired`；
  - 组反馈：订阅算法层 `/ctrl/<组>/output`，采样保持后回发 `/ctrl/<组>/feedback`（软仿真闭环）；
  - 命令：把采样保持的仿真输出按组下发到物理层 5 组命令话题（`Float64MultiArray`）。
- **两个节点**：`chassis_control`（底盘：steering 转向 + wheels 轮子 2 组）、`arm_control`（torso / left_arm / right_arm 3 组）。
- **不做什么**：不实现任何控制算法 / 被控对象模型（都在算法层），不接触 Gazebo。

### 每周期流程（控制层视角）

```
订阅 /desired_joint_states（采样保持）──▶ 发布 /ctrl/<组>/desired ──▶ 算法层仿真控制器
订阅 /ctrl/<组>/output（采样保持） ──▶ 回发 /ctrl/<组>/feedback（软仿真闭环）
                                    ──▶ 发布 5 组命令 ──▶ plant (Gazebo)
```

---

## 二、包结构

```
include/ylr1d_control/
├── groups/  group_forwarder.hpp      # GroupForwarder：一组关节的转发器（期望/反馈/命令）
└── nodes/   chassis_control_node.hpp  arm_control_node.hpp
src/
├── groups/  group_forwarder.cpp
├── nodes/   chassis_control_node.cpp  arm_control_node.cpp  (无 main)
└── main_chassis.cpp  main_arm.cpp     # 独立入口
launch/position_simulate.launch.py     # 仅控制层节点（不再加载 pid.yaml，已移交算法层）
```

依赖：`ylr1d_algorithm_sim`（仅 include 其 `joint_config.hpp` 分组/限位/命令话题常量，**不链接核心算法库**）、`rclcpp / std_msgs / sensor_msgs`。

---

## 三、使用方法

```bash
# 分步启动（控制层 + 算法层，配合 plant + HMI）
ros2 launch ylr1d_plant gazebo.launch.py                            # 终端 1：物理层
ros2 launch ylr1d_algorithm_sim sim_controller.launch.py            # 终端 2：算法层
ros2 launch ylr1d_control position_simulate.launch.py               # 终端 3：控制层

# 完整闭环一键启动
ros2 launch ylr1d_bringup bringup_control.launch.py                 # 或 bringup_translate.launch.py
```

### 发送期望关节值

```bash
# 可以只发送需要控制的关节，未发布的关节保持当前值不变
ros2 topic pub /desired_joint_states sensor_msgs/JointState "
header: auto
name:
  - 'Joint_Base_to_RFWheelF'
  - 'Joint_Base_to_LFWheelF'
  - 'Joint_Base_to_RBWheelF'
  - 'Joint_Base_to_LBWheelF'
  - 'Joint_RFWheelF_to_RFWheel'
  - 'Joint_LFWheelF_to_LFWheel'
  - 'Joint_RBWheelF_to_RBWheel'
  - 'Joint_LBWheelF_to_LBWheel'
position: [0.0, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0]
"
```

### 验证

```bash
ros2 control list_controllers          # 依赖 plant 已启动
ros2 topic echo /joint_states --once   # 观察物理层反馈
ros2 topic echo /ctrl/left_arm/output --once   # 观察算法层仿真输出
```

---

## 四、接口

### 节点

| 节点（可执行名） | 订阅 | 发布 | 控制频率 |
|------|------|------|----------|
| `chassis_control` | `/desired_joint_states`<br>`/ctrl/steering/output`<br>`/ctrl/wheels/output` | `/ctrl/steering/desired`、`/ctrl/steering/feedback`<br>`/ctrl/wheels/desired`、`/ctrl/wheels/feedback`<br>`/chassis_steering_controller/commands`<br>`/chassis_wheels_controller/commands`<br>`/simulated_chassis_states` | 100Hz |
| `arm_control` | `/desired_joint_states`<br>`/ctrl/torso/output`<br>`/ctrl/left_arm/output`<br>`/ctrl/right_arm/output` | `/ctrl/{torso,left_arm,right_arm}/{desired,feedback}`<br>`/torso_controller/commands`<br>`/left_arm_controller/commands`<br>`/right_arm_controller/commands`<br>`/simulated_arm_states` | 100Hz |

> 可执行名与 ROS 节点名一致（`chassis_control` / `arm_control`）。`/ctrl/<组>/*` 话题为**算法层与本层共同约定**的层间接口。

### 话题数据结构

**输入 `/desired_joint_states`**（`sensor_msgs/JointState`）：`name` 列出需控制的关节，`position` / `velocity` 按 `name` 顺序索引对齐。约定：转向 / 躯干 / 臂 / 夹爪用 `position`（26 个），轮子用 `velocity`（4 个）。

**层间 `/ctrl/<组>/{desired,feedback,output}`**（`sensor_msgs/JointState`）：每组含该组关节，位置组填 `position`（+ 可选 velocity）、速度组填 `velocity`。

**输出 5 组命令话题**：均为 `std_msgs/Float64MultiArray`，data 数组长度与该控制器关节数一致（转向 4 / 轮子 4 / 躯干 4 / 左臂 9 / 右臂 9）。角度 rad，棱柱关节 m，轮速 rad/s。

**输出 `/simulated_*_states`**（`sensor_msgs/JointState`）：算法层仿真输出采样保持（底盘、臂各一份），调试用途。

### 参数

- `loop_hz`（默认 100.0）：控制层采样/转发频率。
- pid 参数已**移交算法层**（`ylr1d_algorithm_sim` 的 `config/pid.yaml`），本层不再加载；限位/分组来自算法层 `joint_config.hpp`（编译期常量，include 引用）。

---

## 五、配置

| 文件 / 头文件 | 内容 | 说明 |
|------|------|------|
| `launch/position_simulate.launch.py` | 仅启动 `chassis_control` + `arm_control` 两个节点，传 `loop_hz` | 不再加载 pid.yaml |
| 分组 / 限位 / 命令话题 | `ylr1d_algorithm_sim::joint_config.hpp` | 单一来源在算法层，本层 include 引用；对照 `ylr1d_description/config/limits.yaml` 语境 |
| pid 参数 | `ylr1d_algorithm_sim/config/pid.yaml` | 移交算法层，由算法层 launch 注入 |

---

## 六、关键机制（选读）

### 6.1 30 关节完整列表

所有关节在 `/joint_states` 中以固定顺序发布，命令数组顺序与下表一致。限位数值来源：`ylr1d_algorithm_sim/include/ylr1d_algorithm_sim/config/joint_config.hpp`（迁移自本层，阶段 B 移交）。

**底盘 — 转向（4 关节，位置接口，发布到 `/chassis_steering_controller/commands`）**

| 索引 | URDF 名称 | 限位 (rad) | 初始角度 (rad) |
|------|-----------|------------|----------------|
| 0 | `Joint_Base_to_RFWheelF` | ±3.14 | +2.497 |
| 1 | `Joint_Base_to_LFWheelF` | ±3.14 | -2.497 |
| 2 | `Joint_Base_to_RBWheelF` | ±3.14 | +0.644 |
| 3 | `Joint_Base_to_LBWheelF` | ±3.14 | -0.644 |

**底盘 — 轮子（4 关节，速度接口，发布到 `/chassis_wheels_controller/commands`）**

| 索引 | URDF 名称 | 类型 | 初始值 |
|------|-----------|------|--------|
| 4 | `Joint_RFWheelF_to_RFWheel` | Continuous（连续旋转） | 0 |
| 5 | `Joint_LFWheelF_to_LFWheel` | Continuous | 0 |
| 6 | `Joint_RBWheelF_to_RBWheel` | Continuous | 0 |
| 7 | `Joint_LBWheelF_to_LBWheel` | Continuous | 0 |

> 轮子无位置限位（连续旋转），速度 / 加速度限幅见 `kVelocityLimits`。

**躯干（4 关节，位置接口，发布到 `/torso_controller/commands`）**

| 索引 | URDF 名称 | 限位 (rad) | 说明 |
|------|-----------|------------|------|
| 8 | `Joint_Base_to_Body1` | **[-0.30, +0.30]** | 棱柱关节（升降），单位**米** |
| 9 | `Joint_Body1_to_Body2` | ±3.14 | 偏航 |
| 10 | `Joint_Body2_to_Body3` | ±1.57 | 俯仰 1 |
| 11 | `Joint_Body3_to_Body4` | ±1.57 | 俯仰 2 |

> ⚠️ `Joint_Base_to_Body1` 限位非常小（±0.3），初始位置即为 0.3（上限），正向控制需先反向运动离开限位。

**左臂（9 关节，位置接口，发布到 `/left_arm_controller/commands`）**

| 索引 | URDF 名称 | 限位 (rad) | 说明 |
|------|-----------|------------|------|
| 12 | `Joint_Body2_to_LeftArm1` | ±2.62 | 肩 1 |
| 13 | `Joint_LeftArm1_to_LeftArm2` | **[-1.57, +1.83]** | 肩 2（非对称） |
| 14 | `Joint_LeftArm2_to_LeftArm3` | ±2.62 | 肩 3 |
| 15 | `Joint_LeftArm3_to_LeftArm4` | ±1.57 | 肘 1 |
| 16 | `Joint_LeftArm4_to_LeftArm5` | ±2.62 | 肘 2 |
| 17 | `Joint_LeftArm5_to_LeftArm6` | ±2.09 | 腕 1 |
| 18 | `Joint_LeftArm6_to_LeftArm7` | ±6.28 | 腕 2（范围大） |
| 19 | `Joint_LeftArm7_to_LeftFinger1` | **[-0.014, 0.0]** | 左指 1（棱柱，单位 m） |
| 20 | `Joint_LeftArm7_to_LeftFinger2` | **[-0.014, 0.0]** | 左指 2（棱柱，单位 m） |

**右臂（9 关节，位置接口，发布到 `/right_arm_controller/commands`）**

| 索引 | URDF 名称 | 限位 (rad) | 说明 |
|------|-----------|------------|------|
| 21 | `Joint_Body2_to_RightArm1` | ±2.62 | 肩 1 |
| 22 | `Joint_RightArm1_to_RightArm2` | **[-1.57, +1.83]** | 肩 2（非对称） |
| 23 | `Joint_RightArm2_to_RightArm3` | ±2.62 | 肩 3 |
| 24 | `Joint_RightArm3_to_RightArm4` | ±1.57 | 肘 1 |
| 25 | `Joint_RightArm4_to_RightArm5` | ±2.62 | 肘 2 |
| 26 | `Joint_RightArm5_to_RightArm6` | ±2.09 | 腕 1 |
| 27 | `Joint_RightArm6_to_RightArm7` | ±6.28 | 腕 2（范围大） |
| 28 | `Joint_RightArm7_to_RightFinger1` | **[0.0, +0.014]** | 右指 1（棱柱，单位 m） |
| 29 | `Joint_RightArm7_to_RightFinger2` | **[0.0, +0.014]** | 右指 2（棱柱，单位 m） |

> 夹爪说明：正 q 值使两指沿相反方向平移（张开），负 q 值使两指相向（闭合）。左指 q=-0.014 全开、q=0 全闭；右指 q=0 全开、q=0.014 全闭。

### 6.2 每周期控制流程

```
control: 订阅 /desired_joint_states → set_desired（采样保持）
  → 周期发布 /ctrl/<组>/desired（组期望）
  → 算法层仿真控制器：协同 P=1 → 逐关节 PID → 仿真对象整体推进 → /ctrl/<组>/output
  → control: 订阅 output → on_output（采样保持）→ 回发 /ctrl/<组>/feedback（软仿真闭环）
  → 发布 5 组命令话题 + 填充 /simulated_*_states
```

- 反馈来自算法层软仿真输出（采样保持回发），不是 Gazebo 回读；软仿真闭环在算法层进程内完成。
- `GroupForwarder` 在收到首帧 `output` 前只发布 desired（`have_output_` 前不发 feedback / 命令）。

### 6.3 类结构

- `GroupForwarder`（`groups/group_forwarder.hpp`）：一组关节的转发器，持有 `JointGroupDef`（算法层）与三个 publisher（desired / feedback / cmd），`setup → set_desired → on_output → publish → fill_state_msg`。
- `ChassisControlNode` / `ArmControlNode`：订阅 / 发布 / 定时器，各管辖若干组（底盘 2 组、臂 3 组）；`main()` 独立到 `src/main_*.cpp`，节点类可作库复用。

### 6.4 多线程决策

**结论：不需要多线程。** 每节点用默认 `SingleThreadedExecutor`：100Hz 定时器执行采样保持与转发，订阅回调只写缓存，单线程串行化后无数据竞争；引入多线程反而需加锁。两个节点由各自独立 executor 运行，天然并行。

### 6.5 位置限幅

位置限位来自算法层 `joint_config.hpp` 头文件常量，在算法层各组具名仿真对象内部的积分器（`src/control_law/integrator.cpp`）中**硬钳制**到 `[lower, upper]`（越限目标只会停在限位处，不会越过）。HMI 或上层节点仍应遵守限位约束：转向 ±3.14；轮子无位置限位（速度上限 `max_vel`）；躯干 `Joint_Base_to_Body1` ±0.30（米）；双臂大部分 ±1.57~±2.62、`*Arm6_to_*Arm7` ±6.28；夹爪左指 [-0.014, 0]、右指 [0, 0.014]。

---

## 七、已知限制与注意事项

### 启动与配置

- **控制层只做采样保持，不做算法**：pid / 限位 / 分组均以算法层为单一来源（`config/pid.yaml` + `joint_config.hpp`），改控制参数改算法层后重启算法层容器。
- **层间依赖时序**：`bringup_control.launch.py` 已按「物理层 → 算法层 → 控制层 → HMI」聚合；单独启动时须先起算法层再起控制层（算法层只等首帧期望即初始化，控制层在收到首帧 desired 前不发命令，无死锁）。
- **组件内部参数**：算法层容器组件的 pid 参数只在构造时读取一次，运行中 `ros2 param set` 不生效。
- 订阅 / 发布用相对话题名（`desired_joint_states`、`simulated_*_states`），`/ctrl/<组>/*` 用绝对话题名。节点应运行在根命名空间 `/`；放入子命名空间会导致订阅断链。

### 初始化与话题

- 控制层在收到首帧 `output` 前不发布 feedback / 命令（只发布 desired）；算法层在收到首帧期望后初始化 plant 并开始发布 output。
- `/simulated_chassis_states` / `/simulated_arm_states` 为调试用途：关节顺序与 `/joint_states` 不同（底盘在前），且轮子 position 恒为 0。**勿按索引拼接成 30 关节全集**，请按 `name` 数组查名。

### 仿真语义

- 控制步长固定为 `dt = 1/loop_hz`，不测量真实经过时间；系统繁忙时模拟可能比真实时间慢（漂移）。
- 仿真层 `max_accel` / `max_vel` 只是**运动学限幅器**，不代表 Gazebo 端真实电机 / 关节物理极限。仿真结果用于验证控制逻辑，**不能**用于验证执行器真实性能。
- PID 微分项为裸离散差分，无低通滤波。当前纯仿真无传感器噪声，影响可忽略；若未来直接以带噪声的 `/joint_states` 做反馈，kd 项会放大噪声，需自行加滤波。
