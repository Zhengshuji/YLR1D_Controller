# ylr1d_position_simulate

模拟硬件层位置/速度闭环控制，在 ForwardCommandController 接收的期望值和用户设定的期望值之间插入 PID 过渡。适用于方案一（原始 position/velocity 接口）和 HMI 联合调试。

## 架构总览

```
用户 / 上层节点
       │ 发布 /desired_joint_states (sensor_msgs/JointState)
       ▼
┌─────────────────────────────────────────────────────────────┐
│                  ylr1d_position_simulate                     │
│         config/*.yaml → <关节名>/limit/*、<关节名>/pid/*      │
│                                                             │
│  Node_ChassisSimulate (chassis_simulate 节点)               │
│    ├─ PositionJointGroup (转向×4)  JointSimulator→位置      │
│    │   订阅: /joint_states (反馈)                            │
│    │   发布: /chassis_steering_controller/commands           │
│    └─ VelocityJointGroup (轮子×4)  PID→加速度→速度          │
│        发布: /chassis_wheels_controller/commands             │
│                                                             │
│  Node_ArmSimulate (arm_simulate 节点)                       │
│    └─ std::array<PositionJointGroup, ARM_GROUP_COUNT>        │
│       groups_  [TORSO=0 | LEFT_ARM=1 | RIGHT_ARM=2]         │
│         躯干×4 / 左臂×9 / 右臂×9，每关节 JointSimulator→位置 │
│        订阅: /joint_states (反馈)                            │
│        发布: /torso_controller/commands                      │
│        发布: /left_arm_controller/commands                   │
│        发布: /right_arm_controller/commands                  │
└─────────────────────────────────────────────────────────────┘
       │ 发布 5 组 ForwardCommandController 命令
       ▼
  Gazebo + ros2_control
       │
       ▼
  /joint_states (30 关节全量反馈)
```

**输入**: `/desired_joint_states`（用户/上层设定值）
**反馈**: `/joint_states`（Gazebo 当前关节状态）
**输出**: 5 组 ForwardCommandController 命令话题

## 话题与节点总览

| 节点 | 订阅 | 发布 | 控制频率 |
|------|------|------|----------|
| `chassis_simulate`（类 `Node_ChassisSimulate`） | `/joint_states`<br>`/desired_joint_states` | `/chassis_steering_controller/commands`<br>`/chassis_wheels_controller/commands` | 100Hz |
| `arm_simulate`（类 `Node_ArmSimulate`） | `/joint_states`<br>`/desired_joint_states` | `/torso_controller/commands`<br>`/left_arm_controller/commands`<br>`/right_arm_controller/commands` | 100Hz |

> 可执行名（`chassis_simulate` / `arm_simulate`）与 ROS 节点名保持不变，外部引用稳定；
> 内部类名统一为 `Node_ChassisSimulate` / `Node_ArmSimulate`，与节点定义分离。

## 配置文件（config/）

`config/` 下共三个 yaml，经 launch 加载后合并为两个节点的 ROS2 参数，统一命名
**`<关节名>/limit/<参数名>`** 与 **`<关节名>/pid/<参数名>`**。

| 文件 | 内容 | 说明 |
|------|------|------|
| `position_control_limits.yaml` | 26 个位置关节 `limit: {lower, upper, velocity, accelerate}` | 位置限位 + 速度/加速度限幅 |
| `velocity_control_limits.yaml` | 4 个轮子 `limit: {velocity, accelerate}` | 速度/加速度限幅（连续关节无位置限位） |
| `pid.yaml` | 全部 30 关节 `pid: {kp, ki, kd}` | 每关节独立 PID 增益 |

节点按关节 `declare_parameter` 读取（默认值与旧版硬编码一致，yaml 缺失时仍可运行）。
可通过 `ros2 param get <节点> <关节名>/pid/kp` 在线查询、`ros2 param set` 在线整定。

### limit 参数

| 参数 | 位置关节默认 | 轮子默认 | 说明 |
|------|-------------|---------|------|
| `limit/lower` | 来自 yaml（转向 ±3.14，Body1 ±0.3 等） | — | 位置下限 (rad 或 m) |
| `limit/upper` | 来自 yaml | — | 位置上限 |
| `limit/velocity` | 3.0 | 5.0 | 最大速度 (rad/s) |
| `limit/accelerate` | 50.0 | 20.0 | 最大加速度 (rad/s²) |

### pid 参数

| 参数 | 位置关节默认 | 轮子/夹爪默认 | 说明 |
|------|-------------|--------------|------|
| `pid/kp` | 4.0 | 2.0 | 比例增益 |
| `pid/ki` | 0.0 | 0.0 | 积分增益 |
| `pid/kd` | 0.2 | 0.05 | 微分增益 |

> 转向 / 躯干 / 臂关节 `4/0/0.2`，轮子与夹爪 `2/0/0.05`（见 `pid.yaml`）。

## 关节完整列表（30 关节）

所有关节在 `/joint_states` 中以固定顺序发布，以下按分组列出。

### 底盘 — 转向（4 关节，位置接口）

| 索引 | URDF 名称 | 限位 (rad) | 初始角度 (rad) |
|------|-----------|------------|----------------|
| 0 | `Joint_Base_to_RFWheelF` | ±3.14 | +2.497 |
| 1 | `Joint_Base_to_LFWheelF` | ±3.14 | -2.497 |
| 2 | `Joint_Base_to_RBWheelF` | ±3.14 | +0.644 |
| 3 | `Joint_Base_to_LBWheelF` | ±3.14 | -0.644 |

发布到 `/chassis_steering_controller/commands`，顺序与此表一致。

### 底盘 — 轮子（4 关节，速度接口）

| 索引 | URDF 名称 | 类型 | 初始值 |
|------|-----------|------|--------|
| 4 | `Joint_RFWheelF_to_RFWheel` | Continuous（连续旋转） | 0 |
| 5 | `Joint_LFWheelF_to_LFWheel` | Continuous | 0 |
| 6 | `Joint_RBWheelF_to_RBWheel` | Continuous | 0 |
| 7 | `Joint_LBWheelF_to_LBWheel` | Continuous | 0 |

发布到 `/chassis_wheels_controller/commands`，顺序与此表一致。

> 轮子关节无位置限位（连续旋转关节），速度/加速度限幅见 `limit/velocity`、`limit/accelerate`（`velocity_control_limits.yaml`）。

### 躯干（4 关节，位置接口）

| 索引 | URDF 名称 | 限位 (rad) | 说明 |
|------|-----------|------------|------|
| 8 | `Joint_Base_to_Body1` | **[-0.30, +0.30]** | 棱柱关节（升降），单位**米** |
| 9 | `Joint_Body1_to_Body2` | ±3.14 | 偏航 |
| 10 | `Joint_Body2_to_Body3` | ±1.57 | 俯仰 1 |
| 11 | `Joint_Body3_to_Body4` | ±1.57 | 俯仰 2 |

发布到 `/torso_controller/commands`，顺序与此表一致。

> **注意**: `Joint_Base_to_Body1` 限位非常小（±0.3 rad ≈ ±17°），且初始位置即为 0.3（上限），正向控制需先反向运动离开限位。

### 左臂（9 关节，位置接口）

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

发布到 `/left_arm_controller/commands`，顺序与此表一致。

### 右臂（9 关节，位置接口）

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

发布到 `/right_arm_controller/commands`，顺序与此表一致。

> 夹爪说明：正 q 值使两指沿相反方向平移（张开），负 q 值使两指相向运动（闭合）。
> 左指在 q=-0.014 时全开、q=0 时全闭；右指在 q=0 时全开、q=0.014 时全闭。

## JointState 话题数据结构

### 输入：`/desired_joint_states`

```yaml
header:
  stamp: <now>
name:
  - "Joint_Base_to_RFWheelF"    # position: 期望转向角
  - "Joint_Base_to_LFWheelF"
  - "Joint_Base_to_RBWheelF"
  - "Joint_Base_to_LBWheelF"
  - "Joint_RFWheelF_to_RFWheel" # velocity: 期望轮速
  - "Joint_LFWheelF_to_LFWheel"
  - "Joint_RBWheelF_to_RBWheel"
  - "Joint_LBWheelF_to_LBWheel"
  - "Joint_Base_to_Body1"       # position: 期望位置/升降
  - "Joint_Body1_to_Body2"
  - "Joint_Body2_to_Body3"
  - "Joint_Body3_to_Body4"
  - "Joint_Body2_to_LeftArm1"   # position: 期望位置
  - "Joint_LeftArm1_to_LeftArm2"
  - "Joint_LeftArm2_to_LeftArm3"
  - "Joint_LeftArm3_to_LeftArm4"
  - "Joint_LeftArm4_to_LeftArm5"
  - "Joint_LeftArm5_to_LeftArm6"
  - "Joint_LeftArm6_to_LeftArm7"
  - "Joint_LeftArm7_to_LeftFinger1"
  - "Joint_LeftArm7_to_LeftFinger2"
  - "Joint_Body2_to_RightArm1"     # position: 期望位置
  - "Joint_RightArm1_to_RightArm2"
  - "Joint_RightArm2_to_RightArm3"
  - "Joint_RightArm3_to_RightArm4"
  - "Joint_RightArm4_to_RightArm5"
  - "Joint_RightArm5_to_RightArm6"
  - "Joint_RightArm6_to_RightArm7"
  - "Joint_RightArm7_to_RightFinger1"
  - "Joint_RightArm7_to_RightFinger2"
position: [<转向×4>, <躯干×4>, <左臂×9>, <右臂×9>]  # 共 26 个
velocity: [<轮子×4>]                                   # 共 4 个
```

### 输出：各 ForwardCommandController 命令话题

Gazebo 原始控制器的命令话题，每路均为 `std_msgs/Float64MultiArray`，data 数组长度与关节数一致。

**角度值**: rad（弧度），**棱柱关节**: m（米），**速度**: rad/s（弧度/秒）

## 位置限幅

位置限位来自 `config/position_control_limits.yaml`，节点会在 `JointSimulator::update()` 内
**硬钳制**到 `[lower, upper]`（越限目标只会让关节停在限位处，不会越过）。HMI 或上层节点
仍应遵守以下约束：

**转向关节**: ±3.14 rad（全向范围，初始角度各异）
**轮子关节**: 无位置限位（连续旋转），速度上限由 `limit/velocity`（默认 5.0）控制
**躯干**: 
- `Joint_Base_to_Body1`：**±0.30**（棱柱，米）
- 其余 ±1.57~±3.14
**左右臂**:
- 大部分关节 ±1.57~±2.62
- `Joint_LeftArm6_to_LeftArm7` / `Joint_RightArm6_to_RightArm7`：**±6.28**（范围较大）
- 夹爪：左指 [-0.014, 0]、右指 [0, 0.014]（棱柱，米）

## 使用

```bash
# 终端 1: 启动 Gazebo + 控制器
ros2 launch ylr1d_mid_control gazebo.launch.py

# 终端 2: 启动模拟层
ros2 launch ylr1d_position_simulate position_simulate.launch.py

# 发送期望关节位置（可以只发送需要控制的关节）
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

> 可以只发布部分关节的期望值；未发布的关节保持当前值不变。

## 参数

所有关节参数统一按 **`<关节名>/limit/<参数名>`** 与 **`<关节名>/pid/<参数名>`** 命名，
由 launch 从 `config/` 下三个 yaml 加载（详见上文 [配置文件（config/）](#配置文件config)）。

- `loop_hz`（默认 100.0）：控制频率，launch 中设置
- 其余均为逐关节参数；`declare_parameter` 默认值与旧版硬编码一致，yaml 缺失时节点仍可运行

## 类结构

分层设计：**仿真核心（纯 C++，不依赖 rclcpp）** → **组类（桥接层，持有 publisher）** → **节点类**。

```
JointSimulator    (底层仿真控制器, 纯 C++)       [joint_simulator.{hpp,cpp}]
  - configure(params)  : 配置 PID 增益 + 限幅
  - initialize(pos)    : 从当前实际位置初始化
  - set_target(target) : 设定期望位置
  - update(dt) → 真实位置
      期望位置 → PID(加速度) → 速度(±max_vel 限幅) → 位置([lower,upper] 限幅)
  - position() / velocity() / initialized()
  - 对 PID 类封装，输出结果为真实位置（要求 3）

PID               (纯 C++)
  - compute(error, dt) → 加速度 (带 max_accel / max_vel 限幅)

PositionJointGroup   (一组位置关节, 每关节一个 JointSimulator)
  - setup(names, per_joint_params, topic, node) : 配置逐关节参数、建 publisher
  - init_from(msg)   : 从 JointState 读取当前值
  - set_desired(msg) : 按关节名设定期望位置
  - update(dt)       : 逐关节 JointSimulator::update
  - publish()        : 发布 Float64MultiArray（真实位置）
  - fill_state_msg() : 填充 /simulated_* 状态

VelocityJointGroup   (一组速度关节, 每关节独立 PID + 限幅)
  - setup(names, per_joint_velocity_params, topic, node)
  - set_desired(msg) : 按关节名设定期望速度
  - update(dt)       : PID → 加速度 → 速度(±max_vel 限幅)
  - publish()        : 发布 Float64MultiArray（当前速度）

Node_ChassisSimulate   (节点, chassis_simulate)   [node_chassis_simulate.{hpp,cpp}]
  - PositionJointGroup steering_ + VelocityJointGroup wheels_

Node_ArmSimulate       (节点, arm_simulate)       [node_arm_simulate.{hpp,cpp}]
  - enum ArmGroup : size_t { TORSO=0, LEFT_ARM=1, RIGHT_ARM=2, ARM_GROUP_COUNT=3 }
  - std::array<PositionJointGroup, ARM_GROUP_COUNT> groups_
  - kArmGroupSpecs[3] 静态表（话题 + 关节名），构造/初始化/更新/发布统一遍历
```

类定义与节点分离（要求 5）：仿真核心与组类只描述行为，节点类 `Node_*` 负责订阅/发布/定时器。

## 多线程决策

**结论：不需要多线程。** 每个节点使用默认的 `SingleThreadedExecutor`：

- 100Hz 定时器执行控制计算，订阅回调只写入 `joints_` 状态；单线程串行化后不存在数据竞争。
- 控制计算极轻量（30 关节 × 每关节几行浮点运算），单线程 100Hz 远未饱和。
- 引入多线程反而需要加锁保护共享关节状态，徒增复杂度。
- 两个节点（`chassis_simulate` / `arm_simulate`）由各自独立的单线程 executor 运行，天然并行。

如果未来出现耗时操作（如日志/网络 IO），才考虑把 executor 换成 `MultiThreadedExecutor` 或抽离
到独立回调组；届时再评估加锁方案。

## 已知限制

- 节点启动后需等待 `/joint_states` 首个消息完成内部状态初始化后才开始输出
- 初始化期间（Gazebo 控制器未 active 时）timer 空转不发布
- 轮子无初始化逻辑（连续关节无初始位置），从 0 速度开始
- 位置限位（含 `Joint_Base_to_Body1` 的 ±0.3）在 `JointSimulator::update()` 内硬钳制，
  越限目标只会停在限位处；HMI 仍应遵守限位以得到合理轨迹
