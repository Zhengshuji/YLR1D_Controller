# ylr1d_control_sim — 控制层（软仿真）

模拟硬件层位置/速度闭环控制，在期望值与 Gazebo 反馈之间插入 PID 过渡，生成平滑命令。适用于控制层链路（HMI 直发）与转译层链路（translate 驱动）联合调试。

---

## 一、功能定位

- **架构位置**：控制层，位于 `ylr1d_hmi` / `ylr1d_translate` 与 `ylr1d_plant` 之间。
- **职责**：订阅 `/desired_joint_states`（期望值）+ `/joint_states`（反馈），PID 过渡后发布 5 组 ForwardCommandController 命令话题。
- **两个节点**：`chassis_simulate`（底盘：转向 position + 轮子 velocity）、`arm_simulate`（躯干 / 左臂 / 右臂，均为 position）。
- **不做什么**：不接触 Gazebo，只在期望值与反馈之间做运动学过渡（PID 限幅），不代表真实电机物理极限。

---

## 二、包结构

```
include/ylr1d_control_sim/
├── core/    joint_params.hpp  pid.hpp  joint_simulator.hpp   # 纯 C++，不依赖 rclcpp
├── groups/  joint_group.hpp        # PositionJointGroup / VelocityJointGroup
├── params/  param_reader.hpp       # read_joint_params(node, name, preset)
└── nodes/   chassis_simulate_node.hpp  arm_simulate_node.hpp
src/
├── core/    pid.cpp  joint_simulator.cpp
├── groups/  joint_group.cpp
├── nodes/   chassis_simulate_node.cpp  arm_simulate_node.cpp  (无 main)
└── main_chassis.cpp  main_arm.cpp     # 独立入口
config/
├── position_control_limits.yaml   # 26 个位置关节 limit（lower/upper/velocity/accelerate）
├── velocity_control_limits.yaml   # 4 个轮子 limit（velocity/accelerate）
└── pid.yaml                        # 全部 30 关节 pid（kp/ki/kd）
launch/position_simulate.launch.py  # 必须经此启动（加载 config）
```

---

## 三、使用方法

> ⚠️ **必须通过 `position_simulate.launch.py` 启动**（加载 config 三个 yaml）。直接 `ros2 run` 时位置关节按默认限位 [0,0] 且限位开启，**所有 26 个位置关节被钳死到 0**。

```bash
# 分步启动（配合 plant + HMI）
ros2 launch ylr1d_plant gazebo.launch.py                    # 终端 1：物理层（position 方案）
ros2 launch ylr1d_control_sim position_simulate.launch.py   # 终端 2：控制层

# 完整闭环一键启动
ros2 launch ylr1d_bringup bringup.launch.py
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
ros2 topic echo /joint_states --once   # 观察反馈
```

---

## 四、接口

### 节点

| 节点（可执行名） | 订阅 | 发布 | 控制频率 |
|------|------|------|----------|
| `chassis_simulate` | `/joint_states`<br>`/desired_joint_states` | `/chassis_steering_controller/commands`<br>`/chassis_wheels_controller/commands` | 100Hz |
| `arm_simulate` | `/joint_states`<br>`/desired_joint_states` | `/torso_controller/commands`<br>`/left_arm_controller/commands`<br>`/right_arm_controller/commands` | 100Hz |

> 可执行名与 ROS 节点名保持不变，外部引用稳定；内部类名统一为 `ChassisSimulateNode` / `ArmSimulateNode`。

### 话题数据结构

**输入 `/desired_joint_states`**（`sensor_msgs/JointState`）：`name` 列出需控制的关节，`position` / `velocity` 按 `name` 顺序索引对齐。约定：转向 / 躯干 / 臂 / 夹爪用 `position`（26 个），轮子用 `velocity`（4 个）。

**输出 5 组命令话题**：均为 `std_msgs/Float64MultiArray`，data 数组长度与该控制器关节数一致（顺序见[关键机制](#六关键机制选读) 的关节表）。角度 rad，棱柱关节 m，轮速 rad/s。

### 参数

所有关节参数统一按 **`<关节名>/limit/<参数名>`** 与 **`<关节名>/pid/<参数名>`** 命名，由 launch 从 config 加载。另有 `loop_hz`（默认 100.0）控制频率。参数回退**两层**（默认值只在预设 struct 中定义一次）：

```
yaml (<关节名>/limit/*、<关节名>/pid/*)
  → 节点参数 (declare_parameter 默认 = 预设 struct)
    → 预设 struct defaultPositionParams / defaultVelocityParams（定义于 core/joint_params.hpp）
```

参数仅在**节点构造时读取一次**，运行中 `ros2 param set` 修改**不会生效**，调参需改 yaml 后重启节点。

---

## 五、配置

### config 三个 yaml

| 文件 | 内容 | 说明 |
|------|------|------|
| `position_control_limits.yaml` | 26 个位置关节 `limit: {lower, upper, velocity, accelerate}` | 位置限位 + 速度/加速度限幅 |
| `velocity_control_limits.yaml` | 4 个轮子 `limit: {velocity, accelerate}` | 速度/加速度限幅（连续关节无位置限位） |
| `pid.yaml` | 全部 30 关节 `pid: {kp, ki, kd}` | 每关节独立 PID 增益 |

### limit 参数

| 参数 | 位置关节默认 | 轮子默认 | 说明 |
|------|-------------|---------|------|
| `limit/lower` | 来自 yaml（转向 ±3.14，Body1 ±0.3 等） | — | 位置下限（rad 或 m） |
| `limit/upper` | 来自 yaml | — | 位置上限 |
| `limit/velocity` | 3.0 | 5.0 | 最大速度（rad/s） |
| `limit/accelerate` | 50.0 | 20.0 | 最大加速度（rad/s²） |

### pid 参数

| 参数 | 位置关节默认 | 轮子/夹爪默认 | 说明 |
|------|-------------|--------------|------|
| `pid/kp` | 4.0 | 2.0 | 比例增益 |
| `pid/ki` | 0.0 | 0.0 | 积分增益 |
| `pid/kd` | 0.2 | 0.05 | 微分增益 |

> 上表为**预设默认值**（yaml 缺失时生效），定义于 `core/joint_params.hpp` 的 `defaultPositionParams` / `defaultVelocityParams`。当前 `pid.yaml` 实际调参：转向/躯干/臂关节 kp=150、kd=20，轮子 kp=4、kd=0.1，夹爪 kp=10、kd=20（每关节独立，见 `pid.yaml`）。

---

## 六、关键机制（选读）

### 6.1 30 关节完整列表

所有关节在 `/joint_states` 中以固定顺序发布，命令数组顺序与下表一致。

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

> 轮子无位置限位（连续旋转），速度/加速度限幅见 `limit/velocity`、`limit/accelerate`。

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

### 6.2 类结构

分层设计：**仿真核心（纯 C++，不依赖 rclcpp）→ 组类（桥接层，持有 publisher）→ 节点类**。

- `JointParams`（纯 C++）：统一参数结构体（kp/ki/kd、max_accel、max_vel、has_position_limit、lower/upper），含预设函数 `defaultPositionParams()` / `defaultVelocityParams()`
- `JointSimulator`（位置仿真器）：`configure → initialize(pos) → set_target(target) → update(dt) → position()/velocity()`；内部 `期望位置 → PID(加速度) → 速度(±max_vel) → 位置([lower,upper] 限幅)`
- `PID`（纯 C++）：`compute(error, dt) → 加速度`，带 max_accel 限幅
- `PositionJointGroup` / `VelocityJointGroup`：一组关节，`setup → init_from → set_desired → update → publish`
- `ChassisSimulateNode` / `ArmSimulateNode`：订阅/发布/定时器；`main()` 独立到 `src/main_*.cpp`，节点类可作库复用与单测

### 6.3 多线程决策

**结论：不需要多线程。** 每节点用默认 `SingleThreadedExecutor`：100Hz 定时器执行控制计算，订阅回调只写关节状态，单线程串行化后无数据竞争；控制计算极轻量，单线程远未饱和；引入多线程反而需加锁。两个节点由各自独立 executor 运行，天然并行。若未来出现耗时操作（日志/网络 IO）再评估换 `MultiThreadedExecutor`。

### 6.4 位置限幅

位置限位来自 `config/position_control_limits.yaml`，节点在 `JointSimulator::update()` 内**硬钳制**到 `[lower, upper]`（越限目标只会停在限位处，不会越过）。HMI 或上层节点仍应遵守限位约束：转向 ±3.14；轮子无位置限位（速度上限 `limit/velocity`）；躯干 `Joint_Base_to_Body1` ±0.30（米）；双臂大部分 ±1.57~±2.62、`*Arm6_to_*Arm7` ±6.28；夹爪左指 [-0.014, 0]、右指 [0, 0.014]。

---

## 七、已知限制与注意事项

### 启动与配置

- **必须通过 `position_simulate.launch.py` 启动**（加载 config 三个 yaml），直接 `ros2 run` 时所有 26 个位置关节被钳死到 0
- yaml 中漏关节或关节名拼错**不会报错**：该关节静默使用默认值（配合上一条可能锁死关节）。请保持 30 关节配置完整，改配置后用 `ros2 param list` 核对
- 参数仅在节点构造时读取一次，运行中 `ros2 param set` 不生效
- 订阅/发布用相对话题名（`desired_joint_states` / `joint_states` / `simulated_*_states`）。节点应运行在根命名空间 `/`；放入子命名空间会导致订阅断链

### 初始化与话题

- 节点启动后需等待 `/joint_states` 首个消息完成初始化后才开始输出；初始化期间 timer 空转不发布
- 轮子无初始化逻辑（连续关节无初始位置），从 0 速度开始，不与 Gazebo 当前轮速衔接
- `/simulated_chassis_states` / `/simulated_arm_states` 为调试用途：关节顺序与 `/joint_states` 不同（底盘在前），且轮子 position 恒为 0。**勿按索引拼接成 30 关节全集**，请按 `name` 数组查名
- 位置限位在 `JointSimulator::update()` 内硬钳制，越限目标只会停在限位处

### 仿真语义

- 控制步长固定为 `dt = 1/loop_hz`，不测量真实经过时间；系统繁忙时模拟可能比真实时间慢（漂移）
- 仿真层 `max_accel` / `max_vel` 只是**运动学限幅器**，不代表 Gazebo 端真实电机/关节物理极限。仿真结果用于验证控制逻辑，**不能**用于验证执行器真实性能
- PID 微分项为裸离散差分，无低通滤波。当前纯仿真无传感器噪声，影响可忽略；若未来直接以带噪声的 `/joint_states` 做反馈，kd 项会放大噪声，需自行加滤波
