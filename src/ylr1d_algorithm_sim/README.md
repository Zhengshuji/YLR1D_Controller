# ylr1d_algorithm_sim — 算法层（ROS2 包：仿真控制器）

控制算法 + 被控对象模型的 ROS2 包。核心为**纯 C++ 算法库**（零 rclcpp，可单测），外加 **ROS2 节点壳**把每个"仿真控制器"包成一个组件，经 composition 合成单进程。控制参数（pid、限位、分组）全部移交本包维护，控制层只做通信与采样保持。

---

## 一、功能定位

- **架构位置**：算法层，与控制层（`ylr1d_control`）并列；二者经 **topic 通信**解耦（`/ctrl/<组>/*`），不再是"控制层进程内调用算法层"。
- **职责**：每个**仿真控制器**处理一组关节（按控制层分类后的组），节点内部固定线程框架：
  `输入(期望/反馈) → 协同控制 → 独立控制 → 仿真对象 → 输出`
  - **协同控制器** = 组级 P=1 比例控制器（当前透传，未来协同扩展位）；
  - **独立控制器** = 逐关节 PID（参数与 `pid.yaml` 相同）；
  - **仿真对象** = 整体输入/整体输出（Eigen 向量化），组内所有关节一起推进。
- **不做什么**：不接触 Gazebo / 真机，不直接暴露给 HMI / translate（对外一律经控制层）。

---

## 二、包结构

```
ylr1d_algorithm_sim/
├── controller/                     # 控制算法（纯 C++，零 rclcpp）
│   ├── include/controller/controller.hpp        # 汇总头
│   ├── pid/                        # PidController（逐关节独立控制器）
│   └── proportional/               # ProportionalController（协同控制器，P=1）
├── plant/                          # 被控对象模型（纯 C++，零 rclcpp）
│   ├── include/plant/plant.hpp     # 汇总头
│   ├── position/                   # PositionPlant（标量，1/s²）
│   ├── velocity/                   # VelocityPlant（标量，1/s）
│   ├── position_group/             # PositionGroupPlant（整体，Eigen 向量）
│   └── velocity_group/             # VelocityGroupPlant（整体，Eigen 向量）
├── node/                           # ROS2 节点壳
│   ├── include/algorithm/ros/sim_controller.hpp  # SimControllerNode 模板
│   └── src/sim_controller.cpp      # rclcpp_components 注册（Position/Velocity）
├── include/algorithm/config/joint_config.hpp     # 分组 + 限位 + 命令话题单一来源
├── config/pid.yaml                 # 全部 30 关节 pid（控制参数移交算法层）
├── launch/sim_controller.launch.py # composition 单进程，5 个仿真控制器组件
└── test/                           # 纯 C++ gtest（test_pid/plant/closed_loop/group）
```

> **汇总头约定**：核心层消费方只需 include `controller/controller.hpp` 与 `plant/plant.hpp` 即可获得全部类型。

构建产物：
- `libylr1d_algorithm_sim.so`（算法核心共享库，导出 CMake target `ylr1d_algorithm_sim::ylr1d_algorithm_sim`）；
- `libylr1d_algorithm_components.so`（组件库，含 `PositionSimController` / `VelocitySimController`，经 `rclcpp_components` 加载）。

---

## 三、使用方法

```bash
# 单独启动算法层（composition，5 个仿真控制器节点）
ros2 launch ylr1d_algorithm_sim sim_controller.launch.py

# 完整闭环（含 Gazebo 物理层 + 控制层采样保持 + HMI）
ros2 launch ylr1d_bringup bringup_control.launch.py
```

算法层节点**只等首帧期望**即初始化（plant 初始状态 = 首帧期望，无瞬态；feedback 同步为期望 → 首轮误差为 0），不依赖反馈首帧，脱离 Gazebo 也能跑软仿真闭环。

运行单元测试（纯 C++ 环境，无需 ROS2）：

```bash
colcon test --packages-select ylr1d_algorithm_sim --event-handlers console_direct+
```

---

## 四、对外资产与接口

| 资产 | 说明 |
|------|------|
| 节点 | 5 个仿真控制器（composition 单进程）：`steering_sim` / `wheels_sim` / `torso_sim` / `left_arm_sim` / `right_arm_sim` |
| 话题 | 订阅 `/ctrl/<组>/desired`、`/ctrl/<组>/feedback`；发布 `/ctrl/<组>/output`（均为 `sensor_msgs/JointState`） |
| 参数 | `group_name`、`loop_hz`、`<关节>/pid/*`（pid.yaml 注入）；分组/限位从 `joint_config.hpp` 编译期读取 |
| CMake target | `ylr1d_algorithm_sim::ylr1d_algorithm_sim`（含 include 目录传递，链接需用命名空间 target） |

### 话题约定（`/ctrl/<组>/*`，组见 `joint_config.hpp` 的 `kJointGroups`）

| 组 | 控制类型 | 关节数 | 节点 | 仿真对象 |
|----|---------|--------|------|---------|
| `steering` | 位置 | 4 | `PositionSimController` | `PositionGroupPlant`（1/s²） |
| `wheels` | 速度 | 4 | `VelocitySimController` | `VelocityGroupPlant`（1/s） |
| `torso` | 位置 | 4 | `PositionSimController` | `PositionGroupPlant` |
| `left_arm` | 位置 | 9 | `PositionSimController` | `PositionGroupPlant` |
| `right_arm` | 位置 | 9 | `PositionSimController` | `PositionGroupPlant` |

### 节点固定线程框架（`SimControllerNode<GroupPlantT>`）

```
timer(dt)：
  1. 协同控制  setpoint = cooperative_.compute(desired)      // P=1 比例，当前透传
  2. 独立控制  u[i] = independent_[i].compute(setpoint, feedback, dt)  // 逐关节 PID
  3. 仿真对象  out = plant_.update(u, dt)                    // 整体 Eigen 推进（bypass 直通）
  4. 发布      /ctrl/<组>/output（位置组含 position+velocity，速度组含 velocity）
```

### 核心算法接口

- `PidController`（`controller/pid/pid.hpp`）：`configure(params)` → `initialize()` → `compute(setpoint, feedback, dt)` → `reset()`
- `ProportionalController`（`controller/proportional/proportional_controller.hpp`）：`configure(gain)` → `compute(VectorXd)`（返回 `gain × 输入`）
- `PositionGroupPlant`（`plant/position_group/`）：`configure(vector<PositionPlantParams>)` → `initialize(pos, vel)` → `update(VectorXd u, dt)` → `position()/velocity()`（bypass 时 `update` 直通返回 u）
- `VelocityGroupPlant`（`plant/velocity_group/`）：同上（→ 速度向量）

---

## 五、配置

| 文件 | 内容 | 说明 |
|------|------|------|
| `config/pid.yaml` | 全部 30 关节 `pid: {kp, ki, kd}` | 控制参数移交算法层；launch 全量注入每个组件（组件只 declare 自己组内关节的参数，多余参数无害，保持单一来源） |
| `include/algorithm/config/joint_config.hpp` | 关节分组（`kJointGroups`）+ 限位表（`jointLimitFor`）+ 命令话题常量 | 编译期常量，改后需重新编译；分组/限位/命令话题单一来源，控制层 include 本头 |

> 从 `ylr1d_control` 迁移：pid.yaml 与 joint_config.hpp 阶段 B 移交算法层，控制层不再维护。改 pid 改 `config/pid.yaml` 后重启算法层容器；改限位/分组改头文件后重编。

---

## 六、关键机制（选读）

### 6.1 两层结构：纯 C++ 核心 + ROS2 组件壳

核心层（`controller/`、`plant/`）零 rclcpp 依赖，可独立单元测试；`node/` 只是 ROS2 适配——订阅/发布、参数声明、定时器，调用核心类完成算法。算法调整（换控制器、调参、换被控对象）只动本包，控制层零改动。

### 6.2 协同 / 独立 / 仿真的分工

- 协同控制器是**组级整体**操作（`Eigen::VectorXd`），当前 P=1 即透传；未来协同（动力学补偿等）在此扩展，不动独立 PID 与仿真对象；
- 独立控制器**逐关节** PID，参数来自 pid.yaml；
- 仿真对象**整体输入/整体输出**（Eigen 向量化），内部逐关节复用标量 `PositionPlant` / `VelocityPlant`，限位限速在仿真对象内完成。

### 6.3 直通（bypass）

`*GroupPlant::update()` 在 `bypass` 置位时直接返回输入 u——仿真环被跳过，控制层可把期望原样下发（对接 Gazebo 物理反馈 / 真机在环）。预留开关，当前 launch 未启用。

### 6.4 初始化时序

节点只依赖**首帧期望**完成初始化（`plant.initialize(desired_)`、`feedback_ = desired_`），因此软仿真闭环无 feedback 死锁，可脱离 Gazebo 独立运行。

### 6.5 ThirdParty Eigen

Eigen 3.4 header-only 源码 vendor 于 `<WS_ROOT>/ThirdParty/Eigen`（从 `/usr/include/eigen3` 整体复制，`Eigen/Dense` 位于其下）。算法层 CMake `target_include_directories` 加入该路径，仅构建期使用、不 ament 导出。协同控制器与仿真对象（`Eigen::VectorXd`）实际使用。

---

## 七、已知限制与注意事项

- **当前仅实现 PID + P=1 协同**；MPC / 自适应等算法为后续扩展（同一接口风格新增 `controller/`、`plant/` 下子库）。
- **参数只在节点构造时读取一次**，运行中 `ros2 param set` 不生效；调 pid 改 `pid.yaml` 后重启算法层容器。
- **命名空间 target**：消费方链接必须用 `ylr1d_algorithm_sim::ylr1d_algorithm_sim`（裸库名会丢失 include 目录与编译特性）。
- PID 微分项为裸离散差分，无低通滤波；以带噪声反馈做微分时需自行加滤波。
