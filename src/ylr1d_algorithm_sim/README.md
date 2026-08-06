# ylr1d_algorithm_sim — 算法层（ROS2 包：仿真控制器）

控制算法 + 仿真被控对象模型的 ROS2 包。核心为**纯 C++ 算法库**（零 rclcpp，可单测），外加 **ROS2 节点壳**把每个"仿真控制器"包成一个组件，经 composition 合成单进程。控制参数（pid、限位、分组）全部移交本包维护，控制层只做通信与采样保持。

---

## 一、功能定位

- **架构位置**：算法层，与控制层（`ylr1d_control`）并列；二者经 **topic 通信**解耦（`/ctrl/<组>/*`），不再是"控制层进程内调用算法层"。
- **职责**：每个**仿真控制器**处理一组关节（按控制层分类后的组），节点内部固定线程框架：
  `输入(期望/反馈) → 协同控制 → 独立控制 → 仿真被控对象 → 输出`
  - **协同控制器**（cooperative）= 组级控制器（接口），当前实现 P=1 比例透传；
  - **独立控制器**（independent）= 逐关节控制器（接口），当前实现 PID（参数与 `pid.yaml` 相同）；
  - **仿真被控对象**（plant）= 整体输入/整体输出（Eigen 向量化），组内所有关节一起推进（位置/速度两型）。
- **算法可替换**：三类对象均以**接口**抽象，具体算法是实现；换控制算法只需改对应实现文件与装配点，接口与节点框架不动。
- **不做什么**：不接触 Gazebo / 真机，不直接暴露给 HMI / translate（对外一律经控制层）。

---

## 二、包结构

```
ylr1d_algorithm_sim/
├── include/ylr1d_algorithm_sim/        # 头文件（ROS2 规范 include/<pkg>/）
│   ├── cooperative_controller.hpp      # 虚基类：协同控制器（组级）
│   ├── independent_controller.hpp      # 虚基类：独立控制器（逐关节）
│   ├── plant.hpp                       # 虚基类：仿真被控对象（组级整体）
│   ├── steering.hpp / wheel.hpp / torso.hpp / left_arm.hpp / right_arm.hpp
│   │                                   # 按控制层 5 组对齐的具名类（每组含：协同/逐关节/Plant）
│   ├── control_law/                    # 具体算法（只声明，实现在 src/control_law/）
│   │   ├── proportional.hpp            #   P 比例协同（P=1 即透传）
│   │   ├── pid.hpp                     #   PID 逐关节（含 PidParams）
│   │   └── integrator.hpp              #   位置/速度积分器（标量 + 组级 Eigen 向量）
│   ├── config/joint_config.hpp         # 分组 + 限位 + 命令话题单一来源
│   └── ros/sim_controller.hpp          # SimControllerNode 模板 + 5 个具名节点
├── src/                                # 实现
│   ├── control_law/{proportional,pid,integrator}.cpp   # 具体算法（改算法改这里）
│   ├── {steering,wheel,torso,left_arm,right_arm}.cpp   # 各组具名类实现
│   └── ros/sim_controller.cpp          # 5 个具名节点注册
├── config/pid.yaml                     # 全部 30 关节 pid（控制参数移交算法层）
├── launch/sim_controller.launch.py     # composition 单进程，5 个仿真控制器组件
└── test/                               # 纯 C++ gtest（test_pid/plant/closed_loop/group）
```

构建产物：
- `libylr1d_algorithm_sim.so`（算法核心共享库，导出 CMake target `ylr1d_algorithm_sim::ylr1d_algorithm_sim`）；
- `libylr1d_algorithm_components.so`（组件库，含 5 个具名节点 `SteeringSimNode` / `WheelSimNode` / `TorsoSimNode` / `LeftArmSimNode` / `RightArmSimNode`，经 `rclcpp_components` 加载）。

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

| 组 | 控制类型 | 关节数 | 节点 | 仿真被控对象（具名 Plant） |
|----|---------|--------|------|---------|
| `steering` | 位置 | 4 | `SteeringSimNode` | `SteeringPlant`（1/s²） |
| `wheels` | 速度 | 4 | `WheelSimNode` | `WheelPlant`（1/s） |
| `torso` | 位置 | 4 | `TorsoSimNode` | `TorsoPlant` |
| `left_arm` | 位置 | 9 | `LeftArmSimNode` | `LeftArmPlant` |
| `right_arm` | 位置 | 9 | `RightArmSimNode` | `RightArmPlant` |

### 节点固定线程框架（`SimControllerNode<GroupPlantT>`）

```
timer(dt)：
  1. 协同控制  setpoint = cooperative_->compute(desired)   // 虚基类接口（当前 P=1 透传）
  2. 独立控制  u[i] = joints_[i]->compute(setpoint, feedback, dt)  // 虚基类接口（当前 PID）
  3. 仿真对象  out = plant_->update(u, dt)                  // 整体 Eigen 推进（bypass 直通）
  4. 发布      /ctrl/<组>/output（位置组含 position+velocity，速度组含 velocity）
```

### 三类对象接口与当前实现

| 对象 | 虚基类（`include/ylr1d_algorithm_sim/`） | 具名类（按组，`<组>.hpp`） | 内部算法（`src/control_law/`） | 方法 |
|------|--------------------------------------|------|------|------|
| 协同控制器 | `cooperative_controller.hpp` | `SteeringCooperativeController` / `Wheel…` / `Torso…` / `LeftArm…` / `RightArm…` | `proportional`（P=1 比例） | `set_group_size(n)` → `initialize()` → `compute(VectorXd 组期望) → VectorXd setpoint` |
| 独立控制器 | `independent_controller.hpp` | `SteeringJointController` 等（每关节一个实例） | `pid`（PID，参数 `PidParams`） | `initialize()` → `compute(setpoint, feedback, dt) → double u` |
| 仿真被控对象 | `plant.hpp` | `SteeringPlant` 等（位置型）/ `WheelPlant`（速度型） | `integrator`（位置/速度积分器，Eigen 向量） | `configure(limits, bypass)` → `initialize(state)` → `update(VectorXd u, dt) → state` → `state()/velocity()` |

**换算法怎么做**（以换独立控制器为例）：
1. 改算法本体：在 `src/control_law/` 新增/修改算法实现（如把 PID 换成 LQR，新增 `control_law/lqr.*`）；
2. 改使用方：把对应组的具名类（`src/<组>.cpp`）内部持有的算法类换成新实现；
3. 节点框架、虚基类接口、控制层均零改动。

---

## 五、配置

| 文件 | 内容 | 说明 |
|------|------|------|
| `config/pid.yaml` | 全部 30 关节 `pid: {kp, ki, kd}` | 控制参数移交算法层；launch 全量注入每个组件（组件只 declare 自己组内关节的参数，多余参数无害，保持单一来源） |
| `include/ylr1d_algorithm_sim/config/joint_config.hpp` | 关节分组（`kJointGroups`）+ 限位表（`jointLimitFor`）+ 命令话题常量 | 编译期常量，改后需重新编译；分组/限位/命令话题单一来源，控制层 include 本头 |

> 从 `ylr1d_control` 迁移：pid.yaml 与 joint_config.hpp 阶段 B 移交算法层，控制层不再维护。改 pid 改 `config/pid.yaml` 后重启算法层容器；改限位/分组改头文件后重编。

---

## 六、关键机制（选读）

### 6.1 架构：虚基类 + 按控制层对齐的具名类 + src 算法

核心层（`control_law/` + 各组具名类）零 rclcpp 依赖，可独立单元测试；`ros/` 只是 ROS2 适配——订阅/发布、参数声明、定时器，调用核心接口完成算法。三个虚基类（`cooperative_controller.hpp` / `independent_controller.hpp` / `plant.hpp`）定义"是什么"，各组具名类（`<组>.hpp`）继承并**按控制层分组命名**（`SteeringPlant` 等），"怎么做"在 `src/control_law/`（pid / proportional / integrator）。改算法只动 `src/control_law/` 与对应具名类，控制层零改动。

### 6.2 协同 / 独立 / 仿真的分工与接口

- **协同控制器**是**组级整体**操作（`Eigen::VectorXd`），当前 P=1 即透传；未来协同（动力学补偿等）在 `control_law/` 新增算法并改对应组的具名协同控制器；
- **独立控制器**逐关节，当前 PID（`control_law/pid`），参数来自 pid.yaml；换算法在 `control_law/` 新增实现并改对应组的具名逐关节控制器；
- **仿真被控对象**整体输入/整体输出（Eigen 向量化），内部逐关节复用标量积分器（`control_law/integrator`），限位限速在对象内完成；换模型改 `control_law/integrator` 与对应组的具名 Plant。
- 三者均以虚基类接入模板节点 `SimControllerNode<协同类, 逐关节类, Plant类>`，接口稳定、实现可换。

### 6.3 直通（bypass）

各组具名 Plant 在 `bypass` 置位（`configure(limits, bypass)` 组级开关）时 `update()` 直接返回输入 u——仿真环被跳过，控制层可把期望原样下发（对接 Gazebo 物理反馈 / 真机在环）。预留开关，当前 launch 未启用。

### 6.4 初始化时序

节点只依赖**首帧期望**完成初始化（`plant_->initialize(desired_)`、`feedback_ = desired_`），因此软仿真闭环无 feedback 死锁，可脱离 Gazebo 独立运行。

### 6.5 ThirdParty Eigen

Eigen 3.4 header-only 源码 vendor 于 `<WS_ROOT>/ThirdParty/Eigen`（从 `/usr/include/eigen3` 整体复制，`Eigen/Dense` 位于其下）。算法层 CMake `target_include_directories` 加入该路径，仅构建期使用、不 ament 导出。协同控制器与仿真对象（`Eigen::VectorXd`）实际使用。

---

## 七、已知限制与注意事项

- **当前仅实现 PID + P=1 协同 + 位置/速度双积分模型**；MPC / 自适应 / 动力学模型等为后续扩展（按 6.2 接口风格新增实现类，改 `assembly/` 装配点）。
- **参数只在节点构造时读取一次**，运行中 `ros2 param set` 不生效；调 pid 改 `pid.yaml` 后重启算法层容器。
- **命名空间 target**：消费方链接必须用 `ylr1d_algorithm_sim::ylr1d_algorithm_sim`（裸库名会丢失 include 目录与编译特性）。
- PID 微分项为裸离散差分，无低通滤波；以带噪声反馈做微分时需自行加滤波。
