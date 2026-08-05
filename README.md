# YLR1D 机器人控制项目

双机械臂 + 全向四轮底盘 + 升降躯干的复合移动机器人仿真，基于 **ROS2 Humble + Gazebo Classic**，全模型 30 关节（4 转向 + 4 轮 + 4 躯干 + 9 左臂 + 9 右臂）。

---

## 一、总体架构

项目采用**分层结构**：模型资产 → 物理层 → 控制层 → 转译层 → 人机界面，外加一键启动编排。各包的职责与相互关系见[包一览](#二包一览)。

**核心思路**：`ylr1d_control_sim` 用软件模拟真实硬件的闭环响应特性（PID 过渡），让上层控制逻辑不依赖真实硬件即可开发调试；`ylr1d_plant` 把控制命令落到 Gazebo 物理仿真。两层通过标准 ROS2 话题解耦，可任意替换其中一层（如日后接入真实硬件）。`ylr1d_translate` 位于上层与控制层之间，让上层以"高级指令"而非"关节坐标"的方式操控机器人。

### 数据流

**控制层链路**（HMI 直接下发关节期望值）：

```
HMI ──/desired_joint_states──▶ control_sim ──5组命令话题──▶ plant (Gazebo)
  ▲                                                          │
  └────────────────────── /joint_states 反馈 ◀──────────────┘
```

**转译层链路**（上层通过 action 下发高级指令）：

```
上层 ──action（/chassis_move /arm_move /gripper_move）──▶ translate
   ──/desired_joint_states──▶ control_sim ──5组命令话题──▶ plant (Gazebo)
      ▲                                                     │
      └───────────────── /joint_states 反馈 ◀───────────────┘
```

---

## 二、包一览

| 包 | 功能用途 | 与其他包的关系 |
|----|---------|----------------|
| `ylr1d_description` | 模型资产单一来源：xacro / meshes / 模型 config / rviz / world | 供 `ylr1d_plant` 与各展示 launch 取资产，其余包不依赖它 |
| `ylr1d_plant` | 物理层：管理 Gazebo + ros2_control，把控制命令落到物理仿真 | 承接 `ylr1d_control_sim` 的命令，向全系统发布 30 关节反馈 |
| `ylr1d_control_sim` | 控制层：软仿真，软件模拟硬件位置/速度闭环 | 接收 `ylr1d_hmi` / `ylr1d_translate` 的期望值；命令发往 `ylr1d_plant`；以 `ylr1d_plant` 反馈为输入 |
| `ylr1d_translate` | 转译层：把上层高级指令（action）解算为关节期望值 | 接收上层 / `ylr1d_hmi` 的 action；解算结果发给 `ylr1d_control_sim` |
| `ylr1d_hmi` | 人机界面：Qt5 多面板（控制 / 转译 / 传感器 / 监视） | 订阅 `ylr1d_plant` 反馈；控制层链路发期望值给 `ylr1d_control_sim`，转译层链路发 action 给 `ylr1d_translate` |
| `ylr1d_bringup` | 一键启动：聚合各包 launch（无节点） | 聚合除 `ylr1d_description` 外的各包 launch，提供控制层 / 转译层两套链路（`bringup_control` / `bringup_translate`） |
| `ylr1d_test` | 功能测试包：统一入口跑各层冒烟测试，结果落盘 `test_results/` | 独立运行、独立 launch，不并入 bringup；依赖各包做功能验证，见 [ylr1d_test README](src/ylr1d_tools/ylr1d_test/README.md) |

> `src/ylr1d_tools/` 是**辅助工具容器目录（非 package）**，当前含功能测试包 `ylr1d_test` 与入口脚本
> `src/ylr1d_tools/scripts/run_tests.sh`，详见 [ylr1d_tools README](src/ylr1d_tools/README.md)。

---

## 三、快速开始

### 构建
```bash
source /opt/ros/humble/setup.bash
cd <工作空间根目录>
colcon build
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src
```

### 一键启动（推荐）
```bash
# 控制层链路：Gazebo + 软仿真 + HMI
ros2 launch ylr1d_bringup bringup_control.launch.py

# 转译层链路：Gazebo + 软仿真 + translate + 转译层 HMI
ros2 launch ylr1d_bringup bringup_translate.launch.py
```

WSL 下控制器加载需 30-60s，轮询直到全部 active 再发命令：

```bash
ros2 control list_controllers    # 6 个控制器应全部 active
```

### 分步启动
```bash
ros2 launch ylr1d_plant gazebo.launch.py                    # 物理层
ros2 launch ylr1d_control_sim position_simulate.launch.py   # 控制层
ros2 launch ylr1d_translate translate.launch.py             # 转译层（可选）
ros2 launch ylr1d_hmi hmi.launch.py                         # 人机界面（或 hmi_translate.launch.py）
```

---

## 四、接口速查

> 完整接口与参数见各包 README；此处仅列全局关键接口。

| 接口 | 类型 | 流向 | 说明 |
|------|------|------|------|
| `/desired_joint_states` | `sensor_msgs/JointState` | HMI / translate → control_sim | 期望关节位置/速度 |
| `/joint_states` | `sensor_msgs/JointState` | plant → 全系统 | 30 关节全量反馈（position / velocity / effort） |
| `*_controller/commands` × 5 | `std_msgs/Float64MultiArray` | control_sim → plant | 各控制器命令（position 方案；effort 方案为 `*_effort_controller/commands`） |
| `/chassis_move` `/arm_move` `/gripper_move` | `action` | 上层 → translate | 底盘运动 / 机械臂躯干 / 夹爪开合 |

---

## 五、文档导航

- [Controller.md](Controller.md)：系统架构（感知 / 决策 / 规划 / 控制 / 硬件）
- [CLAUDE.md](CLAUDE.md)：开发备忘、注意事项（含常见陷阱）与工作流程
- [colcon_build.sh](colcon_build.sh)：常用指令速查脚本
- 各包 README：
  - [ylr1d_description](src/ylr1d_description/README.md)
  - [ylr1d_plant](src/ylr1d_plant/README.md)
  - [ylr1d_control_sim](src/ylr1d_control_sim/README.md)
  - [ylr1d_translate](src/ylr1d_translate/README.md)
  - [ylr1d_hmi](src/ylr1d_hmi/README.md)
  - [ylr1d_bringup](src/ylr1d_bringup/README.md)
  - [ylr1d_tools（工具容器）](src/ylr1d_tools/README.md)
  - [ylr1d_test（功能测试）](src/ylr1d_tools/ylr1d_test/README.md)
