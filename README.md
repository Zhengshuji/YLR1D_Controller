# YLR1D 机器人控制项目

双机械臂 + 全向四轮底盘 + 升降躯干的复合移动机器人仿真，基于 **ROS2 Humble + Gazebo Classic**，全模型 30 关节（4 转向 + 4 轮 + 4 躯干 + 9 左臂 + 9 右臂）。

---

## 一、总体架构

项目采用**分层结构**，四个功能包各司其职，`ylr1d_bringup` 负责一键聚合：

```
ylr1d_description   模型资产单一来源（xacro / meshes / config / sensors / rviz / world）
ylr1d_plant         物理层：Gazebo + ros2_control，模拟真实机器人
ylr1d_control_sim   控制层：软仿真，软件模拟硬件位置/速度闭环
ylr1d_hmi           人机界面：Qt5 观测 + 控制
ylr1d_bringup       一键启动：聚合上面三个 launch
```

**核心思路**：`ylr1d_control_sim` 用软件模拟真实硬件的闭环响应特性（PID 过渡），让上层控制逻辑不依赖真实硬件即可开发调试；`ylr1d_plant` 把控制命令落到 Gazebo 物理仿真；两层通过标准 ROS2 话题解耦，可任意替换其中一层（如日后接入真实硬件）。`ylr1d_plant` 另提供两套物理接口：**position 方案**直接下发目标角度，**effort 方案**下发力矩（更贴近真实执行器，适合验证执行效果）。

### 数据流

```
HMI / 上层算法 ──/desired_joint_states──▶ control_sim ──5组命令话题──▶ plant (Gazebo)
      ▲                                       ▲                              │
      └───────────────────────────────────────└── /joint_states 反馈 ◀──────┘
```

- `/desired_joint_states`（`sensor_msgs/JointState`）：期望关节位置/速度，由 HMI 或算法节点发布
- 5 组命令话题：`chassis_steering` / `chassis_wheels` / `torso` / `left_arm` / `right_arm` 各控制器的 `/commands`
- `/joint_states`：30 关节全量反馈（position / velocity / effort），棱柱关节 NaN 由 plant 内 `joint_state_filter` 处理为 0.0

---

## 二、包一览

> 每个包的功能、launch 与通信接口如下；包内细节见[文档导航](#六文档导航)中各包 README。

| 包名 | 功能与用途 | 提供 launch | 通信接口（topic） |
|------|-----------|------------|------------------|
| `ylr1d_description` | 模型资产单一来源。plant 与展示 launch 均从此取 URDF / config / mesh / world | `xacro_display.launch.py`（xacro 动态生成并展示）、`urdf_display.launch.py`（静态自包含 URDF 展示） | 无运行期 topic（纯资产提供方，由 robot_state_publisher 以参数加载 `robot_description`） |
| `ylr1d_plant` | 物理层/中控：管理 Gazebo + ros2_control，spawn 机器人、6 个控制器及 `joint_state_filter` | `gazebo.launch.py`（position 接口）、`gazebo_effort.launch.py`（effort 力控，推荐调试） | 发布 `/joint_states`、`/joint_states_filtered`；接收 `*_controller/commands`（position 方案）/ `*_effort_controller/commands`（effort 方案） |
| `ylr1d_control_sim` | 控制层软仿真：订阅期望关节状态与反馈，PID 过渡后生成平滑命令 | `position_simulate.launch.py`（启动 `chassis_simulate` + `arm_simulate` 两个节点） | 订阅 `/desired_joint_states`、`/joint_states`；发布 5 组 `*_controller/commands` |
| `ylr1d_hmi` | Qt5 人机界面：关节状态观测 + 关节控制 | `hmi.launch.py`（Lite 版，正常）；`hmi_rviz.launch.py`（RViz2 版，存在构建问题） | 订阅 `/joint_states`；发布 `/desired_joint_states` |
| `ylr1d_bringup` | 一键聚合启动以上全部，内部 include 三个 launch | `bringup.launch.py` | 无（纯启动编排） |

---

## 三、快速开始

### 环境
WSL Ubuntu-22.04 + ROS2 Humble + Gazebo Classic（各包依赖、构建细节见各包 README）。

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
ros2 launch ylr1d_bringup bringup.launch.py
```
等价于依次启动 `gazebo.launch.py` + `position_simulate.launch.py` + `hmi.launch.py`。
WSL 下控制器加载需 30-60s，轮询直到全部 active 再发命令：

```bash
ros2 control list_controllers    # 6 个控制器应全部 active
```

### 分步启动
```bash
ros2 launch ylr1d_plant gazebo.launch.py                    # 物理层
ros2 launch ylr1d_control_sim position_simulate.launch.py   # 控制层
ros2 launch ylr1d_hmi hmi.launch.py                         # 人机界面
```

---

## 四、问题排查

Gazebo 残留进程、控制器加载慢、TF_NAN、关节限位等常见问题，见 [CLAUDE.md](CLAUDE.md) 的"已知弯路 / 常见陷阱"。

---

## 五、接口速查

| 接口 | 类型 | 流向 | 说明 |
|------|------|------|------|
| `/desired_joint_states` | `sensor_msgs/JointState` | HMI / 算法 → control_sim | 期望关节位置/速度 |
| `/joint_states` | `sensor_msgs/JointState` | plant → 全系统 | 30 关节全量反馈（position / velocity / effort） |
| `/joint_states_filtered` | `sensor_msgs/JointState` | plant → 全系统 | NaN 已过滤为 0.0（仅 effort 方案） |
| `*_controller/commands` × 5 | `std_msgs/Float64MultiArray` | control_sim → plant | 各控制器命令（position 方案）；effort 方案为 `*_effort_controller/commands` |

---

## 六、文档导航

- [Controller.md](Controller.md)：系统架构（感知 / 决策 / 规划 / 控制 / 硬件）
- [CLAUDE.md](CLAUDE.md)：开发备忘与常见陷阱
- [colcon_build.sh](colcon_build.sh)：常用指令速查脚本
- 各包 README：
  - [ylr1d_description](src/ylr1d_description/README.md)
  - [ylr1d_plant](src/ylr1d_plant/README.md)
  - [ylr1d_control_sim](src/ylr1d_control_sim/README.md)
  - [ylr1d_hmi](src/ylr1d_hmi/README.md)
  - [ylr1d_bringup](src/ylr1d_bringup/README.md)
