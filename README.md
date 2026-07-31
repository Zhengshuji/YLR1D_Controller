# YLR1D 机器人控制项目

双机械臂 + 全向四轮底盘 + 升降躯干的复合移动机器人，基于 **ROS2 Humble + Gazebo Classic** 仿真。

- 底盘：4 转向 + 4 轮（全向）
- 躯干：升降（棱柱）+ 偏航 + 2 俯仰
- 左右臂：各 7-DOF + 2 夹爪指
- 传感器：全局/左/右 RGB-D 相机 + 红外、雷达、IMU、四路超声波
- 全模型 30 个关节

---

## 一、包介绍

| 包名 | 职责 | 说明 |
|------|------|------|
| `ylr1d_description` | **模型资产（单一来源）** | URDF/xacro、meshes、模型配置 yaml、传感器参数、rviz、world。`ylr1d_plant` 与展示 launch 均从这里取资产 |
| `ylr1d_plant` | **物理层（中控）** | 管理 Gazebo + ros2_control 的配置与启动。提供 `gazebo.launch.py`（position 接口）与 `gazebo_effort.launch.py`（effort 接口）两套方案；内含 `joint_state_filter` 节点（NaN → 0.0） |
| `ylr1d_control_sim` | **控制层（软仿真）** | 模拟硬件层位置/速度闭环：订阅 `/desired_joint_states` + `/joint_states`，经 PID 过渡后发布 5 组 ForwardCommandController 命令话题 |
| `ylr1d_hmi` | **人机界面** | Qt5 界面，关节状态观测 + 关节控制。Lite 版（`hmi.launch.py`）正常；RViz2 版存在构建/运行问题 |

### 数据流

```
用户 / HMI ──/desired_joint_states──▶ ylr1d_control_sim ──5组命令话题──▶ ylr1d_plant (Gazebo)
        ▲                                                                        │
        └───────────────────────────── /joint_states (30 关节反馈) ◀──────────────┘
```

- `ylr1d_control_sim`：转向（位置）、轮子（速度）、躯干/左右臂（位置），PID 过渡后输出平滑命令
- `ylr1d_plant`：`gazebo_effort.launch.py` 输出力矩/位置命令，通过 6 个控制器下发到 Gazebo
- `/joint_states`：30 关节全量反馈（position / velocity / effort），棱柱关节 NaN 由 `joint_state_filter` 处理为 0.0

---

## 二、环境准备

- **系统**: WSL Ubuntu-22.04（本项目在 WSL 中运行）
- **ROS2**: Humble
- **Gazebo**: Classic（`gzserver` / `gzclient`）
- **Qt5**: `qtbase5-dev`、`libqt5widgets5`（仅 HMI 需要）
- **额外包**: `ros-humble-gazebo-ros2-control`、`ros-humble-controller-manager`、`ros-humble-ros2-controllers`、`ros-humble-joint-state-broadcaster`、`ros-humble-forward-command-controller`

> 从 Windows CLI 调用 WSL 时，必须加 `MSYS2_ARG_CONV_EXCL="*"` 防止 Git Bash 路径转换（见[常见问题](#六常见问题)）。

---

## 三、构建

```bash
source /opt/ros/humble/setup.bash
cd <工作空间根目录>          # 例如 ~/WorkSpace/test_ylr1d

# 全量构建
colcon build
source install/setup.bash

# 指定包快速重编（改源码后）
colcon build --packages-select ylr1d_description ylr1d_plant ylr1d_control_sim

# 指定包彻底重建（清缓存，排除残留，推荐在改动 include/CMakeLists 后使用）
rm -rf build/ylr1d_plant install/ylr1d_plant
colcon build --packages-select ylr1d_plant
```

环境变量（每次新开终端）：
```bash
source install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<工作空间根目录>/src
```

> 脚本 `colcon_build.sh` 提供常用指令速查。

---

## 四、使用方法

### 1. 物理层仿真（Gazebo + ros2_control）

```bash
# 方案 A：position 接口（原始命令控制器）
ros2 launch ylr1d_plant gazebo.launch.py

# 方案 B：effort 力控（推荐调试）
ros2 launch ylr1d_plant gazebo_effort.launch.py
```

WSL 下 Gazebo 启动很慢（30-60s 控制器才加载完），务必等待全部控制器 `active`：
```bash
ros2 control list_controllers
# 期望（effort 方案）：chassis_steering_effort_controller / chassis_wheels_effort_controller /
# torso_effort_controller / left_arm_effort_controller / right_arm_effort_controller /
# joint_state_broadcaster  全部 active
```

### 2. 控制层软仿真（位置/速度闭环）

```bash
ros2 launch ylr1d_control_sim position_simulate.launch.py
```

> **必须通过 launch 启动**（加载 `config/*.yaml`）。直接 `ros2 run` 时参数未加载，位置关节会被钳死到 0。

### 3. 人机界面（HMI，Lite 版）

```bash
ros2 launch ylr1d_hmi hmi.launch.py
```

### 4. 纯展示（RViz / Gazebo 看模型，无控制器）

```bash
ros2 launch ylr1d_description xacro_display.launch.py   # 动态生成（推荐）
ros2 launch ylr1d_description urdf_display.launch.py     # 静态 URDF（自包含，无外部 yaml 依赖）
```

---

## 五、详细控制方法

### 1. 查看广义坐标

```bash
ros2 topic echo /joint_states --once        # 30 关节 position/velocity/effort
ros2 topic echo /joint_states_filtered --once  # NaN 已过滤为 0.0
```

### 2. 力控（effort 方案）— 发送力矩命令

ForwardCommandController 保持最后值，建议 `--rate 20` 持续发送。

```bash
# 转向 — 30Nm 能看到明显转动
ros2 topic pub --rate 20 /chassis_steering_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [30.0, 30.0, 30.0, 30.0]}"

# 车轮
ros2 topic pub --rate 20 /chassis_wheels_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [20.0, 20.0, 20.0, 20.0]}"

# 躯干 — 注意 Joint_Base_to_Body1 上限 0.3 rad（初始就在上限），先负向离开
ros2 topic pub --rate 20 /torso_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [-100.0, 0.0, 0.0, 0.0]}"

# 左臂（9 关节，最后 2 个是夹爪）
ros2 topic pub --rate 20 /left_arm_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0]}"

# 右臂
ros2 topic pub --rate 20 /right_arm_effort_controller/commands \
  std_msgs/Float64MultiArray "{data: [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0]}"
```

### 3. 软仿真（control_sim 方案）— 发布期望关节状态

```bash
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

可以只发布部分关节的期望值，未发布的关节保持当前值不变。关节限位 / PID 参数见
`ylr1d_control_sim/config/*.yaml`（`<关节名>/limit/*`、`<关节名>/pid/*`）。

---

## 六、常见问题

### 1. WSL 路径转换（MSYS2_ARG_CONV_EXCL）
Git Bash 会把 `/home/...` 转成 `C:/Program Files/Git/...`。所有 `wsl.exe` 调用前加
`MSYS2_ARG_CONV_EXCL="*"`，或在 WSL 终端内直接执行。

### 2. Gazebo 残留进程
`ros2 launch` 退出后 `gzserver` 可能仍在后台，再次启动报 `Entity already exists`。清理：
```bash
pkill -f gzserver; pkill -f gzclient
```

### 3. 控制器加载慢（WSL）
WSL 下控制器从启动到 active 需要 30-60s。用 `ros2 control list_controllers` 轮询，不能抢跑。

### 4. TF_NAN（棱柱关节）
Gazebo 将 prismatic joint 位置初始化为 NaN。`joint_state_filter` 节点（`ylr1d_plant`）自动
NaN → 0.0，过滤后数据在 `/joint_states_filtered`，已集成到 `gazebo_effort.launch.py`。

### 5. Joint_Base_to_Body1 限位
限位 [-0.3, 0.3]，初始位置就是 0.3，正向力矩推不动。调试时先用负向力矩离开限位。

### 6. effort 命令需要持续发送
单次 `--once` 在重力/碰撞下可能不够。用 `--rate 20` 持续发送。

### 7. position_simulate 必须经 launch 启动
直接 `ros2 run` 时 `config/*.yaml` 未加载，位置关节按默认限位 [0,0] 且限位开启，全部被钳死到 0。

### 8. pkill -f 自匹配
`pkill -f chassis_simulate` 会匹配到 bash 自身命令行而杀掉执行 shell。用 `pkill -f "[c]hassis_simulate"`。

---

## 七、补充说明

- 各包详细文档见 `src/<包名>/README.md`，均按"包介绍 → 使用方法 → 详细控制方法 → 补充说明 → 问题解决"组织
- 系统架构文档（感知/决策/规划/控制/硬件）见 `Controller.md`
- 模型资产（xacro/config/meshes/rviz/world）**统一由 `ylr1d_description` 提供**，`ylr1d_plant` 只保留自己的 controllers.yaml
- 静态 `ylr1d_description/urdf/ylr1d.urdf` 为自包含文件（模型配置已内联，不引用任何外部 yaml），可随工作空间移动
