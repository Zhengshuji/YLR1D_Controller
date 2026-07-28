# YLR1D 三层控制架构

## 设计背景

YLR1D 是一个约 30 DOF 的轮式双机械臂机器人：
- 4 个麦克纳姆轮（4 转向 + 4 驱动）
- 4 DOF 躯干（1 升降 + 1 偏航 + 2 俯仰）
- 2 x 9 DOF 手臂（7 自由度 + 2 手指）
- 各类传感器（IMU、Force-Torque、Camera、Lidar）

为降低系统耦合度、分层调试，设计了上中下三层控制架构。

---

## 架构图

```
                    上层
         运动学 / 规划层 (ylr1d_upper_control)

  底盘运动学: 麦克纳姆轮正逆解
    输入: vx, vy, wz  ->  输出: 4舵角 + 4轮速

  双臂运动学: IK 求解器 / MoveIt2
    输入: 末端位姿  ->  输出: 14 关节角度

  躯干规划: 升降 + 腰部耦合
  空间路径: 末端 Cartestian 插值

  输入: 速度指令 / 末端目标位姿 / 任务级命令
  输出: 6 x Float64MultiArray（映射到各控制器 Topic）
                          |
                          | 6 x /<controller_name>/commands
                          v
                    中层  [已验证通过]
        指令转发层 (ylr1d_mid_control)

  ros2_control + ForwardCommandController

  chassis_steering_controller  (4 pos)
  chassis_wheels_controller    (4 vel)
  torso_controller             (4 pos)
  left_arm_controller          (9 pos)
  right_arm_controller         (9 pos)
  joint_state_broadcaster      (30 state)

  桥接: Gazebo Classic 11 + gazebo_ros2_control plugin
  环境: ROS2 Humble, WSL Ubuntu 22.04
                          |
                          | 30 x command_interface
                          v
                    下层  (待实现)
          关节驱动层 (ylr1d_lower_control)

  PD 位置/速度控制器   每个关节一个闭环
  重力补偿             双臂和躯干重力矩前馈
  关节限位保护         软限位 + 缓冲区
  力矩/电流环          力控场景需要

  Gazebo 中: GazeboSystem 内部 PID（已在 URDF 定义）
  实际硬件: 微控制器 / RT Linux 独立进程
```

---

## 分层策略

### 为什么分三层？

1. 解耦 - 运动学、指令转发、关节控制三者的时间尺度和实现技术不同
2. 调试 - 每层可以独立验证，先确保中层能正确转发指令
3. 替换性 - 仿真中用 Gazebo 内置 PID 做下层，换真机时只需替换下层

### 层间接口

| 接口 | 方向 | 数据类型 | 说明 |
|------|------|----------|------|
| /<controller>/commands | 上层 -> 中层 | Float64MultiArray | 关节角度/速度指令 |
| /joint_states | 中层 -> 上层 | JointState | 当前关节状态反馈 |
| command interface | 中层 -> 下层 | double 值 | ros2_control 内部传递 |
| state interface | 下层 -> 中层 | double 值 | ros2_control 内部传递 |

---

## 包依赖关系

```
ylr1d_description              # 共享模型包（被所有包依赖）
  meshes/*.STL                 # 3D 网格文件
  urdf/*.xacro                 # URDF 模型定义
  config/*.yaml                # 颜色、动力学、限位等参数

ylr1d_mid_control              # 中层（已验证）
  config/controllers.yaml      # ros2_control 控制器定义
  urdf/ylr1d_mid.xacro         # ros2_control 硬件接口定义
  launch/gazebo.launch.py      # Gazebo + controller 启动
  config/*.yaml                # links, colors, limits 等

ylr1d_upper_control (待建)     # 上层（运动学/规划）
  src/chassis_kinematics/      # 麦克纳姆轮运动学
  src/arm_kinematics/          # 双臂运动学 (MoveIt2)
  src/planner/                 # 路径规划
  src/teleop/                  # 遥操作接口

ylr1d_lower_control (待建)     # 下层（关节驱动）
  src/pid_controller/          # PD/PID 关节闭环
  src/gravity_compensation/    # 重力补偿
  src/joint_limits/            # 限位保护
  src/hardware_interface/      # 真机硬件接口
```

---

## 当前进展

| 时间 | 里程碑 |
|------|--------|
| - | 完成 URDF 模型定义 (ylr1d_description) |
| OK | 解决 gazebo_ros2_control 插件参数解析 BUG |
| OK | 解决 LD_LIBRARY_PATH 导致插件不加载 |
| OK | 解决 YAML 参数结构错误导致 ForwardCommandController 配置失败 |
| OK | 6 个控制器全部加载、配置、激活 |
| OK | 通过 Topic 发布 Float64MultiArray 可控制 30 个关节 |
| TODO | 实现上层运动学（底盘 + 双臂） |
| TODO | 实现下层关节驱动（真机适用） |
| TODO | 整机运动协调控制 |

---

## 快速开始（验证中层）

```bash
# 在 WSL Ubuntu 22.04 中

# 1. 构建
source /opt/ros/humble/setup.bash
cd ~/WorkSpace/test_ylr1d
colcon build

# 2. 启动仿真
source install/setup.bash
ros2 launch ylr1d_mid_control gazebo.launch.py

# 3. 另开终端，发送测试指令
source ~/WorkSpace/test_ylr1d/install/setup.bash
ros2 topic pub /torso_controller/commands \
  std_msgs/Float64MultiArray "data: [0.2, 0.5, -0.3, 0.1]" --once
```
