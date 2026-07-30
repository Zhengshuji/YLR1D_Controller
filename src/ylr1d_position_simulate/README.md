# ylr1d_position_simulate

模拟硬件层位置/速度闭环控制，在 ForwardCommandController 和用户期望值之间插入 PID 过渡。

## 架构

```
用户发布 /desired_joint_states (JointState)
       │
       ├── chassis_simulate
       │     ├── 转向 4 关节: 位置 PID → /chassis_steering_controller/commands
       │     └── 轮子 4 关节: 速度 PID → /chassis_wheels_controller/commands
       │
       └── arm_simulate
             ├── 躯干 4 关节: 位置 PID → /torso_controller/commands
             ├── 左臂 9 关节: 位置 PID → /left_arm_controller/commands
             └── 右臂 9 关节: 位置 PID → /right_arm_controller/commands
```

## 使用

```bash
# 终端 1: 启动 Gazebo + 控制器
ros2 launch ylr1d_mid_control gazebo.launch.py

# 终端 2: 启动模拟层
ros2 launch ylr1d_position_simulate position_simulate.launch.py

# 发送期望关节位置
ros2 topic pub /desired_joint_states sensor_msgs/JointState "
{header: auto, name: ['Joint_Base_to_Body1'], position: [0.0]}"
```

## 关节约定

- **转向关节**: 在 JointState.position 字段输入期望位置
- **轮子关节**: 在 JointState.velocity 字段输入期望速度
- **躯干/臂关节**: 在 JointState.position 字段输入期望位置

## 参数

### chassis_simulate
| 参数 | 默认值 | 说明 |
|------|--------|------|
| loop_hz | 100.0 | 控制频率 |
| steering/kp | 5.0 | 转向位置 PID 比例 |
| steering/kd | 0.1 | 转向位置 PID 微分 |
| steering/max_accel | 50.0 | 转向最大加速度 |
| steering/max_vel | 3.0 | 转向最大速度 |
| wheel/kp | 2.0 | 轮子速度 PID 比例 |
| wheel/kd | 0.05 | 轮子速度 PID 微分 |
| wheel/max_accel | 20.0 | 轮子最大加速度 |
| wheel/max_vel | 5.0 | 轮子最大速度 |

### arm_simulate
| 参数 | 默认值 | 说明 |
|------|--------|------|
| loop_hz | 100.0 | 控制频率 |
| pid/kp | 5.0 | 位置 PID 比例 |
| pid/kd | 0.1 | 位置 PID 微分 |
| pid/max_accel | 50.0 | 最大加速度 |
| pid/max_vel | 3.0 | 最大速度 |

## 类结构

- **PID** — 纯数值 PID，compute() 输出加速度
- **PositionJointGroup** — 位置 PID 关节组，封装一组关节的 PID + 发布
- **VelocityJointGroup** — 速度 PID 关节组，封装一组关节的 PID + 发布
- **ChassisSimulateNode** — 底盘节点，持有一个 PositionJointGroup(转向) + VelocityJointGroup(轮子)
- **ArmSimulateNode** — 机械臂节点，持有三个 PositionJointGroup(躯干/左臂/右臂)

## 已知限制

- 节点启动后需等待 `/joint_states` 首个消息完成内部状态初始化后才开始输出
- 初始化期间（Gazebo 控制器未 active 时）timer 空转不发布
- 轮子无初始化逻辑（连续关节无初始位置），直接从 0 速度开始
