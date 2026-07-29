# YLR1D 机器人控制项目

## 包结构

| 包名 | 说明 |
|------|------|
| `ylr1d_description` | 机器人模型定义：URDF/xacro、3D 网格、传感器参数、颜色/动力学/限位等配置 |
| `ylr1d_mid_control` | 中层控制：Gazebo 仿真 + ros2_control 关节控制，提供三种控制方案 |

> `ylr1d_mid_control` 的详细文档（控制器清单、关节顺序、控制命令、关键文件等）
> 见 `src/ylr1d_mid_control/README.md`。

## 快速开始

```bash
source /opt/ros/humble/setup.bash
cd ~/WorkSpace/test_ylr1d
colcon build
source install/setup.bash
ros2 launch ylr1d_mid_control gazebo.launch.py
```

**注意：** 需要 gazebo 的 Python 环境，且 `GAZEBO_MODEL_PATH` 需包含 src 目录。
