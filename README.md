# 简介
该项目用于实现ylr1d机器人的控制

# 快速开始（验证中层）

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
注意：
1. 需要注意是否包含gazebo运行所需要的python环境
2. 需要注意GAZEBO_MODEL_PATH是否包含目标文件夹src。否则会无法加载模型文件
3. 
