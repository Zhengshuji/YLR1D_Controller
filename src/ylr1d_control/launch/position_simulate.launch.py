"""控制层（采样保持器 + 通信节点）launch。

链路：
    上层 /desired_joint_states → 控制层采样保持 → /ctrl/<组>/desired、feedback
    → 算法层仿真控制器（ylr1d_algorithm_sim sim_controller.launch.py，composition）
    → /ctrl/<组>/output → 控制层采样保持 → 5 组命令 → 物理层

pid.yaml 已移交算法层（由 sim_controller.launch.py 注入），控制层不再加载。

用法:
    ros2 launch ylr1d_control position_simulate.launch.py
    （算法层需先/同时启动：ros2 launch ylr1d_algorithm_sim sim_controller.launch.py）
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ylr1d_control",
            executable="chassis_control",
            name="chassis_control",
            output="screen",
            parameters=[{"loop_hz": 100.0}],
        ),
        Node(
            package="ylr1d_control",
            executable="arm_control",
            name="arm_control",
            output="screen",
            parameters=[{"loop_hz": 100.0}],
        ),
    ])
