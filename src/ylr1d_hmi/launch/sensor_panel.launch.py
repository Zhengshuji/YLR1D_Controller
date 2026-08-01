import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """独立传感器可视化面板（与控制器 HMI 分开）。

    只订阅并展示各传感器话题（相机/点云/雷达/超声波/IMU），
    不包含任何关节控制逻辑。

    用法:
        ros2 launch ylr1d_hmi sensor_panel.launch.py
    """
    set_env = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")

    return LaunchDescription([
        set_env,
        Node(
            package="ylr1d_hmi",
            executable="ylr1d_sensor_panel",
            name="ylr1d_sensor_panel",
            output="screen",
        ),
    ])
