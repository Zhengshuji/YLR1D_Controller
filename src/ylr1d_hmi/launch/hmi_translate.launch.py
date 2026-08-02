import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """传输层 HMI：向 ylr1d_translate 发送 action goal（/chassis_move /arm_move /gripper_move）。"""
    set_env = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")

    return LaunchDescription([
        set_env,
        Node(
            package="ylr1d_hmi",
            executable="ylr1d_hmi_translate",
            name="ylr1d_hmi_translate",
            output="screen",
        ),
    ])
