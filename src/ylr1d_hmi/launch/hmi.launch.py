import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """控制层 HMI：观测 /joint_states，向控制层发 /desired_joint_states。"""
    set_env = SetEnvironmentVariable(
        "LIBGL_ALWAYS_SOFTWARE", "1"
    )

    return LaunchDescription([
        set_env,
        Node(
            package="ylr1d_hmi",
            executable="ylr1d_hmi_control",
            name="ylr1d_hmi_control",
            output="screen",
        ),
    ])
