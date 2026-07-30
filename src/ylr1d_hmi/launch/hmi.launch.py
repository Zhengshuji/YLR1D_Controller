import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # WSL 下需要软件渲染（但此 lite 版本无 RViz，仅保留以备后续扩展）
    set_env = SetEnvironmentVariable(
        "LIBGL_ALWAYS_SOFTWARE", "1"
    )

    return LaunchDescription([
        set_env,
        Node(
            package="ylr1d_hmi",
            executable="ylr1d_hmi",
            name="ylr1d_hmi",
            output="screen",
        ),
    ])
