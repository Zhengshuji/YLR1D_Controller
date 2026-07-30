import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # WSL software rendering for RViz2
    set_env = SetEnvironmentVariable(
        "LIBGL_ALWAYS_SOFTWARE", "1"
    )

    # RViz config path
    pkg_dir = get_package_share_directory("ylr1d_hmi")
    rviz_config = os.path.join(pkg_dir, "config", "hmi.rviz")
    if not os.path.exists(rviz_config):
        rviz_config = ""

    return LaunchDescription([
        set_env,
        Node(
            package="ylr1d_hmi",
            executable="ylr1d_hmi_rviz",
            name="ylr1d_hmi",
            output="screen",
            parameters=[{
                "rviz_config": rviz_config,
            }],
        ),
    ])
