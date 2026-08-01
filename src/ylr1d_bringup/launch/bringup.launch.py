import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(package_name: str, launch_file: str):
    """Include <package>/launch/<launch_file> by share directory."""
    pkg_share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", launch_file)
        )
    )


def generate_launch_description():
    """一键启动 YLR1D 仿真栈。

    聚合：
        ylr1d_plant         gazebo.launch.py        物理层（Gazebo + 控制器 + rviz2）
        ylr1d_control_sim   position_simulate.launch.py   控制层软仿真
        ylr1d_hmi           hmi.launch.py           人机界面

    用法:
        ros2 launch ylr1d_bringup bringup.launch.py

    被包含的 launch 自带环境设置（LIBGL_ALWAYS_SOFTWARE / GAZEBO_MODEL_PATH /
    LD_LIBRARY_PATH）与 gazebo spawner 的 TimerAction 时序，无需在此重复处理。
    """
    return LaunchDescription([
        _include("ylr1d_plant", "gazebo.launch.py"),
        _include("ylr1d_control_sim", "position_simulate.launch.py"),
        _include("ylr1d_hmi", "hmi.launch.py"),
    ])
