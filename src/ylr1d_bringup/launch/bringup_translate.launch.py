import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package_name: str, launch_file: str, launch_arguments=None):
    """Include <package>/launch/<launch_file> by share directory.

    launch_arguments must be a list of (name, value) tuples (as required by
    IncludeLaunchDescription) — a dict would be iterated as keys only.
    """
    pkg_share = get_package_share_directory(package_name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", launch_file)
        ),
        launch_arguments=list(launch_arguments or []),
    )


def generate_launch_description():
    """转译层完整栈（转译层 → 物理层）。

    聚合：
        ylr1d_plant           gazebo.launch.py           物理层（Gazebo + ros2_control）
        ylr1d_algorithm_sim   sim_controller.launch.py   算法层仿真控制器（composition）
        ylr1d_control         position_simulate.launch.py 控制层采样保持 + 通信
        ylr1d_translate       translate.launch.py        转译层（translate_server，3 个 action server）
        ylr1d_hmi             hmi_translate.launch.py    转译层 HMI（发 action goal）
        ylr1d_hmi             sensor_panel.launch.py     传感器观测面板

    链路：translate HMI → action goal → translate_server → /desired_joint_states
          → control 采样保持 → 算法层仿真控制器（/ctrl/<组>/*）→ control
          → 5 组命令 → plant(Gazebo)

    用法:
        ros2 launch ylr1d_bringup bringup_translate.launch.py [world:=<world 文件名>]

    参数:
        world: Gazebo 世界文件名（ylr1d_description/worlds/ 下），透传给
               ylr1d_plant gazebo.launch.py。默认 empty.world。

    被包含的 launch 自带环境设置（LIBGL_ALWAYS_SOFTWARE / GAZEBO_MODEL_PATH /
    LD_LIBRARY_PATH）与 gazebo spawner 的 TimerAction 时序，无需在此重复处理。
    """
    world_arg = LaunchConfiguration("world")
    declare_world = DeclareLaunchArgument(
        "world", default_value="empty.world",
        description="World file name passed to ylr1d_plant gazebo.launch.py")

    return LaunchDescription([
        declare_world,
        _include("ylr1d_plant", "gazebo.launch.py", [("world", world_arg)]),
        _include("ylr1d_algorithm_sim", "sim_controller.launch.py"),
        _include("ylr1d_control", "position_simulate.launch.py"),
        _include("ylr1d_translate", "translate.launch.py"),
        _include("ylr1d_hmi", "hmi_translate.launch.py"),
        _include("ylr1d_hmi", "sensor_panel.launch.py"),
    ])
