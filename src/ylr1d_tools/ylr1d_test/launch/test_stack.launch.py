import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _launch_path(pkg, name):
    return os.path.join(get_package_share_directory(pkg), "launch", name)


def generate_launch_description():
    """全栈测试 launch：plant（无 rviz）+ control_sim + translate。

    用于 translate_test / full_flow / sensor_probe 分层测试。独立于 bringup
    （bringup 一行不改）；world 参数透传（默认 empty.world，sensor_probe 用
    sensors_test.world）。
    """
    declare_world = DeclareLaunchArgument(
        "world", default_value="empty.world",
        description="World file name under ylr1d_description/worlds/")
    world_arg = LaunchConfiguration("world")

    plant = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path("ylr1d_test", "plant_stack.launch.py")),
        launch_arguments={"world": world_arg}.items(),
    )
    algorithm_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path("ylr1d_algorithm_sim", "sim_controller.launch.py")))
    control_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path("ylr1d_control", "position_simulate.launch.py")))
    translate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path("ylr1d_translate", "translate.launch.py")))

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(plant)
    ld.add_action(algorithm_sim)
    ld.add_action(control_sim)
    ld.add_action(translate)
    return ld
