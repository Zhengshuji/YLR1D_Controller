import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """仿真监视 HMI（独立于 bringup）。

    启动 rviz2 并预加载"YLR1D Monitor"面板（RobotModel 3D 视图 + 监视面板），
    用于监视仿真情况：控制器 / 节点 / 传感器状态、rosout 关键信息、关节分组。
    需在仿真（gazebo.launch.py 或 bringup）运行时另行启动本 launch。
    """
    set_env = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")

    pkg_share = get_package_share_directory("ylr1d_hmi")
    rviz_config = os.path.join(pkg_share, "rviz", "monitor.rviz")

    return LaunchDescription([
        set_env,
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
