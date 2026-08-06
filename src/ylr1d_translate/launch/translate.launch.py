from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动转译层 translate_server（chassis_move / arm_move / gripper_move）。

    依赖下层已运行：
        ros2 launch ylr1d_plant gazebo.launch.py
        ros2 launch ylr1d_control position_simulate.launch.py
    """
    return LaunchDescription([
        Node(
            package="ylr1d_translate",
            executable="translate_server",
            name="translate_server",
            output="screen",
        ),
    ])
