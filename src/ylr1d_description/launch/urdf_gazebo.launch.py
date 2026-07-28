import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "ylr1d_description"
    robot_name = "ylr1d"
    urdf_name = "ylr1d_gazebo.urdf"

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["GAZEBO_MODEL_DATABASE_URI"] = ""

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, "urdf", urdf_name)
    rviz_path = os.path.join(pkg_share, "rviz", "gazebo_display.rviz")

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        arguments=[urdf_path],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_path],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        env=env,
    )

    start_gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose",
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so"],
        output="screen",
        env=env,
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", robot_name, "-file", urdf_path],
        output="screen",
    )

    spawn_delayed = TimerAction(period=5.0, actions=[spawn_entity])

    ld = LaunchDescription()
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    ld.add_action(start_gazebo)
    ld.add_action(spawn_delayed)
    ld.add_action(rviz2)
    return ld
