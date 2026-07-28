import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "ylr1d_description"
    default_urdf = "ylr1d.urdf"
    default_rviz = "display.rviz"

    declare_urdf = DeclareLaunchArgument(
        "urdf_name", default_value=default_urdf,
        description="URDF file name (in urdf/ directory)"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz_config", default_value=default_rviz,
        description="RViz config file name (in rviz/ directory)"
    )

    urdf_name = LaunchConfiguration("urdf_name")
    rviz_config = LaunchConfiguration("rviz_config")

    urdf_path = PathJoinSubstitution([
        FindPackageShare(package_name), "urdf", urdf_name
    ])
    rviz_path = PathJoinSubstitution([
        FindPackageShare(package_name), "rviz", rviz_config
    ])

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_path],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
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

    return LaunchDescription([
        declare_urdf,
        declare_rviz,
        robot_state_publisher,
        joint_state_publisher,
        rviz2,
    ])
