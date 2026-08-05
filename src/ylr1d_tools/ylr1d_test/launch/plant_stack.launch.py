import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# 公共 xacro → URDF 导入逻辑（随 ylr1d_description 安装）
sys.path.insert(0, os.path.join(get_package_share_directory("ylr1d_description"), "launch", "python_utils"))
from xacro_utils import process_xacro_to_urdf  # noqa: E402


def generate_launch_description():
    """测试用 plant 单层栈：Gazebo + ros2_control + spawner（无 rviz2）。

    复制 ylr1d_plant/gazebo.launch.py 结构，去掉 rviz2（无头测试避免其崩溃
    连锁杀掉 launch 进程组）；world 参数透传（默认 empty.world）。
    供 plant_test 单独测物理层，以及 test_stack 复用。
    """
    package_name = "ylr1d_plant"
    robot_name = "ylr1d"

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["GAZEBO_MODEL_DATABASE_URI"] = ""
    lib_paths = [p for p in ["/opt/ros/humble/lib"] if os.path.isdir(p)]
    existing_ld = env.get("LD_LIBRARY_PATH", "")
    if lib_paths:
        env["LD_LIBRARY_PATH"] = (":".join(lib_paths) + ":" + existing_ld
                                  if existing_ld else ":".join(lib_paths))

    pkg_share = get_package_share_directory(package_name)
    pkg_desc = get_package_share_directory("ylr1d_description")

    model_path = os.path.join(pkg_desc, "meshes")
    share_path = os.path.dirname(pkg_desc)
    env["GAZEBO_MODEL_PATH"] = (model_path + ":" + share_path + ":"
                                + env.get("GAZEBO_MODEL_PATH", ""))

    declare_world = DeclareLaunchArgument(
        "world", default_value="empty.world",
        description="World file name under ylr1d_description/worlds/")
    world_arg = LaunchConfiguration("world")
    world_path = PathJoinSubstitution([pkg_desc, "worlds", world_arg])

    robot_desc, urdf_tmp_path = process_xacro_to_urdf(
        os.path.join(pkg_desc, "urdf", "ylr1d.xacro"),
        os.path.join(pkg_desc, "config"),
        os.path.join(pkg_share, "config", "controllers.yaml"),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    start_gazebo = ExecuteProcess(
        cmd=[
            "gzserver", "--verbose",
            world_path,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
        env=env,
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", robot_name, "-file", urdf_tmp_path,
            "-x", "0", "-y", "0", "-z", "0.3",
        ],
        output="screen",
    )

    spawn_controllers = TimerAction(
        period=8.0,
        actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster"], output="screen"),
            Node(package="controller_manager", executable="spawner",
                 arguments=["chassis_steering_controller"], output="screen"),
            Node(package="controller_manager", executable="spawner",
                 arguments=["chassis_wheels_controller"], output="screen"),
            Node(package="controller_manager", executable="spawner",
                 arguments=["torso_controller"], output="screen"),
            Node(package="controller_manager", executable="spawner",
                 arguments=["left_arm_controller"], output="screen"),
            Node(package="controller_manager", executable="spawner",
                 arguments=["right_arm_controller"], output="screen"),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(robot_state_publisher)
    ld.add_action(start_gazebo)
    ld.add_action(TimerAction(period=5.0, actions=[spawn_entity]))
    ld.add_action(spawn_controllers)
    return ld
