import os
import re
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# 公共 xacro → URDF 导入逻辑（随 ylr1d_description 安装）
sys.path.insert(0, os.path.join(get_package_share_directory("ylr1d_description"), "launch", "python_utils"))
from xacro_utils import process_xacro_to_urdf


def _inject_effort_interfaces(content: str) -> str:
    """Inject <command_interface name='effort'/> after each
    existing position/velocity command interface in the ros2_control block."""
    return re.sub(
        r'(<command_interface name="(?:position|velocity)"/>)',
        lambda m: m.group(1) + '\n      <command_interface name="effort"/>',
        content,
    )


def generate_launch_description():
    package_name = "ylr1d_plant"
    robot_name = "ylr1d"

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["GAZEBO_MODEL_DATABASE_URI"] = ""
    
    pkg_desc = get_package_share_directory("ylr1d_description")
    model_path = os.path.join(pkg_desc, "meshes")
    # Gazebo rewrites package://ylr1d_description URIs into model://ylr1d_description.
    # For that URI to resolve, GAZEBO_MODEL_PATH must contain a directory whose
    # child is ylr1d_description (here: the share dir holding the package).
    share_path = os.path.dirname(pkg_desc)
    env["GAZEBO_MODEL_PATH"] = (model_path + ":" + share_path + ":"
                                + env.get("GAZEBO_MODEL_PATH", ""))

    lib_paths = [p for p in ["/opt/ros/humble/lib"] if os.path.isdir(p)]
    existing_ld = env.get("LD_LIBRARY_PATH", "")
    if lib_paths:
        env["LD_LIBRARY_PATH"] = ":".join(lib_paths) + ":" + existing_ld if existing_ld else ":".join(lib_paths)

    pkg_share = get_package_share_directory(package_name)
    pkg_desc = get_package_share_directory("ylr1d_description")
    declare_world = DeclareLaunchArgument(
        "world", default_value="empty.world",
        description="World file name under ylr1d_description/worlds/")
    world_arg = LaunchConfiguration("world")
    world_path = PathJoinSubstitution([pkg_desc, "worlds", world_arg])

    # ── Read original xacro (from ylr1d_description) and inject effort interfaces ──
    robot_desc, urdf_tmp_path = process_xacro_to_urdf(
        os.path.join(pkg_desc, "urdf", "ylr1d.xacro"),
        os.path.join(pkg_desc, "config"),
        os.path.join(pkg_share, "config", "effort_controllers.yaml"),
        transforms=(_inject_effort_interfaces,),
    )

    # ── Nodes ──────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        remappings=[("joint_states", "/joint_states_filtered")],
        output="screen",
    )

    joint_state_filter = Node(
        package="ylr1d_plant",
        executable="joint_state_filter",
        output="screen",
    )

    start_gazebo = ExecuteProcess(
        cmd=[
            "gazebo", "--verbose",
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
            "-unpause",
        ],
        output="screen",
    )

    # ── Spawn effort controllers (after Gazebo is ready) ──────
    spawn_controllers = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            # === Effort controllers ===
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["chassis_steering_effort_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["chassis_wheels_effort_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["torso_effort_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_arm_effort_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_arm_effort_controller"],
                output="screen",
            ),
        ],
    )

    # RViz (config from ylr1d_description)
    rviz_path = os.path.join(pkg_desc, "rviz", "display.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        env=env,
    )

    # ── Assemble ──────────────────────────────────────────────
    ld = LaunchDescription()
    ld.add_action(declare_world)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_filter)
    ld.add_action(start_gazebo)
    ld.add_action(TimerAction(period=5.0, actions=[spawn_entity]))
    ld.add_action(spawn_controllers)
    ld.add_action(rviz2)
    return ld
