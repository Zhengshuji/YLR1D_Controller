import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "ylr1d_description"

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["GAZEBO_MODEL_DATABASE_URI"] = ""
    lib_paths = [p for p in ["/opt/ros/humble/lib"] if os.path.isdir(p)]
    existing_ld = env.get("LD_LIBRARY_PATH", "")
    if lib_paths:
        env["LD_LIBRARY_PATH"] = ":".join(lib_paths) + ":" + existing_ld if existing_ld else ":".join(lib_paths)

    pkg_share = get_package_share_directory(package_name)
    # Gazebo converts package://ylr1d_description/meshes/*.STL into
    # model://ylr1d_description/meshes/*.STL. For that URI to resolve,
    # GAZEBO_MODEL_PATH must contain a directory whose child is
    # ylr1d_description (here: the share dir that holds this package).
    model_path = os.path.join(pkg_share, "meshes")
    share_path = os.path.dirname(pkg_share)
    env["GAZEBO_MODEL_PATH"] = (model_path + ":" + share_path + ":"
                                + env.get("GAZEBO_MODEL_PATH", ""))

    # ── Static URDF (pre-generated from ylr1d.xacro) ───────────
    urdf_path = os.path.join(pkg_share, "urdf", "ylr1d.urdf")
    with open(urdf_path) as f:
        robot_desc = f.read()

    world_path = os.path.join(pkg_share, "worlds", "empty.world")
    rviz_path = os.path.join(pkg_share, "rviz", "display.rviz")

    # ── Nodes ──────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    # Interactive joint control: drag sliders to move the model in RViz
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    start_gazebo = ExecuteProcess(
        cmd=["gzserver", "--verbose", world_path,
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so"],
        output="screen",
        env=env,
    )

    # GUI client that connects to the headless gzserver above
    start_gzclient = ExecuteProcess(
        cmd=["gzclient", "--verbose"],
        output="screen",
        env=env,
    )
    gzclient_delayed = TimerAction(period=5.0, actions=[start_gzclient])

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "ylr1d", "-file", urdf_path],
        output="screen",
    )
    spawn_delayed = TimerAction(period=5.0, actions=[spawn_entity])

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        env=env,
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(start_gazebo)
    ld.add_action(gzclient_delayed)
    ld.add_action(spawn_delayed)
    ld.add_action(rviz2)
    return ld
