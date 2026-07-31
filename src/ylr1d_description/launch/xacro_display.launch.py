import os
import re
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro


def _resolve_yaml_refs(content: str, config_dir: str) -> str:
    """Resolve ${links.X.Y}, ${colors.X}, ${limits.X.Y} from config YAML files.

    The model xacro references values via dotted expressions like
    ${links.Link_Base.mass}. xacro does not load these from yaml by itself,
    so we pre-resolve them to literals here, then let xacro handle the rest.
    """
    configs = {}
    for name in ["links", "colors", "limits", "scale", "calibration", "dynamics"]:
        path = os.path.join(config_dir, f"{name}.yaml")
        if os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f)
            if data is not None:
                configs[name] = data

    def _resolve(match):
        expr = match.group(1).strip()
        # Leave simple variable names (prefix, ...) for xacro to handle
        if re.match(r'^[a-zA-Z_]\w*$', expr):
            return match.group(0)
        parts = expr.split(".")
        if parts[0] in configs:
            try:
                val = configs[parts[0]]
                for p in parts[1:]:
                    val = val[p]
                return str(val)
            except (KeyError, TypeError):
                pass
        return match.group(0)  # keep as-is if unresolvable

    return re.sub(r'\$\{([^}]+)\}', _resolve, content)


def generate_launch_description():
    package_name = "ylr1d_description"

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["GAZEBO_MODEL_DATABASE_URI"] = ""
    # Gazebo Classic needs /opt/ros/humble/lib in LD_LIBRARY_PATH to find
    # gazebo_ros plugin libraries (libgazebo_ros2_control.so, etc.)
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

    # ── Process xacro (same pipeline as ylr1d_mid_control) ─────
    config_dir = os.path.join(pkg_share, "config")
    xacro_path = os.path.join(pkg_share, "urdf", "ylr1d.xacro")
    with open(xacro_path) as f:
        raw = f.read()
    resolved = _resolve_yaml_refs(raw, config_dir)
    # Inject controllers.yaml path into the gazebo_ros2_control plugin
    resolved = resolved.replace("${controllers_yaml_path}",
                                os.path.join(config_dir, "controllers.yaml"))

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".xacro", delete=False)
    tmp.write(resolved)
    tmp.close()
    doc = xacro.process_file(tmp.name, mappings={"config_path": config_dir})
    robot_desc = doc.toxml()
    os.unlink(tmp.name)

    # Save final URDF to a temp file for Gazebo spawn
    urdf_tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
    urdf_tmp.write(robot_desc)
    urdf_tmp.close()

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
        arguments=["-entity", "ylr1d", "-file", urdf_tmp.name],
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
