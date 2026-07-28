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
    """Resolve ${links.X.Y}, ${colors.X}, ${limits.X.Y} from YAML config files."""
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
        return match.group(0)

    return re.sub(r'\$\{([^}]+)\}', _resolve, content)


def generate_launch_description():
    package_name = "ylr1d_description"
    robot_name = "ylr1d"
    xacro_file = "ylr1d_gazebo.xacro"

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["GAZEBO_MODEL_DATABASE_URI"] = ""

    pkg_share = get_package_share_directory(package_name)
    model_path = os.path.join(pkg_share, "meshes")
    env["GAZEBO_MODEL_PATH"] = model_path + ":" + env.get("GAZEBO_MODEL_PATH", "")

    xacro_path = os.path.join(pkg_share, "urdf", xacro_file)
    config_dir = os.path.join(pkg_share, "config")

    # Pre-process: resolve yaml refs (${links.X.Y} -> actual values)
    with open(xacro_path) as f:
        raw = f.read()
    resolved = _resolve_yaml_refs(raw, config_dir)

    # Write processed content to temp file, let xacro handle ${prefix}
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".xacro", delete=False)
    tmp.write(resolved)
    tmp.close()

    doc = xacro.process_file(tmp.name, mappings={"config_path": config_dir})
    robot_desc = doc.toxml()

    os.unlink(tmp.name)  # clean up xacro temp

    # Save final URDF to temp file for robot_state_publisher and Gazebo
    urdf_tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
    urdf_tmp.write(robot_desc)
    urdf_tmp.close()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_tmp.name],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        arguments=[urdf_tmp.name],
    )

    rviz_path = os.path.join(pkg_share, "rviz", "gazebo_display.rviz")

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
        arguments=["-entity", robot_name, "-file", urdf_tmp.name],
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
