import os
import re
import tempfile
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def _resolve_yaml_refs(content: str, config_dir: str) -> str:
    """Resolve ${links.X.Y}, ${colors.X}, ${limits.X.Y} from YAML config files.

    Replaces dotted expressions with actual values from config YAML files.
    Simple variable names like ${prefix} are left untouched for xacro.
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
        # Skip simple variable names (prefix, config_path) ¡ª let xacro handle them
        if re.match(r'^[a-zA-Z_]\w*$', expr):
            return match.group(0)
        # Resolve dotted expressions from config data
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
    xacro_name = "ylr1d.xacro"
    rviz_config = "display.rviz"

    env = os.environ.copy()
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"

    pkg_share = FindPackageShare(package=package_name).find(package_name)

    xacro_path = os.path.join(pkg_share, "urdf", xacro_name)
    rviz_path = os.path.join(pkg_share, "rviz", rviz_config)
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

    os.unlink(tmp.name)  # clean up

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"robot_description": robot_desc}],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        env=env,
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz2)
    return ld
