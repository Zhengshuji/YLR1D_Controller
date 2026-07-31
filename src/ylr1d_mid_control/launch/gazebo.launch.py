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

    # Load sensor configs from config/sensors/*.yaml
    sensors_dir = os.path.join(config_dir, "sensors")
    if os.path.isdir(sensors_dir):
        for fname in sorted(os.listdir(sensors_dir)):
            if fname.endswith(".yaml"):
                name = fname[:-5]  # strip .yaml → e.g. "rgb_camera"
                path = os.path.join(sensors_dir, fname)
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
    package_name = "ylr1d_mid_control"
    robot_name = "ylr1d"

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
    pkg_desc = get_package_share_directory("ylr1d_description")

    model_path = os.path.join(pkg_desc, "meshes")
    env["GAZEBO_MODEL_PATH"] = model_path + ":" + env.get("GAZEBO_MODEL_PATH", "")

    # ── Process xacro ──────────────────────────────────────────
    xacro_path = os.path.join(pkg_share, "urdf", "ylr1d_mid.xacro")
    config_dir = os.path.join(pkg_share, "config")
    controllers_yaml_path = os.path.join(pkg_share, "config", "controllers.yaml")

    with open(xacro_path) as f:
        raw = f.read()
    resolved = _resolve_yaml_refs(raw, config_dir)
    # Replace controller yaml path directly (xacro's ${} evaluation
    # doesn't receive arbitrary mappings, so we do it here)
    resolved = resolved.replace("${controllers_yaml_path}", controllers_yaml_path)

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".xacro", delete=False)
    tmp.write(resolved)
    tmp.close()

    doc = xacro.process_file(tmp.name, mappings={
        "config_path": config_dir,
    })
    robot_desc = doc.toxml()
    os.unlink(tmp.name)

    # Save final URDF to temp file for Gazebo
    urdf_tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".urdf", delete=False)
    urdf_tmp.write(robot_desc)
    urdf_tmp.close()

    # ── Nodes ──────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    # Gazebo server — no --ros-args (gazebo_ros2_control plugin reads
    # controllers.yaml via its <parameters> tag directly after patching
    # the CLI arg parsing bug).
    start_gazebo = ExecuteProcess(
        cmd=[
            "gzserver", "--verbose",
            #"gazebo", "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
        env=env,
    )

    # Spawn robot into Gazebo (this activates gazebo_ros2_control plugin)
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", robot_name, "-file", urdf_tmp.name, 
            "-x", "0", "-y", "0", "-z", "0.3",
        ],
        output="screen",
    )

    # ── Spawn controllers (after Gazebo is ready) ─────────────
    spawn_controllers = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["chassis_steering_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["chassis_wheels_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["torso_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_arm_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_arm_controller"],
                output="screen",
            ),
        ],
    )

    # RViz
    rviz_path = os.path.join(pkg_share, "rviz", "display.rviz")
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
    ld.add_action(robot_state_publisher)
    ld.add_action(start_gazebo)
    ld.add_action(TimerAction(period=5.0, actions=[spawn_entity]))
    ld.add_action(spawn_controllers)
    ld.add_action(rviz2)
    return ld
