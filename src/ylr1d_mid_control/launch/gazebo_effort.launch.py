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


def _inject_effort_interfaces(content: str) -> str:
    """Inject <command_interface name='effort'/> after each
    existing position/velocity command interface in the ros2_control block."""
    return re.sub(
        r'(<command_interface name="(?:position|velocity)"/>)',
        lambda m: m.group(1) + '\n      <command_interface name="effort"/>',
        content,
    )


def _resolve_package_uris(content: str, pkg_name: str) -> str:
    """Replace package://<pkg_name>/ URIs with absolute file paths using ament_index."""
    from ament_index_python.packages import get_package_share_directory
    pkg_path = get_package_share_directory(pkg_name)
    return content.replace(f"package://{pkg_name}", pkg_path)


def generate_launch_description():
    package_name = "ylr1d_mid_control"
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
    world_path = os.path.join(pkg_desc, "worlds", "empty.world")

    # model:// URIs are not used by this robot — package:// URIs are
    # resolved to absolute paths via _resolve_package_uris below,
    # so GAZEBO_MODEL_PATH is left unset to avoid Gazebo scanning
    # unrelated share/ subdirectories (which triggers noisy
    # "Missing model.config" errors).

    # ── Read original xacro (from ylr1d_description) and inject effort interfaces ────
    original_xacro_path = os.path.join(pkg_desc, "urdf", "ylr1d.xacro")
    config_dir = os.path.join(pkg_desc, "config")
    effort_controllers_yaml_path = os.path.join(pkg_share, "config", "effort_controllers.yaml")

    with open(original_xacro_path) as f:
        raw = f.read()

    # Step 1: resolve YAML config references (${links.X.Y}, etc.)
    resolved = _resolve_yaml_refs(raw, config_dir)
    # Step 2: point controllers path to the effort controller config
    resolved = resolved.replace("${controllers_yaml_path}", effort_controllers_yaml_path)
    # Step 3: inject <command_interface name="effort"/> into ros2_control block
    resolved = _inject_effort_interfaces(resolved)
    # Step 4: resolve package:// URIs to absolute file paths so Gazebo can find meshes
    # resolved = _resolve_package_uris(resolved, "ylr1d_description")

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
        remappings=[("joint_states", "/joint_states_filtered")],
        output="screen",
    )

    joint_state_filter = Node(
        package="ylr1d_mid_control",
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
            "-entity", robot_name, "-file", urdf_tmp.name,
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
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_filter)
    ld.add_action(start_gazebo)
    ld.add_action(TimerAction(period=5.0, actions=[spawn_entity]))
    ld.add_action(spawn_controllers)
    ld.add_action(rviz2)
    return ld
