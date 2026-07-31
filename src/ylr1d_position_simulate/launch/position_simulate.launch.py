import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_yaml(path):
    """Load a yaml file into a dict (empty dict if missing/empty)."""
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f)
    return data if data is not None else {}


def _collect_params(limit_files, pid_file):
    """把 config 下的 limit/pid yaml 合并为节点参数。

    统一命名: <关节名>/limit/<参数名> 与 <关节名>/pid/<参数名>
    例如 Joint_Base_to_RFWheelF/limit/lower, Joint_Base_to_RFWheelF/pid/kp
    """
    params = {}
    for lf in limit_files:
        data = _load_yaml(lf)
        for joint, cfg in data.items():
            lim = cfg.get("limit", {})
            for k, v in lim.items():
                params[f"{joint}/limit/{k}"] = v
    data = _load_yaml(pid_file)
    for joint, cfg in data.items():
        pid = cfg.get("pid", {})
        for k, v in pid.items():
            params[f"{joint}/pid/{k}"] = v
    return params


def generate_launch_description():
    """Launch chassis_simulate + arm_simulate。

    用法:
        ros2 launch ylr1d_mid_control gazebo.launch.py
        ros2 launch ylr1d_position_simulate position_simulate.launch.py

    然后发布期望关节状态:
        ros2 topic pub /desired_joint_states sensor_msgs/JointState ...
    """
    pkg_share = get_package_share_directory("ylr1d_position_simulate")
    config_dir = os.path.join(pkg_share, "config")

    params = _collect_params(
        [os.path.join(config_dir, "position_control_limits.yaml"),
         os.path.join(config_dir, "velocity_control_limits.yaml")],
        os.path.join(config_dir, "pid.yaml"),
    )
    params["loop_hz"] = 100.0

    # 两个节点都接收全部关节参数；各自只声明/读取自己管理关节的参数。
    # chassis_simulate: 转向(4) + 轮子(4)；arm_simulate: 躯干(4) + 左臂(9) + 右臂(9)
    return LaunchDescription([
        Node(
            package="ylr1d_position_simulate",
            executable="chassis_simulate",
            name="chassis_simulate",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="ylr1d_position_simulate",
            executable="arm_simulate",
            name="arm_simulate",
            output="screen",
            parameters=[params],
        ),
    ])
