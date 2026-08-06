"""算法层仿真控制器：composition 单进程，5 个"仿真控制器"组件（每组一个）。

每个仿真控制器 = 一组关节的「控制器 + 仿真」整体，固定线程框架：
    订阅 /ctrl/<组>/desired、/ctrl/<组>/feedback
    → 协同控制(P=1) → 独立控制(PID) → 仿真对象(整体 Eigen) → 发布 /ctrl/<组>/output

组（与 algorithm/config/joint_config.hpp 的 kJointGroups 一致）：
    steering(位置4) wheels(速度4) torso(位置4) left_arm(位置9) right_arm(位置9)

参数：config/pid.yaml 全量注入到每个组件（组件只 declare 自己组内关节的参数，
多余参数无害，保持 pid.yaml 单一来源）。group_name 决定装配分组与限位。

用法:
    ros2 launch ylr1d_algorithm_sim sim_controller.launch.py
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _load_yaml(path):
    """Load a yaml file into a dict (empty dict if missing/empty)."""
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f)
    return data if data is not None else {}


def _collect_pid_params(pid_file):
    """把 config/pid.yaml 合并为节点参数：<关节名>/pid/<参数名>。"""
    params = {}
    data = _load_yaml(pid_file)
    for joint, cfg in data.items():
        pid = cfg.get("pid", {})
        for k, v in pid.items():
            params[f"{joint}/pid/{k}"] = v
    return params


# 组名 → 具名仿真控制器节点（与控制层分组一一对应，位置/速度由组定义决定）
GROUPS = [
    ("steering", "ylr1d_algorithm_sim::SteeringSimNode"),
    ("wheels", "ylr1d_algorithm_sim::WheelSimNode"),
    ("torso", "ylr1d_algorithm_sim::TorsoSimNode"),
    ("left_arm", "ylr1d_algorithm_sim::LeftArmSimNode"),
    ("right_arm", "ylr1d_algorithm_sim::RightArmSimNode"),
]


def generate_launch_description():
    pkg_share = get_package_share_directory("ylr1d_algorithm_sim")
    pid_params = _collect_pid_params(os.path.join(pkg_share, "config", "pid.yaml"))

    composable_nodes = []
    for group, plugin in GROUPS:
        params = dict(pid_params)          # 全量 pid 参数（节点只取自己组内关节）
        params["group_name"] = group       # 装配分组 + 限位（头文件编译期单一来源）
        params["loop_hz"] = 100.0
        composable_nodes.append(ComposableNode(
            package="ylr1d_algorithm_sim",
            plugin=plugin,
            name=f"{group}_sim",
            parameters=[params],
        ))

    container = ComposableNodeContainer(
        name="sim_controller_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    return LaunchDescription([container])
