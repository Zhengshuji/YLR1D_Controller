import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch chassis_simulate + arm_simulate alongside Gazebo.

    Usage:
        ros2 launch ylr1d_mid_control gazebo.launch.py
        ros2 launch ylr1d_position_simulate position_simulate.launch.py

    Then publish desired joint states:
        ros2 topic pub /desired_joint_states sensor_msgs/JointState ...
    """
    chassis_params = {
        "loop_hz": 100.0,
        "steering/kp": 4.0,
        "steering/ki": 0.0,
        "steering/kd": 0.2,
        "steering/max_accel": 50.0,
        "steering/max_vel": 3.0,
        "wheel/kp": 2.0,
        "wheel/ki": 0.0,
        "wheel/kd": 0.05,
        "wheel/max_accel": 20.0,
        "wheel/max_vel": 5.0,
    }

    arm_params = {
        "loop_hz": 100.0,
        "pid/kp": 4.0,
        "pid/ki": 0.0,
        "pid/kd": 0.2,
        "pid/max_accel": 50.0,
        "pid/max_vel": 3.0,
    }

    return LaunchDescription([
        Node(
            package="ylr1d_position_simulate",
            executable="chassis_simulate",
            name="chassis_simulate",
            output="screen",
            parameters=[chassis_params],
        ),
        Node(
            package="ylr1d_position_simulate",
            executable="arm_simulate",
            name="arm_simulate",
            output="screen",
            parameters=[arm_params],
        ),
    ])
