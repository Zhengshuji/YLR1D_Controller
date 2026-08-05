#!/usr/bin/env python3
"""Tier2 translate 单层测试：需 plant + control_sim 先运行（test_stack）。

启动 test_stack.launch.py，断言：
  1. /chassis_move /arm_move /gripper_move 三个 action server 在线
  2. 发 gripper_move goal 被接受（ACCEPTED）
  3. /desired_joint_states 收到夹指目标（translate 把 action → 期望值的通路正常）
运动断言留给 full_flow；本测试聚焦 translate 层自身。
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ylr1d_translate.action import GripperMove

from . import common

EXPECT_SERVERS = {"chassis_move", "arm_move", "gripper_move"}
# 左夹爪 open -> 目标 -0.014（见 translate constants.hpp LEFT_FINGER_OPEN）
FINGER_JOINT = "Joint_LeftArm7_to_LeftFinger1"
GRIPPER_TARGET = -0.014


class TranslateProbe(Node):
    def __init__(self):
        super().__init__("translate_probe")
        self.desired = None
        self.client = ActionClient(self, GripperMove, "gripper_move")
        self.create_subscription(JointState, "/desired_joint_states", self.cb, 10)

    def cb(self, msg):
        if self.desired is None:
            self.desired = msg

    def get_finger_pos(self):
        if self.desired is None or FINGER_JOINT not in self.desired.name:
            return None
        i = self.desired.name.index(FINGER_JOINT)
        return self.desired.position[i]


def run():
    """执行 translate 单层测试，返回 (ok, detail_lines, missing_layer)。"""
    lines = []
    launch = None
    ok = False
    try:
        common.log("启动 test_stack.launch.py ...")
        launch = common.start_process_group(
            ["ros2", "launch", "ylr1d_test", "test_stack.launch.py"])

        # 1) action server 在线
        common.log("[1] 轮询 action server 在线 (超时 90s)...")
        seen = common.wait_for_action_servers(EXPECT_SERVERS, timeout=90)
        miss = EXPECT_SERVERS - seen
        lines.append("%s action server: 缺失 %s（已见 %s）" %
                     ("PASS" if not miss else "FAIL", ",".join(sorted(miss)) or "-",
                      ",".join(sorted(seen & EXPECT_SERVERS))))
        if miss:
            return False, lines, "translate"

        # 2) 发 gripper_move goal（左夹爪 open），断言 ACCEPTED
        rclpy.init()
        node = TranslateProbe()
        spin = rclpy.executors.SingleThreadedExecutor()
        spin.add_node(node)

        common.log("[2] 发送 gripper_move goal (part=left, open=true) ...")
        goal = GripperMove.Goal()
        goal.part = 0
        goal.open = True
        if not node.client.wait_for_server(timeout_sec=15.0):
            lines.append("FAIL gripper_move action server 连接超时")
            return False, lines, "translate"
        future = node.client.send_goal_async(goal)
        accepted = False
        end = time.time() + 15.0
        while time.time() < end and not future.done():
            spin.spin_once(timeout_sec=0.1)
        if future.done():
            gh = future.result()
            accepted = bool(gh and gh.accepted)
        lines.append("%s goal ACCEPTED" % ("PASS" if accepted else "FAIL"))
        if not accepted:
            return False, lines, "translate"

        # 3) /desired_joint_states 收到夹指目标
        common.log("[3] 等待 /desired_joint_states 收到夹指目标 ...")
        end = time.time() + 15.0
        got = None
        while time.time() < end and got is None:
            spin.spin_once(timeout_sec=0.1)
            got = node.get_finger_pos()
        if got is not None and abs(got - GRIPPER_TARGET) < 1e-3:
            lines.append("PASS desired_joint_states 夹指目标 %.4f" % got)
            ok = True
        else:
            lines.append("FAIL desired_joint_states 夹指目标（期望 %.4f，收到 %s）"
                         % (GRIPPER_TARGET, "None" if got is None else "%.4f" % got))
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if launch is not None:
            common.kill_process_group(launch)
        common.cleanup_residual(verbose=False)
    return ok, lines, ("translate" if not ok else None)
