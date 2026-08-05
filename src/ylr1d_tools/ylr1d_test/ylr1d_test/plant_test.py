#!/usr/bin/env python3
"""Tier2 plant 单层测试：需 Gazebo。

启动 plant_stack.launch.py（无 rviz 的物理层栈），断言：
  1. 6 个控制器全部 active（WSL 加载慢，超时 120s）
  2. /joint_states 有数据（30 关节）
  3. gzserver 存活未崩
失败提示参考 CLAUDE 坑 2（gzserver 残留）/ 坑 3（控制器加载慢）。
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from . import common

MIN_JOINTS = 30


class JsProbe(Node):
    def __init__(self):
        super().__init__("plant_probe")
        self.js = None
        self.create_subscription(JointState, "/joint_states", self.cb, 10)

    def cb(self, msg):
        if self.js is None:
            self.js = msg


def run():
    """执行 plant 单层测试，返回 (ok, detail_lines, missing_layer)。"""
    lines = []
    launch = None
    ok = False
    try:
        common.log("启动 plant_stack.launch.py ...")
        launch = common.start_process_group(
            ["ros2", "launch", "ylr1d_test", "plant_stack.launch.py"])

        # 1) 控制器 active
        common.log("[1] 轮询 6 控制器 active (超时 120s)...")
        active = common.wait_for_controllers(expect=6, timeout=120)
        lines.append("%s 控制器 active: %d/6" % ("PASS" if active >= 6 else "FAIL", active))
        if active < 6:
            return False, lines, "plant"

        # 2) /joint_states 有数据
        rclpy.init()
        node = JsProbe()
        spin = rclpy.executors.SingleThreadedExecutor()
        spin.add_node(node)
        end = time.time() + 20
        while time.time() < end and node.js is None:
            spin.spin_once(timeout_sec=0.2)
        if node.js is not None:
            n = len(node.js.name)
            lines.append("%s /joint_states 有数据 (%d 关节)" % ("PASS" if n >= MIN_JOINTS else "FAIL", n))
        else:
            lines.append("FAIL /joint_states 20s 内无消息")
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        # 3) gzserver 存活
        rc, out, _ = common.run_cmd(["pgrep", "-x", "gzserver"], timeout=15)
        gz_alive = rc == 0 and out.strip()
        lines.append("%s gzserver 存活" % ("PASS" if gz_alive else "FAIL"))

        ok = (active >= 6 and node.js is not None and len(node.js.name) >= MIN_JOINTS
              and gz_alive)
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if launch is not None:
            common.kill_process_group(launch)
        common.cleanup_residual(verbose=False)
    return ok, lines, ("plant" if not ok else None)
