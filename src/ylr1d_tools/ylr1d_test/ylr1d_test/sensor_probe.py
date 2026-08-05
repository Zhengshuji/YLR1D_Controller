#!/usr/bin/env python3
"""Tier2 传感器动态探测（可选，默认跳过）：迁移自 ylr1d_hmi/test/scan_probe.py。

启动 plant_stack 于 sensors_test.world，探测 /radar/scan + 4 超声 /range
（均 sensor_msgs/LaserScan）。收到消息 → PASS；60s 无消息 → FAIL
（提示需传感器世界 / GUI）。
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from . import common

SCAN_TOPICS = ["/radar/scan",
               "/lf_ultrasonic/range", "/rf_ultrasonic/range",
               "/lb_ultrasonic/range", "/rb_ultrasonic/range"]
PROBE_TIMEOUT = 60.0


class ScanProbe(Node):
    def __init__(self):
        super().__init__("sensor_probe")
        self.got = set()
        for t in SCAN_TOPICS:
            self.create_subscription(LaserScan, t, self._mk_cb(t), 10)

    def _mk_cb(self, topic):
        def cb(_msg):
            self.got.add(topic)
        return cb

    def done(self):
        return len(self.got) >= len(SCAN_TOPICS)


def run():
    """执行传感器探测，返回 (ok, detail_lines, missing_layer)。"""
    lines = []
    launch = None
    ok = False
    try:
        common.log("启动 plant_stack.launch.py (world=sensors_test.world) ...")
        launch = common.start_process_group(
            ["ros2", "launch", "ylr1d_test", "plant_stack.launch.py",
             "world:=sensors_test.world"])

        # 先等机器人 spawn + 控制器加载，确保传感器插件就位
        common.log("[1] 等待控制器 active（确保模型已 spawn）...")
        active = common.wait_for_controllers(expect=6, timeout=120)
        lines.append("%s 控制器 active: %d/6" % ("PASS" if active >= 6 else "FAIL", active))
        if active < 6:
            return False, lines, "plant"

        # 探测传感器话题
        common.log("[2] 探测 5 个传感器话题 (超时 %.0fs)..." % PROBE_TIMEOUT)
        rclpy.init()
        node = ScanProbe()
        spin = rclpy.executors.SingleThreadedExecutor()
        spin.add_node(node)
        end = time.time() + PROBE_TIMEOUT
        while time.time() < end and not node.done():
            spin.spin_once(timeout_sec=0.2)
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        missing = [t for t in SCAN_TOPICS if t not in node.got]
        ok = not missing
        lines.append("%s 传感器消息：收到 %d/%d %s" %
                     ("PASS" if ok else "FAIL", len(node.got), len(SCAN_TOPICS),
                      ("全部收到" if ok else "缺失: " + ",".join(missing))))
        if not ok:
            lines.append("提示：需 sensors_test.world + Gazebo GUI/传感器世界；无消息可能是传感器未使能")
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if launch is not None:
            common.kill_process_group(launch)
        common.cleanup_residual(verbose=False)
    return ok, lines, ("sensor" if not ok else None)
