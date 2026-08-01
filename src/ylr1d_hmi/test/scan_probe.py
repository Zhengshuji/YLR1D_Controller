#!/usr/bin/env python3
"""Probe one LaserScan message and summarize the reading distribution."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def summarize(topic: str):
    node = Node(f"probe_{topic.strip('/').replace('/', '_')}")
    got = []

    def cb(msg):
        got.append(msg)
        node.destroy_node()
        rclpy.shutdown()

    node.create_subscription(LaserScan, topic, cb, 10)
    spin_time = 0.0
    while rclpy.ok() and not got:
        rclpy.spin_once(node, timeout_sec=0.5)
        spin_time += 0.5
        if spin_time > 60.0:
            break
    if not got:
        print(f"{topic}: NO MESSAGE in 60s")
        return
    m = got[0]
    fin = [r for r in m.ranges if math.isfinite(r)]
    print(f"{topic}: angle [{m.angle_min:.3f},{m.angle_max:.3f}] "
          f"range [{m.range_min:.2f},{m.range_max:.2f}] "
          f"samples {len(m.ranges)}  finite {len(fin)}")
    if fin:
        fin.sort()
        print(f"  finite: min {fin[0]:.3f}  median {fin[len(fin)//2]:.3f}  max {fin[-1]:.3f}")
        # histogram buckets in meters
        buckets = [0] * 6
        for r in fin:
            b = min(5, int(r))  # 0-1,1-2,...,5+
            buckets[b] += 1
        print(f"  buckets(<1,1-2,2-3,3-4,4-5,5+): {buckets}")
        print(f"  few around 0.2-0.4: {sum(1 for r in fin if r < 0.5)}"
              f"  few around 0.5-2: {sum(1 for r in fin if 0.5 <= r < 2.0)}"
              f"  few >= 2: {sum(1 for r in fin if r >= 2.0)}")


def main():
    rclpy.init()
    summarize("/radar/scan")
    for t in ["/lf_ultrasonic/range", "/rf_ultrasonic/range",
              "/lb_ultrasonic/range", "/rb_ultrasonic/range"]:
        summarize(t)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
