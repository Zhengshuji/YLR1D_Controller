#!/usr/bin/env python3
"""决定性测试：发布期望轮速并回显验证。
- 订阅 /desired_joint_states 确认自己发出的期望确实在话题上（值应=3.0）
- 订阅 /chassis_wheels_controller/commands 看模拟层响应
- 订阅 /joint_states 看 Gazebo 实际轮速"""
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

WHEELS = ["Joint_RFWheelF_to_RFWheel", "Joint_LFWheelF_to_LFWheel",
          "Joint_RBWheelF_to_RBWheel", "Joint_LBWheelF_to_LBWheel"]
DES = [2.0, 2.0, 2.0, 2.0]


class LoopTest(Node):
    def __init__(self):
        super().__init__("wheel_loop_test")
        self.cmd = None
        self.js = None
        self.des_echo = None
        self.pub = self.create_publisher(JointState, "/desired_joint_states", 10)
        self.create_subscription(Float64MultiArray, "/chassis_wheels_controller/commands",
                                 lambda m: setattr(self, "cmd", m), 10)
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: setattr(self, "js", m), 10)
        self.create_subscription(JointState, "/desired_joint_states",
                                 lambda m: setattr(self, "des_echo", m), 10)

    def send_desired(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        for w, v in zip(WHEELS, DES):
            m.name.append(w)
            m.position.append(0.0)
            m.velocity.append(v)
        self.pub.publish(m)

    def wheels(self, js, field):
        out = {}
        if js is not None:
            for w in WHEELS:
                if w in js.name:
                    i = js.name.index(w)
                    arr = getattr(js, field)
                    out[w] = arr[i] if i < len(arr) else float("nan")
        return out


def main():
    rclpy.init()
    t = LoopTest()
    t0 = time.time()
    last = -1.0
    print("t(s) | 期望echo | 模拟层cmd | Gazebo实际轮速")
    while time.time() - t0 < 10:
        t.send_desired()
        rclpy.spin_once(t, timeout_sec=0.05)
        now = time.time() - t0
        if now - last >= 1.0:
            last = now
            de = t.wheels(t.des_echo, "velocity")
            cmd = [round(x, 3) for x in t.cmd.data] if t.cmd is not None else ["--"] * 4
            act = t.wheels(t.js, "velocity")
            print(f"{now:4.1f} | {str([round(de.get(w,float('nan')),1) for w in WHEELS]):>16} | "
                  f"{str(cmd):>20} | "
                  f"{act.get(WHEELS[0],float('nan')):5.2f} {act.get(WHEELS[1],float('nan')):5.2f} "
                  f"{act.get(WHEELS[2],float('nan')):5.2f} {act.get(WHEELS[3],float('nan')):5.2f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
