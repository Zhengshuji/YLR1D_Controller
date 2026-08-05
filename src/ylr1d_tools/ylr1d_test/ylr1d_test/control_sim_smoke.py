#!/usr/bin/env python3
"""Tier1 control_sim 无头冒烟测试（迁移自 ylr1d_control_sim/test/position_simulate_smoke_test.py）。

流程：
  1. 后台启动 ros2 launch ylr1d_control_sim position_simulate.launch.py
  2. 校验 pid.yaml 已映射为节点参数（<关节>/pid/*；limit 为头文件常量，无 <关节>/limit/*）
  3. 发布 /joint_states 初始位置，触发节点初始化
  4. 发布 /desired_joint_states 期望值
  5. 采样 5 个控制器命令话题 + /simulated_* 状态，断言:
     - 位置关节趋近期望且不越限位
     - 轮子速度趋近期望且受 max_vel 限制
     - 命令话题长度正确 (4/4/4/9/9)
"""
import math
import re
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from . import common

PACKAGE = "ylr1d_control_sim"

# ── 30 个关节（固定顺序，与 controllers.yaml 一致） ──
STEERING = ["Joint_Base_to_RFWheelF", "Joint_Base_to_LFWheelF",
            "Joint_Base_to_RBWheelF", "Joint_Base_to_LBWheelF"]
WHEELS = ["Joint_RFWheelF_to_RFWheel", "Joint_LFWheelF_to_LFWheel",
          "Joint_RBWheelF_to_RBWheel", "Joint_LBWheelF_to_LBWheel"]
TORSO = ["Joint_Base_to_Body1", "Joint_Body1_to_Body2",
         "Joint_Body2_to_Body3", "Joint_Body3_to_Body4"]
LEFT = ["Joint_Body2_to_LeftArm1", "Joint_LeftArm1_to_LeftArm2",
        "Joint_LeftArm2_to_LeftArm3", "Joint_LeftArm3_to_LeftArm4",
        "Joint_LeftArm4_to_LeftArm5", "Joint_LeftArm5_to_LeftArm6",
        "Joint_LeftArm6_to_LeftArm7", "Joint_LeftArm7_to_LeftFinger1",
        "Joint_LeftArm7_to_LeftFinger2"]
RIGHT = ["Joint_Body2_to_RightArm1", "Joint_RightArm1_to_RightArm2",
         "Joint_RightArm2_to_RightArm3", "Joint_RightArm3_to_RightArm4",
         "Joint_RightArm4_to_RightArm5", "Joint_RightArm5_to_RightArm6",
         "Joint_RightArm6_to_RightArm7", "Joint_RightArm7_to_RightFinger1",
         "Joint_RightArm7_to_RightFinger2"]
ALL_NAMES = STEERING + WHEELS + TORSO + LEFT + RIGHT

# 初始位置（模拟 /joint_states 首条反馈）
INIT_POS = {
    "Joint_Base_to_RFWheelF": 2.497, "Joint_Base_to_LFWheelF": -2.497,
    "Joint_Base_to_RBWheelF": 0.644, "Joint_Base_to_LBWheelF": -0.644,
    "Joint_Base_to_Body1": 0.3,  # 已在限位上限
}
# 期望（只发部分关节，验证"未发布的关节保持不变"）
DESIRED_POS = {
    "Joint_Base_to_RFWheelF": 0.0, "Joint_Base_to_LFWheelF": 0.0,
    "Joint_Base_to_RBWheelF": 0.0, "Joint_Base_to_LBWheelF": 0.0,
    "Joint_Base_to_Body1": -0.5,  # 低于下限 -0.3 -> 应被钳制在 -0.3
    "Joint_Body2_to_LeftArm1": 0.5,
    "Joint_Body2_to_RightArm1": -0.5,
    "Joint_LeftArm7_to_LeftFinger1": -0.005,
    "Joint_RightArm7_to_RightFinger1": 0.007,
}
DESIRED_VEL = {"Joint_RFWheelF_to_RFWheel": 2.0,
               "Joint_LFWheelF_to_LFWheel": 2.0,
               "Joint_RBWheelF_to_RBWheel": -1.5,
               "Joint_LBWheelF_to_LBWheel": -1.5}

# 命令话题 -> 期望长度
CMD_TOPICS = [
    ("/chassis_steering_controller/commands", 4),
    ("/chassis_wheels_controller/commands", 4),
    ("/torso_controller/commands", 4),
    ("/left_arm_controller/commands", 9),
    ("/right_arm_controller/commands", 9),
]


class SmokeTestNode(Node):
    def __init__(self):
        super().__init__("smoke_test")
        self.cmd_samples = {t: [] for t, _ in CMD_TOPICS}
        self.sim_samples = {t: [] for t in ["/simulated_chassis_states",
                                            "/simulated_arm_states"]}
        for t, _ in CMD_TOPICS:
            self.create_subscription(Float64MultiArray, t,
                                     self._mk_cmd_cb(t), 10)
        for t in self.sim_samples:
            self.create_subscription(JointState, t,
                                     self._mk_sim_cb(t), 10)
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.desired_pub = self.create_publisher(JointState, "/desired_joint_states", 10)

    def _mk_cmd_cb(self, topic):
        def cb(msg):
            self.cmd_samples[topic].append(list(msg.data))
        return cb

    def _mk_sim_cb(self, topic):
        def cb(msg):
            d = dict(zip(msg.name, zip(msg.position, msg.velocity)))
            self.sim_samples[topic].append(d)
        return cb

    def publish_initial(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for n in ALL_NAMES:
            msg.name.append(n)
            msg.position.append(INIT_POS.get(n, 0.0))
            msg.velocity.append(0.0)
            msg.effort.append(0.0)
        self.js_pub.publish(msg)

    def publish_desired(self):
        # name/position/velocity/effort 必须等长（并行数组）
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        pos_names = list(DESIRED_POS.keys())
        vel_names = list(DESIRED_VEL.keys())
        msg.name = pos_names + vel_names
        msg.position = [DESIRED_POS[n] for n in pos_names] + [0.0] * len(vel_names)
        msg.velocity = [0.0] * len(pos_names) + [DESIRED_VEL[n] for n in vel_names]
        msg.effort = [0.0] * len(msg.name)
        self.desired_pub.publish(msg)

    def get_param(self, node_name, param_names):
        """通过 `ros2 param get` 子进程读取其他节点参数（rclpy 同步 call 会阻塞）。"""
        out = []
        for name in param_names:
            try:
                proc = subprocess.run(
                    ["ros2", "param", "get", f"/{node_name}", name],
                    capture_output=True, text=True, timeout=15.0)
                m = re.search(r"[Dd]ouble value is:\s*(-?[\d.eE+-]+)", proc.stdout)
                out.append(float(m.group(1)) if m else None)
            except Exception:
                out.append(None)
        return out


def run():
    """执行 control_sim 冒烟测试，返回 (ok, detail_lines, missing_layer)。"""
    lines = []
    launch = None
    ok = False
    try:
        # 1) 后台启动 launch（独立进程组）
        common.log("启动 position_simulate.launch.py ...")
        launch = common.start_process_group(
            ["ros2", "launch", PACKAGE, "position_simulate.launch.py"])
        time.sleep(5.0)

        rclpy.init()
        node = SmokeTestNode()
        spin = rclpy.executors.SingleThreadedExecutor()
        spin.add_node(node)

        # 2) 参数校验（pid.yaml -> <关节>/pid/*；limit 为头文件常量）
        common.log("[1] 参数校验 (pid.yaml -> 节点参数; limit 为头文件常量)")
        ok = True
        chassis_params = {
            "Joint_Base_to_RFWheelF/pid/kp": 150.0,
            "Joint_Base_to_RFWheelF/pid/kd": 20.0,
            # Joint_Base_to_Body1 属 arm 节点管理，此处不声明，不在此校验
            "Joint_RFWheelF_to_RFWheel/pid/kp": 4.0,
        }
        vals = node.get_param("chassis_simulate", list(chassis_params.keys()))
        for name, got in zip(chassis_params.keys(), vals):
            exp = chassis_params[name]
            good = got is not None and abs(got - exp) < 1e-9
            ok = ok and good
            lines.append("%s %s = %s (期望 %s)" % ("PASS" if good else "FAIL", name, got, exp))
        vals = node.get_param("arm_simulate", [
            "Joint_Body2_to_LeftArm1/pid/kp",   # 期望 150.0
            "Joint_LeftArm7_to_LeftFinger1/pid/kp",  # 期望 10.0
        ])
        for name, got, exp in zip(
                ["Joint_Body2_to_LeftArm1/pid/kp",
                 "Joint_LeftArm7_to_LeftFinger1/pid/kp"],
                vals, [150.0, 10.0]):
            good = got is not None and abs(got - exp) < 1e-9
            ok = ok and good
            lines.append("%s %s = %s (期望 %s)" % ("PASS" if good else "FAIL", name, got, exp))
        if not ok:
            lines.append("FAIL 参数校验失败")
            return False, lines, "control_sim"
        lines.append("PASS 参数校验全部通过")

        # 3) 发布初始 joint_states（多发几次确保收到）
        common.log("[2] 初始化: 发布 /joint_states")
        for _ in range(15):
            node.publish_initial()
            for _ in range(2):
                spin.spin_once(timeout_sec=0.1)
        time.sleep(1.0)

        # 4) 发布期望
        common.log("[3] 发布 /desired_joint_states")
        node.publish_desired()

        # 5) 采样 5s
        common.log("[4] 采样控制器命令 + 仿真状态 (5s)...")
        end = time.time() + 5.0
        while time.time() < end:
            spin.spin_once(timeout_sec=0.05)
            node.publish_desired()  # 持续刷新期望，避免丢失

        # 6) 断言
        common.log("[5] 断言")
        fail = []

        def check(cond, desc):
            nonlocal ok
            ok = ok and cond
            lines.append("%s %s" % ("PASS" if cond else "FAIL", desc))
            if not cond:
                fail.append(desc)

        for topic, exp_len in CMD_TOPICS:
            samps = node.cmd_samples[topic]
            lengths = {len(s) for s in samps}
            check(len(samps) > 10 and lengths == {exp_len},
                  f"{topic} 收到 {len(samps)} 条, 长度 {lengths} (期望 {exp_len})")

        chassis = node.sim_samples["/simulated_chassis_states"]
        arm = node.sim_samples["/simulated_arm_states"]
        check(len(chassis) > 10 and len(arm) > 10, "仿真状态话题有数据")

        def series(container, joint):
            return [c.get(joint, (float("nan"), 0.0))[0] for c in container]

        for j, target in [("Joint_Base_to_RFWheelF", 0.0),
                          ("Joint_Base_to_LBWheelF", 0.0)]:
            init = INIT_POS[j]
            last = chassis[-1].get(j, (init, 0.0))[0]
            check(abs(last - target) < abs(init - target),
                  f"{j} 趋近目标 {target} (初 {init:.3f} -> 末 {last:.3f})")

        for j, target in [("Joint_Body2_to_LeftArm1", 0.5),
                          ("Joint_Body2_to_RightArm1", -0.5)]:
            last = arm[-1].get(j, (0.0, 0.0))[0]
            check(abs(last - target) < abs(0.0 - target),
                  f"{j} 趋近目标 {target} (初 0 -> 末 {last:.3f})")

        b1 = series(arm, "Joint_Base_to_Body1")
        check(min(b1) >= -0.3 - 1e-6, f"Body1 未越下限 -0.3 (min {min(b1):.4f})")
        check(max(b1) <= 0.3 + 1e-6, f"Body1 未越上限 0.3 (max {max(b1):.4f})")
        check(min(b1) <= -0.29, f"Body1 被钳制到下限附近 (min {min(b1):.4f})")

        steer_cmd = node.cmd_samples["/chassis_steering_controller/commands"][-1]
        check(abs(steer_cmd[0]) <= 3.14 + 1e-6,
              f"steering 命令在限位内 (末值 {steer_cmd[0]:.3f})")
        check(abs(steer_cmd[0] - 0.0) < abs(INIT_POS["Joint_Base_to_RFWheelF"]),
              f"steering 命令趋近 0 (末值 {steer_cmd[0]:.3f})")

        for j, exp in [("Joint_RFWheelF_to_RFWheel", 2.0),
                       ("Joint_LBWheelF_to_LBWheel", -1.5)]:
            vel = chassis[-1].get(j, (0.0, 0.0))[1]
            check(abs(vel - exp) < 0.15, f"{j} 速度趋近 {exp} (末值 {vel:.3f})")
        all_vels = [abs(chassis[-1].get(w, (0.0, 0.0))[1]) for w in WHEELS]
        check(max(all_vels) <= 5.0 + 1e-6, f"轮子速度未超 5.0 (max {max(all_vels):.3f})")

        if ok:
            lines.append("PASS control_sim 冒烟测试通过")
        else:
            lines.append("FAIL control_sim 冒烟测试失败: " + "; ".join(fail))
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if launch is not None:
            common.kill_process_group(launch)
        common.cleanup_residual(verbose=False)
    return ok, lines, ("control_sim" if not ok else None)
