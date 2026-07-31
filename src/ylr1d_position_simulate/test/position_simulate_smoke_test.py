#!/usr/bin/env python3
"""ylr1d_position_simulate 无头冒烟测试（无需 Gazebo）。

流程:
  1. 后台启动 ros2 launch ylr1d_position_simulate position_simulate.launch.py
  2. 校验 yaml 配置已映射为节点参数 (<关节>/limit/*、<关节>/pid/*)
  3. 发布 /joint_states 初始位置，触发节点初始化
  4. 发布 /desired_joint_states 期望值
  5. 采样 5 个控制器命令话题 + /simulated_* 状态，断言:
     - 位置关节趋近期望且不越限位
     - 轮子速度趋近期望且受 max_vel 限制
     - 命令话题长度正确 (4/4/4/9/9)

用法（WSL）:
  source /home/zsj/WorkSpace/test_ylr1d/install/setup.bash
  python3 src/ylr1d_position_simulate/test/position_simulate_smoke_test.py
"""
import math
import os
import re
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

WORKSPACE = "/home/zsj/WorkSpace/test_ylr1d"
PACKAGE = "ylr1d_position_simulate"

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
        # name/position/velocity/effort 必须等长（并行数组），否则节点按
        # name 索引取 velocity 会越界，轮子期望速度将不被设置。
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
        """通过 `ros2 param get` 子进程读取其他节点参数。

        rclpy 同步 call() 在该 WSL 环境会无限阻塞，改用 CLI 更可靠。
        """
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


def main():
    # 1) 后台启动 launch
    log_path = "/tmp/position_simulate_launch.log"
    log = open(log_path, "w")
    launch = subprocess.Popen(
        ["ros2", "launch", PACKAGE, "position_simulate.launch.py"],
        stdout=log, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,  # 独立进程组，便于清理
    )
    try:
        time.sleep(5.0)

        rclpy.init()
        node = SmokeTestNode()
        spin = rclpy.executors.SingleThreadedExecutor()
        spin.add_node(node)

        # 2) 参数校验（yaml -> <关节>/limit/*、<关节>/pid/*）
        print("\n[1] 参数校验 (yaml -> 节点参数)")
        ok = True
        chassis_params = {
            "Joint_Base_to_RFWheelF/limit/lower": -3.14,
            "Joint_Base_to_RFWheelF/limit/upper": 3.14,
            "Joint_Base_to_RFWheelF/limit/velocity": 3.0,
            "Joint_Base_to_RFWheelF/limit/accelerate": 5.0,
            "Joint_Base_to_RFWheelF/pid/kp": 4.0,
            "Joint_Base_to_RFWheelF/pid/kd": 0.2,
            # Joint_Base_to_Body1 属 arm 节点管理，此处不声明，不在此校验
            "Joint_RFWheelF_to_RFWheel/limit/accelerate": 10.0,
            "Joint_RFWheelF_to_RFWheel/pid/kp": 2.0,
        }
        vals = node.get_param("chassis_simulate", list(chassis_params.keys()))
        for name, got in zip(chassis_params.keys(), vals):
            exp = chassis_params[name]
            good = got is not None and abs(got - exp) < 1e-9
            ok = ok and good
            print(f"  {'PASS' if good else 'FAIL'} {name} = {got} (期望 {exp})")
        # 臂节点读其关节的 pid（夹爪 kp=2.0 与普通臂 4.0 不同 -> 证明逐关节独立）
        vals = node.get_param("arm_simulate", [
            "Joint_Body2_to_LeftArm1/pid/kp",   # 期望 4.0
            "Joint_LeftArm7_to_LeftFinger1/pid/kp",  # 期望 2.0
            "Joint_Base_to_Body1/limit/velocity",    # 期望 0.6782
        ])
        for name, got, exp in zip(
                ["Joint_Body2_to_LeftArm1/pid/kp",
                 "Joint_LeftArm7_to_LeftFinger1/pid/kp",
                 "Joint_Base_to_Body1/limit/velocity"],
                vals, [4.0, 2.0, 0.6782]):
            good = got is not None and abs(got - exp) < 1e-9
            ok = ok and good
            print(f"  {'PASS' if good else 'FAIL'} {name} = {got} (期望 {exp})")
        if not ok:
            raise AssertionError("参数校验失败")
        print("  [OK] 参数校验全部通过")

        # 3) 发布初始 joint_states（多发几次确保收到）
        print("\n[2] 初始化: 发布 /joint_states")
        for _ in range(15):
            node.publish_initial()
            for _ in range(2):
                spin.spin_once(timeout_sec=0.1)
        time.sleep(1.0)

        # 4) 发布期望
        print("[3] 发布 /desired_joint_states")
        node.publish_desired()

        # 5) 采样 5s
        print("[4] 采样控制器命令 + 仿真状态 (5s)...")
        end = time.time() + 5.0
        while time.time() < end:
            spin.spin_once(timeout_sec=0.05)
            node.publish_desired()  # 持续刷新期望，避免丢失

        # 6) 断言
        print("[5] 断言")
        ok = True
        fail = []

        def check(cond, desc):
            nonlocal ok
            ok = ok and cond
            print(f"  {'PASS' if cond else 'FAIL'} {desc}")
            if not cond:
                fail.append(desc)

        # 命令话题长度
        for topic, exp_len in CMD_TOPICS:
            samps = node.cmd_samples[topic]
            lengths = {len(s) for s in samps}
            check(len(samps) > 10 and lengths == {exp_len},
                  f"{topic} 收到 {len(samps)} 条, 长度 {lengths} (期望 {exp_len})")

        # 位置趋近期望
        chassis = node.sim_samples["/simulated_chassis_states"]
        arm = node.sim_samples["/simulated_arm_states"]
        check(len(chassis) > 10 and len(arm) > 10, "仿真状态话题有数据")

        def series(container, joint):
            return [c.get(joint, (float("nan"), 0.0))[0] for c in container]

        # 位置关节: 末值比初始更接近目标（欠阻尼 PD 会围绕目标振荡，故不断言收敛）
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

        # Body1: 目标 -0.5 低于下限 -0.3 -> 钳制在下限，且全程不越上/下限
        b1 = series(arm, "Joint_Base_to_Body1")
        check(min(b1) >= -0.3 - 1e-6, f"Body1 未越下限 -0.3 (min {min(b1):.4f})")
        check(max(b1) <= 0.3 + 1e-6, f"Body1 未越上限 0.3 (max {max(b1):.4f})")
        check(min(b1) <= -0.29, f"Body1 被钳制到下限附近 (min {min(b1):.4f})")

        # 转向命令输出: 在限位内且比初始更接近目标（命令 = 仿真位置）
        steer_cmd = node.cmd_samples["/chassis_steering_controller/commands"][-1]
        check(abs(steer_cmd[0]) <= 3.14 + 1e-6,
              f"steering 命令在限位内 (末值 {steer_cmd[0]:.3f})")
        check(abs(steer_cmd[0] - 0.0) < abs(INIT_POS["Joint_Base_to_RFWheelF"]),
              f"steering 命令趋近 0 (末值 {steer_cmd[0]:.3f})")

        # 轮子速度: 速度环为一阶，5s 内应收敛，且受 max_vel=5 限制
        for j, exp in [("Joint_RFWheelF_to_RFWheel", 2.0),
                       ("Joint_LBWheelF_to_LBWheel", -1.5)]:
            vel = chassis[-1].get(j, (0.0, 0.0))[1]
            check(abs(vel - exp) < 0.15, f"{j} 速度趋近 {exp} (末值 {vel:.3f})")
        all_vels = [abs(chassis[-1].get(w, (0.0, 0.0))[1]) for w in WHEELS]
        check(max(all_vels) <= 5.0 + 1e-6, f"轮子速度未超 5.0 (max {max(all_vels):.3f})")

        print()
        if ok:
            print("SMOKE TEST PASSED")
        else:
            print("SMOKE TEST FAILED:", fail)
            sys.exit(1)
    finally:
        rclpy.shutdown()
        # 清理 launch 及其子进程
        try:
            os.killpg(os.getpgid(launch.pid), signal.SIGTERM)
        except Exception:
            pass
        time.sleep(1.0)
        subprocess.run(["pkill", "-f", "chassis_simulate"], capture_output=True)
        subprocess.run(["pkill", "-f", "arm_simulate"], capture_output=True)
        log.close()
        print(f"\nlaunch 日志: {log_path}")


if __name__ == "__main__":
    main()
