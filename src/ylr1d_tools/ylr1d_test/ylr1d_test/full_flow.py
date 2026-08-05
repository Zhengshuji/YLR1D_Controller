#!/usr/bin/env python3
"""Tier2 全流程测试 + 内置分层隔离诊断。

启动 test_stack.launch.py（plant + control_sim + translate），分层收集证据：
  E1  6 控制器 active        -> 否 => plant 层
  E2  /joint_states 有数据    -> 否 => plant/gazebo 层
  E3  3 action server 在线    -> 否 => translate 层
  E4  chassis goal 后 desired 有期望 -> 否 => translate 层
  E5  5 命令话题在发           -> 否 => control_sim 层
  E6  运动断言（转向/轮速）     -> 否且 E1-E5 全过 => 各层通路正常，物理/时序/环境层
返回 missing_layer，runner 据此自动补跑对应层测试。
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from ylr1d_translate.action import ChassisMove

from . import common
from .control_sim_smoke import STEERING, WHEELS

CMD_TOPICS = ["/chassis_steering_controller/commands",
              "/chassis_wheels_controller/commands",
              "/torso_controller/commands",
              "/left_arm_controller/commands",
              "/right_arm_controller/commands"]
EXPECT_SERVERS = {"chassis_move", "arm_move", "gripper_move"}

# ChassisMove 常量（见 ylr1d_translate constants.hpp）
MODE_TRANSLATE = 0
STEERING_TARGET = 0.0     # direction=0 -> 转向到 0
MOVE_SPEED = 2.0          # 平移速度（rad/s），经 control_sim 限幅到 max_vel 5.0
MOVE_DURATION = 5.0


def _val(msg, name, field):
    """从 JointState 按名取 position/velocity；不存在返回 None。"""
    if msg is None or name not in msg.name:
        return None
    i = msg.name.index(name)
    arr = getattr(msg, field)
    return arr[i] if i < len(arr) else None


class FlowProbe(Node):
    def __init__(self):
        super().__init__("flow_probe")
        self.js = []
        self.desired = None
        self.cmd_counts = {t: 0 for t in CMD_TOPICS}
        self.client = ActionClient(self, ChassisMove, "chassis_move")
        self.create_subscription(JointState, "/joint_states", self.js_cb, 10)
        self.create_subscription(JointState, "/desired_joint_states", self.des_cb, 10)
        for t in CMD_TOPICS:
            self.create_subscription(Float64MultiArray, t, self._mk_cmd(t), 10)

    def js_cb(self, msg):
        self.js.append(msg)
        if len(self.js) > 3000:
            self.js.pop(0)

    def des_cb(self, msg):
        if self.desired is None:
            self.desired = msg

    def _mk_cmd(self, topic):
        def cb(_m):
            self.cmd_counts[topic] += 1
        return cb

    def latest_js(self):
        return self.js[-1] if self.js else None


def run():
    """执行全流程测试，返回 (ok, detail_lines, missing_layer)。"""
    lines = []
    launch = None
    missing = None
    ok = False
    node = None
    try:
        common.log("启动 test_stack.launch.py ...")
        launch = common.start_process_group(
            ["ros2", "launch", "ylr1d_test", "test_stack.launch.py"])

        # E1 控制器 active
        common.log("[E1] 轮询 6 控制器 active (超时 120s)...")
        active = common.wait_for_controllers(expect=6, timeout=120)
        lines.append("%s E1 控制器 active: %d/6" % ("PASS" if active >= 6 else "FAIL", active))
        if active < 6:
            return False, lines, "plant"

        # 初始化 rclpy + 探针
        rclpy.init()
        node = FlowProbe()
        spin = rclpy.executors.SingleThreadedExecutor()
        spin.add_node(node)

        # E2 /joint_states 有数据
        common.log("[E2] 等待 /joint_states 数据 ...")
        end = time.time() + 25.0
        while time.time() < end and node.latest_js() is None:
            spin.spin_once(timeout_sec=0.2)
        n_joints = len(node.latest_js().name) if node.latest_js() else 0
        lines.append("%s E2 /joint_states 有数据 (%d 关节)" %
                     ("PASS" if n_joints >= 30 else "FAIL", n_joints))
        if n_joints < 30:
            return False, lines, "plant"

        # E3 action server 在线
        common.log("[E3] 轮询 action server 在线 (超时 90s)...")
        seen = common.wait_for_action_servers(EXPECT_SERVERS, timeout=90)
        miss = EXPECT_SERVERS - seen
        lines.append("%s E3 action server: 缺失 %s" %
                     ("PASS" if not miss else "FAIL", ",".join(sorted(miss)) or "-"))
        if miss:
            return False, lines, "translate"

        # 发送 chassis_move goal（平移，方向 0，速度 2，时长 5s）
        common.log("[E4] 发送 chassis_move goal ...")
        goal = ChassisMove.Goal()
        goal.mode = MODE_TRANSLATE
        goal.direction = STEERING_TARGET
        goal.speed = MOVE_SPEED
        goal.duration = MOVE_DURATION
        if not node.client.wait_for_server(timeout_sec=15.0):
            lines.append("FAIL E4 chassis_move server 连接超时")
            return False, lines, "translate"
        future = node.client.send_goal_async(goal)
        end = time.time() + 15.0
        while time.time() < end and not future.done():
            spin.spin_once(timeout_sec=0.1)
        if future.done():
            gh = future.result()
            if not (gh and gh.accepted):
                lines.append("FAIL E4 chassis_move goal 被拒绝")
                return False, lines, "translate"
        else:
            lines.append("FAIL E4 chassis_move goal 发送超时")
            return False, lines, "translate"
        lines.append("PASS E4 chassis_move goal ACCEPTED")

        # E4b desired 收到转向期望
        end = time.time() + 15.0
        got = None
        while time.time() < end and got is None:
            spin.spin_once(timeout_sec=0.1)
            got = _val(node.desired, STEERING[0], "position")
        good_des = got is not None and abs(got - STEERING_TARGET) < 0.01
        lines.append("%s E4 desired_joint_states 转向期望 %.3f (期望 %.1f)" %
                     ("PASS" if good_des else "FAIL", got if got is not None else float("nan"),
                      STEERING_TARGET))
        if not good_des:
            return False, lines, "translate"

        # E5 命令话题在发
        common.log("[E5] 采样 5 命令话题 8s ...")
        end = time.time() + 8.0
        while time.time() < end:
            spin.spin_once(timeout_sec=0.1)
        quiet = [t for t, c in node.cmd_counts.items() if c == 0]
        lines.append("%s E5 命令话题：%d/5 在发%s" %
                     ("PASS" if not quiet else "FAIL", len(CMD_TOPICS) - len(quiet),
                      "（静默: " + ",".join(quiet) + "）" if quiet else ""))
        if quiet:
            return False, lines, "control_sim"

        # E6 运动断言
        common.log("[E6] 采样运动 12s ...")
        first = node.latest_js()
        start_steer = _val(first, STEERING[0], "position")
        end = time.time() + 12.0
        while time.time() < end:
            spin.spin_once(timeout_sec=0.1)
        last = node.latest_js()
        end_steer = _val(last, STEERING[0], "position")
        wheel_vels = [_val(last, w, "velocity") for w in WHEELS]
        wheel_vels = [v for v in wheel_vels if v is not None]
        max_wheel = max(abs(v) for v in wheel_vels) if wheel_vels else 0.0

        steer_moved = False
        if start_steer is not None and end_steer is not None:
            steer_moved = (abs(end_steer) < abs(start_steer) - 0.05 or
                           abs(start_steer - end_steer) > 0.05)
        # 平移模式转向目标即 0：初始已在目标附近则无需转向变化，轮速响应即运动证据
        steer_at_target = start_steer is not None and abs(start_steer) < 0.05
        motion_ok = max_wheel > 0.3 and (steer_at_target or steer_moved)
        lines.append("%s E6 运动: 转向 %.3f -> %.3f, 最大轮速 %.2f%s" %
                     ("PASS" if motion_ok else "FAIL", start_steer or float("nan"),
                      end_steer or float("nan"), max_wheel,
                      "（转向已在目标，轮速响应即运动）" if steer_at_target else ""))
        if not motion_ok:
            # 各层通路已通，可能是物理/时序/环境层问题
            lines.append("提示：E1-E5 均通过，但物理层未观察到运动——检查 Gazebo 仿真是否推进"
                         "（/clock、碰撞、初始位置限位），或环境性能不足")
            return False, lines, None

        ok = True
        lines.append("PASS 全流程测试通过")
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        if launch is not None:
            common.kill_process_group(launch)
        common.cleanup_residual(verbose=False)
    return ok, lines, missing
