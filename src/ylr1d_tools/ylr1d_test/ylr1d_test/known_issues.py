#!/usr/bin/env python3
"""失败输出 → 问题信息 + 解决方案 映射（面向用户）。

runner 在测试 FAIL 时，用每条规则的 match 正则扫描测试日志/摘要；
命中则打印「问题信息 + 解决方案建议」。规则覆盖 CLAUDE.md 已知弯路与
WSL / GUI / 构建环境问题。
"""
import re

KNOWN_ISSUES = [
    {
        "match": r"gzserver|Entity already exists|gzclient",
        "problem": "Gazebo 残留进程冲突（上次 gzserver/gzclient 未完全退出）。",
        "solution": "先清理：pkill -f gzserver; pkill -f gzclient，再重跑测试。",
    },
    {
        "match": r"not active|controllers? .*inactive|unconfigured|Failed to start controller",
        "problem": "控制器未加载为 active（WSL 下 Gazebo 控制器加载需 30-60s，或资源不足 / gzserver 残留）。",
        "solution": "用 ros2 control list_controllers 轮询至全部 active；先 pkill 清理残留；确认已 source install/setup.bash。",
    },
    {
        "match": r"Joint_Base_to_Body1",
        "problem": "Joint_Base_to_Body1 初始即在限位上限（+0.30 m），正向目标推不动。",
        "solution": "先发负向目标离开限位，再控制到期望值。",
    },
    {
        "match": r"is not defined|links' is not defined",
        "problem": "裸 xacro 命令不可用（${links.X.Y} 占位符需 yaml 预替换）。",
        "solution": "必须用 xacro_utils.process_xacro_to_urdf（见 ylr1d_description/launch/python_utils/xacro_utils.py），勿直接跑 xacro。",
    },
    {
        "match": r"TF_NAN|prismatic.*nan|NAN.*prismatic",
        "problem": "GazeboSystem spawn 阶段 prismatic 关节位置 NaN 导致 TF_NAN 刷屏。",
        "solution": "joint_state_filter 已把 NaN 置 0（gazebo_effort.launch.py 已集成）；position 方案如需可自行加过滤。",
    },
    {
        "match": r"offscreen|QProcess|QXcbConnection|Could not connect to display|Failed to load platform plugin",
        "problem": "GUI 无法启动（无 X server / offscreen 平台加载失败）。",
        "solution": "无头环境设 QT_QPA_PLATFORM=offscreen；有头环境确认 DISPLAY 与 WSLg（/mnt/wslg）正常。",
    },
    {
        "match": r"font|字符集|方框",
        "problem": "Qt 界面中文显示为方框（缺中文字体）。",
        "solution": "安装中文字体（如 fonts-noto-cjk），或在 WSL 端 apt 安装后重启应用。",
    },
    {
        "match": r"MSYS2|Argument.*conversion|path conversion",
        "problem": "从 Windows CLI 调 WSL 时路径被 Git Bash/MSYS2 转换。",
        "solution": "wsl.exe 调用前加 MSYS2_ARG_CONV_EXCL=\"*\"。",
    },
    {
        "match": r"LIBGL_ALWAYS_SOFTWARE|libGL error|GLEW",
        "problem": "WSL 下 GPU 渲染受限，Gazebo/RViz 可能报 GL 错误。",
        "solution": "设置 LIBGL_ALWAYS_SOFTWARE=1（各 launch 已带，测试栈也已设置）。",
    },
    {
        "match": r"param get.*block|同步 call.*阻塞",
        "problem": "rclpy 同步参数/服务调用在该 WSL 环境可能无限阻塞。",
        "solution": "用 ros2 param get / ros2 topic 等 CLI 子进程方式读取。",
    },
    {
        "match": r"spawner.*cannot|controller_manager.*not.*reachable|Could not connect to.*controller_manager",
        "problem": "spawner 在 Gazebo 完全加载前启动，连不上 controller_manager 服务。",
        "solution": "launch 用 TimerAction(period>=8.0) 延迟 spawner（现有 launch 已处理）。",
    },
    {
        "match": r"action server|send_goal|Goal.*rejected|no action",
        "problem": "translate action server 不在线。",
        "solution": "translate 依赖 plant + control_sim 已运行，先用 test_stack.launch.py 起全栈再测。",
    },
    {
        "match": r"Desired joint state|desired_joint_states|JointState.*length",
        "problem": "JointState name/position/velocity 数组不等长或字段缺失。",
        "solution": "name/position/velocity/effort 须等长且按索引对齐；未用字段填 0 占位。",
    },
]


def advice_for(text):
    """扫描文本，返回命中的问题-解决方案列表（去重）。"""
    hits = []
    for item in KNOWN_ISSUES:
        if re.search(item["match"], text, re.IGNORECASE):
            hits.append((item["problem"], item["solution"]))
    return hits
