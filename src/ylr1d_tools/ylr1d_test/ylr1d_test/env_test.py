#!/usr/bin/env python3
"""Tier0 环境测试：无 ROS 依赖，秒级，逐项 PASS/FAIL/WARN。

检查项：
  1. 工具链    ros2 / colcon / gazebo / xacro（xacro 为空是正常，需经 xacro_utils）
  2. Python 依赖 rclpy / yaml / ament_index_python / launch / launch_ros
  3. 工作空间   install/setup.bash、7 包 install/share 齐全、各包 launch/config 存在
  4. 配置完整性 description limits.yaml、control joint_config.hpp + pid.yaml、plant controllers.yaml
  5. 残留进程   gzserver/gzclient 有 → WARN
  6. WSL 环境   /proc/version 含 microsoft、DISPLAY、/mnt/wslg
  7. GAZEBO_MODEL_PATH 是否含 src
  8. 软渲染     LIBGL_ALWAYS_SOFTWARE 未设置 → WARN
"""
import os
import sys

from . import common

PACKAGES = ["ylr1d_description", "ylr1d_plant", "ylr1d_control",
            "ylr1d_algorithm_sim", "ylr1d_translate", "ylr1d_hmi", "ylr1d_bringup"]

# 各包期望存在的 launch / config 文件（相对 src/<pkg>/）
PACKAGE_FILES = {
    "ylr1d_description": ["launch", "urdf/ylr1d.xacro", "config/limits.yaml",
                          "config/sensors.yaml", "worlds/empty.world"],
    "ylr1d_plant": ["launch/gazebo.launch.py", "config/controllers.yaml"],
    "ylr1d_control": ["launch/position_simulate.launch.py"],
    "ylr1d_algorithm_sim": ["launch/sim_controller.launch.py",
                            "config/pid.yaml",
                            "include/algorithm/config/joint_config.hpp",
                            "controller/include/controller/controller.hpp",
                            "plant/include/plant/plant.hpp"],
    "ylr1d_translate": ["launch/translate.launch.py"],
    "ylr1d_hmi": ["launch/hmi.launch.py"],
    "ylr1d_bringup": ["launch/bringup_control.launch.py",
                      "launch/bringup_translate.launch.py"],
}

CONFIG_CHECKS = [
    "src/ylr1d_description/config/limits.yaml",
    "src/ylr1d_algorithm_sim/include/algorithm/config/joint_config.hpp",
    "src/ylr1d_algorithm_sim/config/pid.yaml",
    "src/ylr1d_plant/config/controllers.yaml",
]


def _which(tool):
    rc, out, _ = common.run_cmd(["which", tool], timeout=15)
    return (rc == 0 and out.strip()) or None


def _import_ok(mod):
    try:
        __import__(mod)
        return True
    except ImportError:
        return False


def run():
    """执行环境检查，返回 (ok, detail_lines, None)。"""
    ws = common.find_ws_root()
    lines = []
    fails = []

    def check(name, ok, warn=False, note=""):
        status = "FAIL" if not ok else ("WARN" if warn else "PASS")
        lines.append("%-4s %-22s %s" % (status, name, note))
        if not ok and not warn:
            fails.append(name)

    # 1) 工具链
    ros2 = _which("ros2")
    colcon = _which("colcon")
    gazebo = _which("gzserver") or _which("gazebo")
    xacro = _which("xacro")
    check("ros2", bool(ros2), note=ros2 or "未找到 ros2（需 source install/setup.bash）")
    check("colcon", bool(colcon), note=colcon or "未找到 colcon")
    check("gazebo", bool(gazebo), note=gazebo or "未找到 gzserver/gazebo")
    check("xacro", True, warn=True,
          note="裸 xacro 不可用属正常（须经 xacro_utils）" if not xacro
          else "检测到 xacro（部分可用，正式用 xacro_utils）")

    # 2) Python 依赖
    for mod in ["rclpy", "yaml", "ament_index_python", "launch", "launch_ros"]:
        check("py:" + mod, _import_ok(mod), note="import %s" % mod)

    # 3) 工作空间
    check("ws_root", bool(ws), note=ws or "未定位到工作空间根（需在 WS 下运行）")
    if ws:
        check("install/setup.bash",
              os.path.isfile(os.path.join(ws, "install", "setup.bash")),
              note="install/setup.bash" + (" 存在" if os.path.isfile(os.path.join(ws, "install", "setup.bash")) else " 缺失（未构建）"))
        def _installed(p):
            # colcon 布局两种可能：install/<pkg>/share/<pkg> 或 install/share/<pkg>
            return (os.path.isdir(os.path.join(ws, "install", p, "share", p)) or
                    os.path.isdir(os.path.join(ws, "install", "share", p)))
        missing_pkgs = [p for p in PACKAGES if not _installed(p)]
        check("7 包已安装", not missing_pkgs,
              note=("齐全" if not missing_pkgs else "缺失: " + ",".join(missing_pkgs)))
        src_missing = {}
        for p, rels in PACKAGE_FILES.items():
            bad = [r for r in rels if not os.path.exists(os.path.join(ws, "src", p, r))]
            if bad:
                src_missing[p] = bad
        check("各包 launch/config", not src_missing,
              note=("齐全" if not src_missing else
                    "; ".join("%s:%s" % (p, ",".join(b)) for p, b in src_missing.items())))

    # 4) 配置完整性
    if ws:
        miss = [c for c in CONFIG_CHECKS if not os.path.exists(os.path.join(ws, c))]
        check("配置完整性", not miss,
              note=("齐全" if not miss else "缺失: " + ",".join(miss)))

    # 5) 残留进程
    rc, out, _ = common.run_cmd(["pgrep", "-x", "gzserver"], timeout=15)
    gzserver_pid = rc == 0 and out.strip()
    rc, out, _ = common.run_cmd(["pgrep", "-x", "gzclient"], timeout=15)
    gzclient_pid = rc == 0 and out.strip()
    if gzserver_pid or gzclient_pid:
        check("残留进程", True, warn=True,
              note="gzserver=%s gzclient=%s（建议 pkill 后再测）" % (gzserver_pid, gzclient_pid))
    else:
        check("残留进程", True, note="无 gzserver/gzclient 残留")

    # 6) WSL 环境
    is_wsl = False
    try:
        with open("/proc/version") as f:
            is_wsl = "microsoft" in f.read().lower()
    except OSError:
        pass
    check("WSL", is_wsl, warn=not is_wsl,
          note="检测到 WSL" if is_wsl else "非 WSL 或无法读取 /proc/version")
    display = os.environ.get("DISPLAY", "")
    wslg = os.path.isdir("/mnt/wslg")
    if not display and not wslg:
        check("X server", True, warn=True,
              note="DISPLAY 未设置且无 /mnt/wslg（GUI 相关测试需 X server/offscreen）")
    else:
        check("X server", True, note="DISPLAY=%s wslg=%s" % (display or "-", wslg))

    # 7) GAZEBO_MODEL_PATH
    gmp = os.environ.get("GAZEBO_MODEL_PATH", "")
    if ws and gmp and ("src" in gmp or ws in gmp):
        check("GAZEBO_MODEL_PATH", True, note="已含 WS/src 资产路径")
    else:
        check("GAZEBO_MODEL_PATH", True, warn=True,
              note="未含 src 资产路径（Gazebo 可能找不到模型），可 export GAZEBO_MODEL_PATH=$PWD/src")

    # 8) 软渲染
    if os.environ.get("LIBGL_ALWAYS_SOFTWARE"):
        check("软渲染", True, note="LIBGL_ALWAYS_SOFTWARE=1")
    else:
        check("软渲染", True, warn=True,
              note="LIBGL_ALWAYS_SOFTWARE 未设置（WSL 下 Gazebo 建议置 1）")

    ok = not fails
    return ok, lines, None
