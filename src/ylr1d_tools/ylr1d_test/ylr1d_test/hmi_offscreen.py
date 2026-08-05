#!/usr/bin/env python3
"""Tier1 HMI 无头冒烟测试：QT_QPA_PLATFORM=offscreen 下各面板起 5s 不崩。

依次启动 monitor / sensor / control 三个面板，进程存活未异常退出即 PASS；
异常退出 → FAIL（提示 offscreen / X server / Qt 字体问题）。每个面板测完即 kill。
"""
import time

from . import common

# 面板可执行名（与 ylr1d_hmi 一致）
PANELS = ["ylr1d_hmi_monitor", "ylr1d_hmi_sensor", "ylr1d_hmi_control"]


def run():
    """执行 HMI offscreen 测试，返回 (ok, detail_lines, missing_layer)。"""
    lines = []
    ok = True
    env = {"QT_QPA_PLATFORM": "offscreen"}
    for exe in PANELS:
        common.log("启动 %s (offscreen, 存活 5s)..." % exe)
        proc = common.start_process_group(["ros2", "run", "ylr1d_hmi", exe], env=env)
        time.sleep(5.0)
        rc = proc.poll()
        if rc is None:
            lines.append("PASS %s 存活 5s 未崩" % exe)
        else:
            lines.append("FAIL %s 异常退出 rc=%s（检查 offscreen/X server/Qt 字体）" % (exe, rc))
            ok = False
        common.kill_process_group(proc)
        common.cleanup_residual(verbose=False)
    return ok, lines, ("hmi" if not ok else None)
