#!/usr/bin/env python3
"""ylr1d_test 公共工具：进程组管理 / 残留清理 / 结果记录 / 路径定位。

约定：
- 每个测试独立进程组启动（os.setsid），结束后 killpg 整组清理（修订 4）。
- 残留清理一律用字符类 pkill（"[g]zserver" 等），避免 pkill 自匹配（CLAUDE 坑 11）。
"""
import os
import signal
import subprocess
import time

# 测试相关进程模式（用于残留核查与清理）。
# 用字符类正则避免 pkill/pgrep 自匹配到命令行里的同名文本。
PROCESS_PATTERNS = [
    "[g]zserver",
    "[g]zclient",
    "[c]hassis_simulate",
    "[a]rm_simulate",
    "[t]ranslate_server",
    "[s]pawner",
    "[r]obot_state_publisher",
    "rviz2",
    "ylr1d_hmi",
]

# 当前测试的日志文件路径（run_all 在每个测试前设置）
log_path = None


def log(msg):
    """写当前测试日志（文件 + 控制台）。"""
    line = "[%s] %s" % (time.strftime("%H:%M:%S"), msg)
    print(line, flush=True)
    if log_path:
        try:
            with open(log_path, "a") as f:
                f.write(line + "\n")
        except OSError:
            pass


def find_ws_root():
    """从 CWD 向上定位工作空间根（含 install/ 与 src/ 的目录）。"""
    env = os.environ.get("YLR1D_WS_ROOT")
    if env and os.path.isdir(os.path.join(env, "install")):
        return env
    d = os.getcwd()
    while True:
        if os.path.isdir(os.path.join(d, "install")) and os.path.isdir(os.path.join(d, "src")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return None
        d = parent


def run_cmd(cmd, timeout=30, cwd=None, env=None):
    """运行命令，返回 (returncode, stdout, stderr)。超时抛 TimeoutExpired。"""
    proc = subprocess.run(cmd, capture_output=True, text=True,
                          timeout=timeout, cwd=cwd, env=env)
    return proc.returncode, proc.stdout, proc.stderr


def start_process_group(cmd, env=None):
    """在独立进程组启动命令，输出追加到当前测试日志。返回 Popen（非阻塞）。"""
    log_fd = open(log_path, "a") if log_path else None
    full_env = os.environ.copy()
    if env:
        full_env.update(env)
    proc = subprocess.Popen(
        cmd, stdout=log_fd, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid, env=full_env,
    )
    proc._log_fd = log_fd  # 保持引用，避免 fd 被回收
    return proc


def kill_process_group(proc):
    """SIGTERM 进程组，宽限后 SIGKILL。"""
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    try:
        if proc._log_fd is not None:
            proc._log_fd.close()
    except (OSError, AttributeError):
        pass


def pgrep_matching():
    """返回当前匹配测试相关模式的进程 PID 集合（不含本进程）。"""
    pids = set()
    for pat in PROCESS_PATTERNS:
        try:
            proc = subprocess.run(["pgrep", "-f", pat],
                                  capture_output=True, text=True, timeout=15)
        except subprocess.TimeoutExpired:
            continue
        if proc.returncode == 0:
            for line in proc.stdout.split():
                try:
                    pids.add(int(line))
                except ValueError:
                    pass
    return pids


def cleanup_residual(verbose=True):
    """清理测试相关残留进程，返回 (被杀进程数, 残留 PID 数)。"""
    before = pgrep_matching()
    for pat in PROCESS_PATTERNS:
        try:
            subprocess.run(["pkill", "-f", pat],
                           capture_output=True, timeout=15)
        except subprocess.TimeoutExpired:
            pass
    # pkill 是异步信号，稍等再核查
    time.sleep(1.0)
    after = pgrep_matching()
    if verbose and before:
        log("清理残留进程 %d 个" % len(before))
    return len(before), len(after)


def wait_for_controllers(expect=6, timeout=120):
    """轮询 ros2 control list_controllers 直到 expect 个 active。返回最终 active 数。"""
    active = 0
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            rc, out, err = run_cmd(["ros2", "control", "list_controllers"], timeout=15)
            text = out or err
            active = sum(1 for ln in text.splitlines() if "active" in ln)
            if active >= expect:
                return active
        except subprocess.TimeoutExpired:
            pass
        time.sleep(3.0)
    return active


def wait_for_action_servers(expect, timeout=60):
    """轮询 ros2 action list 直到期望的 action server 全部在线。返回已见集合。"""
    seen = set()
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            rc, out, err = run_cmd(["ros2", "action", "list"], timeout=15)
            text = out or err
            for ln in text.splitlines():
                name = ln.strip().strip("/")
                if name:
                    seen.add(name)
            if expect <= seen:
                return seen
        except subprocess.TimeoutExpired:
            pass
        time.sleep(3.0)
    return seen
