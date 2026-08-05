#!/usr/bin/env python3
"""ylr1d_test 统一入口：解析参数、串行调度、汇总、落盘、分层补跑、清理核查。

用法：
  python3 -m ylr1d_test.run_all [--tests env,desc,control_sim,hmi]
                                [--all] [--skip-gazebo] [--out <dir>]
  ros2 run ylr1d_test ylr1d-test --tests env
入口脚本 scripts/run_tests.sh 已 source install/setup.bash 后调用本模块。

结果落盘：<WS_ROOT>/test_results/<YYYYmmdd_HHMMSS>/{summary.json,summary.txt,logs/<test>.log}
另写 test_results/latest/ 固定指向最新一次运行。任一 FAIL → 返回码非 0。
"""
import argparse
import json
import os
import shutil
import sys
import time

from . import common
from . import known_issues
from . import env_test, desc_test, control_sim_smoke, hmi_offscreen
from . import plant_test, translate_test, sensor_probe, full_flow

# 测试注册表：id / 模块 / Tier / 是否需 Gazebo / 默认跳过 / 失败定位层
TESTS = [
    {"id": "env", "mod": env_test, "tier": "Tier0", "gazebo": False, "default_skip": False},
    {"id": "desc", "mod": desc_test, "tier": "Tier1", "gazebo": False, "default_skip": False},
    {"id": "control_sim", "mod": control_sim_smoke, "tier": "Tier1", "gazebo": False, "default_skip": False},
    {"id": "hmi", "mod": hmi_offscreen, "tier": "Tier1", "gazebo": False, "default_skip": False},
    {"id": "plant", "mod": plant_test, "tier": "Tier2", "gazebo": True, "default_skip": False},
    {"id": "translate", "mod": translate_test, "tier": "Tier2", "gazebo": True, "default_skip": False},
    {"id": "sensor", "mod": sensor_probe, "tier": "Tier2", "gazebo": True, "default_skip": True},
    {"id": "full_flow", "mod": full_flow, "tier": "Tier2", "gazebo": True, "default_skip": False},
]
TESTS_BY_ID = {t["id"]: t for t in TESTS}
DEFAULT_TESTS = "env,desc,control_sim,hmi"
# 分层补跑：full_flow 失败返回 missing_layer -> 对应单层测试 id
LAYER_TEST = {"plant": "plant", "translate": "translate",
              "control_sim": "control_sim", "hmi": "hmi", "sensor": "sensor"}


def _select_tests(args):
    """返回 (选中的测试 id 列表, 被跳过的 id 列表)。

    默认跳过的测试（sensor）仅在**显式 --tests 指定**或 --all 时运行；
    用默认集合（未传 --tests）时跳过，避免默认路径误开重活。
    """
    if args.all:
        ids = list(TESTS_BY_ID.keys())
    else:
        ids = [x.strip() for x in (args.tests or DEFAULT_TESTS).split(",") if x.strip()]
    # 校验 id
    unknown = [i for i in ids if i not in TESTS_BY_ID]
    if unknown:
        sys.exit("未知测试 id: %s（可用: %s）" % (",".join(unknown), ",".join(TESTS_BY_ID)))
    explicit = args.tests is not None  # 用户显式 --tests，含默认跳过项时应运行
    skipped = []
    selected = []
    for i in ids:
        t = TESTS_BY_ID[i]
        if t["default_skip"] and not args.all and not explicit:
            skipped.append(i)
        elif args.skip_gazebo and t["gazebo"]:
            skipped.append(i)
        else:
            selected.append(i)
    return selected, skipped


def _write_summary(run_dir, results, cleanup, skipped):
    summary = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "tests": results,
        "skipped": skipped,
        "cleanup_check": cleanup,
    }
    with open(os.path.join(run_dir, "summary.json"), "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    lines = []
    lines.append("=" * 62)
    lines.append("ylr1d_test 功能测试汇总  %s" % summary["timestamp"])
    lines.append("=" * 62)
    for r in results:
        lines.append("%-5s %-12s %6.1fs  %s" %
                     (r["status"], r["id"], r["duration"], r["summary"]))
    if skipped:
        lines.append("SKIP  跳过: " + ", ".join(skipped))
    lines.append("-" * 62)
    lines.append("cleanup_check: %s" % cleanup["text"])
    lines.append("=" * 62)
    with open(os.path.join(run_dir, "summary.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    return lines


def _run_one(test, logs_dir):
    """执行单个测试，返回结果 dict。"""
    tid = test["id"]
    log_file = os.path.join(logs_dir, tid + ".log")
    common.log_path = log_file
    with open(log_file, "w"):
        pass
    common.log("===== 开始测试 [%s] (%s) =====" % (tid, test["tier"]))
    start = time.time()
    ok, detail_lines, missing_layer = False, [], None
    try:
        ok, detail_lines, missing_layer = test["mod"].run()
    except Exception as e:  # 测试异常按 FAIL 计
        detail_lines.append("测试抛异常: %r" % e)
        ok = False
    duration = time.time() - start
    for ln in detail_lines:
        common.log(ln)
    status = "PASS" if ok else "FAIL"
    common.log("===== 测试 [%s] %s (%.1fs) =====" % (tid, status, duration))
    fails = [ln for ln in detail_lines if ln.startswith("FAIL")]
    passes = [ln for ln in detail_lines if ln.startswith("PASS")]
    summary = (fails[0] if fails else passes[-1] if passes
               else (detail_lines[-1] if detail_lines else ""))
    return {
        "id": tid,
        "tier": test["tier"],
        "status": status,
        "duration": round(duration, 1),
        "summary": summary,
        "detail": detail_lines,
        "missing_layer": missing_layer,
        "log": os.path.join("logs", tid + ".log"),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(prog="ylr1d-test",
                                     description="YLR1D 功能测试统一入口")
    parser.add_argument("--tests", default=None,
                        help="逗号分隔的测试 id（默认: %s）" % DEFAULT_TESTS)
    parser.add_argument("--all", action="store_true",
                        help="运行全部测试（含默认跳过的 sensor）")
    parser.add_argument("--skip-gazebo", action="store_true",
                        help="跳过需 Gazebo 的测试（plant/translate/full_flow/sensor）")
    parser.add_argument("--out", default=None, help="结果目录（默认 <WS_ROOT>/test_results/）")
    args = parser.parse_args(argv)

    ws = common.find_ws_root()
    out_root = args.out or (os.path.join(ws, "test_results") if ws
                            else os.path.join(os.getcwd(), "test_results"))
    run_dir = os.path.join(out_root, time.strftime("%Y%m%d_%H%M%S"))
    logs_dir = os.path.join(run_dir, "logs")
    os.makedirs(logs_dir, exist_ok=True)

    selected, skipped = _select_tests(args)
    print("工作空间根: %s" % (ws or "（未识别，按 cwd 处理）"))
    print("结果目录: %s" % run_dir)
    print("本次测试: %s" % (", ".join(selected) if selected else "（无）"))
    if skipped:
        print("跳过: %s" % ", ".join(skipped))

    results = []
    ran_ids = set()
    try:
        for tid in selected:
            # 测试前后环境清理（隔离）
            common.cleanup_residual(verbose=False)
            common.log_path = os.path.join(logs_dir, tid + ".log")
            res = _run_one(TESTS_BY_ID[tid], logs_dir)
            results.append(res)
            ran_ids.add(tid)

            # 分层补跑：full_flow 失败且给出层定位，自动跑该层单测
            if (res["status"] == "FAIL" and res["missing_layer"]
                    and res["missing_layer"] in LAYER_TEST):
                layer_tid = LAYER_TEST[res["missing_layer"]]
                if layer_tid not in ran_ids:
                    common.log("== 分层诊断：full_flow 定位到 %s 层，自动补跑 [%s] ==" %
                               (res["missing_layer"], layer_tid))
                    res2 = _run_one(TESTS_BY_ID[layer_tid], logs_dir)
                    results.append(res2)
                    ran_ids.add(layer_tid)
                    res["summary"] += " | 已补跑 [%s] %s" % (layer_tid, res2["status"])
    finally:
        # 最终清理核查（修订 4 硬性要求）
        killed, residual = common.cleanup_residual(verbose=True)
        if residual:
            cleanup = {"status": "FAIL", "killed": killed, "residual": residual,
                       "text": "FAIL 清理后仍有 %d 个残留进程（pid: %s）" %
                               (len(residual), ",".join(map(str, sorted(residual))))}
        elif killed:
            cleanup = {"status": "WARN", "killed": killed, "residual": 0,
                       "text": "WARN 已清理 %d 个测试残留进程，最终无残留" % killed}
        else:
            cleanup = {"status": "PASS", "killed": 0, "residual": 0,
                       "text": "PASS 无测试残留进程"}

    summary_lines = _write_summary(run_dir, results, cleanup, skipped)
    for ln in summary_lines:
        print(ln)

    # 测试 FAIL 时输出 known_issues 诊断（只扫 FAIL 行，避免 WARN 文本误触发）
    for r in results:
        if r["status"] == "FAIL":
            text = "\n".join(ln for ln in r["detail"] if ln.startswith("FAIL"))
            hits = known_issues.advice_for(text)
            if hits:
                print("\n[%s] 可能原因与建议：" % r["id"])
                seen = set()
                for problem, solution in hits:
                    if problem in seen:
                        continue
                    seen.add(problem)
                    print("  - %s" % problem)
                    print("    解决: %s" % solution)

    # latest/ 固定指向最新
    latest_dir = os.path.join(out_root, "latest")
    try:
        if os.path.isdir(latest_dir):
            shutil.rmtree(latest_dir)
        shutil.copytree(run_dir, latest_dir)
        print("\n最新结果: %s" % latest_dir)
    except OSError as e:
        print("\n写入 latest/ 失败（忽略）: %s" % e)

    failed = [r for r in results if r["status"] == "FAIL"]
    print("\n结果: %d PASS, %d FAIL, %d SKIP（总 %d）" %
          (sum(1 for r in results if r["status"] == "PASS"),
           len(failed),
           sum(1 for r in results if r["status"] == "SKIP"),
           len(results)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
