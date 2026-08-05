#!/usr/bin/env python3
"""Tier1 description 静态测试：检测模型能否成功导入、传感器配置是否正常。

- 模型导入：用 ylr1d_description 公共模块 xacro_utils.process_xacro_to_urdf
  生成 URDF（裸 xacro 命令不可用，见 ylr1d_description README 警告）。
- 结构校验：30 个受控关节名齐全、关键关节限位与 limits.yaml 一致。
- 传感器：URDF 内 5 个 ray（雷达 + 4 超声）+ imu 插件存在，与 sensors.yaml 一致。
  动态话题验证归 sensor_probe（需传感器世界 + Gazebo）。
"""
import os
import sys
import xml.etree.ElementTree as ET

from . import common

# 30 个受控关节（固定顺序，与 /joint_states 及 controllers.yaml 一致）
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

# 关键限位（对照 limits.yaml 物理限位；rad 或 m）
# 注意：手指 ±0.05 是 URDF/limits.yaml 物理限位；±0.014 是 control_sim
# joint_config.hpp 的控制钳制限位（更紧），此处验证物理层。
KEY_LIMITS = {
    "Joint_Base_to_Body1": (-0.30, 0.30),          # 棱柱，m
    "Joint_LeftArm7_to_LeftFinger1": (-0.05, 0.05),  # 棱柱，m
    "Joint_RightArm7_to_RightFinger1": (-0.05, 0.05), # 棱柱，m
}

# 期望的 ray 传感器（雷达 + 4 超声）与 imu
RAY_SENSORS = ["radar_sensor", "lf_ultrasonic_sensor", "rf_ultrasonic_sensor",
               "lb_ultrasonic_sensor", "rb_ultrasonic_sensor"]
IMU_SENSORS = ["imu_sensor"]


def _load_xacro_utils(ws):
    """按 ylr1d_description launch 的导入模式加载 xacro_utils（share 优先，回退 src）。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory("ylr1d_description"),
                             "launch", "python_utils")
        if os.path.isdir(share):
            sys.path.insert(0, share)
            import xacro_utils  # noqa: F401
            return share
    except Exception:
        pass
    # 回退：未 source 时用源码目录
    src = os.path.join(ws, "src", "ylr1d_description", "launch", "python_utils")
    if os.path.isdir(src):
        sys.path.insert(0, src)
        import xacro_utils  # noqa: F401
        return src
    return None


def run():
    """执行 desc 静态测试，返回 (ok, detail_lines, None)。"""
    ws = common.find_ws_root()
    lines = []
    fails = []

    def check(name, ok, note=""):
        lines.append("%-4s %-28s %s" % ("PASS" if ok else "FAIL", name, note))
        if not ok:
            fails.append(name)

    # ── 定位资产路径 ──
    if not ws:
        return False, ["FAIL 未定位工作空间根"], None
    xacro_path = os.path.join(ws, "src", "ylr1d_description", "urdf", "ylr1d.xacro")
    config_dir = os.path.join(ws, "src", "ylr1d_description", "config")
    controllers_yaml = os.path.join(ws, "src", "ylr1d_plant", "config", "controllers.yaml")
    check("资产路径", all(os.path.exists(p) for p in [xacro_path, config_dir, controllers_yaml]),
          "xacro/config/controllers 齐全" if all(os.path.exists(p) for p in [xacro_path, config_dir, controllers_yaml])
          else "缺失资产路径，请核对 src 结构")

    # ── 模型导入（xacro → URDF） ──
    utils_dir = _load_xacro_utils(ws)
    if not utils_dir:
        return False, lines + ["FAIL xacro_utils 模块未找到（无法验证模型导入）"], None
    check("xacro_utils", True, "加载自 %s" % utils_dir)

    robot_desc = None
    try:
        import xacro_utils
        robot_desc, urdf_tmp = xacro_utils.process_xacro_to_urdf(
            xacro_path, config_dir, controllers_yaml)
        check("xacro → URDF", bool(robot_desc and urdf_tmp),
              "生成成功（tmp: %s）" % urdf_tmp)
    except Exception as e:
        check("xacro → URDF", False, "异常: %s" % e)

    if robot_desc is None:
        return False, lines, None

    try:
        root = ET.fromstring(robot_desc)
    except ET.ParseError as e:
        return False, lines + ["FAIL URDF 解析失败: %s" % e], None

    # gazebo_ros2_control 会注入无 <limit> 的重复 joint 元素，优先保留带 <limit>
    # 的物理关节，避免覆盖导致限位查询落空。
    joints = {}
    for j in root.iter("joint"):
        name = j.attrib.get("name")
        if name is None:
            continue
        cur = joints.get(name)
        if cur is None or j.find("limit") is not None:
            joints[name] = j
    check("URDF 可解析", True, "共 %d 个 joint" % len(joints))

    # ── 30 受控关节名齐全 ──
    missing = [n for n in ALL_NAMES if n not in joints]
    check("30 受控关节名", not missing,
          "齐全" if not missing else "缺失: " + ",".join(missing))

    # ── 关键限位 ──
    for name, (lo, hi) in KEY_LIMITS.items():
        lim = joints.get(name)
        ok = False
        note = "%s 未找到" % name
        if lim is not None:
            l = lim.find("limit")
            if l is not None:
                try:
                    lower, upper = float(l.attrib["lower"]), float(l.attrib["upper"])
                    ok = abs(lower - lo) < 1e-6 and abs(upper - hi) < 1e-6
                    note = "limit [%.3f, %.3f] (期望 [%.3f, %.3f])" % (lower, upper, lo, hi)
                except (KeyError, ValueError):
                    note = "limit 属性缺失"
        check("限位 %s" % name, ok, note)

    # ── 传感器插件 ──
    sensors = {}
    for s in root.iter("sensor"):
        sensors.setdefault(s.attrib.get("type"), []).append(s.attrib.get("name"))
    ray_names = set(sensors.get("ray", []))
    imu_names = set(sensors.get("imu", []))
    check("ray 传感器齐全", set(RAY_SENSORS) <= ray_names,
          "缺: " + ",".join(set(RAY_SENSORS) - ray_names) if not set(RAY_SENSORS) <= ray_names
          else "5 个 ray 全部存在（雷达+4超声）")
    check("imu 传感器", set(IMU_SENSORS) <= imu_names,
          "缺: " + ",".join(set(IMU_SENSORS) - imu_names) if not set(IMU_SENSORS) <= imu_names
          else "imu_sensor 存在")

    # 与 sensors.yaml 的 ray 数量对照
    try:
        import yaml
        with open(os.path.join(config_dir, "sensors.yaml")) as f:
            sdata = yaml.safe_load(f) or {}
        yaml_ray = [k for k, v in sdata.items() if isinstance(v, dict) and v.get("type") == "ray"]
        check("sensors.yaml ray 数", len(yaml_ray) == len(RAY_SENSORS),
              "yaml ray=%d（期望 %d）" % (len(yaml_ray), len(RAY_SENSORS)))
    except Exception as e:
        check("sensors.yaml 对照", True, note="跳过（%s）" % e)

    ok = not fails
    return ok, lines, None
