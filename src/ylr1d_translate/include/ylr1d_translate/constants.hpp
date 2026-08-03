#ifndef YLR1D_TRANSLATE__CONSTANTS_HPP_
#define YLR1D_TRANSLATE__CONSTANTS_HPP_

#include <array>

namespace ylr1d_translate {
namespace constants {

// ── ChassisMove.mode ────────────────────────────────
constexpr int8_t MODE_TRANSLATE = 0;  // 平移：四轮同向、同速
constexpr int8_t MODE_ROTATE = 1;     // 原地旋转：固定转向角
constexpr int8_t MODE_STOP = 2;       // 停车：固定转向角、轮速 0

// ── ArmMove.part ────────────────────────────────────
// 机械臂与夹爪解耦：夹爪两指由 GripperMove 控制，不在 ArmMove 关节表中
constexpr int8_t PART_TORSO = 0;    // 躯干（4 关节）
constexpr int8_t PART_LEFT_ARM = 1; // 左臂（7 关节）
constexpr int8_t PART_RIGHT_ARM = 2;// 右臂（7 关节）

// ── GripperMove.part / open ─────────────────────────
constexpr int8_t GRIPPER_LEFT = 0;  // 左夹爪
constexpr int8_t GRIPPER_RIGHT = 1; // 右夹爪
constexpr bool GRIPPER_OPEN = true; // 开（张到限位）
constexpr bool GRIPPER_CLOSE = false; // 关（并拢）

// 夹指开合目标位置（棱柱关节，单位 m）
constexpr double LEFT_FINGER_OPEN = -0.014;   // 左夹爪开
constexpr double LEFT_FINGER_CLOSE = 0.0;     // 左夹爪关
constexpr double RIGHT_FINGER_OPEN = 0.0;     // 右夹爪开
constexpr double RIGHT_FINGER_CLOSE = 0.014;  // 右夹爪关

// ── 底座运动学参数（Controller.md 2.3）─────────────
// 原地旋转模式固定转向角 [RF, LF, RB, LB]
constexpr std::array<double, 4> ROTATE_STEERING = {-2.2150, -0.9265, 2.2150, 0.9265};
// 停车模式固定转向角 [RF, LF, RB, LB]
constexpr std::array<double, 4> STOP_STEERING = {-0.6443, 0.6443, -2.4973, 2.4973};
// 旋转换算：车体角速度 ω = ROTATE_RATIO * 轮速
constexpr double ROTATE_RATIO = -3.7538;
// 行进换算：车体角速度 v = TRANSLATE_RATIO * 轮速
constexpr double TRANSLATE_RATIO = 0.5;

// ── 到位容差 / 超时 ────────────────────────────────
constexpr double TURN_TOL = 0.03;    // 转向到位容差 (rad)
constexpr double ARM_TOL = 0.02;     // 机械臂到位容差 (rad)
constexpr double GRIPPER_TOL = 0.002;// 夹爪到位容差 (m)；夹指行程仅 0.014，须远小于行程
constexpr double ARM_TIMEOUT = 30.0; // 机械臂 / 夹爪到位超时 (s)
constexpr double LOOP_DT = 0.05;     // 控制 / 检查循环周期 (s)

}  // namespace constants
}  // namespace ylr1d_translate

#endif  // YLR1D_TRANSLATE__CONSTANTS_HPP_
