#ifndef ALGORITHM__CONFIG__JOINT_CONFIG_HPP_
#define ALGORITHM__CONFIG__JOINT_CONFIG_HPP_

#include <cstddef>
#include <string>

namespace ylr1d_algorithm_sim {

/// 单个关节的仿真限幅配置（静态配置单一来源，编译期确定，不再经 yaml）。
/// 数值基准：ylr1d_description/config/limits.yaml 的物理限位语境。
/// 原位于 ylr1d_control/include/ylr1d_control/config/joint_config.hpp，
/// 阶段 B 随"控制参数移交算法层"整体迁移至此。
struct JointLimit {
  bool has_position_limit{false};
  double lower{0.0};         // 位置下限 (rad 或 m)
  double upper{0.0};         // 位置上限 (rad 或 m)
  double max_vel{3.0};       // 速度限幅 (rad/s 或 m/s)
  double max_accel{50.0};    // 加速度限幅 (rad/s^2 或 m/s^2)
};

/// 位置控制关节限位条目（按 URDF 关节名查表）。
struct PositionLimitEntry {
  const char * name;
  JointLimit limit;
};

/// 位置控制关节限位表（26 项，逐字取自原 config/position_control_limits.yaml）。
/// 单位：旋转关节 rad，棱柱关节 m（Joint_Base_to_Body1、夹爪）。
inline constexpr PositionLimitEntry kPositionLimits[] = {
  {"Joint_Base_to_RFWheelF",   {true,  -3.14,   3.14,   3.0,   5.0}},
  {"Joint_Base_to_LFWheelF",   {true,  -3.14,   3.14,   3.0,   5.0}},
  {"Joint_Base_to_RBWheelF",   {true,  -3.14,   3.14,   3.0,   5.0}},
  {"Joint_Base_to_LBWheelF",   {true,  -3.14,   3.14,   3.0,   5.0}},
  {"Joint_Base_to_Body1",      {true,  -0.30,   0.30,   0.6782, 5.0}},
  {"Joint_Body1_to_Body2",     {true,  -3.14,   3.14,   2.83,  5.0}},
  {"Joint_Body2_to_Body3",     {true,  -1.57,   1.57,   5.65,  5.0}},
  {"Joint_Body3_to_Body4",     {true,  -1.57,   1.57,   5.65,  5.0}},
  {"Joint_Body2_to_LeftArm1",  {true,  -2.62,   2.62,   2.83,  5.0}},
  {"Joint_LeftArm1_to_LeftArm2", {true, -1.57,  1.83,   2.83,  5.0}},
  {"Joint_LeftArm2_to_LeftArm3", {true, -2.62,  2.62,   5.65,  5.0}},
  {"Joint_LeftArm3_to_LeftArm4", {true, -1.57,  1.57,   5.65,  5.0}},
  {"Joint_LeftArm4_to_LeftArm5", {true, -2.62,  2.62,   5.65,  5.0}},
  {"Joint_LeftArm5_to_LeftArm6", {true, -2.09,  2.09,   5.65,  5.0}},
  {"Joint_LeftArm6_to_LeftArm7", {true, -6.28,  6.28,   5.65,  5.0}},
  {"Joint_LeftArm7_to_LeftFinger1", {true, -0.014, 0.0,  0.05, 1.0}},
  {"Joint_LeftArm7_to_LeftFinger2", {true, -0.014, 0.0,  0.05, 1.0}},
  {"Joint_Body2_to_RightArm1", {true,  -2.62,   2.62,   2.83,  5.0}},
  {"Joint_RightArm1_to_RightArm2", {true, -1.57, 1.83,   2.83,  5.0}},
  {"Joint_RightArm2_to_RightArm3", {true, -2.62, 2.62,   5.65,  5.0}},
  {"Joint_RightArm3_to_RightArm4", {true, -1.57, 1.57,   5.65,  5.0}},
  {"Joint_RightArm4_to_RightArm5", {true, -2.62, 2.62,   5.65,  5.0}},
  {"Joint_RightArm5_to_RightArm6", {true, -2.09, 2.09,   5.65,  5.0}},
  {"Joint_RightArm6_to_RightArm7", {true, -6.28, 6.28,   5.65,  5.0}},
  {"Joint_RightArm7_to_RightFinger1", {true, 0.0, 0.014, 0.05, 1.0}},
  {"Joint_RightArm7_to_RightFinger2", {true, 0.0, 0.014, 0.05, 1.0}},
};
inline constexpr size_t kPositionLimitCount =
    sizeof(kPositionLimits) / sizeof(kPositionLimits[0]);

/// 速度控制关节（轮子）限位条目（连续旋转，无位置限位）。
struct VelocityLimitEntry {
  const char * name;
  JointLimit limit;
};

/// 速度控制关节限位表（4 项，逐字取自原 config/velocity_control_limits.yaml）。
/// 单位：速度 rad/s，加速度 rad/s^2。accelerate=10.0 为 yaml 实际生效值。
inline constexpr VelocityLimitEntry kVelocityLimits[] = {
  {"Joint_LFWheelF_to_LFWheel", {false, 0.0, 0.0, 5.0, 10.0}},
  {"Joint_LBWheelF_to_LBWheel", {false, 0.0, 0.0, 5.0, 10.0}},
  {"Joint_RBWheelF_to_RBWheel", {false, 0.0, 0.0, 5.0, 10.0}},
  {"Joint_RFWheelF_to_RFWheel", {false, 0.0, 0.0, 5.0, 10.0}},
};
inline constexpr size_t kVelocityLimitCount =
    sizeof(kVelocityLimits) / sizeof(kVelocityLimits[0]);

/// 线性查表取关节限位，未命中返回默认 JointLimit{}（无位置限位 + 默认限幅，防御未知关节）。
inline JointLimit jointLimitFor(const std::string & name) {
  for (const auto & e : kPositionLimits) {
    if (name == e.name) return e.limit;
  }
  for (const auto & e : kVelocityLimits) {
    if (name == e.name) return e.limit;
  }
  return JointLimit{};
}

// ── 关节分组（名/顺序与命令数组索引一致） ──
inline constexpr const char * kSteeringJoints[] = {
  "Joint_Base_to_RFWheelF", "Joint_Base_to_LFWheelF",
  "Joint_Base_to_RBWheelF", "Joint_Base_to_LBWheelF"};
inline constexpr size_t kSteeringJointCount = 4;

inline constexpr const char * kWheelJoints[] = {
  "Joint_RFWheelF_to_RFWheel", "Joint_LFWheelF_to_LFWheel",
  "Joint_RBWheelF_to_RBWheel", "Joint_LBWheelF_to_LBWheel"};
inline constexpr size_t kWheelJointCount = 4;

inline constexpr const char * kTorsoJoints[] = {
  "Joint_Base_to_Body1", "Joint_Body1_to_Body2",
  "Joint_Body2_to_Body3", "Joint_Body3_to_Body4"};
inline constexpr size_t kTorsoJointCount = 4;

inline constexpr const char * kLeftArmJoints[] = {
  "Joint_Body2_to_LeftArm1", "Joint_LeftArm1_to_LeftArm2",
  "Joint_LeftArm2_to_LeftArm3", "Joint_LeftArm3_to_LeftArm4",
  "Joint_LeftArm4_to_LeftArm5", "Joint_LeftArm5_to_LeftArm6",
  "Joint_LeftArm6_to_LeftArm7", "Joint_LeftArm7_to_LeftFinger1",
  "Joint_LeftArm7_to_LeftFinger2"};
inline constexpr size_t kLeftArmJointCount = 9;

inline constexpr const char * kRightArmJoints[] = {
  "Joint_Body2_to_RightArm1", "Joint_RightArm1_to_RightArm2",
  "Joint_RightArm2_to_RightArm3", "Joint_RightArm3_to_RightArm4",
  "Joint_RightArm4_to_RightArm5", "Joint_RightArm5_to_RightArm6",
  "Joint_RightArm6_to_RightArm7", "Joint_RightArm7_to_RightFinger1",
  "Joint_RightArm7_to_RightFinger2"};
inline constexpr size_t kRightArmJointCount = 9;

// 命令话题（ForwardCommandController）——由控制层引用，单一来源在本包
inline constexpr const char * kSteeringTopic = "/chassis_steering_controller/commands";
inline constexpr const char * kWheelTopic = "/chassis_wheels_controller/commands";
inline constexpr const char * kTorsoTopic = "/torso_controller/commands";
inline constexpr const char * kLeftArmTopic = "/left_arm_controller/commands";
inline constexpr const char * kRightArmTopic = "/right_arm_controller/commands";

// ── 组注册表（阶段 B 新增） ──
/// 仿真控制器组定义：组名（仿真控制器 group_name 参数）→ 关节列表 + 控制类型。
/// is_position=true → 位置型（PositionGroupPlant，1/s²）；false → 速度型（VelocityGroupPlant，1/s）。
struct JointGroupDef {
  const char * name;             // 组名：steering / wheels / torso / left_arm / right_arm
  const char * const * joints;   // 关节名数组
  size_t count;                  // 组内关节数
  bool is_position;              // 控制类型
};

inline constexpr JointGroupDef kJointGroups[] = {
  {"steering",   kSteeringJoints, kSteeringJointCount, true},
  {"wheels",     kWheelJoints,    kWheelJointCount,    false},
  {"torso",      kTorsoJoints,    kTorsoJointCount,    true},
  {"left_arm",   kLeftArmJoints,  kLeftArmJointCount,  true},
  {"right_arm",  kRightArmJoints, kRightArmJointCount, true},
};
inline constexpr size_t kJointGroupCount =
    sizeof(kJointGroups) / sizeof(kJointGroups[0]);

/// 按组名查组定义，未命中返回 nullptr。
inline const JointGroupDef * jointGroupFor(const std::string & name) {
  for (const auto & g : kJointGroups) {
    if (name == g.name) return &g;
  }
  return nullptr;
}

}  // namespace ylr1d_algorithm_sim

#endif  // ALGORITHM__CONFIG__JOINT_CONFIG_HPP_
