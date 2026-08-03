#ifndef YLR1D_TRANSLATE__JOINT_NAMES_HPP_
#define YLR1D_TRANSLATE__JOINT_NAMES_HPP_

#include <array>
#include <string>
#include <vector>

namespace ylr1d_translate {
namespace joints {

/// 四轮转向关节（顺序与控制层 /chassis_steering_controller 一致）
constexpr std::array<const char *, 4> kSteering = {
    "Joint_Base_to_RFWheelF", "Joint_Base_to_LFWheelF",
    "Joint_Base_to_RBWheelF", "Joint_Base_to_LBWheelF"};

/// 四轮驱动关节（顺序与控制层 /chassis_wheels_controller 一致）
constexpr std::array<const char *, 4> kWheels = {
    "Joint_RFWheelF_to_RFWheel", "Joint_LFWheelF_to_LFWheel",
    "Joint_RBWheelF_to_RBWheel", "Joint_LBWheelF_to_LBWheel"};

/// 躯干（顺序与控制层 /torso_controller 一致）
constexpr std::array<const char *, 4> kTorso = {
    "Joint_Base_to_Body1", "Joint_Body1_to_Body2",
    "Joint_Body2_to_Body3", "Joint_Body3_to_Body4"};

/// 左臂（顺序与控制层 /left_arm_controller 一致；夹爪两指归 GripperMove，不在此表）
constexpr std::array<const char *, 7> kLeftArm = {
    "Joint_Body2_to_LeftArm1", "Joint_LeftArm1_to_LeftArm2",
    "Joint_LeftArm2_to_LeftArm3", "Joint_LeftArm3_to_LeftArm4",
    "Joint_LeftArm4_to_LeftArm5", "Joint_LeftArm5_to_LeftArm6",
    "Joint_LeftArm6_to_LeftArm7"};

/// 右臂（顺序与控制层 /right_arm_controller 一致；夹爪两指归 GripperMove，不在此表）
constexpr std::array<const char *, 7> kRightArm = {
    "Joint_Body2_to_RightArm1", "Joint_RightArm1_to_RightArm2",
    "Joint_RightArm2_to_RightArm3", "Joint_RightArm3_to_RightArm4",
    "Joint_RightArm4_to_RightArm5", "Joint_RightArm5_to_RightArm6",
    "Joint_RightArm6_to_RightArm7"};

/// 夹爪两指（左/右各 2）
constexpr std::array<const char *, 2> kLeftFingers = {
    "Joint_LeftArm7_to_LeftFinger1", "Joint_LeftArm7_to_LeftFinger2"};
constexpr std::array<const char *, 2> kRightFingers = {
    "Joint_RightArm7_to_RightFinger1", "Joint_RightArm7_to_RightFinger2"};

/// 部件关节表（与 ArmMove.action 的 part 枚举对应）
/// 0=躯干(4) 1=左臂(7) 2=右臂(7)；夹爪两指归 GripperMove（kLeftFingers / kRightFingers）
inline const std::vector<std::vector<std::string>> & kPartJoints() {
  static const std::vector<std::vector<std::string>> table = {
      {kTorso.begin(), kTorso.end()},
      {kLeftArm.begin(), kLeftArm.end()},
      {kRightArm.begin(), kRightArm.end()},
  };
  return table;
}

}  // namespace joints
}  // namespace ylr1d_translate

#endif  // YLR1D_TRANSLATE__JOINT_NAMES_HPP_
