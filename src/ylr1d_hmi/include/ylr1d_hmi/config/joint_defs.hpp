#ifndef YLR1D_HMI__CONFIG__JOINT_DEFS_HPP_
#define YLR1D_HMI__CONFIG__JOINT_DEFS_HPP_

#include <QString>

#include <string>
#include <vector>

namespace ylr1d_hmi {

/// One static joint definition. Limits/units are the HMI display envelope and
/// stay in sync with the robot model authority:
///   src/ylr1d_description/config/limits.yaml   (what Gazebo actually loads)
///   src/ylr1d_control_sim/config/position_control_limits.yaml
/// Note: the gripper fingers use the ±0.015 m display envelope here — the
/// translate layer resolves GripperMove with ±0.014 (see ylr1d_translate
/// constants), so the two are intentionally slightly different.
struct JointDef {
  std::string name;       // URDF joint name
  QString label;          // display short name
  bool is_velocity{false};     // true = wheel (velocity control)
  bool is_prismatic{false};    // true = translational joint (unit: m / m/s)
  double lower{-3.14};         // SI: rad / m
  double upper{3.14};
};

/// One dropdown entry in a joint-group selector.
struct JointGroup {
  QString name;
  std::vector<JointDef> joints;
};

// ── Atomic collections (single source of the 30-joint table) ────────────
inline const std::vector<JointDef> & steeringDefs() {
  static const std::vector<JointDef> d = {
    {"Joint_Base_to_RFWheelF",    "RF_Steer",  false, false, -3.14, 3.14},
    {"Joint_Base_to_LFWheelF",    "LF_Steer",  false, false, -3.14, 3.14},
    {"Joint_Base_to_RBWheelF",    "RB_Steer",  false, false, -3.14, 3.14},
    {"Joint_Base_to_LBWheelF",    "LB_Steer",  false, false, -3.14, 3.14},
  };
  return d;
}

inline const std::vector<JointDef> & wheelsDefs() {
  static const std::vector<JointDef> d = {
    {"Joint_RFWheelF_to_RFWheel", "RF_Wheel",  true},
    {"Joint_LFWheelF_to_LFWheel", "LF_Wheel",  true},
    {"Joint_RBWheelF_to_RBWheel", "RB_Wheel",  true},
    {"Joint_LBWheelF_to_LBWheel", "LB_Wheel",  true},
  };
  return d;
}

inline const std::vector<JointDef> & torsoDefs() {
  static const std::vector<JointDef> d = {
    {"Joint_Base_to_Body1",    "Lift",    false, true,  -0.30,  0.30},
    {"Joint_Body1_to_Body2",   "Yaw",     false, false, -3.14,  3.14},
    {"Joint_Body2_to_Body3",   "Pitch1",  false, false, -1.57,  1.57},
    {"Joint_Body3_to_Body4",   "Pitch2",  false, false, -1.57,  1.57},
  };
  return d;
}

inline const std::vector<JointDef> & leftArmDefs() {
  static const std::vector<JointDef> d = {
    {"Joint_Body2_to_LeftArm1",    "Shoulder1", false, false, -2.62,  2.62},
    {"Joint_LeftArm1_to_LeftArm2", "Shoulder2", false, false, -1.57,  1.83},
    {"Joint_LeftArm2_to_LeftArm3", "Shoulder3", false, false, -2.62,  2.62},
    {"Joint_LeftArm3_to_LeftArm4", "Elbow1",    false, false, -1.57,  1.57},
    {"Joint_LeftArm4_to_LeftArm5", "Elbow2",    false, false, -2.62,  2.62},
    {"Joint_LeftArm5_to_LeftArm6", "Wrist1",    false, false, -2.09,  2.09},
    {"Joint_LeftArm6_to_LeftArm7", "Wrist2",    false, false, -6.28,  6.28},
  };
  return d;
}

inline const std::vector<JointDef> & rightArmDefs() {
  static const std::vector<JointDef> d = {
    {"Joint_Body2_to_RightArm1",   "Shoulder1", false, false, -2.62,  2.62},
    {"Joint_RightArm1_to_RightArm2","Shoulder2", false, false, -1.57,  1.83},
    {"Joint_RightArm2_to_RightArm3","Shoulder3", false, false, -2.62,  2.62},
    {"Joint_RightArm3_to_RightArm4","Elbow1",    false, false, -1.57,  1.57},
    {"Joint_RightArm4_to_RightArm5","Elbow2",    false, false, -2.62,  2.62},
    {"Joint_RightArm5_to_RightArm6","Wrist1",    false, false, -2.09,  2.09},
    {"Joint_RightArm6_to_RightArm7","Wrist2",    false, false, -6.28,  6.28},
  };
  return d;
}

inline const std::vector<JointDef> & leftFingersDefs() {
  static const std::vector<JointDef> d = {
    {"Joint_LeftArm7_to_LeftFinger1", "Finger1", false, true, -0.015, 0.0},
    {"Joint_LeftArm7_to_LeftFinger2", "Finger2", false, true, -0.015, 0.0},
  };
  return d;
}

inline const std::vector<JointDef> & rightFingersDefs() {
  static const std::vector<JointDef> d = {
    {"Joint_RightArm7_to_RightFinger1", "Finger1", false, true, 0.0, 0.015},
    {"Joint_RightArm7_to_RightFinger2", "Finger2", false, true, 0.0, 0.015},
  };
  return d;
}

// ── HmiWindow view: four tabs (Chassis 8 / Torso 4 / Left 9 / Right 9) ──
inline const std::vector<JointDef> & chassis_joints() {
  static const std::vector<JointDef> d = [] {
    std::vector<JointDef> v;
    const auto & s = steeringDefs();
    const auto & w = wheelsDefs();
    v.insert(v.end(), s.begin(), s.end());
    v.insert(v.end(), w.begin(), w.end());
    return v;
  }();
  return d;
}

inline const std::vector<JointDef> & torso_joints() { return torsoDefs(); }

inline const std::vector<JointDef> & left_arm_joints() {
  static const std::vector<JointDef> d = [] {
    std::vector<JointDef> v;
    const auto & a = leftArmDefs();
    const auto & f = leftFingersDefs();
    v.insert(v.end(), a.begin(), a.end());
    v.insert(v.end(), f.begin(), f.end());
    return v;
  }();
  return d;
}

inline const std::vector<JointDef> & right_arm_joints() {
  static const std::vector<JointDef> d = [] {
    std::vector<JointDef> v;
    const auto & a = rightArmDefs();
    const auto & f = rightFingersDefs();
    v.insert(v.end(), a.begin(), a.end());
    v.insert(v.end(), f.begin(), f.end());
    return v;
  }();
  return d;
}

// ── Translate view: three parts (Torso 4 / Left 7 / Right 7), no fingers ──
// The fingers belong to GripperMove, exactly like ylr1d_translate's
// kPartJoints table.
inline const std::vector<std::vector<JointDef>> & armPartJoints() {
  static const std::vector<std::vector<JointDef>> table = {
      torsoDefs(), leftArmDefs(), rightArmDefs(),
  };
  return table;
}

// ── Monitor view: five dropdown groups (all 30 joints) ─────────────────
inline std::vector<JointGroup> makeJointGroups() {
  return {
    {"Chassis Steering", steeringDefs()},
    {"Chassis Wheels",   wheelsDefs()},
    {"Torso",            torsoDefs()},
    {"Left Arm",         [] {
      std::vector<JointDef> v;
      const auto & a = leftArmDefs();
      const auto & f = leftFingersDefs();
      v.insert(v.end(), a.begin(), a.end());
      v.insert(v.end(), f.begin(), f.end());
      return v;
    }()},
    {"Right Arm",        [] {
      std::vector<JointDef> v;
      const auto & a = rightArmDefs();
      const auto & f = rightFingersDefs();
      v.insert(v.end(), a.begin(), a.end());
      v.insert(v.end(), f.begin(), f.end());
      return v;
    }()},
  };
}

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__CONFIG__JOINT_DEFS_HPP_
