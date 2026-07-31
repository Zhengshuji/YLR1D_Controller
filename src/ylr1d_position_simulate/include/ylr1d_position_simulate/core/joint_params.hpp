#ifndef YLR1D_POSITION_SIMULATE__CORE__JOINT_PARAMS_HPP_
#define YLR1D_POSITION_SIMULATE__CORE__JOINT_PARAMS_HPP_

namespace ylr1d_position_simulate {

/// 单关节仿真参数（PID 增益 + 限幅）。
/// 位置与速度关节共用同一结构，作为 yaml / 节点参数未提供时的预设默认值。
struct JointParams {
  double kp{4.0};
  double ki{0.0};
  double kd{0.2};
  double max_accel{50.0};      // 加速度限幅 (rad/s^2 或 m/s^2)
  double max_vel{3.0};         // 速度限幅 (rad/s 或 m/s)
  bool has_position_limit{false};
  double lower{0.0};           // 位置下限 (rad 或 m)
  double upper{0.0};           // 位置上限 (rad 或 m)
};

/// 位置关节预设：kp=4/0/0.2，加速 50，速度 3，带位置限位
inline JointParams defaultPositionParams() {
  JointParams p;
  p.kp = 4.0;
  p.kd = 0.2;
  p.max_accel = 50.0;
  p.max_vel = 3.0;
  p.has_position_limit = true;
  return p;
}

/// 速度关节（轮子）预设：kp=2/0/0.05，加速 20，速度 5，无位置限位
inline JointParams defaultVelocityParams() {
  JointParams p;
  p.kp = 2.0;
  p.kd = 0.05;
  p.max_accel = 20.0;
  p.max_vel = 5.0;
  p.has_position_limit = false;
  return p;
}

}  // namespace ylr1d_position_simulate

#endif  // YLR1D_POSITION_SIMULATE__CORE__JOINT_PARAMS_HPP_
