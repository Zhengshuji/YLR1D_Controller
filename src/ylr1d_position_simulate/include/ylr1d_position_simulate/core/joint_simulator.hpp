#ifndef YLR1D_POSITION_SIMULATE__CORE__JOINT_SIMULATOR_HPP_
#define YLR1D_POSITION_SIMULATE__CORE__JOINT_SIMULATOR_HPP_

#include "ylr1d_position_simulate/core/joint_params.hpp"
#include "ylr1d_position_simulate/core/pid.hpp"

namespace ylr1d_position_simulate {

/// 底层仿真控制器：对 PID 类进行封装，输出结果为真实位置。
/// 链路：期望位置 -> PID(加速度) -> 速度(限幅) -> 位置(限幅)
class JointSimulator {
public:
  JointSimulator() = default;

  /// 一次性配置 PID 增益与限幅（限幅值同时用于 PID 内部加速度限幅）
  void configure(const JointParams & params);

  /// 从当前实际位置初始化（清空 PID 积分/微分历史）
  void initialize(double position, double velocity = 0.0);

  /// 设定期望位置
  void set_target(double target);

  /// 前进一步，返回当前真实位置
  double update(double dt);

  double position() const { return position_; }
  double velocity() const { return velocity_; }
  bool initialized() const { return initialized_; }

private:
  PID pid_;
  JointParams params_;
  double position_{0.0};
  double velocity_{0.0};
  double target_{0.0};
  bool initialized_{false};
};

}  // namespace ylr1d_position_simulate

#endif  // YLR1D_POSITION_SIMULATE__CORE__JOINT_SIMULATOR_HPP_
