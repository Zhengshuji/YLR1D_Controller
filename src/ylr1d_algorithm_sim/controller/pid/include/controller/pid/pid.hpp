#ifndef YLR1D_ALGORITHM_SIM__CONTROLLER__PID__PID_HPP_
#define YLR1D_ALGORITHM_SIM__CONTROLLER__PID__PID_HPP_

namespace ylr1d_algorithm_sim {

/// PID 控制器参数
struct PidParams {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double output_limit{0.0};  // 输出限幅；<=0 表示不限幅（语义由外层解释，不绑定加速度等）
};

/// 通用 PID 控制器：输入期望与反馈，输出控制量 u。
/// 不关心 u 的语义（加速度/位置/速度…），语义由被控对象模型 / 控制层解释。
class PidController {
public:
  PidController() = default;

  /// 配置参数（可重复调用，支持运行时热更新）
  void configure(const PidParams & params);

  /// 初始化（清空积分/微分历史）
  void initialize(double state = 0.0);

  /// 核心控制律：u = clamp(p + i + d, ±output_limit)
  double compute(double setpoint, double feedback, double dt);

  /// 重置积分与微分历史
  void reset();

  const PidParams & params() const { return params_; }

private:
  PidParams params_;
  double integral_{0.0};
  double prev_error_{0.0};
};

}  // namespace ylr1d_algorithm_sim

#endif  // YLR1D_ALGORITHM_SIM__CONTROLLER__PID__PID_HPP_
