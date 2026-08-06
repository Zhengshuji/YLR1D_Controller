#ifndef YLR1D_ALGORITHM_SIM__CONTROL_LAW__PID_HPP_
#define YLR1D_ALGORITHM_SIM__CONTROL_LAW__PID_HPP_

namespace ylr1d_algorithm_sim {

/// PID 控制律参数（来自 config/pid.yaml 的 <关节>/pid/*）。
struct PidParams {
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double output_limit{0.0};  // 输出限幅；<=0 表示不限幅
};

/// 独立控制律（算法）：PID。
///
/// 输入期望 setpoint 与反馈 feedback，输出控制量 u（u 的语义由被控对象解释，
/// 本律不绑定）。这是"怎么做"的实现细节（src/control_law/pid.cpp），被各组具名
/// 逐关节控制器（src/<组>.cpp）内部持有；换算法在 src/control_law/ 新增实现。
class PidLaw {
public:
  PidLaw() = default;

  /// 配置参数（可重复调用，支持运行时热更新）。
  void configure(const PidParams & params);

  /// 重置积分与微分历史。
  void initialize();

  /// 核心控制律：u = clamp(p + i + d, ±output_limit)。
  double update(double setpoint, double feedback, double dt);

  const PidParams & params() const { return params_; }

private:
  PidParams params_;
  double integral_{0.0};
  double prev_error_{0.0};
};

}  // namespace ylr1d_algorithm_sim

#endif  // YLR1D_ALGORITHM_SIM__CONTROL_LAW__PID_HPP_
