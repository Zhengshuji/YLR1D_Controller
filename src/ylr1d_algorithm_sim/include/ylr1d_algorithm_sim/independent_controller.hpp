#ifndef YLR1D_ALGORITHM_SIM__INDEPENDENT_CONTROLLER_HPP_
#define YLR1D_ALGORITHM_SIM__INDEPENDENT_CONTROLLER_HPP_

namespace ylr1d_algorithm_sim {

/// 独立控制器（架构层虚基类，逐关节）。
///
/// "是什么"：单关节局部期望 setpoint / 反馈 feedback → 控制量 u。
/// "怎么做"（PID、LQR、滑模…）是实现，在 src/control_law/ 与各组的具名
/// 逐关节控制器（src/<组>.cpp）中，改算法只改 src。本接口与节点框架不动。
class IndependentController {
public:
  virtual ~IndependentController() = default;

  /// 初始化（清空内部历史状态）。
  virtual void initialize() = 0;

  /// 单关节控制律：setpoint 与 feedback → 控制量 u。
  virtual double compute(double setpoint, double feedback, double dt) = 0;
};

}  // namespace ylr1d_algorithm_sim

#endif  // YLR1D_ALGORITHM_SIM__INDEPENDENT_CONTROLLER_HPP_
