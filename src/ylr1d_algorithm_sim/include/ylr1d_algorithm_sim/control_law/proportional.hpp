#ifndef YLR1D_ALGORITHM_SIM__CONTROL_LAW__PROPORTIONAL_HPP_
#define YLR1D_ALGORITHM_SIM__CONTROL_LAW__PROPORTIONAL_HPP_

#include <Eigen/Dense>

#include <cstddef>

namespace ylr1d_algorithm_sim {

/// 协同控制律（算法）：P 比例（默认 P=1，组期望逐元素透传为各关节 setpoint）。
///
/// 这是"怎么做"的实现细节（src/control_law/proportional.cpp），被各组具名
/// 协同控制器（src/<组>.cpp）内部持有；换协同算法在 src/control_law/ 新增实现
/// 并改对应具名类，接口与节点框架不动。
class ProportionalLaw {
public:
  ProportionalLaw() = default;
  explicit ProportionalLaw(double gain);

  /// 组级算法不需要按关节分配状态（纯比例透传），保留接口以便未来协同扩展。
  void set_group_size(std::size_t count);

  /// 清空内部状态（当前无状态）。
  void initialize();

  /// 组期望向量 → 逐关节 setpoint 向量（尺寸与输入一致）。
  Eigen::VectorXd compute(const Eigen::VectorXd & group_desired);

  double gain() const { return gain_; }
  void set_gain(double gain) { gain_ = gain; }

private:
  double gain_{1.0};
};

}  // namespace ylr1d_algorithm_sim

#endif  // YLR1D_ALGORITHM_SIM__CONTROL_LAW__PROPORTIONAL_HPP_
