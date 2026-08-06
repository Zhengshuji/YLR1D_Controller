#ifndef CONTROLLER__PROPORTIONAL__PROPORTIONAL_CONTROLLER_HPP_
#define CONTROLLER__PROPORTIONAL__PROPORTIONAL_CONTROLLER_HPP_

#include <Eigen/Dense>

namespace ylr1d_algorithm_sim {

/// 组级协同控制器：当前实现为 P=1 比例（透传）。
/// 输入组期望向量，输出组内各关节的局部期望（setpoint）向量。
/// P=1 时逐元素恒等透传；未来协同（动力学补偿、差速分配、双臂协同等）
/// 在"组期望 → 逐关节 setpoint"这一层扩展，不改变独立控制器与仿真对象接口。
class ProportionalController {
public:
  ProportionalController() = default;

  /// 设置比例增益（默认 1.0，即透传）。可重复调用（支持热更新）。
  void configure(double gain = 1.0);

  /// 清空内部状态（当前无状态，为风格一致保留）。
  void initialize();

  /// 组期望 → 逐关节局部期望向量（尺寸与输入一致）。
  Eigen::VectorXd compute(const Eigen::VectorXd & group_desired) const;

  double gain() const { return gain_; }

private:
  double gain_{1.0};
};

}  // namespace ylr1d_algorithm_sim

#endif  // CONTROLLER__PROPORTIONAL__PROPORTIONAL_CONTROLLER_HPP_
