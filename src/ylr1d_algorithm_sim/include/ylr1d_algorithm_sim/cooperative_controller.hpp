#ifndef YLR1D_ALGORITHM_SIM__COOPERATIVE_CONTROLLER_HPP_
#define YLR1D_ALGORITHM_SIM__COOPERATIVE_CONTROLLER_HPP_

#include <Eigen/Dense>

#include <cstddef>

namespace ylr1d_algorithm_sim {

/// 协同控制器（架构层虚基类，组级）。
///
/// "是什么"：把一组关节的组期望 → 组内各关节的局部期望（setpoint）向量。
/// "怎么做"（P=1 比例透传、动力学补偿、差速/双臂分配…）是实现，在
/// src/control_law/ 与各组的具名协同控制器（src/<组>.cpp）中，改算法只改 src。
/// 本接口与节点框架不动。
class CooperativeController {
public:
  virtual ~CooperativeController() = default;

  /// 装配时告知组内关节数（实现按需分配内部状态向量）。
  virtual void set_group_size(std::size_t count) = 0;

  /// 初始化（清空内部状态）。
  virtual void initialize() = 0;

  /// 组期望向量 → 逐关节 setpoint 向量（尺寸与输入一致）。
  virtual Eigen::VectorXd compute(const Eigen::VectorXd & group_desired) = 0;
};

}  // namespace ylr1d_algorithm_sim

#endif  // YLR1D_ALGORITHM_SIM__COOPERATIVE_CONTROLLER_HPP_
