#include "ylr1d_algorithm_sim/control_law/proportional.hpp"

namespace ylr1d_algorithm_sim {

ProportionalLaw::ProportionalLaw(double gain) : gain_(gain) {}

void ProportionalLaw::set_group_size(std::size_t) {
  // P 比例透传不依赖组大小；未来带耦合的协同算法在此分配状态。
}

void ProportionalLaw::initialize() {
  // 当前无内部状态（纯比例透传）。
}

Eigen::VectorXd ProportionalLaw::compute(
    const Eigen::VectorXd & group_desired) {
  return gain_ * group_desired;
}

}  // namespace ylr1d_algorithm_sim
