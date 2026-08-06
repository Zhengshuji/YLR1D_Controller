#include "controller/proportional/proportional_controller.hpp"

namespace ylr1d_algorithm_sim {

void ProportionalController::configure(double gain) {
  gain_ = gain;
  initialize();
}

void ProportionalController::initialize() {
  // 当前无内部状态（纯比例透传）。
}

Eigen::VectorXd ProportionalController::compute(
    const Eigen::VectorXd & group_desired) const {
  return gain_ * group_desired;
}

}  // namespace ylr1d_algorithm_sim
