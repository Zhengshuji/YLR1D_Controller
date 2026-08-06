#include "ylr1d_algorithm_sim/torso.hpp"

namespace ylr1d_algorithm_sim {

// ── 协同控制器：组期望 → 逐关节 setpoint（内部算法：P 比例，control_law/proportional） ──

void TorsoCooperativeController::set_group_size(std::size_t count) {
  law_.set_group_size(count);
}

void TorsoCooperativeController::initialize() {
  law_.initialize();
}

Eigen::VectorXd TorsoCooperativeController::compute(
    const Eigen::VectorXd & group_desired) {
  return law_.compute(group_desired);
}

// ── 逐关节控制器：setpoint + feedback → u（内部算法：PID，control_law/pid） ──

void TorsoJointController::configure(const PidParams & params) {
  law_.configure(params);
}

void TorsoJointController::initialize() {
  law_.initialize();
}

double TorsoJointController::compute(double setpoint, double feedback,
                                     double dt) {
  return law_.update(setpoint, feedback, dt);
}

// ── 仿真被控对象：位置型 1/s²（内部模型：位置积分器组，control_law/integrator） ──

void TorsoPlant::configure(const std::vector<JointLimit> & limits,
                           bool bypass) {
  std::vector<IntegratorParams> params(limits.size());
  for (std::size_t i = 0; i < limits.size(); ++i) {
    params[i].max_vel = limits[i].max_vel;
    params[i].has_position_limit = limits[i].has_position_limit;
    params[i].lower = limits[i].lower;
    params[i].upper = limits[i].upper;
  }
  engine_.configure(params, bypass);
}

std::size_t TorsoPlant::size() const { return engine_.size(); }

void TorsoPlant::initialize(const Eigen::VectorXd & state) {
  engine_.initialize(state);
}

Eigen::VectorXd TorsoPlant::update(const Eigen::VectorXd & input, double dt) {
  return engine_.update(input, dt);
}

const Eigen::VectorXd & TorsoPlant::state() const {
  return engine_.position();
}

const Eigen::VectorXd & TorsoPlant::velocity() const {
  return engine_.velocity();
}

bool TorsoPlant::initialized() const { return engine_.initialized(); }

bool TorsoPlant::bypass() const { return engine_.bypass(); }

}  // namespace ylr1d_algorithm_sim
