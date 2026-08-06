#include "ylr1d_algorithm_sim/wheel.hpp"

namespace ylr1d_algorithm_sim {

// ── 协同控制器：组期望 → 逐关节 setpoint（内部算法：P 比例，control_law/proportional） ──

void WheelCooperativeController::set_group_size(std::size_t count) {
  law_.set_group_size(count);
}

void WheelCooperativeController::initialize() {
  law_.initialize();
}

Eigen::VectorXd WheelCooperativeController::compute(
    const Eigen::VectorXd & group_desired) {
  return law_.compute(group_desired);
}

// ── 逐关节控制器：setpoint + feedback → u（内部算法：PID，control_law/pid） ──

void WheelJointController::configure(const PidParams & params) {
  law_.configure(params);
}

void WheelJointController::initialize() {
  law_.initialize();
}

double WheelJointController::compute(double setpoint, double feedback,
                                     double dt) {
  return law_.update(setpoint, feedback, dt);
}

// ── 仿真被控对象：速度型 1/s（内部模型：速度积分器组，control_law/integrator） ──

void WheelPlant::configure(const std::vector<JointLimit> & limits,
                           bool bypass) {
  // 轮子无位置限位（连续旋转），仅取速度限幅。
  std::vector<IntegratorParams> params(limits.size());
  for (std::size_t i = 0; i < limits.size(); ++i) {
    params[i].max_vel = limits[i].max_vel;
  }
  engine_.configure(params, bypass);
}

std::size_t WheelPlant::size() const { return engine_.size(); }

void WheelPlant::initialize(const Eigen::VectorXd & state) {
  engine_.initialize(state);
}

Eigen::VectorXd WheelPlant::update(const Eigen::VectorXd & input, double dt) {
  return engine_.update(input, dt);
}

const Eigen::VectorXd & WheelPlant::state() const {
  return engine_.velocity();
}

const Eigen::VectorXd & WheelPlant::velocity() const {
  return engine_.velocity();
}

bool WheelPlant::initialized() const { return engine_.initialized(); }

bool WheelPlant::bypass() const { return engine_.bypass(); }

}  // namespace ylr1d_algorithm_sim
