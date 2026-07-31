#include "ylr1d_position_simulate/core/joint_simulator.hpp"

#include <algorithm>

namespace ylr1d_position_simulate {

void JointSimulator::configure(const JointParams & params) {
  params_ = params;
  pid_.set_gains(params.kp, params.ki, params.kd);
  pid_.set_limits(params.max_accel);
  pid_.reset();
}

void JointSimulator::initialize(double position, double velocity) {
  position_ = position;
  velocity_ = velocity;
  target_ = position;
  pid_.reset();
  initialized_ = true;
}

void JointSimulator::set_target(double target) {
  target_ = target;
}

double JointSimulator::update(double dt) {
  double error = target_ - position_;
  double accel = pid_.compute(error, dt);
  velocity_ += accel * dt;
  velocity_ = std::clamp(velocity_, -params_.max_vel, params_.max_vel);
  position_ += velocity_ * dt;
  if (params_.has_position_limit) {
    position_ = std::clamp(position_, params_.lower, params_.upper);
  }
  return position_;
}

}  // namespace ylr1d_position_simulate
