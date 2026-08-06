#include "controller/pid/pid.hpp"

#include <algorithm>

namespace ylr1d_algorithm_sim {

void PidController::configure(const PidParams & params) {
  params_ = params;
  reset();
}

void PidController::initialize(double) {
  reset();
}

double PidController::compute(double setpoint, double feedback, double dt) {
  double error = setpoint - feedback;

  double p_term = params_.kp * error;

  integral_ += error * dt;
  double i_term = params_.ki * integral_;

  double derivative = (error - prev_error_) / dt;
  double d_term = params_.kd * derivative;

  prev_error_ = error;

  double u = p_term + i_term + d_term;
  if (params_.output_limit > 0.0) {
    u = std::clamp(u, -params_.output_limit, params_.output_limit);
  }
  return u;
}

void PidController::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
}

}  // namespace ylr1d_algorithm_sim
