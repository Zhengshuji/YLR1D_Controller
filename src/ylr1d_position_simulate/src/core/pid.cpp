#include "ylr1d_position_simulate/core/pid.hpp"

#include <algorithm>
#include <cmath>

namespace ylr1d_position_simulate {

PID::PID(double kp, double ki, double kd, double max_accel)
  : kp_(kp), kd_(kd), ki_(ki), max_accel_(max_accel) {}

double PID::compute(double error, double dt) {
  // 比例项
  double p_term = kp_ * error;

  // 积分项 (带抗饱和)
  integral_ += error * dt;
  double i_term = ki_ * integral_;

  // 微分项
  double derivative = (error - prev_error_) / dt;
  double d_term = kd_ * derivative;

  prev_error_ = error;

  double output = p_term + i_term + d_term;

  // 限幅 (加速度限幅)
  output = std::clamp(output, -max_accel_, max_accel_);

  return output;
}

void PID::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
}

void PID::set_gains(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PID::set_limits(double max_accel) {
  max_accel_ = max_accel;
}

}  // namespace ylr1d_position_simulate
