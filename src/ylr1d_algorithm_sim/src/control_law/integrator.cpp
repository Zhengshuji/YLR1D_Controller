#include "ylr1d_algorithm_sim/control_law/integrator.hpp"

#include <algorithm>

namespace ylr1d_algorithm_sim {

// ── 标量积分器 ──

void PositionIntegrator::configure(const IntegratorParams & params) {
  params_ = params;
}

void PositionIntegrator::initialize(double position, double velocity) {
  position_ = position;
  velocity_ = velocity;
  initialized_ = true;
}

double PositionIntegrator::update(double u, double dt) {
  velocity_ += u * dt;
  velocity_ = std::clamp(velocity_, -params_.max_vel, params_.max_vel);

  position_ += velocity_ * dt;
  if (params_.has_position_limit) {
    position_ = std::clamp(position_, params_.lower, params_.upper);
  }
  return position_;
}

void VelocityIntegrator::configure(const IntegratorParams & params) {
  params_ = params;
}

void VelocityIntegrator::initialize(double velocity) {
  velocity_ = velocity;
  initialized_ = true;
}

double VelocityIntegrator::update(double u, double dt) {
  velocity_ += u * dt;
  velocity_ = std::clamp(velocity_, -params_.max_vel, params_.max_vel);
  return velocity_;
}

// ── 组级积分器（整体输入/整体输出，Eigen 向量） ──

void PositionIntegratorGroup::configure(
    const std::vector<IntegratorParams> & params, bool bypass) {
  units_.clear();
  units_.reserve(params.size());
  for (const auto & p : params) {
    PositionIntegrator unit;
    unit.configure(p);
    units_.push_back(std::move(unit));
  }
  position_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(params.size()));
  velocity_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(params.size()));
  bypass_ = bypass;
  initialized_ = false;
}

void PositionIntegratorGroup::initialize(const Eigen::VectorXd & position) {
  position_ = position;
  velocity_ = Eigen::VectorXd::Zero(position.size());
  for (std::size_t i = 0; i < units_.size(); ++i) {
    units_[i].initialize(position_[static_cast<Eigen::Index>(i)],
                         velocity_[static_cast<Eigen::Index>(i)]);
  }
  initialized_ = true;
}

Eigen::VectorXd PositionIntegratorGroup::update(const Eigen::VectorXd & u,
                                                double dt) {
  if (bypass_) return u;
  for (std::size_t i = 0; i < units_.size(); ++i) {
    position_[static_cast<Eigen::Index>(i)] =
        units_[i].update(u[static_cast<Eigen::Index>(i)], dt);
    velocity_[static_cast<Eigen::Index>(i)] = units_[i].velocity();
  }
  return position_;
}

void VelocityIntegratorGroup::configure(
    const std::vector<IntegratorParams> & params, bool bypass) {
  units_.clear();
  units_.reserve(params.size());
  for (const auto & p : params) {
    VelocityIntegrator unit;
    unit.configure(p);
    units_.push_back(std::move(unit));
  }
  velocity_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(params.size()));
  bypass_ = bypass;
  initialized_ = false;
}

void VelocityIntegratorGroup::initialize(const Eigen::VectorXd & velocity) {
  velocity_ = velocity;
  for (std::size_t i = 0; i < units_.size(); ++i) {
    units_[i].initialize(velocity_[static_cast<Eigen::Index>(i)]);
  }
  initialized_ = true;
}

Eigen::VectorXd VelocityIntegratorGroup::update(const Eigen::VectorXd & u,
                                                double dt) {
  if (bypass_) return u;
  for (std::size_t i = 0; i < units_.size(); ++i) {
    velocity_[static_cast<Eigen::Index>(i)] =
        units_[i].update(u[static_cast<Eigen::Index>(i)], dt);
  }
  return velocity_;
}

}  // namespace ylr1d_algorithm_sim
