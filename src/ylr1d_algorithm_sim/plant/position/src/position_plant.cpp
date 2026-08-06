#include "plant/position/position_plant.hpp"

#include <algorithm>

namespace ylr1d_algorithm_sim {

void PositionPlant::configure(const PositionPlantParams & params) {
  params_ = params;
}

void PositionPlant::initialize(double position, double velocity) {
  position_ = position;
  velocity_ = velocity;
  initialized_ = true;
}

double PositionPlant::update(double u, double dt) {
  if (params_.bypass) return u;

  velocity_ += u * dt;
  velocity_ = std::clamp(velocity_, -params_.max_vel, params_.max_vel);

  position_ += velocity_ * dt;
  if (params_.has_position_limit) {
    position_ = std::clamp(position_, params_.lower, params_.upper);
  }
  return position_;
}

}  // namespace ylr1d_algorithm_sim
