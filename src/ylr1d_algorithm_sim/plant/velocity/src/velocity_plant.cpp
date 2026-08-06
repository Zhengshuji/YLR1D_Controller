#include "plant/velocity/velocity_plant.hpp"

#include <algorithm>

namespace ylr1d_algorithm_sim {

void VelocityPlant::configure(const VelocityPlantParams & params) {
  params_ = params;
}

void VelocityPlant::initialize(double velocity) {
  velocity_ = velocity;
  initialized_ = true;
}

double VelocityPlant::update(double u, double dt) {
  if (params_.bypass) return u;

  velocity_ += u * dt;
  velocity_ = std::clamp(velocity_, -params_.max_vel, params_.max_vel);
  return velocity_;
}

}  // namespace ylr1d_algorithm_sim
