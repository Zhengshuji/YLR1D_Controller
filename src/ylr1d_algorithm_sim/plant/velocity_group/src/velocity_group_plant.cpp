#include "plant/velocity_group/velocity_group_plant.hpp"

namespace ylr1d_algorithm_sim {

void VelocityGroupPlant::configure(
    const std::vector<VelocityPlantParams> & params) {
  plants_.clear();
  plants_.reserve(params.size());
  for (const auto & p : params) {
    VelocityPlant plant;
    plant.configure(p);
    plants_.push_back(std::move(plant));
  }
  velocity_ = Eigen::VectorXd::Zero(params.size());
  bypass_ = !params.empty() && params[0].bypass;
  initialized_ = false;
}

void VelocityGroupPlant::initialize(const Eigen::VectorXd & velocity) {
  velocity_ = velocity;
  for (std::size_t i = 0; i < plants_.size(); ++i) {
    plants_[i].initialize(velocity_[i]);
  }
  initialized_ = true;
}

Eigen::VectorXd VelocityGroupPlant::update(const Eigen::VectorXd & u,
                                           double dt) {
  if (bypass_) return u;
  for (std::size_t i = 0; i < plants_.size(); ++i) {
    velocity_[i] = plants_[i].update(u[i], dt);
  }
  return velocity_;
}

}  // namespace ylr1d_algorithm_sim
