#include "plant/position_group/position_group_plant.hpp"

namespace ylr1d_algorithm_sim {

void PositionGroupPlant::configure(
    const std::vector<PositionPlantParams> & params) {
  plants_.clear();
  plants_.reserve(params.size());
  for (const auto & p : params) {
    PositionPlant plant;
    plant.configure(p);
    plants_.push_back(std::move(plant));
  }
  position_ = Eigen::VectorXd::Zero(params.size());
  velocity_ = Eigen::VectorXd::Zero(params.size());
  bypass_ = !params.empty() && params[0].bypass;
  initialized_ = false;
}

void PositionGroupPlant::initialize(const Eigen::VectorXd & position,
                                    const Eigen::VectorXd & velocity) {
  position_ = position;
  velocity_ = velocity.size() == position.size()
                  ? velocity
                  : Eigen::VectorXd::Zero(position.size());
  for (std::size_t i = 0; i < plants_.size(); ++i) {
    plants_[i].initialize(position_[i], velocity_[i]);
  }
  initialized_ = true;
}

Eigen::VectorXd PositionGroupPlant::update(const Eigen::VectorXd & u,
                                           double dt) {
  if (bypass_) return u;
  for (std::size_t i = 0; i < plants_.size(); ++i) {
    position_[i] = plants_[i].update(u[i], dt);
    velocity_[i] = plants_[i].velocity();
  }
  return position_;
}

}  // namespace ylr1d_algorithm_sim
