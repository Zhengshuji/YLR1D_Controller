#include "ylr1d_position_simulate/groups/joint_group.hpp"

#include <algorithm>
#include <cmath>

namespace ylr1d_position_simulate {

// ===================================================================
// PositionJointGroup
// ===================================================================

void PositionJointGroup::setup(const std::vector<std::string> & names,
                               const std::vector<JointParams> & params,
                               const std::string & topic,
                               rclcpp::Node * node) {
  joints_.reserve(names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    Joint j;
    j.name = names[i];
    j.sim.configure(i < params.size() ? params[i] : JointParams{});
    joints_.push_back(std::move(j));
    name_to_idx_[names[i]] = i;
  }
  pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
}

void PositionJointGroup::init_from(const sensor_msgs::msg::JointState & msg) {
  if (initialized_) return;

  for (size_t i = 0; i < msg.name.size(); ++i) {
    auto it = name_to_idx_.find(msg.name[i]);
    if (it != name_to_idx_.end()) {
      auto & j = joints_[it->second];
      if (!j.sim.initialized() && i < msg.position.size()) {
        double pos = std::isnan(msg.position[i]) ? 0.0 : msg.position[i];
        j.sim.initialize(pos, 0.0);
      }
    }
  }

  for (auto & j : joints_) if (!j.sim.initialized()) return;
  initialized_ = true;
}

void PositionJointGroup::set_desired(const sensor_msgs::msg::JointState & msg) {
  for (size_t i = 0; i < msg.name.size(); ++i) {
    auto it = name_to_idx_.find(msg.name[i]);
    if (it != name_to_idx_.end() && i < msg.position.size()) {
      joints_[it->second].sim.set_target(msg.position[i]);
    }
  }
}

void PositionJointGroup::update(double dt) {
  if (!initialized_) return;
  for (auto & j : joints_) {
    j.sim.update(dt);
  }
}

void PositionJointGroup::publish() {
  if (!initialized_) return;
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data.reserve(joints_.size());
  for (auto & j : joints_) msg.data.push_back(j.sim.position());
  pub_->publish(msg);
}

void PositionJointGroup::fill_state_msg(sensor_msgs::msg::JointState & msg) const {
  for (auto & j : joints_) {
    msg.name.push_back(j.name);
    msg.position.push_back(j.sim.position());
    msg.velocity.push_back(j.sim.velocity());
    msg.effort.push_back(0.0);
  }
}

// ===================================================================
// VelocityJointGroup
// ===================================================================

void VelocityJointGroup::setup(const std::vector<std::string> & names,
                               const std::vector<JointParams> & params,
                               const std::string & topic,
                               rclcpp::Node * node) {
  joints_.reserve(names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    Joint j;
    j.name = names[i];
    const JointParams & p = i < params.size() ? params[i] : JointParams{};
    j.pid = PID(p.kp, p.ki, p.kd, p.max_accel);
    j.max_accel = p.max_accel;
    j.max_vel = p.max_vel;
    joints_.push_back(std::move(j));
    name_to_idx_[names[i]] = i;
  }
  pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
}

void VelocityJointGroup::set_desired(const sensor_msgs::msg::JointState & msg) {
  for (size_t i = 0; i < msg.name.size(); ++i) {
    auto it = name_to_idx_.find(msg.name[i]);
    if (it != name_to_idx_.end() && i < msg.velocity.size()) {
      joints_[it->second].desired = msg.velocity[i];
    }
  }
}

void VelocityJointGroup::update(double dt) {
  for (auto & j : joints_) {
    double error = j.desired - j.velocity;
    double accel = j.pid.compute(error, dt);
    j.velocity += accel * dt;
    j.velocity = std::clamp(j.velocity, -j.max_vel, j.max_vel);
  }
}

void VelocityJointGroup::publish() {
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data.reserve(joints_.size());
  for (auto & j : joints_) msg.data.push_back(j.velocity);
  pub_->publish(msg);
}

void VelocityJointGroup::fill_state_msg(sensor_msgs::msg::JointState & msg) const {
  for (auto & j : joints_) {
    msg.name.push_back(j.name);
    msg.position.push_back(0.0);
    msg.velocity.push_back(j.velocity);
    msg.effort.push_back(0.0);
  }
}

}  // namespace ylr1d_position_simulate
