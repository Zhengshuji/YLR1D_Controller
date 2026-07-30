#include "ylr1d_position_simulate/joint_group.hpp"
#include <algorithm>

namespace ylr1d_position_simulate {

// ===================================================================
// PositionJointGroup
// ===================================================================

void PositionJointGroup::setup(const std::vector<std::string> & names,
                                const PID & pid,
                                const std::string & topic,
                                rclcpp::Node * node) {
  joints_.reserve(names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    Joint j;
    j.name = names[i];
    j.pid = pid;
    joints_.push_back(j);
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
      if (!j.initialized && i < msg.position.size()) {
        j.position = std::isnan(msg.position[i]) ? 0.0 : msg.position[i];
        j.desired = j.position;
        j.initialized = true;
      }
    }
  }

  for (auto & j : joints_) if (!j.initialized) return;
  initialized_ = true;
}

void PositionJointGroup::set_desired(const sensor_msgs::msg::JointState & msg) {
  for (size_t i = 0; i < msg.name.size(); ++i) {
    auto it = name_to_idx_.find(msg.name[i]);
    if (it != name_to_idx_.end() && i < msg.position.size()) {
      double val = msg.position[i];
      // 输入限幅
      if (it->second < lower_limits_.size()) {
        val = std::clamp(val, lower_limits_[it->second], upper_limits_[it->second]);
      }
      joints_[it->second].desired = val;
    }
  }
}

void PositionJointGroup::set_limits(const std::vector<std::pair<double, double>> & limits) {
  lower_limits_.resize(joints_.size(), 0.0);
  upper_limits_.resize(joints_.size(), 0.0);
  for (size_t i = 0; i < limits.size() && i < joints_.size(); ++i) {
    lower_limits_[i] = limits[i].first;
    upper_limits_[i] = limits[i].second;
  }
}

void PositionJointGroup::update(double dt) {
  if (!initialized_) return;
  for (size_t i = 0; i < joints_.size(); ++i) {
    auto & j = joints_[i];
    double error = j.desired - j.position;
    double accel = j.pid.compute(error, dt);
    j.velocity += accel * dt;
    j.velocity = std::clamp(j.velocity, -j.pid.max_vel_, j.pid.max_vel_);
    j.position += j.velocity * dt;
    // 输出限幅
    if (i < lower_limits_.size()) {
      j.position = std::clamp(j.position, lower_limits_[i], upper_limits_[i]);
    }
  }
}

void PositionJointGroup::publish() {
  if (!initialized_) return;
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data.reserve(joints_.size());
  for (auto & j : joints_) msg.data.push_back(j.position);
  pub_->publish(msg);
}

void PositionJointGroup::fill_state_msg(sensor_msgs::msg::JointState & msg) const {
  for (auto & j : joints_) {
    msg.name.push_back(j.name);
    msg.position.push_back(j.position);
    msg.velocity.push_back(j.velocity);
    msg.effort.push_back(0.0);
  }
}

// ===================================================================
// VelocityJointGroup
// ===================================================================

void VelocityJointGroup::setup(const std::vector<std::string> & names,
                                const PID & pid,
                                const std::string & topic,
                                rclcpp::Node * node) {
  joints_.reserve(names.size());
  for (size_t i = 0; i < names.size(); ++i) {
    Joint j;
    j.name = names[i];
    j.pid = pid;
    joints_.push_back(j);
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
    j.velocity = std::clamp(j.velocity, -j.pid.max_vel_, j.pid.max_vel_);
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
