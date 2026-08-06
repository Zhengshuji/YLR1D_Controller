#include "ylr1d_control/groups/group_forwarder.hpp"

#include <cmath>

namespace ylr1d_control {

namespace {

/// 从 JointState 按关节名取 position（is_position=true）或 velocity 字段之一。
double sample(const sensor_msgs::msg::JointState & msg, const std::string & name,
              bool is_position) {
  const auto & vec = is_position ? msg.position : msg.velocity;
  for (size_t i = 0; i < msg.name.size() && i < vec.size(); ++i) {
    if (msg.name[i] == name) {
      double v = vec[i];
      return std::isnan(v) ? 0.0 : v;
    }
  }
  return 0.0;
}

}  // namespace

void GroupForwarder::setup(const ylr1d_algorithm_sim::JointGroupDef * def,
                           const std::string & cmd_topic, rclcpp::Node * node) {
  def_ = def;
  desired_.assign(def_->count, 0.0);
  output_.assign(def_->count, 0.0);
  const std::string base = "/ctrl/" + std::string(def_->name);
  desired_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(base + "/desired", 10);
  feedback_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(base + "/feedback", 10);
  cmd_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(cmd_topic, 10);
}

void GroupForwarder::set_desired(const sensor_msgs::msg::JointState & msg) {
  if (!def_) return;
  for (size_t i = 0; i < def_->count; ++i) {
    desired_[i] = sample(msg, def_->joints[i], def_->is_position);
  }
  have_desired_ = true;
}

void GroupForwarder::on_output(const sensor_msgs::msg::JointState & msg) {
  if (!def_) return;
  for (size_t i = 0; i < def_->count; ++i) {
    output_[i] = sample(msg, def_->joints[i], def_->is_position);
  }
  have_output_ = true;
}

void GroupForwarder::publish() {
  if (!def_) return;

  // 组期望（无条件发：算法层节点用首帧期望初始化）
  auto d = sensor_msgs::msg::JointState();
  for (size_t i = 0; i < def_->count; ++i) {
    d.name.push_back(def_->joints[i]);
    if (def_->is_position) {
      d.position.push_back(desired_[i]);
    } else {
      d.velocity.push_back(desired_[i]);
    }
  }
  desired_pub_->publish(d);

  // 组反馈 + 命令：软仿真闭环 = 仿真输出采样保持
  if (!have_output_) return;
  auto f = sensor_msgs::msg::JointState();
  for (size_t i = 0; i < def_->count; ++i) {
    f.name.push_back(def_->joints[i]);
    if (def_->is_position) {
      f.position.push_back(output_[i]);
    } else {
      f.velocity.push_back(output_[i]);
    }
  }
  feedback_pub_->publish(f);

  auto cmd = std_msgs::msg::Float64MultiArray();
  cmd.data = output_;
  cmd_pub_->publish(cmd);
}

void GroupForwarder::fill_state_msg(sensor_msgs::msg::JointState & msg) const {
  if (!def_) return;
  for (size_t i = 0; i < def_->count; ++i) {
    msg.name.push_back(def_->joints[i]);
    // 并行数组等长约定：position / velocity 均填充（速度组 position 恒 0）
    msg.position.push_back(def_->is_position ? output_[i] : 0.0);
    msg.velocity.push_back(def_->is_position ? 0.0 : output_[i]);
    msg.effort.push_back(0.0);
  }
}

}  // namespace ylr1d_control
