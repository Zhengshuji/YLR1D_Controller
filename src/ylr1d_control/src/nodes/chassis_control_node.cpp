#include "ylr1d_control/nodes/chassis_control_node.hpp"

#include "algorithm/config/joint_config.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

namespace ylr1d_control {

ChassisControlNode::ChassisControlNode() : Node("chassis_control") {
  double loop_hz = declare_parameter("loop_hz", 100.0);
  dt_ = 1.0 / loop_hz;

  // 两组：steering(位置接口) + wheels(速度接口)（组定义来自算法层 joint_config.hpp）
  struct GroupSpec {
    const char * name;
    const char * cmd_topic;
  };
  constexpr GroupSpec kSpecs[2] = {
      {"steering", ylr1d_algorithm_sim::kSteeringTopic},
      {"wheels",   ylr1d_algorithm_sim::kWheelTopic},
  };

  for (size_t g = 0; g < 2; ++g) {
    const auto * def = ylr1d_algorithm_sim::jointGroupFor(kSpecs[g].name);
    if (!def) {
      RCLCPP_FATAL(get_logger(), "unknown group '%s'", kSpecs[g].name);
      throw std::runtime_error("invalid control group");
    }
    groups_[g].setup(def, kSpecs[g].cmd_topic, this);
    output_subs_[g] = create_subscription<sensor_msgs::msg::JointState>(
        "/ctrl/" + std::string(kSpecs[g].name) + "/output", 10,
        [this, g](const sensor_msgs::msg::JointState::SharedPtr msg) {
          groups_[g].on_output(*msg);
        });
  }

  desired_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "desired_joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { desired_callback(msg); });

  sim_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("simulated_chassis_states", 10);

  timer_ = create_wall_timer(std::chrono::duration<double>(dt_),
                             [this]() { update(); });

  RCLCPP_INFO(get_logger(),
              "chassis_control (sampler/forwarder) started (%.1f Hz, steering + wheels)",
              loop_hz);
}

void ChassisControlNode::desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (auto & g : groups_) g.set_desired(*msg);
}

void ChassisControlNode::update() {
  for (auto & g : groups_) g.publish();

  auto state = sensor_msgs::msg::JointState();
  state.header.stamp = now();
  for (auto & g : groups_) g.fill_state_msg(state);
  sim_state_pub_->publish(state);
}

}  // namespace ylr1d_control
