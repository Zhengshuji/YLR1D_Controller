#include "ylr1d_control/nodes/arm_control_node.hpp"

#include "algorithm/config/joint_config.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

namespace ylr1d_control {

ArmControlNode::ArmControlNode() : Node("arm_control") {
  double loop_hz = declare_parameter("loop_hz", 100.0);
  dt_ = 1.0 / loop_hz;

  // 三组：torso / left_arm / right_arm（组定义来自算法层 joint_config.hpp，单一来源）
  struct GroupSpec {
    const char * name;
    const char * cmd_topic;
  };
  constexpr GroupSpec kSpecs[3] = {
      {"torso",     ylr1d_algorithm_sim::kTorsoTopic},
      {"left_arm",  ylr1d_algorithm_sim::kLeftArmTopic},
      {"right_arm", ylr1d_algorithm_sim::kRightArmTopic},
  };

  for (size_t g = 0; g < 3; ++g) {
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

  sim_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("simulated_arm_states", 10);

  timer_ = create_wall_timer(std::chrono::duration<double>(dt_),
                             [this]() { update(); });

  RCLCPP_INFO(get_logger(), "arm_control (sampler/forwarder) started (%.1f Hz, 3 groups)",
              loop_hz);
}

void ArmControlNode::desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (auto & g : groups_) g.set_desired(*msg);
}

void ArmControlNode::update() {
  for (auto & g : groups_) g.publish();

  auto state = sensor_msgs::msg::JointState();
  state.header.stamp = now();
  for (auto & g : groups_) g.fill_state_msg(state);
  sim_state_pub_->publish(state);
}

}  // namespace ylr1d_control
