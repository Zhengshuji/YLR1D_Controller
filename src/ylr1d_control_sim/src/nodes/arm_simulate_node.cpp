#include "ylr1d_control_sim/nodes/arm_simulate_node.hpp"
#include "ylr1d_control_sim/params/param_reader.hpp"

#include <memory>
#include <string>
#include <vector>

namespace ylr1d_control_sim {

ArmSimulateNode::ArmSimulateNode() : Node("arm_simulate") {
  double loop_hz = declare_parameter("loop_hz", 100.0);
  dt_ = 1.0 / loop_hz;

  // 数组 + 枚举统一配置三组（躯干 / 左臂 / 右臂），组定义取自 config/joint_config.hpp
  for (size_t g = 0; g < ARM_GROUP_COUNT; ++g) {
    const ArmGroupDef & def = kArmGroups[g];
    std::vector<std::string> names(def.joints, def.joints + def.count);
    std::vector<JointParams> params;
    for (const auto & n : names) {
      params.push_back(read_joint_params(*this, n, defaultPositionParams()));
    }
    groups_[g].setup(names, params, def.topic, this);
  }

  // 订阅
  desired_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "desired_joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { desired_callback(msg); });

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { init_callback(msg); });

  sim_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("simulated_arm_states", 10);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(dt_),
    [this]() { update(); });

  RCLCPP_INFO(get_logger(), "arm_simulate started (%.1f Hz, %zu groups, %zu joints)",
              loop_hz, ARM_GROUP_COUNT,
              kTorsoJointCount + kLeftArmJointCount + kRightArmJointCount);
}

void ArmSimulateNode::init_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (initialized_) return;
  bool all = true;
  for (auto & g : groups_) {
    g.init_from(*msg);
    all = all && g.initialized();
  }
  if (all) {
    initialized_ = true;
    RCLCPP_INFO(get_logger(), "Arm joints initialized from /joint_states");
    joint_state_sub_.reset();
  }
}

void ArmSimulateNode::desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (auto & g : groups_) {
    g.set_desired(*msg);
  }
}

void ArmSimulateNode::update() {
  if (!initialized_) return;
  for (auto & g : groups_) {
    g.update(dt_);
    g.publish();
  }

  auto state = sensor_msgs::msg::JointState();
  state.header.stamp = now();
  for (auto & g : groups_) {
    g.fill_state_msg(state);
  }
  sim_state_pub_->publish(state);
}

}  // namespace ylr1d_control_sim
