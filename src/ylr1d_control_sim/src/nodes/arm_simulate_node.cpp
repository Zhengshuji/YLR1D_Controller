#include "ylr1d_control_sim/nodes/arm_simulate_node.hpp"
#include "ylr1d_control_sim/params/param_reader.hpp"

#include <memory>
#include <vector>

namespace ylr1d_control_sim {

namespace {

/// 每组的位置关节名 + 控制器命令话题（顺序与 enum ArmGroup 一致）
struct ArmGroupSpec {
  const char * topic;
  std::vector<std::string> joints;
};

const std::array<ArmGroupSpec, ARM_GROUP_COUNT> kArmGroupSpecs = {{
  {"/torso_controller/commands",
   {"Joint_Base_to_Body1", "Joint_Body1_to_Body2",
    "Joint_Body2_to_Body3", "Joint_Body3_to_Body4"}},
  {"/left_arm_controller/commands",
   {"Joint_Body2_to_LeftArm1", "Joint_LeftArm1_to_LeftArm2",
    "Joint_LeftArm2_to_LeftArm3", "Joint_LeftArm3_to_LeftArm4",
    "Joint_LeftArm4_to_LeftArm5", "Joint_LeftArm5_to_LeftArm6",
    "Joint_LeftArm6_to_LeftArm7", "Joint_LeftArm7_to_LeftFinger1",
    "Joint_LeftArm7_to_LeftFinger2"}},
  {"/right_arm_controller/commands",
   {"Joint_Body2_to_RightArm1", "Joint_RightArm1_to_RightArm2",
    "Joint_RightArm2_to_RightArm3", "Joint_RightArm3_to_RightArm4",
    "Joint_RightArm4_to_RightArm5", "Joint_RightArm5_to_RightArm6",
    "Joint_RightArm6_to_RightArm7", "Joint_RightArm7_to_RightFinger1",
    "Joint_RightArm7_to_RightFinger2"}},
}};

}  // namespace

ArmSimulateNode::ArmSimulateNode() : Node("arm_simulate") {
  double loop_hz = declare_parameter("loop_hz", 100.0);
  dt_ = 1.0 / loop_hz;

  // 数组 + 枚举统一配置三组（躯干 / 左臂 / 右臂）
  for (size_t g = 0; g < ARM_GROUP_COUNT; ++g) {
    std::vector<JointParams> params;
    for (const auto & n : kArmGroupSpecs[g].joints) {
      params.push_back(read_joint_params(*this, n, defaultPositionParams()));
    }
    groups_[g].setup(kArmGroupSpecs[g].joints, params, kArmGroupSpecs[g].topic, this);
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
              loop_hz, ARM_GROUP_COUNT, 4ul + 9ul + 9ul);
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
