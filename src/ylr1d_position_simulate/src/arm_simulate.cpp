#include "ylr1d_position_simulate/arm_simulate.hpp"
#include <memory>

namespace ylr1d_position_simulate {

ArmSimulateNode::ArmSimulateNode() : Node("arm_simulate") {
  double loop_hz = declare_parameter("loop_hz", 100.0);
  double kp = declare_parameter("pid/kp", 5.0);
  double ki = declare_parameter("pid/ki", 0.0);
  double kd = declare_parameter("pid/kd", 0.1);
  double max_accel = declare_parameter("pid/max_accel", 50.0);
  double max_vel = declare_parameter("pid/max_vel", 3.0);

  dt_ = 1.0 / loop_hz;
  PID pid(kp, ki, kd, max_accel, max_vel);

  // 三个 PositionJointGroup，结构完全一致
  torso_.setup(
    {"Joint_Base_to_Body1", "Joint_Body1_to_Body2",
     "Joint_Body2_to_Body3", "Joint_Body3_to_Body4"},
    pid, "/torso_controller/commands", this);

  left_arm_.setup(
    {"Joint_Body2_to_LeftArm1", "Joint_LeftArm1_to_LeftArm2",
     "Joint_LeftArm2_to_LeftArm3", "Joint_LeftArm3_to_LeftArm4",
     "Joint_LeftArm4_to_LeftArm5", "Joint_LeftArm5_to_LeftArm6",
     "Joint_LeftArm6_to_LeftArm7", "Joint_LeftArm7_to_LeftFinger1",
     "Joint_LeftArm7_to_LeftFinger2"},
    pid, "/left_arm_controller/commands", this);

  right_arm_.setup(
    {"Joint_Body2_RightArm1", "Joint_RightArm1_to_RightArm2",
     "Joint_RightArm2_to_RightArm3", "Joint_RightArm3_to_RightArm4",
     "Joint_RightArm4_to_RightArm5", "Joint_RightArm5_to_RightArm6",
     "Joint_RightArm6_to_RightArm7", "Joint_RightArm7_to_RightFinger1",
     "Joint_RightArm7_to_RightFinger2"},
    pid, "/right_arm_controller/commands", this);

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

  RCLCPP_INFO(get_logger(), "arm_simulate started (%.1f Hz, %zu+%zu+%zu joints)",
              loop_hz, 4ul, 9ul, 9ul);
}

void ArmSimulateNode::init_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (initialized_) return;
  torso_.init_from(*msg);
  left_arm_.init_from(*msg);
  right_arm_.init_from(*msg);
  if (torso_.initialized() && left_arm_.initialized() && right_arm_.initialized()) {
    initialized_ = true;
    RCLCPP_INFO(get_logger(), "Arm joints initialized from /joint_states");
    joint_state_sub_.reset();
  }
}

void ArmSimulateNode::desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  torso_.set_desired(*msg);
  left_arm_.set_desired(*msg);
  right_arm_.set_desired(*msg);
}

void ArmSimulateNode::update() {
  if (!initialized_) return;
  torso_.update(dt_);   torso_.publish();
  left_arm_.update(dt_);  left_arm_.publish();
  right_arm_.update(dt_); right_arm_.publish();

  auto state = sensor_msgs::msg::JointState();
  state.header.stamp = now();
  torso_.fill_state_msg(state);
  left_arm_.fill_state_msg(state);
  right_arm_.fill_state_msg(state);
  sim_state_pub_->publish(state);
}

}  // namespace ylr1d_position_simulate

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ylr1d_position_simulate::ArmSimulateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
