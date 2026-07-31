#include "ylr1d_position_simulate/nodes/chassis_simulate_node.hpp"
#include "ylr1d_position_simulate/params/param_reader.hpp"

#include <memory>
#include <vector>

namespace ylr1d_position_simulate {

ChassisSimulateNode::ChassisSimulateNode() : Node("chassis_simulate") {
  double loop_hz = declare_parameter("loop_hz", 100.0);
  dt_ = 1.0 / loop_hz;

  // ── 转向（位置接口，4 关节，独立 PID + 限位） ──
  const std::vector<std::string> steering_names = {
    "Joint_Base_to_RFWheelF", "Joint_Base_to_LFWheelF",
    "Joint_Base_to_RBWheelF", "Joint_Base_to_LBWheelF"};
  std::vector<JointParams> steering_params;
  for (const auto & n : steering_names) {
    steering_params.push_back(read_joint_params(*this, n, defaultPositionParams()));
  }
  steering_.setup(steering_names, steering_params,
                  "/chassis_steering_controller/commands", this);

  // ── 轮子（速度接口，4 关节，独立 PID + 速度/加速度限幅） ──
  const std::vector<std::string> wheel_names = {
    "Joint_RFWheelF_to_RFWheel", "Joint_LFWheelF_to_LFWheel",
    "Joint_RBWheelF_to_RBWheel", "Joint_LBWheelF_to_LBWheel"};
  std::vector<JointParams> wheel_params;
  for (const auto & n : wheel_names) {
    wheel_params.push_back(read_joint_params(*this, n, defaultVelocityParams()));
  }
  wheels_.setup(wheel_names, wheel_params,
                "/chassis_wheels_controller/commands", this);

  // 订阅
  desired_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "desired_joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { desired_callback(msg); });

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { init_callback(msg); });

  sim_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("simulated_chassis_states", 10);

  timer_ = create_wall_timer(
    std::chrono::duration<double>(dt_),
    [this]() { update(); });

  RCLCPP_INFO(get_logger(), "chassis_simulate started (%.1f Hz, %zu steering + %zu wheels)",
              loop_hz, 4ul, 4ul);
}

void ChassisSimulateNode::init_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (initialized_) return;
  steering_.init_from(*msg);
  if (steering_.initialized()) {
    initialized_ = true;
    RCLCPP_INFO(get_logger(), "Chassis initialized from /joint_states");
    joint_state_sub_.reset();
  }
}

void ChassisSimulateNode::desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  steering_.set_desired(*msg);
  wheels_.set_desired(*msg);
}

void ChassisSimulateNode::update() {
  if (!initialized_) return;
  steering_.update(dt_);  steering_.publish();
  wheels_.update(dt_);    wheels_.publish();

  auto state = sensor_msgs::msg::JointState();
  state.header.stamp = now();
  steering_.fill_state_msg(state);
  wheels_.fill_state_msg(state);
  sim_state_pub_->publish(state);
}

}  // namespace ylr1d_position_simulate
