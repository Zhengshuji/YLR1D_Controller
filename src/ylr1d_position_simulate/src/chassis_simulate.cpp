#include "ylr1d_position_simulate/chassis_simulate.hpp"
#include <memory>

namespace ylr1d_position_simulate {

ChassisSimulateNode::ChassisSimulateNode() : Node("chassis_simulate") {
  double loop_hz = declare_parameter("loop_hz", 100.0);

  double skp = declare_parameter("steering/kp", 5.0);
  double ski = declare_parameter("steering/ki", 0.0);
  double skd = declare_parameter("steering/kd", 0.1);
  double s_accel = declare_parameter("steering/max_accel", 50.0);
  double s_vel   = declare_parameter("steering/max_vel", 3.0);

  double wkp = declare_parameter("wheel/kp", 2.0);
  double wki = declare_parameter("wheel/ki", 0.0);
  double wkd = declare_parameter("wheel/kd", 0.05);
  double w_accel = declare_parameter("wheel/max_accel", 20.0);
  double w_vel   = declare_parameter("wheel/max_vel", 5.0);

  dt_ = 1.0 / loop_hz;

  steering_.setup(
    {"Joint_Base_to_RFWheelF", "Joint_Base_to_LFWheelF",
     "Joint_Base_to_RBWheelF", "Joint_Base_to_LBWheelF"},
    PID(skp, ski, skd, s_accel, s_vel),
    "/chassis_steering_controller/commands", this);
  steering_.set_limits({{-3.14, 3.14}, {-3.14, 3.14}, {-3.14, 3.14}, {-3.14, 3.14}});

  wheels_.setup(
    {"Joint_RFWheelF_to_RFWheel", "Joint_LFWheelF_to_LFWheel",
     "Joint_RBWheelF_to_RBWheel", "Joint_LBWheelF_to_LBWheel"},
    PID(wkp, wki, wkd, w_accel, w_vel),
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

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ylr1d_position_simulate::ChassisSimulateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
