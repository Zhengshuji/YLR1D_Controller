#ifndef YLR1D_CONTROL_SIM__NODES__ARM_SIMULATE_NODE_HPP_
#define YLR1D_CONTROL_SIM__NODES__ARM_SIMULATE_NODE_HPP_

#include "ylr1d_control_sim/config/joint_config.hpp"
#include "ylr1d_control_sim/groups/joint_group.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>

namespace ylr1d_control_sim {

/// 机械臂模拟节点：躯干(4) + 左臂(9) + 右臂(9)，用数组 + 枚举统一管理
/// （enum ArmGroup / kArmGroups 定义于 config/joint_config.hpp，单一来源）
class ArmSimulateNode : public rclcpp::Node {
public:
  ArmSimulateNode();

private:
  void init_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void update();

  double dt_{0.01};
  std::array<PositionJointGroup, ARM_GROUP_COUNT> groups_;
  bool initialized_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ylr1d_control_sim

#endif  // YLR1D_CONTROL_SIM__NODES__ARM_SIMULATE_NODE_HPP_
