#ifndef YLR1D_POSITION_SIMULATE__NODE_ARM_SIMULATE_HPP_
#define YLR1D_POSITION_SIMULATE__NODE_ARM_SIMULATE_HPP_

#include "ylr1d_position_simulate/joint_group.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <string>
#include <vector>

namespace ylr1d_position_simulate {

/// 机械臂组索引（躯干 / 左臂 / 右臂）
enum ArmGroup : size_t {
  TORSO = 0,
  LEFT_ARM = 1,
  RIGHT_ARM = 2,
  ARM_GROUP_COUNT = 3,
};

/// 机械臂模拟节点：躯干(4) + 左臂(9) + 右臂(9)，用数组 + 枚举统一管理
class Node_ArmSimulate : public rclcpp::Node {
public:
  Node_ArmSimulate();

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

}  // namespace ylr1d_position_simulate

#endif  // YLR1D_POSITION_SIMULATE__NODE_ARM_SIMULATE_HPP_
