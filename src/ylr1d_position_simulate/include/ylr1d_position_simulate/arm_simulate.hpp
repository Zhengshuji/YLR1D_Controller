#ifndef YLR1D_POSITION_SIMULATE__ARM_SIMULATE_HPP_
#define YLR1D_POSITION_SIMULATE__ARM_SIMULATE_HPP_

#include "ylr1d_position_simulate/joint_group.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

namespace ylr1d_position_simulate {

/// 机械臂模拟节点
/// 躯干(4) + 左臂(9) + 右臂(9)，每个为独立 PositionJointGroup
class ArmSimulateNode : public rclcpp::Node {
public:
  ArmSimulateNode();

private:
  void init_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void update();

  double dt_{0.01};
  PositionJointGroup torso_;
  PositionJointGroup left_arm_;
  PositionJointGroup right_arm_;
  bool initialized_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ylr1d_position_simulate

#endif  // YLR1D_POSITION_SIMULATE__ARM_SIMULATE_HPP_
