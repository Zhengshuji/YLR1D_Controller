#ifndef YLR1D_POSITION_SIMULATE__NODE_CHASSIS_SIMULATE_HPP_
#define YLR1D_POSITION_SIMULATE__NODE_CHASSIS_SIMULATE_HPP_

#include "ylr1d_position_simulate/joint_group.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace ylr1d_position_simulate {

/// 底盘模拟节点：转向(4, 位置接口) + 轮子(4, 速度接口)
class Node_ChassisSimulate : public rclcpp::Node {
public:
  Node_ChassisSimulate();

private:
  void init_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void update();

  double dt_{0.01};
  PositionJointGroup steering_;
  VelocityJointGroup wheels_;
  bool initialized_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ylr1d_position_simulate

#endif  // YLR1D_POSITION_SIMULATE__NODE_CHASSIS_SIMULATE_HPP_
