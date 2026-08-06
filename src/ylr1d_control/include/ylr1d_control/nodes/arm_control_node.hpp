#ifndef YLR1D_CONTROL__NODES__ARM_CONTROL_NODE_HPP_
#define YLR1D_CONTROL__NODES__ARM_CONTROL_NODE_HPP_

#include "ylr1d_control/groups/group_forwarder.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>

namespace ylr1d_control {

/// 机械臂控制节点（采样保持器 + 通信节点）：躯干 / 左臂 / 右臂三组。
/// 控制层退化为"期望/反馈采样保持 + 命令转发"，不做任何算法计算；
/// 算法计算在算法层仿真控制器节点（ylr1d_algorithm_sim，composition）。
class ArmControlNode : public rclcpp::Node {
public:
  ArmControlNode();

private:
  void desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void update();

  double dt_{0.01};
  std::array<GroupForwarder, 3> groups_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr, 3> output_subs_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ylr1d_control

#endif  // YLR1D_CONTROL__NODES__ARM_CONTROL_NODE_HPP_
