#ifndef YLR1D_CONTROL__GROUPS__GROUP_FORWARDER_HPP_
#define YLR1D_CONTROL__GROUPS__GROUP_FORWARDER_HPP_

#include "algorithm/config/joint_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <string>
#include <vector>

namespace ylr1d_control {

/// 控制层转发器：管理一组关节的「期望/反馈采样保持 + 仿真输出 + 命令下发」。
///
/// 控制层退化为采样保持器 + 通信节点后，不再做任何算法计算：
///   - 组期望（来自 /desired_joint_states 采样保持）→ 发布 /ctrl/<组>/desired
///   - 仿真输出（订阅 /ctrl/<组>/output 采样保持）→ 回发 /ctrl/<组>/feedback（软仿真闭环）
///   - 仿真输出 → 物理层命令话题（Float64MultiArray）
///
/// 组定义（关节名/类型/限位）来自算法层 joint_config.hpp（单一来源）。
class GroupForwarder {
public:
  GroupForwarder() = default;

  /// @param def       组定义（算法层 kJointGroups 之一）
  /// @param cmd_topic 物理层命令话题（如 /left_arm_controller/commands）
  void setup(const ylr1d_algorithm_sim::JointGroupDef * def,
             const std::string & cmd_topic, rclcpp::Node * node);

  /// 从全量期望 JointState 提取本组期望（采样保持）。
  void set_desired(const sensor_msgs::msg::JointState & msg);

  /// 采样保持本组仿真输出（同时作为软仿真反馈源与命令源）。
  void on_output(const sensor_msgs::msg::JointState & msg);

  /// 周期发布：组期望 /ctrl/<组>/desired、组反馈 /ctrl/<组>/feedback、物理层命令。
  void publish();

  /// 填充本组状态到调试话题（simulated_*_states）。
  void fill_state_msg(sensor_msgs::msg::JointState & msg) const;

  bool have_output() const { return have_output_; }

private:
  const ylr1d_algorithm_sim::JointGroupDef * def_{nullptr};
  std::vector<double> desired_;
  std::vector<double> output_;
  bool have_desired_{false};
  bool have_output_{false};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
};

}  // namespace ylr1d_control

#endif  // YLR1D_CONTROL__GROUPS__GROUP_FORWARDER_HPP_
