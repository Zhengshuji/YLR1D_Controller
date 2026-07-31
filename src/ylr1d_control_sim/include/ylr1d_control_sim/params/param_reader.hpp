#ifndef YLR1D_CONTROL_SIM__PARAMS__PARAM_READER_HPP_
#define YLR1D_CONTROL_SIM__PARAMS__PARAM_READER_HPP_

#include "ylr1d_control_sim/core/joint_params.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace ylr1d_control_sim {

/// 从 ROS 参数读取单个关节的仿真参数，统一命名 <关节名>/pid/*、<关节名>/limit/*。
/// 默认值取自 preset（yaml / 节点参数未提供时回退到预设），不重复硬编码。
/// 回退语义：yaml 配置 -> 节点参数（默认 = preset）-> JointParams 预设。
inline JointParams read_joint_params(rclcpp::Node & node, const std::string & name,
                                     const JointParams & preset) {
  JointParams p = preset;
  p.kp = node.declare_parameter(name + "/pid/kp", preset.kp);
  p.ki = node.declare_parameter(name + "/pid/ki", preset.ki);
  p.kd = node.declare_parameter(name + "/pid/kd", preset.kd);
  p.max_accel = node.declare_parameter(name + "/limit/accelerate", preset.max_accel);
  p.max_vel = node.declare_parameter(name + "/limit/velocity", preset.max_vel);
  if (preset.has_position_limit) {
    p.lower = node.declare_parameter(name + "/limit/lower", preset.lower);
    p.upper = node.declare_parameter(name + "/limit/upper", preset.upper);
  }
  return p;
}

}  // namespace ylr1d_control_sim

#endif  // YLR1D_CONTROL_SIM__PARAMS__PARAM_READER_HPP_
