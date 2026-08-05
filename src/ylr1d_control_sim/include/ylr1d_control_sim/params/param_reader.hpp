#ifndef YLR1D_CONTROL_SIM__PARAMS__PARAM_READER_HPP_
#define YLR1D_CONTROL_SIM__PARAMS__PARAM_READER_HPP_

#include "ylr1d_control_sim/core/joint_params.hpp"
#include "ylr1d_control_sim/config/joint_config.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace ylr1d_control_sim {

/// 从 ROS 参数读取单个关节的仿真参数。
/// pid 部分命名 <关节名>/pid/*（来自 pid.yaml，可通过 launch 参数覆盖）；
/// limit 部分改读头文件常量 jointLimitFor(name)（编译期确定，不再经 ROS 参数）。
/// 默认值取自 preset（yaml / 节点参数未提供时回退到预设），不重复硬编码。
inline JointParams read_joint_params(rclcpp::Node & node, const std::string & name,
                                     const JointParams & preset) {
  JointParams p = preset;
  p.kp = node.declare_parameter(name + "/pid/kp", preset.kp);
  p.ki = node.declare_parameter(name + "/pid/ki", preset.ki);
  p.kd = node.declare_parameter(name + "/pid/kd", preset.kd);
  // 限幅来自头文件静态配置（单一来源），覆盖 preset 中的 limit 字段
  const JointLimit lim = jointLimitFor(name);
  p.has_position_limit = lim.has_position_limit;
  p.lower = lim.lower;
  p.upper = lim.upper;
  p.max_vel = lim.max_vel;
  p.max_accel = lim.max_accel;
  return p;
}

}  // namespace ylr1d_control_sim

#endif  // YLR1D_CONTROL_SIM__PARAMS__PARAM_READER_HPP_
