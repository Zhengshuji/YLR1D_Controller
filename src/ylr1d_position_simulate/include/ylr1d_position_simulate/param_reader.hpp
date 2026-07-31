#ifndef YLR1D_POSITION_SIMULATE__PARAM_READER_HPP_
#define YLR1D_POSITION_SIMULATE__PARAM_READER_HPP_

#include "ylr1d_position_simulate/joint_group.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace ylr1d_position_simulate {

/// 读取单个位置关节的参数（统一命名 <关节名>/pid/*、<关节名>/limit/*）。
/// 默认值与旧版硬编码一致，yaml 未提供时节点仍可运行。
inline JointSimParams read_position_params(rclcpp::Node & node, const std::string & name) {
  JointSimParams p;
  p.kp = node.declare_parameter(name + "/pid/kp", 4.0);
  p.ki = node.declare_parameter(name + "/pid/ki", 0.0);
  p.kd = node.declare_parameter(name + "/pid/kd", 0.2);
  p.max_accel = node.declare_parameter(name + "/limit/accelerate", 50.0);
  p.max_vel = node.declare_parameter(name + "/limit/velocity", 3.0);
  p.lower = node.declare_parameter(name + "/limit/lower", 0.0);
  p.upper = node.declare_parameter(name + "/limit/upper", 0.0);
  p.has_position_limit = true;
  return p;
}

/// 读取单个速度关节（轮子）的参数：无位置限位，只有速度/加速度限幅。
inline VelocitySimParams read_velocity_params(rclcpp::Node & node, const std::string & name) {
  VelocitySimParams p;
  p.kp = node.declare_parameter(name + "/pid/kp", 2.0);
  p.ki = node.declare_parameter(name + "/pid/ki", 0.0);
  p.kd = node.declare_parameter(name + "/pid/kd", 0.05);
  p.max_accel = node.declare_parameter(name + "/limit/accelerate", 20.0);
  p.max_vel = node.declare_parameter(name + "/limit/velocity", 5.0);
  return p;
}

}  // namespace ylr1d_position_simulate

#endif  // YLR1D_POSITION_SIMULATE__PARAM_READER_HPP_
