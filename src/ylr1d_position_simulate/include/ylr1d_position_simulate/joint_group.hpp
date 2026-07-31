#ifndef YLR1D_POSITION_SIMULATE__JOINT_GROUP_HPP_
#define YLR1D_POSITION_SIMULATE__JOINT_GROUP_HPP_

#include "ylr1d_position_simulate/joint_simulator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <map>
#include <string>
#include <vector>

namespace ylr1d_position_simulate {

/// 位置 PID 关节组
/// 统一管理一组位置关节，每个关节持有独立的 JointSimulator（PID + 限幅）。
/// 输出：各关节真实位置 -> 对应 ForwardCommandController 命令话题
class PositionJointGroup {
public:
  PositionJointGroup() = default;

  /// @param names   关节名列表（顺序与 controller 的 joints 一致）
  /// @param params  每个关节独立的仿真参数（长度与 names 一致）
  /// @param topic   发布到的 controller command topic
  /// @param node    父节点（用于创建 publisher）
  void setup(const std::vector<std::string> & names,
             const std::vector<JointSimParams> & params,
             const std::string & topic,
             rclcpp::Node * node);

  /// 从 /joint_states 初始化位置（仅一次）
  void init_from(const sensor_msgs::msg::JointState & msg);

  /// 设置期望位置
  void set_desired(const sensor_msgs::msg::JointState & msg);

  /// 执行一步更新
  void update(double dt);

  /// 发布位置命令
  void publish();

  /// 填充 JointState 消息（用于调试发布）
  void fill_state_msg(sensor_msgs::msg::JointState & msg) const;

  bool initialized() const { return initialized_; }

private:
  struct Joint {
    std::string name;
    JointSimulator sim;
  };

  std::vector<Joint> joints_;
  std::map<std::string, size_t> name_to_idx_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  bool initialized_{false};
};

/// 单关节速度仿真参数
struct VelocitySimParams {
  double kp{2.0};
  double ki{0.0};
  double kd{0.05};
  double max_accel{20.0};
  double max_vel{5.0};
};

/// 速度 PID 关节组
/// 统一管理一组速度关节（轮子）：期望速度 -> PID -> 加速度 -> 速度(限幅)
class VelocityJointGroup {
public:
  VelocityJointGroup() = default;

  /// @param names   关节名列表（顺序与 controller 的 joints 一致）
  /// @param params  每个关节独立的仿真参数（长度与 names 一致）
  /// @param topic   发布到的 controller command topic
  /// @param node    父节点（用于创建 publisher）
  void setup(const std::vector<std::string> & names,
             const std::vector<VelocitySimParams> & params,
             const std::string & topic,
             rclcpp::Node * node);

  void set_desired(const sensor_msgs::msg::JointState & msg);
  void update(double dt);
  void publish();
  void fill_state_msg(sensor_msgs::msg::JointState & msg) const;

private:
  struct Joint {
    std::string name;
    double velocity{0.0};
    double desired{0.0};
    PID pid;
    double max_accel{20.0};
    double max_vel{5.0};
  };

  std::vector<Joint> joints_;
  std::map<std::string, size_t> name_to_idx_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
};

}  // namespace ylr1d_position_simulate

#endif  // YLR1D_POSITION_SIMULATE__JOINT_GROUP_HPP_
