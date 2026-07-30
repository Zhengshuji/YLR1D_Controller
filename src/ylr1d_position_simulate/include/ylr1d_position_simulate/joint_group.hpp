#ifndef YLR1D_POSITION_SIMULATE__JOINT_GROUP_HPP_
#define YLR1D_POSITION_SIMULATE__JOINT_GROUP_HPP_

#include "ylr1d_position_simulate/pid.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <map>
#include <string>
#include <vector>

namespace ylr1d_position_simulate {

/// 位置 PID 关节组
/// 统一管理一组关节：position PID -> accel限幅 -> 速度 -> vel限幅 -> 位置
class PositionJointGroup {
public:
  PositionJointGroup() = default;

  /// @param names  关节名列表（顺序与 controller 的 joints 一致）
  /// @param pid    PID 参数
  /// @param topic  发布到的 controller command topic
  /// @param node   父节点（用于创建 publisher）
  void setup(const std::vector<std::string> & names,
             const PID & pid,
             const std::string & topic,
             rclcpp::Node * node);

  /// 从 /joint_states 初始化位置
  void init_from(const sensor_msgs::msg::JointState & msg);

  /// 设置期望位置
  void set_desired(const sensor_msgs::msg::JointState & msg);

  /// 设置位置限幅 [lower, upper] per joint (长度须与 setup names 一致)
  void set_limits(const std::vector<std::pair<double, double>> & limits);

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
    double position{0.0};
    double velocity{0.0};
    double desired{0.0};
    PID pid;
    bool initialized{false};
  };

  std::vector<Joint> joints_;
  std::map<std::string, size_t> name_to_idx_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  bool initialized_{false};
  std::vector<double> lower_limits_;   // 位置下限 (size=0 表示不限幅)
  std::vector<double> upper_limits_;   // 位置上限
};

/// 速度 PID 关节组
/// 统一管理一组关节：velocity PID -> accel限幅 -> 速度 -> vel限幅
class VelocityJointGroup {
public:
  VelocityJointGroup() = default;

  void setup(const std::vector<std::string> & names,
             const PID & pid,
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
  };

  std::vector<Joint> joints_;
  std::map<std::string, size_t> name_to_idx_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
};

}  // namespace ylr1d_position_simulate

#endif  // YLR1D_POSITION_SIMULATE__JOINT_GROUP_HPP_
