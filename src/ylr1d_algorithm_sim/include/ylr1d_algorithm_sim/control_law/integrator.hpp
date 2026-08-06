#ifndef YLR1D_ALGORITHM_SIM__CONTROL_LAW__INTEGRATOR_HPP_
#define YLR1D_ALGORITHM_SIM__CONTROL_LAW__INTEGRATOR_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <vector>

namespace ylr1d_algorithm_sim {

/// 积分器（仿真被控对象模型）参数：限速 + 可选位置限位。
/// 数值来源：config/joint_config.hpp 的 JointLimit（各组具名 Plant 装配时换算）。
struct IntegratorParams {
  double max_vel{3.0};        // 速度限幅 (rad/s 或 m/s)
  bool has_position_limit{false};
  double lower{0.0};          // 位置下限（对无限位关节忽略）
  double upper{0.0};          // 位置上限
};

/// 位置型标量积分器（1/s²）：把控制量 u 解释为加速度 → 二重积分 → 位置。
/// 纯运动学模型，不含物理动力学参数；是"怎么做"的实现细节（src/control_law/）。
class PositionIntegrator {
public:
  PositionIntegrator() = default;

  void configure(const IntegratorParams & params);

  void initialize(double position, double velocity = 0.0);

  /// 输入 u（加速度语义），输出位置。
  double update(double control_input, double dt);

  double position() const { return position_; }
  double velocity() const { return velocity_; }
  bool initialized() const { return initialized_; }

private:
  IntegratorParams params_;
  double position_{0.0};
  double velocity_{0.0};
  bool initialized_{false};
};

/// 速度型标量积分器（1/s）：把控制量 u 解释为加速度 → 单重积分 → 速度。
class VelocityIntegrator {
public:
  VelocityIntegrator() = default;

  void configure(const IntegratorParams & params);

  void initialize(double velocity);

  /// 输入 u（加速度语义），输出速度。
  double update(double control_input, double dt);

  double velocity() const { return velocity_; }
  bool initialized() const { return initialized_; }

private:
  IntegratorParams params_;
  double velocity_{0.0};
  bool initialized_{false};
};

/// 位置型组级积分器（整体输入/整体输出，Eigen 向量）：内部逐关节复用
/// PositionIntegrator；bypass 时 update 直通返回输入向量（Gazebo/真机在环）。
class PositionIntegratorGroup {
public:
  PositionIntegratorGroup() = default;

  void configure(const std::vector<IntegratorParams> & params, bool bypass);

  void initialize(const Eigen::VectorXd & position);

  /// 输入 u 向量（加速度语义）→ 推进 → 返回位置向量；bypass 直通返回 u。
  Eigen::VectorXd update(const Eigen::VectorXd & u, double dt);

  const Eigen::VectorXd & position() const { return position_; }
  const Eigen::VectorXd & velocity() const { return velocity_; }
  std::size_t size() const { return units_.size(); }
  bool initialized() const { return initialized_; }
  bool bypass() const { return bypass_; }

private:
  std::vector<PositionIntegrator> units_;
  Eigen::VectorXd position_;
  Eigen::VectorXd velocity_;
  bool bypass_{false};
  bool initialized_{false};
};

/// 速度型组级积分器（整体输入/整体输出）：内部逐关节复用 VelocityIntegrator。
class VelocityIntegratorGroup {
public:
  VelocityIntegratorGroup() = default;

  void configure(const std::vector<IntegratorParams> & params, bool bypass);

  void initialize(const Eigen::VectorXd & velocity);

  /// 输入 u 向量（加速度语义）→ 推进 → 返回速度向量；bypass 直通返回 u。
  Eigen::VectorXd update(const Eigen::VectorXd & u, double dt);

  const Eigen::VectorXd & velocity() const { return velocity_; }
  std::size_t size() const { return units_.size(); }
  bool initialized() const { return initialized_; }
  bool bypass() const { return bypass_; }

private:
  std::vector<VelocityIntegrator> units_;
  Eigen::VectorXd velocity_;
  bool bypass_{false};
  bool initialized_{false};
};

}  // namespace ylr1d_algorithm_sim

#endif  // YLR1D_ALGORITHM_SIM__CONTROL_LAW__INTEGRATOR_HPP_
