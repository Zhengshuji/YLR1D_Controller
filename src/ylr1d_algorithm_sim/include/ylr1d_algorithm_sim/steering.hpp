#ifndef YLR1D_ALGORITHM_SIM__STEERING_HPP_
#define YLR1D_ALGORITHM_SIM__STEERING_HPP_

#include "ylr1d_algorithm_sim/control_law/integrator.hpp"
#include "ylr1d_algorithm_sim/control_law/pid.hpp"
#include "ylr1d_algorithm_sim/control_law/proportional.hpp"
#include "ylr1d_algorithm_sim/cooperative_controller.hpp"
#include "ylr1d_algorithm_sim/independent_controller.hpp"
#include "ylr1d_algorithm_sim/plant.hpp"

#include <Eigen/Dense>

#include <cstddef>
#include <vector>

namespace ylr1d_algorithm_sim {

/// steering 组（4 转向，位置型）协同控制器：组期望 → 逐关节 setpoint。
/// 内部算法：P 比例（control_law/proportional），实现见 src/steering.cpp。
class SteeringCooperativeController final : public CooperativeController {
public:
  void set_group_size(std::size_t count) override;
  void initialize() override;
  Eigen::VectorXd compute(const Eigen::VectorXd & group_desired) override;

private:
  ProportionalLaw law_;
};

/// steering 组逐关节控制器（每关节一个实例）：setpoint + feedback → u。
/// 内部算法：PID（control_law/pid），实现见 src/steering.cpp。
class SteeringJointController final : public IndependentController {
public:
  /// 配置 PID 参数（来自 pid.yaml 的 <关节>/pid/*）。
  void configure(const PidParams & params);
  void initialize() override;
  double compute(double setpoint, double feedback, double dt) override;

private:
  PidLaw law_;
};

/// steering 组仿真被控对象（位置型，1/s²）：整体输入/整体输出。
/// 内部模型：位置积分器组（control_law/integrator），实现见 src/steering.cpp。
class SteeringPlant final : public GroupPlant {
public:
  void configure(const std::vector<JointLimit> & limits, bool bypass) override;
  std::size_t size() const override;
  void initialize(const Eigen::VectorXd & state) override;
  Eigen::VectorXd update(const Eigen::VectorXd & input, double dt) override;
  const Eigen::VectorXd & state() const override;
  const Eigen::VectorXd & velocity() const override;
  bool initialized() const override;
  bool bypass() const;

private:
  PositionIntegratorGroup engine_;
};

}  // namespace ylr1d_algorithm_sim

#endif  // YLR1D_ALGORITHM_SIM__STEERING_HPP_
