#ifndef YLR1D_ALGORITHM_SIM__PLANT_HPP_
#define YLR1D_ALGORITHM_SIM__PLANT_HPP_

#include "ylr1d_algorithm_sim/config/joint_config.hpp"

#include <Eigen/Dense>

#include <cstddef>
#include <vector>

namespace ylr1d_algorithm_sim {

/// 仿真被控对象（架构层虚基类，组级整体）。
///
/// "是什么"：整体输入控制量 u 向量 → 推进 → 输出状态向量。
/// 位置型（1/s²，状态=位置）与速度型（1/s，状态=速度）语义不同但对外接口一致；
/// "怎么做"（积分器、动力学模型…）是实现，在 src/control_law/ 与各组的具名
/// 仿真对象（src/<组>.cpp）中，改模型只改 src。本接口与节点框架不动。
class GroupPlant {
public:
  virtual ~GroupPlant() = default;

  /// 按关节限位（joint_config.hpp 单一来源）配置 + 组级 bypass 开关。
  virtual void configure(const std::vector<JointLimit> & limits, bool bypass) = 0;

  virtual std::size_t size() const = 0;

  /// 初始化主状态向量（位置组=位置、速度组=速度；位置组内部速度初始为 0）。
  virtual void initialize(const Eigen::VectorXd & state) = 0;

  /// 输入控制量向量 → 推进 → 返回主状态向量（bypass 时直通返回输入）。
  virtual Eigen::VectorXd update(const Eigen::VectorXd & input, double dt) = 0;

  /// 主状态：位置组=位置，速度组=速度。
  virtual const Eigen::VectorXd & state() const = 0;

  /// 速度：位置组=位置导数，速度组=主状态本身。
  virtual const Eigen::VectorXd & velocity() const = 0;

  virtual bool initialized() const = 0;
};

}  // namespace ylr1d_algorithm_sim

#endif  // YLR1D_ALGORITHM_SIM__PLANT_HPP_
