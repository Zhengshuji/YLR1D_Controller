#ifndef PLANT__POSITION_GROUP__POSITION_GROUP_PLANT_HPP_
#define PLANT__POSITION_GROUP__POSITION_GROUP_PLANT_HPP_

#include "plant/position/position_plant.hpp"

#include <Eigen/Dense>

#include <cstddef>
#include <vector>

namespace ylr1d_algorithm_sim {

/// 位置型组级仿真对象：整体输入/整体输出（Eigen 向量）。
/// u 为组内各关节加速度向量，输出组内各关节位置向量。
/// 内部逐关节复用标量 PositionPlant（1/s² + 限位/限速）；
/// bypass 时 update 直通返回输入向量（Gazebo / 真机在环）。
class PositionGroupPlant {
public:
  PositionGroupPlant() = default;

  /// 装配组内各关节参数（长度 = 组内关节数）；bypass 取整体开关。
  void configure(const std::vector<PositionPlantParams> & params);

  /// 初始化各关节位置/速度向量；velocity 缺省为全零。
  void initialize(const Eigen::VectorXd & position,
                  const Eigen::VectorXd & velocity = Eigen::VectorXd());

  /// 输入加速度向量 → 返回位置向量；bypass 时返回 u。
  Eigen::VectorXd update(const Eigen::VectorXd & u, double dt);

  const Eigen::VectorXd & position() const { return position_; }
  const Eigen::VectorXd & velocity() const { return velocity_; }
  bool initialized() const { return initialized_; }
  std::size_t size() const { return plants_.size(); }
  bool bypass() const { return bypass_; }

private:
  std::vector<PositionPlant> plants_;
  Eigen::VectorXd position_;
  Eigen::VectorXd velocity_;
  bool bypass_{false};
  bool initialized_{false};
};

}  // namespace ylr1d_algorithm_sim

#endif  // PLANT__POSITION_GROUP__POSITION_GROUP_PLANT_HPP_
