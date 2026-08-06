#ifndef YLR1D_ALGORITHM_SIM__PLANT__POSITION__POSITION_PLANT_HPP_
#define YLR1D_ALGORITHM_SIM__PLANT__POSITION__POSITION_PLANT_HPP_

namespace ylr1d_algorithm_sim {

/// 位置型被控对象参数（1/s² 双积分器）
struct PositionPlantParams {
  double max_vel{3.0};        // 速度限幅 (rad/s 或 m/s)
  bool has_position_limit{false};
  double lower{0.0};          // 位置下限（对无限位关节忽略）
  double upper{0.0};          // 位置上限
  bool bypass{false};         // 直通：update 直接 return 输入（Gazebo/真机在环）
};

/// 位置型被控对象：把控制量 u 解释为加速度 → 二重积分（1/s²）→ 位置。
/// 是"加速度 → 速度 → 位置"的纯积分器，不含任何物理动力学参数（质量/惯量/摩擦等）；
/// 具体控制哪个关节、语义如何，由控制层装配时赋予。
class PositionPlant {
public:
  PositionPlant() = default;

  void configure(const PositionPlantParams & params);

  void initialize(double position, double velocity = 0.0);

  /// 输入控制量 u（加速度语义），输出位置；bypass 时 return u
  double update(double control_input, double dt);

  double position() const { return position_; }
  double velocity() const { return velocity_; }
  bool initialized() const { return initialized_; }

private:
  PositionPlantParams params_;
  double position_{0.0};
  double velocity_{0.0};
  bool initialized_{false};
};

}  // namespace ylr1d_algorithm_sim

#endif  // YLR1D_ALGORITHM_SIM__PLANT__POSITION__POSITION_PLANT_HPP_
