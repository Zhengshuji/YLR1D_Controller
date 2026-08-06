#ifndef YLR1D_ALGORITHM_SIM__PLANT__VELOCITY__VELOCITY_PLANT_HPP_
#define YLR1D_ALGORITHM_SIM__PLANT__VELOCITY__VELOCITY_PLANT_HPP_

namespace ylr1d_algorithm_sim {

/// 速度型被控对象参数（1/s 单积分器）
struct VelocityPlantParams {
  double max_vel{3.0};        // 速度限幅 (rad/s 或 m/s)
  bool bypass{false};         // 直通：update 直接 return 输入（Gazebo/真机在环）
};

/// 速度型被控对象：把控制量 u 解释为加速度 → 单重积分（1/s）→ 速度。
/// 是"加速度 → 速度"的纯积分器，不含任何物理动力学参数；
/// 具体控制哪个关节（如轮子）、语义如何，由控制层装配时赋予。
class VelocityPlant {
public:
  VelocityPlant() = default;

  void configure(const VelocityPlantParams & params);

  void initialize(double velocity);

  /// 输入控制量 u（加速度语义），输出速度；bypass 时 return u
  double update(double control_input, double dt);

  double velocity() const { return velocity_; }
  bool initialized() const { return initialized_; }

private:
  VelocityPlantParams params_;
  double velocity_{0.0};
  bool initialized_{false};
};

}  // namespace ylr1d_algorithm_sim

#endif  // YLR1D_ALGORITHM_SIM__PLANT__VELOCITY__VELOCITY_PLANT_HPP_
