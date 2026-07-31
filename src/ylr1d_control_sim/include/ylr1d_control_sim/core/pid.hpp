#ifndef YLR1D_CONTROL_SIM__CORE__PID_HPP_
#define YLR1D_CONTROL_SIM__CORE__PID_HPP_

namespace ylr1d_control_sim {

/// 简易 PID 控制器
/// compute() 输出加速度值，外部积分得到速度和位置
class PID {
public:
  PID() = default;

  /// @param kp   比例增益
  /// @param ki   积分增益
  /// @param kd   微分增益
  /// @param max_accel  加速度限幅 (输出限幅)
  PID(double kp, double ki, double kd, double max_accel);

  /// @param error  当前误差 (期望 - 实际)
  /// @param dt     时间步长 (秒)
  /// @return       加速度指令
  double compute(double error, double dt);

  /// 重置积分和前一误差
  void reset();

  // 参数运行时修改
  void set_gains(double kp, double ki, double kd);
  void set_limits(double max_accel);

private:
  double kp_ = 0.0;
  double kd_ = 0.0;
  double ki_ = 0.0;
  double max_accel_ = 0.0;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
};

}  // namespace ylr1d_control_sim

#endif  // YLR1D_CONTROL_SIM__CORE__PID_HPP_
