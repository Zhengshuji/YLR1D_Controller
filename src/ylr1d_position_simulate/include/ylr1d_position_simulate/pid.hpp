#ifndef YLR1D_POSITION_SIMULATE__PID_HPP_
#define YLR1D_POSITION_SIMULATE__PID_HPP_

namespace ylr1d_position_simulate {

/// 简易 PID 控制器
/// compute() 输出加速度值，外部积分得到速度和位置
class PID {
public:
  PID() = default;

  /// @param kp   比例增益
  /// @param ki   积分增益
  /// @param kd   微分增益
  /// @param max_accel  加速度限幅 (输出限幅)
  /// @param max_vel    速度限幅 (外部使用)
  PID(double kp, double ki, double kd, double max_accel, double max_vel);

  /// @param error  当前误差 (期望 - 实际)
  /// @param dt     时间步长 (秒)
  /// @return       加速度指令
  double compute(double error, double dt);

  /// 重置积分和前一误差
  void reset();

  // 参数运行时修改
  void set_gains(double kp, double ki, double kd);
  void set_limits(double max_accel, double max_vel);

  // 限幅值在外部也需要使用
  double max_accel_ = 50.0;
  double max_vel_ = 3.0;

private:
  double kp_ = 5.0;
  double kd_ = 0.1;
  double ki_ = 0.0;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
};

}  // namespace ylr1d_position_simulate

#endif  // YLR1D_POSITION_SIMULATE__PID_HPP_
