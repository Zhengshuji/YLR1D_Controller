#include "ylr1d_algorithm_sim/control_law/integrator.hpp"
#include "ylr1d_algorithm_sim/control_law/pid.hpp"

#include <gtest/gtest.h>

#include <cmath>

using ylr1d_algorithm_sim::IntegratorParams;
using ylr1d_algorithm_sim::PidLaw;
using ylr1d_algorithm_sim::PidParams;
using ylr1d_algorithm_sim::PositionIntegrator;
using ylr1d_algorithm_sim::VelocityIntegrator;

/// 位置闭环：PidLaw + PositionIntegrator 单位负反馈，阶跃输入。
/// 反馈 = itg.position()；期望 r 阶跃到 target。
/// 位置型被控对象（1/s²）用 PD 即可无静差跟踪阶跃。
TEST(ClosedLoop, PositionStepResponse) {
  PidLaw pid;
  PidParams pp;
  pp.kp = 100.0;   // 特征 s² + kd·s + kp = s² + 20s + 100 = (s+10)²，临界阻尼
  pp.kd = 20.0;
  pp.output_limit = 0.0;   // 不限幅，观察纯控制律行为
  pid.configure(pp);

  PositionIntegrator itg;
  IntegratorParams sp;
  sp.max_vel = 10.0;
  sp.has_position_limit = true;
  sp.lower = -1.0;
  sp.upper = 1.0;
  itg.configure(sp);
  itg.initialize(0.0);

  constexpr double dt = 0.001;
  constexpr double target = 1.0;   // 限位 [-1, 1] 内
  constexpr int n = 2000;          // 仿真 2 s
  double pos = 0.0;
  for (int i = 0; i < n; ++i) {
    double u = pid.update(target, itg.position(), dt);
    pos = itg.update(u, dt);
  }
  EXPECT_NEAR(pos, target, 0.01);
  EXPECT_LE(itg.velocity(), sp.max_vel + 1e-9);
}

/// 速度闭环：PidLaw + VelocityIntegrator 单位负反馈，阶跃输入。
/// 反馈 = itg.velocity()；速度型被控对象（1/s）需 PI 消除阶跃静差。
TEST(ClosedLoop, VelocityStepResponse) {
  PidLaw pid;
  PidParams pp;
  pp.kp = 20.0;
  pp.ki = 100.0;   // 特征 s² + kp·s + ki = s² + 20s + 100 = (s+10)²
  pp.output_limit = 0.0;
  pid.configure(pp);

  VelocityIntegrator itg;
  IntegratorParams sp;
  sp.max_vel = 10.0;
  itg.configure(sp);
  itg.initialize(0.0);

  constexpr double dt = 0.001;
  constexpr double target = 2.0;
  constexpr int n = 2000;          // 仿真 2 s
  double vel = 0.0;
  for (int i = 0; i < n; ++i) {
    double u = pid.update(target, itg.velocity(), dt);
    vel = itg.update(u, dt);
  }
  EXPECT_NEAR(vel, target, 0.02);
}

/// 位置闭环 + 限位：期望超出限位，闭环下位置仍被钳制在限位内（不越过）。
TEST(ClosedLoop, PositionStepClampedByLimit) {
  PidLaw pid;
  PidParams pp;
  pp.kp = 100.0;
  pp.kd = 20.0;
  pp.output_limit = 0.0;
  pid.configure(pp);

  PositionIntegrator itg;
  IntegratorParams sp;
  sp.max_vel = 5.0;
  sp.has_position_limit = true;
  sp.lower = -1.0;
  sp.upper = 1.0;
  itg.configure(sp);
  itg.initialize(0.0);

  constexpr double dt = 0.001;
  constexpr double target = 5.0;   // 远超市内限位
  constexpr int n = 2000;
  double pos = 0.0;
  for (int i = 0; i < n; ++i) {
    double u = pid.update(target, itg.position(), dt);
    pos = itg.update(u, dt);
  }
  EXPECT_NEAR(pos, 1.0, 0.01);     // 收敛到上限
  EXPECT_LE(pos, 1.0 + 1e-9);      // 不越限
}
