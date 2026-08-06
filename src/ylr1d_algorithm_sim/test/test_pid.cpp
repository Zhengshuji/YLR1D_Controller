#include "controller/pid/pid.hpp"

#include <gtest/gtest.h>

#include <cmath>

using ylr1d_algorithm_sim::PidController;
using ylr1d_algorithm_sim::PidParams;

TEST(PidController, ProportionalOnlyTracksSetpoint) {
  PidController pid;
  PidParams p;
  p.kp = 2.0;
  p.output_limit = 100.0;
  pid.configure(p);
  // 期望 10、反馈 0，误差 10 → u = kp*error = 20
  EXPECT_NEAR(pid.compute(10.0, 0.0, 0.01), 20.0, 1e-9);
}

TEST(PidController, OutputLimitClamps) {
  PidController pid;
  PidParams p;
  p.kp = 100.0;
  p.output_limit = 5.0;
  pid.configure(p);
  // 误差 1 → 无限制输出 100，被限到 ±5
  EXPECT_NEAR(pid.compute(1.0, 0.0, 0.01), 5.0, 1e-9);
}

TEST(PidController, ZeroLimitIsUnbounded) {
  PidController pid;
  PidParams p;
  p.kp = 1e6;
  p.output_limit = 0.0;  // <=0 表示不限幅
  pid.configure(p);
  EXPECT_GT(pid.compute(1.0, 0.0, 0.01), 1e5);
}

TEST(PidController, ResetClearsHistory) {
  PidController pid;
  PidParams p;
  p.kp = 1.0;
  p.ki = 1.0;
  p.kd = 1.0;
  p.output_limit = 100.0;
  pid.configure(p);
  pid.compute(1.0, 0.0, 0.01);  // 累计积分与微分历史
  pid.reset();
  // 重置后误差突变到 0：积分清零、微分项为 0 → u = 0
  EXPECT_NEAR(pid.compute(1.0, 1.0, 0.01), 0.0, 1e-9);
}

TEST(PidController, ConfigureReapplyIsRuntimeUpdate) {
  PidController pid;
  PidParams p;
  p.kp = 1.0;
  p.output_limit = 100.0;
  pid.configure(p);
  EXPECT_NEAR(pid.compute(2.0, 0.0, 0.01), 2.0, 1e-9);
  p.kp = 10.0;  // 运行时改增益后再次 configure
  pid.configure(p);
  EXPECT_NEAR(pid.compute(2.0, 0.0, 0.01), 20.0, 1e-9);
}
