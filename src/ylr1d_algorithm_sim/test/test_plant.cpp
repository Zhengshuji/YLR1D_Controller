#include "ylr1d_algorithm_sim/control_law/integrator.hpp"

#include <gtest/gtest.h>

using ylr1d_algorithm_sim::IntegratorParams;
using ylr1d_algorithm_sim::PositionIntegrator;
using ylr1d_algorithm_sim::VelocityIntegrator;

// ── 位置型积分器（1/s²，加速度 → 位置） ──

TEST(PositionIntegrator, DoubleIntegration) {
  PositionIntegrator itg;
  IntegratorParams p;
  p.max_vel = 10.0;
  p.has_position_limit = true;
  p.lower = -1.0;
  p.upper = 1.0;
  itg.configure(p);
  itg.initialize(0.0);
  // 恒加速度 2，1 秒：理论位移 0.5*2*1² = 1.0，正好顶到上限
  for (int i = 0; i < 100; ++i) itg.update(2.0, 0.01);
  EXPECT_NEAR(itg.position(), 1.0, 1e-6);
  EXPECT_LE(itg.position(), 1.0 + 1e-9);
}

TEST(PositionIntegrator, PositionLimitClamps) {
  PositionIntegrator itg;
  IntegratorParams p;
  p.max_vel = 100.0;
  p.has_position_limit = true;
  p.lower = -0.3;
  p.upper = 0.3;
  itg.configure(p);
  itg.initialize(0.0);
  for (int i = 0; i < 1000; ++i) itg.update(50.0, 0.01);
  EXPECT_NEAR(itg.position(), 0.3, 1e-6);
}

TEST(PositionIntegrator, VelocityLimitClamps) {
  PositionIntegrator itg;
  IntegratorParams p;
  p.max_vel = 1.0;
  itg.configure(p);
  itg.initialize(0.0);
  for (int i = 0; i < 100; ++i) itg.update(100.0, 0.01);
  EXPECT_NEAR(itg.velocity(), 1.0, 1e-9);
}

TEST(PositionIntegrator, InitializeSetsState) {
  PositionIntegrator itg;
  IntegratorParams p;
  p.max_vel = 10.0;
  itg.configure(p);
  itg.initialize(1.2, -0.5);
  itg.update(0.0, 0.1);          // 加速度 0：速度不变 -0.5，位置 += -0.5*0.1 = -0.05
  EXPECT_NEAR(itg.velocity(), -0.5, 1e-9);
  EXPECT_NEAR(itg.position(), 1.15, 1e-9);
}

// ── 速度型积分器（1/s，加速度 → 速度） ──

TEST(VelocityIntegrator, SingleIntegration) {
  VelocityIntegrator itg;
  IntegratorParams p;
  p.max_vel = 5.0;
  itg.configure(p);
  itg.initialize(0.0);
  itg.update(2.0, 0.1);          // v = 0 + 2*0.1 = 0.2
  EXPECT_NEAR(itg.update(0.0, 0.1), 0.2, 1e-9);
}

TEST(VelocityIntegrator, VelocityLimitClamps) {
  VelocityIntegrator itg;
  IntegratorParams p;
  p.max_vel = 1.0;
  itg.configure(p);
  itg.initialize(0.0);
  for (int i = 0; i < 100; ++i) itg.update(100.0, 0.01);
  EXPECT_NEAR(itg.velocity(), 1.0, 1e-9);
}
