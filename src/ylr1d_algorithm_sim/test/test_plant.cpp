#include "plant/position/position_plant.hpp"
#include "plant/velocity/velocity_plant.hpp"

#include <gtest/gtest.h>

using ylr1d_algorithm_sim::PositionPlant;
using ylr1d_algorithm_sim::PositionPlantParams;
using ylr1d_algorithm_sim::VelocityPlant;
using ylr1d_algorithm_sim::VelocityPlantParams;

// ── 位置型被控对象（1/s²，加速度 → 位置） ──

TEST(PositionPlant, DoubleIntegration) {
  PositionPlant plant;
  PositionPlantParams p;
  p.max_vel = 10.0;
  p.has_position_limit = true;
  p.lower = -1.0;
  p.upper = 1.0;
  plant.configure(p);
  plant.initialize(0.0);
  // 恒加速度 2，1 秒：理论位移 0.5*2*1² = 1.0，正好顶到上限
  for (int i = 0; i < 100; ++i) plant.update(2.0, 0.01);
  EXPECT_NEAR(plant.position(), 1.0, 1e-6);
  EXPECT_LE(plant.position(), 1.0 + 1e-9);
}

TEST(PositionPlant, PositionLimitClamps) {
  PositionPlant plant;
  PositionPlantParams p;
  p.max_vel = 100.0;
  p.has_position_limit = true;
  p.lower = -0.3;
  p.upper = 0.3;
  plant.configure(p);
  plant.initialize(0.0);
  for (int i = 0; i < 1000; ++i) plant.update(50.0, 0.01);
  EXPECT_NEAR(plant.position(), 0.3, 1e-6);
}

TEST(PositionPlant, VelocityLimitClamps) {
  PositionPlant plant;
  PositionPlantParams p;
  p.max_vel = 1.0;
  plant.configure(p);
  plant.initialize(0.0);
  for (int i = 0; i < 100; ++i) plant.update(100.0, 0.01);
  EXPECT_NEAR(plant.velocity(), 1.0, 1e-9);
}

TEST(PositionPlant, InitializeSetsState) {
  PositionPlant plant;
  PositionPlantParams p;
  p.max_vel = 10.0;
  plant.configure(p);
  plant.initialize(1.2, -0.5);
  plant.update(0.0, 0.1);          // 加速度 0：速度不变 -0.5，位置 += -0.5*0.1 = -0.05
  EXPECT_NEAR(plant.velocity(), -0.5, 1e-9);
  EXPECT_NEAR(plant.position(), 1.15, 1e-9);
}

// ── 速度型被控对象（1/s，加速度 → 速度） ──

TEST(VelocityPlant, SingleIntegration) {
  VelocityPlant plant;
  VelocityPlantParams p;
  p.max_vel = 5.0;
  plant.configure(p);
  plant.initialize(0.0);
  plant.update(2.0, 0.1);          // v = 0 + 2*0.1 = 0.2
  EXPECT_NEAR(plant.update(0.0, 0.1), 0.2, 1e-9);
}

TEST(VelocityPlant, VelocityLimitClamps) {
  VelocityPlant plant;
  VelocityPlantParams p;
  p.max_vel = 1.0;
  plant.configure(p);
  plant.initialize(0.0);
  for (int i = 0; i < 100; ++i) plant.update(100.0, 0.01);
  EXPECT_NEAR(plant.velocity(), 1.0, 1e-9);
}

// ── 直通（bypass：update 直接 return 输入） ──

TEST(PositionPlant, BypassReturnsInput) {
  PositionPlant plant;
  PositionPlantParams p;
  p.bypass = true;
  plant.configure(p);
  plant.initialize(0.0);
  EXPECT_DOUBLE_EQ(plant.update(3.14, 0.01), 3.14);
  EXPECT_DOUBLE_EQ(plant.update(-1.5, 0.01), -1.5);
}

TEST(VelocityPlant, BypassReturnsInput) {
  VelocityPlant plant;
  VelocityPlantParams p;
  p.bypass = true;
  plant.configure(p);
  plant.initialize(0.0);
  EXPECT_DOUBLE_EQ(plant.update(2.0, 0.01), 2.0);
  EXPECT_DOUBLE_EQ(plant.update(-0.7, 0.01), -0.7);
}
