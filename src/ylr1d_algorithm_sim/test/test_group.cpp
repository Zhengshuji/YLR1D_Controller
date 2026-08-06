#include "controller/controller.hpp"
#include "plant/plant.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <vector>

using ylr1d_algorithm_sim::PositionGroupPlant;
using ylr1d_algorithm_sim::PositionPlantParams;
using ylr1d_algorithm_sim::ProportionalController;
using ylr1d_algorithm_sim::VelocityGroupPlant;
using ylr1d_algorithm_sim::VelocityPlantParams;

// ── 协同控制器（P=1 比例） ──

TEST(ProportionalController, PassThroughAtGainOne) {
  ProportionalController c;
  c.configure(1.0);
  Eigen::VectorXd d(3);
  d << 1.0, -2.0, 0.5;
  Eigen::VectorXd out = c.compute(d);
  ASSERT_EQ(out.size(), 3);
  EXPECT_DOUBLE_EQ(out[0], 1.0);
  EXPECT_DOUBLE_EQ(out[1], -2.0);
  EXPECT_DOUBLE_EQ(out[2], 0.5);
}

TEST(ProportionalController, ScalesByGain) {
  ProportionalController c;
  c.configure(2.0);
  Eigen::VectorXd d(2);
  d << 1.0, -3.0;
  Eigen::VectorXd out = c.compute(d);
  EXPECT_DOUBLE_EQ(out[0], 2.0);
  EXPECT_DOUBLE_EQ(out[1], -6.0);
}

// ── 位置型组级仿真对象（整体输入/输出，1/s²） ──

TEST(PositionGroupPlant, IntegratesWholeGroup) {
  PositionGroupPlant plant;
  std::vector<PositionPlantParams> params(2);
  params[0].max_vel = 10.0;
  params[0].has_position_limit = true;
  params[0].lower = -1.0;
  params[0].upper = 1.0;
  params[1].max_vel = 10.0;
  plant.configure(params);
  plant.initialize(Eigen::VectorXd::Zero(2));

  Eigen::VectorXd u(2);
  u << 2.0, -1.0;
  // 恒加速度 1 秒（dt=0.01，100 步）：
  //   关节0：欧拉位移 → 2*0.01²*100*101/2 = 1.01 > 上限 1.0，被钳制到 1.0
  //   关节1：欧拉位移 → 1*0.01²*100*101/2 = 0.505（连续解析解 0.5，离散累积差 0.005）
  for (int i = 0; i < 100; ++i) plant.update(u, 0.01);
  Eigen::VectorXd p = plant.position();
  EXPECT_NEAR(p[0], 1.0, 1e-6);
  EXPECT_NEAR(p[1], -0.505, 1e-9);
}

TEST(PositionGroupPlant, BypassReturnsInputVector) {
  PositionGroupPlant plant;
  std::vector<PositionPlantParams> params(2);
  params[0].bypass = true;
  params[1].bypass = true;
  plant.configure(params);
  plant.initialize(Eigen::VectorXd::Zero(2));
  Eigen::VectorXd u(2);
  u << 1.5, -0.3;
  Eigen::VectorXd out = plant.update(u, 0.01);
  EXPECT_DOUBLE_EQ(out[0], 1.5);
  EXPECT_DOUBLE_EQ(out[1], -0.3);
}

// ── 速度型组级仿真对象（整体输入/输出，1/s） ──

TEST(VelocityGroupPlant, IntegratesWholeGroup) {
  VelocityGroupPlant plant;
  std::vector<VelocityPlantParams> params(2);
  params[0].max_vel = 5.0;
  params[1].max_vel = 5.0;
  plant.configure(params);
  plant.initialize(Eigen::VectorXd::Zero(2));

  Eigen::VectorXd u(2);
  u << 2.0, 3.0;
  plant.update(u, 0.1);  // v += u*dt
  Eigen::VectorXd v = plant.velocity();
  EXPECT_NEAR(v[0], 0.2, 1e-9);
  EXPECT_NEAR(v[1], 0.3, 1e-9);
}

TEST(VelocityGroupPlant, BypassReturnsInputVector) {
  VelocityGroupPlant plant;
  std::vector<VelocityPlantParams> params(2);
  params[0].bypass = true;
  params[1].bypass = true;
  plant.configure(params);
  plant.initialize(Eigen::VectorXd::Zero(2));
  Eigen::VectorXd u(2);
  u << 2.0, -0.7;
  Eigen::VectorXd out = plant.update(u, 0.01);
  EXPECT_DOUBLE_EQ(out[0], 2.0);
  EXPECT_DOUBLE_EQ(out[1], -0.7);
}
