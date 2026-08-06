#include "ylr1d_algorithm_sim/control_law/integrator.hpp"
#include "ylr1d_algorithm_sim/control_law/pid.hpp"
#include "ylr1d_algorithm_sim/control_law/proportional.hpp"
#include "ylr1d_algorithm_sim/steering.hpp"
#include "ylr1d_algorithm_sim/wheel.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <vector>

using ylr1d_algorithm_sim::IntegratorParams;
using ylr1d_algorithm_sim::JointLimit;
using ylr1d_algorithm_sim::PidParams;
using ylr1d_algorithm_sim::PositionIntegratorGroup;
using ylr1d_algorithm_sim::ProportionalLaw;
using ylr1d_algorithm_sim::SteeringCooperativeController;
using ylr1d_algorithm_sim::SteeringJointController;
using ylr1d_algorithm_sim::SteeringPlant;
using ylr1d_algorithm_sim::VelocityIntegratorGroup;
using ylr1d_algorithm_sim::WheelCooperativeController;
using ylr1d_algorithm_sim::WheelJointController;
using ylr1d_algorithm_sim::WheelPlant;

// ── 协同控制律（P 比例，透传） ──

TEST(ProportionalLaw, PassThroughAtGainOne) {
  ProportionalLaw law(1.0);
  law.set_group_size(3);
  law.initialize();
  Eigen::VectorXd d(3);
  d << 1.0, -2.0, 0.5;
  Eigen::VectorXd out = law.compute(d);
  ASSERT_EQ(out.size(), 3);
  EXPECT_DOUBLE_EQ(out[0], 1.0);
  EXPECT_DOUBLE_EQ(out[1], -2.0);
  EXPECT_DOUBLE_EQ(out[2], 0.5);
}

TEST(ProportionalLaw, ScalesByGain) {
  ProportionalLaw law(2.0);
  law.set_group_size(2);
  law.initialize();
  Eigen::VectorXd d(2);
  d << 1.0, -3.0;
  Eigen::VectorXd out = law.compute(d);
  EXPECT_DOUBLE_EQ(out[0], 2.0);
  EXPECT_DOUBLE_EQ(out[1], -6.0);
}

// ── 位置型组级积分器（整体输入/输出，1/s²） ──

TEST(PositionIntegratorGroup, IntegratesWholeGroup) {
  PositionIntegratorGroup group;
  std::vector<IntegratorParams> params(2);
  params[0].max_vel = 10.0;
  params[0].has_position_limit = true;
  params[0].lower = -1.0;
  params[0].upper = 1.0;
  params[1].max_vel = 10.0;
  group.configure(params, false);
  group.initialize(Eigen::VectorXd::Zero(2));

  Eigen::VectorXd u(2);
  u << 2.0, -1.0;
  // 恒加速度 1 秒（dt=0.01，100 步）：
  //   关节0：欧拉位移 → 2*0.01²*100*101/2 = 1.01 > 上限 1.0，被钳制到 1.0
  //   关节1：欧拉位移 → 1*0.01²*100*101/2 = 0.505（连续解析解 0.5，离散累积差 0.005）
  for (int i = 0; i < 100; ++i) group.update(u, 0.01);
  Eigen::VectorXd p = group.position();
  EXPECT_NEAR(p[0], 1.0, 1e-6);
  EXPECT_NEAR(p[1], -0.505, 1e-9);
}

TEST(PositionIntegratorGroup, BypassReturnsInputVector) {
  PositionIntegratorGroup group;
  std::vector<IntegratorParams> params(2);
  group.configure(params, true);  // 组级 bypass
  group.initialize(Eigen::VectorXd::Zero(2));
  Eigen::VectorXd u(2);
  u << 1.5, -0.3;
  Eigen::VectorXd out = group.update(u, 0.01);
  EXPECT_DOUBLE_EQ(out[0], 1.5);
  EXPECT_DOUBLE_EQ(out[1], -0.3);
}

// ── 速度型组级积分器（整体输入/输出，1/s） ──

TEST(VelocityIntegratorGroup, IntegratesWholeGroup) {
  VelocityIntegratorGroup group;
  std::vector<IntegratorParams> params(2);
  params[0].max_vel = 5.0;
  params[1].max_vel = 5.0;
  group.configure(params, false);
  group.initialize(Eigen::VectorXd::Zero(2));

  Eigen::VectorXd u(2);
  u << 2.0, 3.0;
  group.update(u, 0.1);  // v += u*dt
  Eigen::VectorXd v = group.velocity();
  EXPECT_NEAR(v[0], 0.2, 1e-9);
  EXPECT_NEAR(v[1], 0.3, 1e-9);
}

TEST(VelocityIntegratorGroup, BypassReturnsInputVector) {
  VelocityIntegratorGroup group;
  std::vector<IntegratorParams> params(2);
  group.configure(params, true);
  group.initialize(Eigen::VectorXd::Zero(2));
  Eigen::VectorXd u(2);
  u << 2.0, -0.7;
  Eigen::VectorXd out = group.update(u, 0.01);
  EXPECT_DOUBLE_EQ(out[0], 2.0);
  EXPECT_DOUBLE_EQ(out[1], -0.7);
}

// ── 具名类（与控制层分组对齐）冒烟 ──

TEST(SteeringNamed, CooperativeControllerPassThrough) {
  SteeringCooperativeController c;
  c.set_group_size(4);
  c.initialize();
  Eigen::VectorXd d(4);
  d << 0.1, -0.2, 0.3, 0.4;
  Eigen::VectorXd out = c.compute(d);
  ASSERT_EQ(out.size(), 4);
  EXPECT_DOUBLE_EQ(out[0], 0.1);
  EXPECT_DOUBLE_EQ(out[1], -0.2);
  EXPECT_DOUBLE_EQ(out[3], 0.4);
}

TEST(SteeringNamed, JointControllerPid) {
  SteeringJointController c;
  PidParams p;
  p.kp = 2.0;
  p.output_limit = 100.0;
  c.configure(p);
  c.initialize();
  EXPECT_NEAR(c.compute(10.0, 0.0, 0.01), 20.0, 1e-9);
}

TEST(SteeringNamed, PlantPositionIntegration) {
  SteeringPlant plant;
  std::vector<JointLimit> limits(2);
  limits[0].max_vel = 10.0;
  limits[0].has_position_limit = true;
  limits[0].lower = -1.0;
  limits[0].upper = 1.0;
  limits[1].max_vel = 10.0;
  plant.configure(limits, false);
  plant.initialize(Eigen::VectorXd::Zero(2));

  Eigen::VectorXd u(2);
  u << 2.0, -1.0;
  for (int i = 0; i < 100; ++i) plant.update(u, 0.01);
  EXPECT_NEAR(plant.state()[0], 1.0, 1e-6);
  EXPECT_NEAR(plant.state()[1], -0.505, 1e-9);
}

TEST(SteeringNamed, PlantBypassReturnsInput) {
  SteeringPlant plant;
  std::vector<JointLimit> limits(2);
  plant.configure(limits, true);
  plant.initialize(Eigen::VectorXd::Zero(2));
  Eigen::VectorXd u(2);
  u << 1.5, -0.3;
  Eigen::VectorXd out = plant.update(u, 0.01);
  EXPECT_DOUBLE_EQ(out[0], 1.5);
  EXPECT_DOUBLE_EQ(out[1], -0.3);
}

TEST(WheelNamed, CooperativeAndJointWork) {
  WheelCooperativeController coop;
  coop.set_group_size(4);
  coop.initialize();
  WheelJointController j;
  PidParams p;
  p.kp = 1.0;
  p.output_limit = 100.0;
  j.configure(p);
  j.initialize();

  Eigen::VectorXd d = Eigen::VectorXd::Ones(4) * 5.0;
  Eigen::VectorXd sp = coop.compute(d);
  EXPECT_DOUBLE_EQ(sp[0], 5.0);
  EXPECT_DOUBLE_EQ(j.compute(sp[0], 0.0, 0.01), 5.0);
}

TEST(WheelNamed, PlantVelocityIntegration) {
  WheelPlant plant;
  std::vector<JointLimit> limits(2);
  limits[0].max_vel = 5.0;
  limits[1].max_vel = 5.0;
  plant.configure(limits, false);
  plant.initialize(Eigen::VectorXd::Zero(2));

  Eigen::VectorXd u(2);
  u << 2.0, 3.0;
  plant.update(u, 0.1);  // v += u*dt
  EXPECT_NEAR(plant.velocity()[0], 0.2, 1e-9);
  EXPECT_NEAR(plant.velocity()[1], 0.3, 1e-9);
}
