#ifndef YLR1D_ALGORITHM_SIM__ROS__SIM_CONTROLLER_HPP_
#define YLR1D_ALGORITHM_SIM__ROS__SIM_CONTROLLER_HPP_

#include "ylr1d_algorithm_sim/config/joint_config.hpp"
#include "ylr1d_algorithm_sim/control_law/pid.hpp"
#include "ylr1d_algorithm_sim/cooperative_controller.hpp"
#include "ylr1d_algorithm_sim/independent_controller.hpp"
#include "ylr1d_algorithm_sim/left_arm.hpp"
#include "ylr1d_algorithm_sim/plant.hpp"
#include "ylr1d_algorithm_sim/right_arm.hpp"
#include "ylr1d_algorithm_sim/steering.hpp"
#include "ylr1d_algorithm_sim/torso.hpp"
#include "ylr1d_algorithm_sim/wheel.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

namespace ylr1d_algorithm_sim {

/// 仿真控制器节点（模板）：一组关节的「控制器 + 仿真被控对象」整体。
///
/// 模板参数是该组的三个具名类（与控制层分组一一对应），算法全部在 src 中：
///   CooperativeT  协同控制器（组级），如 SteeringCooperativeController
///   JointT        逐关节控制器（每关节一个实例），如 SteeringJointController；
///                 模板要求提供 configure(const PidParams&)（pid.yaml 注入）
///   PlantT        仿真被控对象（组级整体），如 SteeringPlant
///
/// 固定线程框架（输入/输出交给 ROS2 topic，算法核心为纯 C++ 可单测）：
///   订阅 /ctrl/<组>/desired、/ctrl/<组>/feedback（缓存最新值）
///   timer(dt)：
///     1. 协同控制：组期望 → 逐关节 setpoint
///     2. 独立控制：逐关节控制量 u（内部算法经各组 src/<组>.cpp 选择）
///     3. 仿真被控对象：整体输入 u 向量，整体推进，输出物理量向量（bypass 直通）
///     4. 发布 /ctrl/<组>/output
///
/// 位置/速度差异由 group_def_->is_position 决定（与控制层对齐），不靠模板类型。
template <typename CooperativeT, typename JointT, typename PlantT>
class SimControllerNode : public rclcpp::Node {
public:
  static_assert(std::is_base_of<CooperativeController, CooperativeT>::value,
                "CooperativeT must derive from CooperativeController");
  static_assert(std::is_base_of<IndependentController, JointT>::value,
                "JointT must derive from IndependentController");
  static_assert(std::is_base_of<GroupPlant, PlantT>::value,
                "PlantT must derive from GroupPlant");

  explicit SimControllerNode(const rclcpp::NodeOptions & options)
      : Node("sim_controller", options) {
    group_name_ = declare_parameter<std::string>("group_name", "left_arm");
    double loop_hz = declare_parameter("loop_hz", 100.0);
    dt_ = 1.0 / loop_hz;

    group_def_ = jointGroupFor(group_name_);
    if (!group_def_) {
      RCLCPP_ERROR(get_logger(),
                   "unknown group_name '%s'（可选: steering/wheels/torso/left_arm/right_arm）",
                   group_name_.c_str());
      throw std::runtime_error("invalid group_name: " + group_name_);
    }
    const bool is_pos = group_def_->is_position;

    // 1) 协同控制器（本组具名类，内部算法经 src/control_law 选择）
    cooperative_ = std::make_unique<CooperativeT>();
    cooperative_->set_group_size(group_def_->count);

    // 2) 逐关节控制器（本组具名类，参数 <关节>/pid/* 来自 pid.yaml）
    joints_.reserve(group_def_->count);
    const double def_kp = is_pos ? 150.0 : 4.0;
    const double def_kd = is_pos ? 20.0 : 0.1;
    for (size_t i = 0; i < group_def_->count; ++i) {
      const std::string & n = group_def_->joints[i];
      PidParams p;
      p.kp = declare_parameter(n + "/pid/kp", def_kp);
      p.ki = declare_parameter(n + "/pid/ki", 0.0);
      p.kd = declare_parameter(n + "/pid/kd", def_kd);
      p.output_limit = jointLimitFor(n).max_accel;
      auto j = std::make_unique<JointT>();
      j->configure(p);
      joints_.push_back(std::move(j));
    }

    // 3) 仿真被控对象（本组具名类）：限位来自头文件静态配置（单一来源）
    plant_ = std::make_unique<PlantT>();
    std::vector<JointLimit> limits(group_def_->count);
    for (size_t i = 0; i < group_def_->count; ++i) {
      limits[i] = jointLimitFor(group_def_->joints[i]);
    }
    plant_->configure(limits, false);

    desired_.setZero(static_cast<Eigen::Index>(group_def_->count));
    feedback_.setZero(static_cast<Eigen::Index>(group_def_->count));

    // 4) 通信（topic 由本层约定，控制层对应转发）
    const std::string base = "/ctrl/" + group_name_;
    output_pub_ = create_publisher<sensor_msgs::msg::JointState>(base + "/output", 10);
    desired_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        base + "/desired", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr m) { desired_callback(m); });
    feedback_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        base + "/feedback", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr m) { feedback_callback(m); });

    timer_ = create_wall_timer(std::chrono::duration<double>(dt_),
                               [this]() { update(); });

    RCLCPP_INFO(get_logger(), "%s sim started (%s, %zu joints, %.1f Hz)",
                group_name_.c_str(), is_pos ? "position" : "velocity",
                group_def_->count, loop_hz);
  }

private:
  bool is_position() const { return group_def_->is_position; }

  /// 从 JointState 按关节名取值：位置组取 position、速度组取 velocity（与控制层对齐）。
  static double field_of(const sensor_msgs::msg::JointState & msg,
                         const std::string & name, bool position_field) {
    const std::vector<double> & vec = position_field ? msg.position : msg.velocity;
    for (size_t i = 0; i < msg.name.size() && i < vec.size(); ++i) {
      if (msg.name[i] == name) {
        double v = vec[i];
        return std::isnan(v) ? 0.0 : v;
      }
    }
    return 0.0;
  }

  void desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < group_def_->count; ++i) {
      desired_[static_cast<Eigen::Index>(i)] =
          field_of(*msg, group_def_->joints[i], is_position());
    }
    have_desired_ = true;
  }

  void feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < group_def_->count; ++i) {
      feedback_[static_cast<Eigen::Index>(i)] =
          field_of(*msg, group_def_->joints[i], is_position());
    }
    have_feedback_ = true;
  }

  void update() {
    if (!initialized_) {
      if (!have_desired_) return;  // 只等首帧期望（软仿真不依赖反馈首帧，避免死锁）
      // 初始化：plant 初始状态 = 首帧期望（物理一致性：位置组先按限位钳制，避免越限状态），
      // feedback 同步为 plant 初始状态 → 首轮误差为 0（无瞬态）
      Eigen::VectorXd init = desired_;
      if (is_position()) {
        for (size_t i = 0; i < group_def_->count; ++i) {
          const JointLimit lim = jointLimitFor(group_def_->joints[i]);
          if (lim.has_position_limit) {
            init[static_cast<Eigen::Index>(i)] =
                std::clamp(init[i], lim.lower, lim.upper);
          }
        }
      }
      plant_->initialize(init);
      feedback_ = init;
      cooperative_->initialize();
      for (auto & ctrl : joints_) ctrl->initialize();
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "%s initialized from first desired", group_name_.c_str());
    }

    // 1) 协同控制：组期望 → 逐关节 setpoint
    const Eigen::VectorXd setpoint = cooperative_->compute(desired_);
    // 2) 独立控制：逐关节
    Eigen::VectorXd u(static_cast<Eigen::Index>(joints_.size()));
    for (size_t i = 0; i < joints_.size(); ++i) {
      u[static_cast<Eigen::Index>(i)] =
          joints_[i]->compute(setpoint[static_cast<Eigen::Index>(i)],
                              feedback_[static_cast<Eigen::Index>(i)], dt_);
    }
    // 3) 仿真被控对象：整体推进（bypass 直通）
    const Eigen::VectorXd out = plant_->update(u, dt_);
    // 4) 发布 output
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    for (size_t i = 0; i < group_def_->count; ++i) {
      msg.name.push_back(group_def_->joints[i]);
      if (is_position()) {
        msg.position.push_back(out[static_cast<Eigen::Index>(i)]);
        msg.velocity.push_back(plant_->velocity()[static_cast<Eigen::Index>(i)]);
      } else {
        msg.velocity.push_back(out[static_cast<Eigen::Index>(i)]);
      }
      msg.effort.push_back(0.0);
    }
    output_pub_->publish(msg);
  }

  std::string group_name_;
  const JointGroupDef * group_def_{nullptr};
  double dt_{0.01};

  std::unique_ptr<CooperativeController> cooperative_;       // 协同控制器（本组具名类）
  std::vector<std::unique_ptr<IndependentController>> joints_;  // 逐关节控制器（每关节一个实例）
  std::unique_ptr<GroupPlant> plant_;                        // 仿真被控对象（本组具名类）

  Eigen::VectorXd desired_;
  Eigen::VectorXd feedback_;
  bool have_desired_{false};
  bool have_feedback_{false};
  bool initialized_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr output_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/// 5 个具名仿真控制器节点（与控制层分组一一对应，供 composition 加载）。
class SteeringSimNode
    : public SimControllerNode<SteeringCooperativeController,
                               SteeringJointController, SteeringPlant> {
public:
  explicit SteeringSimNode(const rclcpp::NodeOptions & options)
      : SimControllerNode<SteeringCooperativeController, SteeringJointController,
                          SteeringPlant>(options) {}
};

class WheelSimNode
    : public SimControllerNode<WheelCooperativeController,
                               WheelJointController, WheelPlant> {
public:
  explicit WheelSimNode(const rclcpp::NodeOptions & options)
      : SimControllerNode<WheelCooperativeController, WheelJointController,
                          WheelPlant>(options) {}
};

class TorsoSimNode
    : public SimControllerNode<TorsoCooperativeController,
                               TorsoJointController, TorsoPlant> {
public:
  explicit TorsoSimNode(const rclcpp::NodeOptions & options)
      : SimControllerNode<TorsoCooperativeController, TorsoJointController,
                          TorsoPlant>(options) {}
};

class LeftArmSimNode
    : public SimControllerNode<LeftArmCooperativeController,
                               LeftArmJointController, LeftArmPlant> {
public:
  explicit LeftArmSimNode(const rclcpp::NodeOptions & options)
      : SimControllerNode<LeftArmCooperativeController, LeftArmJointController,
                          LeftArmPlant>(options) {}
};

class RightArmSimNode
    : public SimControllerNode<RightArmCooperativeController,
                               RightArmJointController, RightArmPlant> {
public:
  explicit RightArmSimNode(const rclcpp::NodeOptions & options)
      : SimControllerNode<RightArmCooperativeController, RightArmJointController,
                          RightArmPlant>(options) {}
};

}  // namespace ylr1d_algorithm_sim

#endif  // YLR1D_ALGORITHM_SIM__ROS__SIM_CONTROLLER_HPP_
