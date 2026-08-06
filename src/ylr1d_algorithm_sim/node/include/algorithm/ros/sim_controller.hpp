#ifndef ALGORITHM__ROS__SIM_CONTROLLER_HPP_
#define ALGORITHM__ROS__SIM_CONTROLLER_HPP_

#include "algorithm/config/joint_config.hpp"
#include "controller/controller.hpp"
#include "plant/plant.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <type_traits>
#include <vector>

namespace ylr1d_algorithm_sim {

/// 仿真控制器节点（模板）：一组关节的"控制器 + 仿真"整体。
///
/// 固定线程框架（输入/输出交给 ROS2 topic，算法核心为纯 C++ 可单测）：
///   订阅 /ctrl/<组>/desired、/ctrl/<组>/feedback（缓存最新值）
///   timer(dt)：
///     1. 协同控制：组期望 → 逐关节 setpoint（当前 P=1 比例，透传）
///     2. 独立控制：逐关节 PID 计算控制量 u（参数 <关节>/pid/*，来自 pid.yaml）
///     3. 仿真对象：整体输入 u 向量，整体推进，输出物理量向量（bypass 直通）
///     4. 发布 /ctrl/<组>/output
///
/// @tparam GroupPlantT 仿真对象类型：PositionGroupPlant（位置，1/s²）或
///                     VelocityGroupPlant（速度，1/s）；决定装配参数与反馈取值。
template <typename GroupPlantT>
class SimControllerNode : public rclcpp::Node {
public:
  explicit SimControllerNode(const rclcpp::NodeOptions & options)
      : Node("sim_controller", options) {
    group_name_ = declare_parameter<std::string>("group_name", "left_arm");
    double loop_hz = declare_parameter("loop_hz", 100.0);
    dt_ = 1.0 / loop_hz;

    group_def_ = jointGroupFor(group_name_);
    if (!group_def_) {
      RCLCPP_ERROR(get_logger(), "unknown group_name '%s'（可选: steering/wheels/torso/left_arm/right_arm）",
                   group_name_.c_str());
      throw std::runtime_error("invalid group_name: " + group_name_);
    }

    // ── 独立控制器（逐关节 PID）：参数 <关节>/pid/*（pid.yaml 经 launch 注入） ──
    independent_.reserve(group_def_->count);
    for (size_t i = 0; i < group_def_->count; ++i) {
      const std::string & n = group_def_->joints[i];
      PidParams p;
      if constexpr (std::is_same_v<GroupPlantT, PositionGroupPlant>) {
        p.kp = declare_parameter(n + "/pid/kp", 150.0);
        p.ki = declare_parameter(n + "/pid/ki", 0.0);
        p.kd = declare_parameter(n + "/pid/kd", 20.0);
      } else {
        p.kp = declare_parameter(n + "/pid/kp", 4.0);
        p.ki = declare_parameter(n + "/pid/ki", 0.0);
        p.kd = declare_parameter(n + "/pid/kd", 0.1);
      }
      p.output_limit = jointLimitFor(n).max_accel;
      PidController pid;
      pid.configure(p);
      independent_.push_back(std::move(pid));
    }

    // ── 仿真对象（整体）：限位来自头文件静态配置（单一来源） ──
    if constexpr (std::is_same_v<GroupPlantT, PositionGroupPlant>) {
      std::vector<PositionPlantParams> params(group_def_->count);
      for (size_t i = 0; i < group_def_->count; ++i) {
        const JointLimit lim = jointLimitFor(group_def_->joints[i]);
        params[i].max_vel = lim.max_vel;
        params[i].has_position_limit = lim.has_position_limit;
        params[i].lower = lim.lower;
        params[i].upper = lim.upper;
      }
      plant_.configure(params);
    } else {
      std::vector<VelocityPlantParams> params(group_def_->count);
      for (size_t i = 0; i < group_def_->count; ++i) {
        params[i].max_vel = jointLimitFor(group_def_->joints[i]).max_vel;
      }
      plant_.configure(params);
    }

    // ── 协同控制器（P=1 比例，当前透传） ──
    cooperative_.configure(1.0);

    desired_.setZero(group_def_->count);
    feedback_.setZero(group_def_->count);

    // ── 通信（topic 由本层约定，控制层对应转发） ──
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

    RCLCPP_INFO(get_logger(), "%s sim_controller started (%s, %zu joints, %.1f Hz)",
                group_name_.c_str(),
                std::is_same_v<GroupPlantT, PositionGroupPlant> ? "position" : "velocity",
                group_def_->count, loop_hz);
  }

private:
  /// 从 JointState 按关节名取值：位置组取 position、速度组取 velocity（模板决定）。
  static double field_of(const sensor_msgs::msg::JointState & msg, const std::string & name) {
    const std::vector<double> * vec = nullptr;
    if constexpr (std::is_same_v<GroupPlantT, PositionGroupPlant>) {
      vec = &msg.position;
    } else {
      vec = &msg.velocity;
    }
    for (size_t i = 0; i < msg.name.size() && i < vec->size(); ++i) {
      if (msg.name[i] == name) {
        double v = (*vec)[i];
        return std::isnan(v) ? 0.0 : v;
      }
    }
    return 0.0;
  }

  void desired_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < group_def_->count; ++i) {
      desired_[i] = field_of(*msg, group_def_->joints[i]);
    }
    have_desired_ = true;
  }

  void feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < group_def_->count; ++i) {
      feedback_[i] = field_of(*msg, group_def_->joints[i]);
    }
    have_feedback_ = true;
  }

  void update() {
    if (!initialized_) {
      if (!have_desired_) return;  // 只等首帧期望（软仿真不依赖反馈首帧，避免死锁）
      // 初始化：plant 初始状态 = 首帧期望（物理一致性：位置组先按限位钳制，避免越限状态），
      // feedback 同步为 plant 初始状态 → 首轮误差为 0（无瞬态）
      if constexpr (std::is_same_v<GroupPlantT, PositionGroupPlant>) {
        Eigen::VectorXd init = desired_;
        for (size_t i = 0; i < group_def_->count; ++i) {
          const JointLimit lim = jointLimitFor(group_def_->joints[i]);
          if (lim.has_position_limit) {
            init[i] = std::clamp(init[i], lim.lower, lim.upper);
          }
        }
        plant_.initialize(init, Eigen::VectorXd::Zero(init.size()));
        feedback_ = init;
      } else {
        plant_.initialize(desired_);
        feedback_ = desired_;
      }
      cooperative_.initialize();
      for (auto & pid : independent_) pid.initialize();
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "%s initialized from first desired", group_name_.c_str());
    }

    // 1) 协同控制：组期望 → 逐关节 setpoint（P=1 透传，未来协同在此扩展）
    const Eigen::VectorXd setpoint = cooperative_.compute(desired_);
    // 2) 独立控制：逐关节 PID
    Eigen::VectorXd u(independent_.size());
    for (size_t i = 0; i < independent_.size(); ++i) {
      u[i] = independent_[i].compute(setpoint[i], feedback_[i], dt_);
    }
    // 3) 仿真对象：整体推进（bypass 直通）
    const Eigen::VectorXd out = plant_.update(u, dt_);
    // 4) 发布 output
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    for (size_t i = 0; i < group_def_->count; ++i) {
      msg.name.push_back(group_def_->joints[i]);
      if constexpr (std::is_same_v<GroupPlantT, PositionGroupPlant>) {
        msg.position.push_back(out[i]);
        msg.velocity.push_back(plant_.velocity()[i]);
      } else {
        msg.velocity.push_back(out[i]);
      }
      msg.effort.push_back(0.0);
    }
    output_pub_->publish(msg);
  }

  std::string group_name_;
  const JointGroupDef * group_def_{nullptr};
  double dt_{0.01};

  ProportionalController cooperative_;       // 协同控制器（P=1）
  std::vector<PidController> independent_;  // 独立控制器（逐关节 PID）
  GroupPlantT plant_;                       // 仿真对象（整体）

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

/// 位置型仿真控制器（steering / torso / left_arm / right_arm）。
class PositionSimController : public SimControllerNode<PositionGroupPlant> {
public:
  explicit PositionSimController(const rclcpp::NodeOptions & options)
      : SimControllerNode<PositionGroupPlant>(options) {}
};

/// 速度型仿真控制器（wheels）。
class VelocitySimController : public SimControllerNode<VelocityGroupPlant> {
public:
  explicit VelocitySimController(const rclcpp::NodeOptions & options)
      : SimControllerNode<VelocityGroupPlant>(options) {}
};

}  // namespace ylr1d_algorithm_sim

#endif  // ALGORITHM__ROS__SIM_CONTROLLER_HPP_
