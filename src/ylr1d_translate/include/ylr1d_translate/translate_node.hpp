#ifndef YLR1D_TRANSLATE__TRANSLATE_NODE_HPP_
#define YLR1D_TRANSLATE__TRANSLATE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ylr1d_translate/action/arm_move.hpp>
#include <ylr1d_translate/action/chassis_move.hpp>
#include <ylr1d_translate/action/gripper_move.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ylr1d_translate {

/// 转译层节点：接收上层 action 指令，解算后发布 /desired_joint_states 到控制层。
///
/// 提供三个 action server：
///   - chassis_move   : 小车底座（运动方式 + 方向 + 速度 → 转向角 + 轮速）
///   - arm_move       : 机械臂 / 躯干（部位名 + 关节坐标 → 期望位置）
///   - gripper_move   : 夹爪（布尔开合 → 夹指位置）
///
/// 统一定时器推进三个目标；"先转向后移动"在底盘目标内分两阶段完成。
class TranslateNode : public rclcpp::Node {
public:
  TranslateNode();

  using ChassisMove = ylr1d_translate::action::ChassisMove;
  using ArmMove = ylr1d_translate::action::ArmMove;
  using GripperMove = ylr1d_translate::action::GripperMove;

  using ChassisGoalHandle = rclcpp_action::ServerGoalHandle<ChassisMove>;
  using ArmGoalHandle = rclcpp_action::ServerGoalHandle<ArmMove>;
  using GripperGoalHandle = rclcpp_action::ServerGoalHandle<GripperMove>;

  // ── 底盘目标阶段 ────────────────────────────────────
  enum class ChassisPhase { kIdle, kSteering, kMoving };

private:
  // ── 底盘目标状态 ────────────────────────────────────
  struct ChassisGoalState {
    std::shared_ptr<ChassisGoalHandle> handle;
    int8_t mode{0};
    double direction{0.0};
    double speed{0.0};
    double duration{0.0};
    ChassisPhase phase{ChassisPhase::kIdle};
    rclcpp::Time move_start;
    bool active() const { return handle != nullptr; }
  };
  ChassisGoalState chassis_;

  // ── 机械臂 / 夹爪目标状态 ────────────────────────────
  struct ArmGoalState {
    std::shared_ptr<ArmGoalHandle> handle;
    int8_t part{0};
    std::vector<double> positions;
    rclcpp::Time start;
    bool active() const { return handle != nullptr; }
  };
  ArmGoalState arm_;

  struct GripperGoalState {
    std::shared_ptr<GripperGoalHandle> handle;
    int8_t part{0};
    bool open{false};
    rclcpp::Time start;
    bool active() const { return handle != nullptr; }
  };
  GripperGoalState gripper_;

  // ── 话题 ────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::map<std::string, double> cur_pos_;  // 当前关节位置（来自 /joint_states）

  // ── action server ───────────────────────────────────
  rclcpp_action::Server<ChassisMove>::SharedPtr chassis_server_;
  rclcpp_action::Server<ArmMove>::SharedPtr arm_server_;
  rclcpp_action::Server<GripperMove>::SharedPtr gripper_server_;

  // ── 回调 ────────────────────────────────────────────
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void timer_cb();

  rclcpp_action::GoalResponse chassis_goal_cb(const ChassisMove::Goal & goal);
  rclcpp_action::CancelResponse chassis_cancel_cb(std::shared_ptr<ChassisGoalHandle> h);
  void chassis_execute_cb(std::shared_ptr<ChassisGoalHandle> h);

  rclcpp_action::GoalResponse arm_goal_cb(const ArmMove::Goal & goal);
  rclcpp_action::CancelResponse arm_cancel_cb(std::shared_ptr<ArmGoalHandle> h);
  void arm_execute_cb(std::shared_ptr<ArmGoalHandle> h);

  rclcpp_action::GoalResponse gripper_goal_cb(const GripperMove::Goal & goal);
  rclcpp_action::CancelResponse gripper_cancel_cb(std::shared_ptr<GripperGoalHandle> h);
  void gripper_execute_cb(std::shared_ptr<GripperGoalHandle> h);

  // ── 内部逻辑 ────────────────────────────────────────
  void tick_chassis();
  void tick_arm();
  void tick_gripper();
  void publish_desired();

  bool all_reached(const std::vector<std::string> & names,
                   const std::vector<double> & targets, double tol) const;
  void set_result(std::shared_ptr<ChassisGoalHandle> h, bool ok, const std::string & msg);
  void set_result(std::shared_ptr<ArmGoalHandle> h, bool ok, const std::string & msg);
  void set_result(std::shared_ptr<GripperGoalHandle> h, bool ok, const std::string & msg);
};

}  // namespace ylr1d_translate

#endif  // YLR1D_TRANSLATE__TRANSLATE_NODE_HPP_
