#include "ylr1d_translate/translate_node.hpp"
#include "ylr1d_translate/constants.hpp"
#include "ylr1d_translate/joint_names.hpp"

#include <algorithm>
#include <cmath>

namespace ylr1d_translate {

// 命名空间作用域的 action 类型别名（供自由辅助函数 resolve_chassis / finger_target 使用）
using ChassisMove = ylr1d_translate::action::ChassisMove;
using ArmMove = ylr1d_translate::action::ArmMove;
using GripperMove = ylr1d_translate::action::GripperMove;

TranslateNode::TranslateNode() : Node("translate_server") {
  // 订阅控制层反馈（用于转向 / 到位检测）
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { joint_state_cb(msg); });

  // 发布期望关节状态到控制层
  desired_pub_ = create_publisher<sensor_msgs::msg::JointState>("desired_joint_states", 10);

  // ── action server ──────────────────────────────────
  chassis_server_ = rclcpp_action::create_server<ChassisMove>(
      this, "chassis_move",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ChassisMove::Goal> g) {
        return chassis_goal_cb(*g);
      },
      [this](std::shared_ptr<ChassisGoalHandle> h) { return chassis_cancel_cb(h); },
      [this](std::shared_ptr<ChassisGoalHandle> h) { chassis_execute_cb(h); });

  arm_server_ = rclcpp_action::create_server<ArmMove>(
      this, "arm_move",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ArmMove::Goal> g) {
        return arm_goal_cb(*g);
      },
      [this](std::shared_ptr<ArmGoalHandle> h) { return arm_cancel_cb(h); },
      [this](std::shared_ptr<ArmGoalHandle> h) { arm_execute_cb(h); });

  gripper_server_ = rclcpp_action::create_server<GripperMove>(
      this, "gripper_move",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const GripperMove::Goal> g) {
        return gripper_goal_cb(*g);
      },
      [this](std::shared_ptr<GripperGoalHandle> h) { return gripper_cancel_cb(h); },
      [this](std::shared_ptr<GripperGoalHandle> h) { gripper_execute_cb(h); });

  timer_ = create_wall_timer(std::chrono::duration<double>(constants::LOOP_DT),
                             [this]() { timer_cb(); });

  RCLCPP_INFO(get_logger(), "translate_server started: chassis_move / arm_move / gripper_move");
}

// ===================================================================
// 话题回调
// ===================================================================

void TranslateNode::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (!std::isnan(msg->position[i])) {
      cur_pos_[msg->name[i]] = msg->position[i];
    }
  }
}

void TranslateNode::timer_cb() {
  tick_chassis();
  tick_arm();
  tick_gripper();
  publish_desired();
}

// ===================================================================
// 底盘（先转向，后移动）
// ===================================================================

rclcpp_action::GoalResponse TranslateNode::chassis_goal_cb(const ChassisMove::Goal & goal) {
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TranslateNode::chassis_cancel_cb(std::shared_ptr<ChassisGoalHandle> h) {
  (void)h;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TranslateNode::chassis_execute_cb(std::shared_ptr<ChassisGoalHandle> h) {
  RCLCPP_INFO(get_logger(), "chassis_execute_cb invoked (is_executing=%d)", (int)h->is_executing());
  // ACCEPT_AND_EXECUTE 时 rcl_action 可能已使 goal 进入 EXECUTING，重复 execute() 会抛错
  if (!h->is_executing()) {
    h->execute();
  }
  if (chassis_.active() && chassis_.handle != h) {
    set_result(chassis_.handle, false, "superseded by new goal");
  }
  const auto & goal = h->get_goal();
  chassis_.handle = h;
  chassis_.mode = goal->mode;
  chassis_.direction = goal->direction;
  chassis_.speed = goal->speed;
  chassis_.duration = goal->duration;
  chassis_.phase = ChassisPhase::kSteering;
}

/// 依据模式解算 4 转向角 与 4 轮速（未做"先转向"钳制，钳制在 publish 层）
static void resolve_chassis(const ChassisMove::Goal & goal,
                            std::array<double, 4> & steer,
                            std::array<double, 4> & wheel) {
  using namespace constants;
  switch (goal.mode) {
    case MODE_TRANSLATE:
      steer = {goal.direction, goal.direction, goal.direction, goal.direction};
      wheel = {goal.speed, goal.speed, goal.speed, goal.speed};
      break;
    case MODE_ROTATE:
      steer = ROTATE_STEERING;
      wheel = {goal.speed / ROTATE_RATIO, goal.speed / ROTATE_RATIO,
               goal.speed / ROTATE_RATIO, goal.speed / ROTATE_RATIO};
      break;
    case MODE_STOP:
    default:
      steer = STOP_STEERING;
      wheel = {0.0, 0.0, 0.0, 0.0};
      break;
  }
}

void TranslateNode::tick_chassis() {
  using namespace constants;
  if (!chassis_.active()) return;
  auto h = chassis_.handle;

  if (h->is_canceling()) {
    // 取消：停车（轮速归零），保持当前转向
    set_result(h, false, "canceled");
    return;
  }

  const auto & goal = h->get_goal();
  std::array<double, 4> steer_targets{};
  std::array<double, 4> wheel_targets{};
  resolve_chassis(*goal, steer_targets, wheel_targets);

  auto steering_names = std::vector<std::string>(joints::kSteering.begin(),
                                                 joints::kSteering.end());
  auto steering_goal = std::vector<double>(steer_targets.begin(), steer_targets.end());

  // 停靠目标：转向到位即完成
  if (goal->mode == MODE_STOP) {
    if (all_reached(steering_names, steering_goal, TURN_TOL)) {
      set_result(h, true, "stopped");
    }
    return;
  }

  // 平移 / 旋转：先转向（轮速 0），到位后进入移动阶段
  const bool steered = all_reached(steering_names, steering_goal, TURN_TOL);

  if (chassis_.phase == ChassisPhase::kSteering) {
    if (steered) {
      chassis_.phase = ChassisPhase::kMoving;
      chassis_.move_start = now();
    }
  } else {  // kMoving
    if (chassis_.duration > 0.0 &&
        (now() - chassis_.move_start).seconds() >= chassis_.duration) {
      set_result(h, true, "move done");
      return;
    }
  }

  // 发布反馈
  auto fb = std::make_shared<ChassisMove::Feedback>();
  switch (chassis_.phase) {
    case ChassisPhase::kSteering: fb->phase = "steering"; break;
    case ChassisPhase::kMoving: fb->phase = "moving"; break;
    default: fb->phase = "stopped"; break;
  }
  h->publish_feedback(fb);
}

// ===================================================================
// 机械臂 / 躯干
// ===================================================================

rclcpp_action::GoalResponse TranslateNode::arm_goal_cb(const ArmMove::Goal & goal) {
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TranslateNode::arm_cancel_cb(std::shared_ptr<ArmGoalHandle> h) {
  (void)h;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TranslateNode::arm_execute_cb(std::shared_ptr<ArmGoalHandle> h) {
  RCLCPP_INFO(get_logger(), "arm_execute_cb invoked (is_executing=%d)", (int)h->is_executing());
  if (!h->is_executing()) {
    h->execute();
  }
  if (arm_.active() && arm_.handle != h) {
    set_result(arm_.handle, false, "superseded by new goal");
  }
  arm_.handle = h;
  arm_.part = h->get_goal()->part;
  arm_.positions = h->get_goal()->positions;
  arm_.start = now();
}

void TranslateNode::tick_arm() {
  using namespace constants;
  if (!arm_.active()) return;
  auto h = arm_.handle;
  if (h->is_canceling()) {
    set_result(h, false, "canceled");
    return;
  }

  const auto & names = joints::kPartJoints()[arm_.part];
  std::vector<std::string> ns(names.begin(), names.end());

  if (arm_.positions.size() != ns.size()) {
    set_result(h, false, "positions size mismatch");
    return;
  }

  // 反馈：已到位关节占比
  size_t reached = 0;
  for (size_t i = 0; i < ns.size(); ++i) {
    auto it = cur_pos_.find(ns[i]);
    if (it != cur_pos_.end() && std::abs(it->second - arm_.positions[i]) <= ARM_TOL) {
      ++reached;
    }
  }
  auto fb = std::make_shared<ArmMove::Feedback>();
  fb->progress = ns.empty() ? 1.0 : static_cast<double>(reached) / ns.size();
  h->publish_feedback(fb);

  if (reached == ns.size()) {
    set_result(h, true, "arm reached target");
    return;
  }
  if ((now() - arm_.start).seconds() > ARM_TIMEOUT) {
    set_result(h, false, "timeout");
  }
}

// ===================================================================
// 夹爪
// ===================================================================

rclcpp_action::GoalResponse TranslateNode::gripper_goal_cb(const GripperMove::Goal & goal) {
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TranslateNode::gripper_cancel_cb(std::shared_ptr<GripperGoalHandle> h) {
  (void)h;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TranslateNode::gripper_execute_cb(std::shared_ptr<GripperGoalHandle> h) {
  RCLCPP_INFO(get_logger(), "gripper_execute_cb invoked (is_executing=%d)", (int)h->is_executing());
  if (!h->is_executing()) {
    h->execute();
  }
  if (gripper_.active() && gripper_.handle != h) {
    set_result(gripper_.handle, false, "superseded by new goal");
  }
  gripper_.handle = h;
  gripper_.part = h->get_goal()->part;
  gripper_.open = h->get_goal()->open;
  gripper_.start = now();
}

/// 夹爪开合 → 夹指目标位置
static double finger_target(int8_t part, bool open) {
  using namespace constants;
  if (part == GRIPPER_LEFT) {
    return open ? LEFT_FINGER_OPEN : LEFT_FINGER_CLOSE;
  }
  return open ? RIGHT_FINGER_OPEN : RIGHT_FINGER_CLOSE;
}

void TranslateNode::tick_gripper() {
  using namespace constants;
  if (!gripper_.active()) return;
  auto h = gripper_.handle;
  if (h->is_canceling()) {
    set_result(h, false, "canceled");
    return;
  }

  const double target = finger_target(gripper_.part, gripper_.open);
  const auto & fingers = (gripper_.part == GRIPPER_LEFT) ? joints::kLeftFingers
                                                         : joints::kRightFingers;

  size_t reached = 0;
  for (const auto & n : fingers) {
    auto it = cur_pos_.find(n);
    if (it != cur_pos_.end() && std::abs(it->second - target) <= ARM_TOL) {
      ++reached;
    }
  }
  auto fb = std::make_shared<GripperMove::Feedback>();
  fb->progress = static_cast<double>(reached) / fingers.size();
  h->publish_feedback(fb);

  if (reached == fingers.size()) {
    set_result(h, true, "gripper set");
    return;
  }
  if ((now() - gripper_.start).seconds() > ARM_TIMEOUT) {
    set_result(h, false, "timeout");
  }
}

// ===================================================================
// 发布 /desired_joint_states（按关节名，控制层按名更新）
// ===================================================================

void TranslateNode::publish_desired() {
  using namespace constants;
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = now();

  if (chassis_.active()) {
    std::array<double, 4> steer_targets{};
    std::array<double, 4> wheel_targets{};
    resolve_chassis(*chassis_.handle->get_goal(), steer_targets, wheel_targets);

    // 平移 / 旋转模式：转向阶段轮速保持 0（先转向后移动）
    if (chassis_.mode != MODE_STOP && chassis_.phase == ChassisPhase::kSteering) {
      wheel_targets = {0.0, 0.0, 0.0, 0.0};
    }
    // 注：JointState 要求 name/position/velocity 三数组等长且按索引对齐，
    // 控制层按 name[i] 取 position[i] / velocity[i]。未用字段填 0 占位。
    for (size_t i = 0; i < joints::kSteering.size(); ++i) {
      msg.name.push_back(joints::kSteering[i]);
      msg.position.push_back(steer_targets[i]);
      msg.velocity.push_back(0.0);
    }
    for (size_t i = 0; i < joints::kWheels.size(); ++i) {
      msg.name.push_back(joints::kWheels[i]);
      msg.position.push_back(0.0);
      msg.velocity.push_back(wheel_targets[i]);
    }
  }

  if (arm_.active()) {
    const auto & names = joints::kPartJoints()[arm_.part];
    for (size_t i = 0; i < names.size() && i < arm_.positions.size(); ++i) {
      msg.name.push_back(names[i]);
      msg.position.push_back(arm_.positions[i]);
      msg.velocity.push_back(0.0);
    }
  }

  if (gripper_.active()) {
    const double target = finger_target(gripper_.part, gripper_.open);
    const auto & fingers = (gripper_.part == GRIPPER_LEFT) ? joints::kLeftFingers
                                                           : joints::kRightFingers;
    for (const auto & n : fingers) {
      msg.name.push_back(n);
      msg.position.push_back(target);
      msg.velocity.push_back(0.0);
    }
  }

  if (!msg.name.empty()) {
    desired_pub_->publish(msg);
  }
}

// ===================================================================
// 工具
// ===================================================================

bool TranslateNode::all_reached(const std::vector<std::string> & names,
                                const std::vector<double> & targets, double tol) const {
  if (names.empty()) return false;
  for (size_t i = 0; i < names.size(); ++i) {
    auto it = cur_pos_.find(names[i]);
    if (it == cur_pos_.end() || std::abs(it->second - targets[i]) > tol) {
      return false;
    }
  }
  return true;
}

void TranslateNode::set_result(std::shared_ptr<ChassisGoalHandle> h, bool ok, const std::string & msg) {
  if (!h) return;
  auto result = std::make_shared<ChassisMove::Result>();
  result->success = ok;
  result->message = msg;
  if (ok) h->succeed(result);
  else h->abort(result);
  if (chassis_.handle == h) {
    chassis_.handle.reset();
    chassis_.phase = ChassisPhase::kIdle;
  }
  RCLCPP_INFO(get_logger(), "chassis_move %s: %s", ok ? "succeeded" : "aborted", msg.c_str());
}

void TranslateNode::set_result(std::shared_ptr<ArmGoalHandle> h, bool ok, const std::string & msg) {
  if (!h) return;
  auto result = std::make_shared<ArmMove::Result>();
  result->success = ok;
  result->message = msg;
  if (ok) h->succeed(result);
  else h->abort(result);
  if (arm_.handle == h) arm_.handle.reset();
  RCLCPP_INFO(get_logger(), "arm_move %s: %s", ok ? "succeeded" : "aborted", msg.c_str());
}

void TranslateNode::set_result(std::shared_ptr<GripperGoalHandle> h, bool ok, const std::string & msg) {
  if (!h) return;
  auto result = std::make_shared<GripperMove::Result>();
  result->success = ok;
  result->message = msg;
  if (ok) h->succeed(result);
  else h->abort(result);
  if (gripper_.handle == h) gripper_.handle.reset();
  RCLCPP_INFO(get_logger(), "gripper_move %s: %s", ok ? "succeeded" : "aborted", msg.c_str());
}

}  // namespace ylr1d_translate
