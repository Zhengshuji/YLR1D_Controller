#ifndef YLR1D_HMI__PANELS__TRANSLATE_PANEL_HPP_
#define YLR1D_HMI__PANELS__TRANSLATE_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ylr1d_translate/action/chassis_move.hpp"
#include "ylr1d_translate/action/arm_move.hpp"
#include "ylr1d_translate/action/gripper_move.hpp"

#include <QWidget>
#include <QLabel>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QStackedWidget>

#include <memory>
#include <string>
#include <vector>

namespace ylr1d_hmi {

/// 传输层 HMI：向 ylr1d_translate 发送 action goal
/// （/chassis_move、/arm_move、/gripper_move）。
///
/// 输出语义与 ylr1d_translate/action/*.action 对齐：
///   - ChassisMove: mode(0 平移 / 1 旋转 / 2 停车), direction(rad), speed, duration
///   - ArmMove:     part(0 躯干 / 1 左臂 / 2 右臂), positions(全部关节)
///   - GripperMove: part(0 左 / 1 右), open
class TranslatePanel : public QWidget {
  Q_OBJECT

public:
  explicit TranslatePanel(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);

private:
  using ChassisMove = ylr1d_translate::action::ChassisMove;
  using ArmMove = ylr1d_translate::action::ArmMove;
  using GripperMove = ylr1d_translate::action::GripperMove;
  using ChassisGoalHandle = rclcpp_action::ClientGoalHandle<ChassisMove>;
  using ArmGoalHandle = rclcpp_action::ClientGoalHandle<ArmMove>;
  using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperMove>;

  /// 单个关节控制行（滑块 + 数值框）
  struct JointRow {
    QSlider * slider{nullptr};
    QDoubleSpinBox * spin{nullptr};
    double desired{0.0};
  };

  // ── ROS ──
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ChassisMove>::SharedPtr chassis_client_;
  rclcpp_action::Client<ArmMove>::SharedPtr arm_client_;
  rclcpp_action::Client<GripperMove>::SharedPtr gripper_client_;

  // ── Chassis ──
  QComboBox * ch_mode_cb_{nullptr};
  QDoubleSpinBox * ch_dir_sb_{nullptr};
  QDoubleSpinBox * ch_speed_sb_{nullptr};
  QDoubleSpinBox * ch_dur_sb_{nullptr};
  QLabel * ch_status_lbl_{nullptr};

  // ── Arm ──
  QComboBox * arm_part_cb_{nullptr};
  QStackedWidget * arm_stack_{nullptr};
  std::vector<std::vector<JointRow>> arm_rows_;  // [part][joint]
  QLabel * arm_status_lbl_{nullptr};

  // ── Gripper ──
  QComboBox * g_part_cb_{nullptr};
  QLabel * g_status_lbl_{nullptr};

  // UI builders
  void buildUi();
  QWidget * buildCard(const QString & title, const QString & color, QWidget * content);
  QWidget * buildChassisTab();
  QWidget * buildArmTab();
  QWidget * buildGripperTab();

  // Actions
  void sendChassis();
  void sendArm();
  void sendGripper(bool open);

  void setStatus(QLabel * lbl, const QString & text, bool ok);
};

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__PANELS__TRANSLATE_PANEL_HPP_
