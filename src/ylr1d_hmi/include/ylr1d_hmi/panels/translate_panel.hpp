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
#include <QPlainTextEdit>
#include <QTimer>

#include <memory>
#include <string>
#include <vector>

namespace ylr1d_hmi {

class ChassisDirectionWidget;

/// 传输层 HMI：向 ylr1d_translate 发送 action goal
/// （/chassis_move、/arm_move、/gripper_move）。
///
/// 面板底部提供 action 通信状态区：三个 server 的连接指示（1s 刷新）+ 事件日志。
/// Chassis 标签页按运动模式联动：Translate 速度单位 m/s（车体线速度）、
/// Rotate 单位 rad/s（车体角速度）；无效控制量禁用并重置为默认。
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
  ChassisDirectionWidget * ch_dir_widget_{nullptr};
  QSlider * ch_dir_slider_{nullptr};
  QDoubleSpinBox * ch_dir_sb_{nullptr};
  QSlider * ch_speed_slider_{nullptr};
  QDoubleSpinBox * ch_speed_sb_{nullptr};
  QSlider * ch_dur_slider_{nullptr};
  QDoubleSpinBox * ch_dur_sb_{nullptr};
  QLabel * ch_hint_lbl_{nullptr};
  QLabel * ch_status_lbl_{nullptr};

  // ── Arm ──
  QComboBox * arm_part_cb_{nullptr};
  QStackedWidget * arm_stack_{nullptr};
  std::vector<std::vector<JointRow>> arm_rows_;  // [part][joint]
  QLabel * arm_status_lbl_{nullptr};

  // ── Gripper ──
  QComboBox * g_part_cb_{nullptr};
  QLabel * g_status_lbl_{nullptr};

  // ── ROS / action status ──
  QLabel * conn_lbl_{nullptr};
  QPlainTextEdit * log_view_{nullptr};
  QTimer * conn_timer_{nullptr};

  // feedback 节流记录状态（避免高频反馈刷屏）
  QString last_ch_phase_;
  double last_arm_prog_{-1.0};
  double last_gripper_prog_{-1.0};

  // UI builders
  void buildUi();
  QWidget * buildCard(const QString & title, const QString & color, QWidget * content);
  QWidget * buildChassisTab();
  QWidget * buildArmTab();
  QWidget * buildGripperTab();
  QWidget * buildStatusPanel();

  // Chassis helpers
  void applyChassisMode();
  void updateDirectionWidget();

  // Actions
  void sendChassis();
  void sendArm();
  void sendGripper(bool open);

  void setStatus(QLabel * lbl, const QString & text, bool ok);
  void appendLog(const QString & msg);
  void refreshConnections();
};

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__PANELS__TRANSLATE_PANEL_HPP_
