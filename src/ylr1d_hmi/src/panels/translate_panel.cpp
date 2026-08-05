#include "ylr1d_hmi/panels/translate_panel.hpp"

#include "ylr1d_hmi/config/joint_defs.hpp"
#include "ylr1d_hmi/common/action_sender.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QTabWidget>
#include <QScrollArea>
#include <QToolButton>
#include <QTime>

#include <algorithm>
#include <cmath>

namespace ylr1d_hmi {

namespace {

// ── 人类可读名称（用于日志 / 状态）───────────────────────────────
const char * chassisModeName(int8_t m) {
  switch (m) {
    case 0: return "Translate";
    case 1: return "Rotate";
    case 2: return "Stop";
    default: return "?";
  }
}
const char * armPartName(int8_t p) {
  switch (p) {
    case 0: return "Torso";
    case 1: return "LeftArm";
    case 2: return "RightArm";
    default: return "?";
  }
}
const char * gripperPartName(int8_t p) {
  switch (p) {
    case 0: return "Left";
    case 1: return "Right";
    default: return "?";
  }
}

}  // namespace

// ============================================================
// Construction: action clients + UI
// ============================================================
TranslatePanel::TranslatePanel(rclcpp::Node::SharedPtr node, QWidget * parent)
  : QWidget(parent), node_(node)
{
  // ── Action clients ──────────────────────────────────────
  chassis_client_ = rclcpp_action::create_client<ChassisMove>(node_, "chassis_move");
  arm_client_ = rclcpp_action::create_client<ArmMove>(node_, "arm_move");
  gripper_client_ = rclcpp_action::create_client<GripperMove>(node_, "gripper_move");

  buildUi();
}

// ============================================================
// UI skeleton: three tabs + bottom status panel
// ============================================================
void TranslatePanel::buildUi() {
  auto main_lay = new QVBoxLayout(this);
  main_lay->setContentsMargins(6, 6, 6, 2);
  main_lay->setSpacing(6);

  auto tabs = new QTabWidget();
  tabs->setDocumentMode(true);
  tabs->addTab(buildCard(QStringLiteral("Chassis Move"), QStringLiteral("#2d6cdf"),
                         buildChassisTab()),
               QStringLiteral("Chassis"));
  tabs->addTab(buildCard(QStringLiteral("Arm Move"), QStringLiteral("#e07b00"),
                         buildArmTab()),
               QStringLiteral("Arm"));
  tabs->addTab(buildCard(QStringLiteral("Gripper Move"), QStringLiteral("#8e44ad"),
                         buildGripperTab()),
               QStringLiteral("Gripper"));

  main_lay->addWidget(tabs, 3);
  main_lay->addWidget(buildStatusPanel(), 1);
}

QWidget * TranslatePanel::buildCard(const QString & title, const QString & color,
                                    QWidget * content) {
  auto card = new QGroupBox();
  card->setStyleSheet(QString(
    "QGroupBox{border:1px solid %1; border-radius:6px; margin-top:10px;}"
    "QGroupBox::title{subcontrol-origin:margin; left:8px; padding:0 4px;"
    " color:%1; font-weight:bold;}").arg(color));
  card->setTitle(title);
  auto lay = new QVBoxLayout(card);
  lay->setContentsMargins(6, 6, 6, 6);
  lay->setSpacing(4);
  lay->addWidget(content, 1);
  return card;
}

// ============================================================
// Bottom status panel: server connection dots + event log
// ============================================================
QWidget * TranslatePanel::buildStatusPanel() {
  auto w = new QWidget();
  auto lay = new QVBoxLayout(w);
  lay->setContentsMargins(0, 0, 0, 0);
  lay->setSpacing(2);

  conn_lbl_ = new QLabel(QStringLiteral("Checking action servers..."));
  conn_lbl_->setTextFormat(Qt::RichText);
  conn_lbl_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  lay->addWidget(conn_lbl_);

  log_view_ = new QPlainTextEdit();
  log_view_->setReadOnly(true);
  log_view_->setMaximumBlockCount(500);
  log_view_->setMaximumHeight(130);
  log_view_->setPlaceholderText(QStringLiteral("Action communication log..."));
  lay->addWidget(log_view_, 1);

  conn_timer_ = new QTimer(this);
  conn_timer_->setInterval(1000);
  connect(conn_timer_, &QTimer::timeout, this, &TranslatePanel::refreshConnections);
  conn_timer_->start();
  refreshConnections();

  return w;
}

// ============================================================
// Chassis tab
// ============================================================
QWidget * TranslatePanel::buildChassisTab() {
  auto w = new QWidget();
  auto lay = new QVBoxLayout(w);
  lay->setContentsMargins(0, 0, 0, 0);
  lay->setSpacing(4);

  // ── mode ────────────────────────────────────────────────
  auto mode_row = new QHBoxLayout();
  mode_row->addWidget(new QLabel(QStringLiteral("Mode: ")));
  ch_mode_cb_ = new QComboBox();
  ch_mode_cb_->addItem(QStringLiteral("Translate"), 0);
  ch_mode_cb_->addItem(QStringLiteral("Rotate"), 1);
  ch_mode_cb_->addItem(QStringLiteral("Stop"), 2);
  mode_row->addWidget(ch_mode_cb_, 1);
  lay->addLayout(mode_row);

  // ── direction visualization ─────────────────────────────
  ch_dir_widget_ = new ChassisDirectionWidget();
  ch_dir_widget_->setMinimumWidth(200);
  lay->addWidget(ch_dir_widget_);

  // ── direction ───────────────────────────────────────────
  auto dir_row = new QHBoxLayout();
  dir_row->addWidget(new QLabel(QStringLiteral("Direction: ")));
  ch_dir_slider_ = new QSlider(Qt::Horizontal);
  ch_dir_slider_->setRange(-314, 314);  // [-pi, pi] rad * 100
  ch_dir_slider_->setValue(0);
  dir_row->addWidget(ch_dir_slider_, 1);
  ch_dir_sb_ = new QDoubleSpinBox();
  ch_dir_sb_->setDecimals(3);
  ch_dir_sb_->setRange(-3.141592653589793, 3.141592653589793);
  ch_dir_sb_->setSingleStep(0.05);
  ch_dir_sb_->setValue(0.0);
  ch_dir_sb_->setSuffix(QStringLiteral(" rad"));
  dir_row->addWidget(ch_dir_sb_);
  lay->addLayout(dir_row);

  // ── speed ───────────────────────────────────────────────
  auto speed_row = new QHBoxLayout();
  speed_row->addWidget(new QLabel(QStringLiteral("Speed: ")));
  ch_speed_slider_ = new QSlider(Qt::Horizontal);
  ch_speed_slider_->setRange(-250, 250);  // [-2.5, 2.5] m/s * 100
  ch_speed_slider_->setValue(0);
  speed_row->addWidget(ch_speed_slider_, 1);
  ch_speed_sb_ = new QDoubleSpinBox();
  ch_speed_sb_->setDecimals(2);
  ch_speed_sb_->setRange(-2.5, 2.5);
  ch_speed_sb_->setSingleStep(0.05);
  ch_speed_sb_->setValue(0.0);
  ch_speed_sb_->setSuffix(QStringLiteral(" m/s"));
  speed_row->addWidget(ch_speed_sb_);
  lay->addLayout(speed_row);

  // ── duration ────────────────────────────────────────────
  auto dur_row = new QHBoxLayout();
  dur_row->addWidget(new QLabel(QStringLiteral("Duration: ")));
  ch_dur_slider_ = new QSlider(Qt::Horizontal);
  ch_dur_slider_->setRange(0, 6000);  // [0, 60] s * 100
  ch_dur_slider_->setValue(0);
  dur_row->addWidget(ch_dur_slider_, 1);
  ch_dur_sb_ = new QDoubleSpinBox();
  ch_dur_sb_->setDecimals(1);
  ch_dur_sb_->setRange(0.0, 60.0);
  ch_dur_sb_->setSingleStep(0.5);
  ch_dur_sb_->setValue(0.0);
  ch_dur_sb_->setSuffix(QStringLiteral(" s"));
  dur_row->addWidget(ch_dur_sb_);
  lay->addLayout(dur_row);

  // slider <-> spin（同步后刷新方向可视化）
  connect(ch_dir_slider_, &QSlider::valueChanged, this, [this](int v) {
    if (!ch_dir_sb_->hasFocus()) ch_dir_sb_->setValue(v / 100.0);
    updateDirectionWidget();
  });
  connect(ch_dir_sb_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this](double v) {
            ch_dir_slider_->setValue(static_cast<int>(v * 100));
            updateDirectionWidget();
          });
  connect(ch_speed_slider_, &QSlider::valueChanged, this, [this](int v) {
    if (!ch_speed_sb_->hasFocus()) ch_speed_sb_->setValue(v / 100.0);
    updateDirectionWidget();
  });
  connect(ch_speed_sb_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this](double v) {
            ch_speed_slider_->setValue(static_cast<int>(v * 100));
            updateDirectionWidget();
          });
  connect(ch_dur_slider_, &QSlider::valueChanged, this, [this](int v) {
    if (!ch_dur_sb_->hasFocus()) ch_dur_sb_->setValue(v / 100.0);
  });
  connect(ch_dur_sb_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this](double v) { ch_dur_slider_->setValue(static_cast<int>(v * 100)); });

  // mode 切换：单位 / 有效控制量联动
  connect(ch_mode_cb_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this](int) { applyChassisMode(); });

  // ── buttons ─────────────────────────────────────────────
  auto btn_row = new QHBoxLayout();
  auto send_btn = new QPushButton(QStringLiteral("Send"));
  auto stop_btn = new QPushButton(QStringLiteral("Stop"));
  btn_row->addWidget(send_btn, 1);
  btn_row->addWidget(stop_btn, 1);
  lay->addLayout(btn_row);
  connect(send_btn, &QPushButton::clicked, this, &TranslatePanel::sendChassis);
  connect(stop_btn, &QPushButton::clicked, this, [this]() {
    ch_mode_cb_->setCurrentIndex(2);  // MODE_STOP
    sendChassis();
  });

  ch_status_lbl_ = new QLabel(QStringLiteral("Ready"));
  ch_status_lbl_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  lay->addWidget(ch_status_lbl_);

  ch_hint_lbl_ = new QLabel();
  ch_hint_lbl_->setWordWrap(true);
  ch_hint_lbl_->setStyleSheet(QStringLiteral("color:#777; font-size:11px;"));
  lay->addWidget(ch_hint_lbl_);

  applyChassisMode();
  return w;
}

// ============================================================
// Chassis mode helpers
// ============================================================
void TranslatePanel::applyChassisMode() {
  const int mode = ch_mode_cb_->currentData().toInt();
  const bool tr = (mode == 0);
  const bool rot = (mode == 1);
  const bool stop = (mode == 2);

  // direction：仅平移有效
  ch_dir_slider_->setEnabled(tr);
  ch_dir_sb_->setEnabled(tr);
  if (!tr) {
    ch_dir_slider_->setValue(0);
    ch_dir_sb_->setValue(0);
  }

  // speed：平移(车体线速度 m/s) / 旋转(车体角速度 rad/s)；停车无效
  ch_speed_slider_->setEnabled(!stop);
  ch_speed_sb_->setEnabled(!stop);
  if (stop) {
    ch_speed_slider_->setValue(0);
    ch_speed_sb_->setValue(0);
  }
  if (tr) {
    ch_speed_slider_->setRange(-250, 250);
    ch_speed_sb_->setRange(-2.5, 2.5);
    ch_speed_sb_->setSuffix(QStringLiteral(" m/s"));
  } else if (rot) {
    ch_speed_slider_->setRange(-500, 500);
    ch_speed_sb_->setRange(-5.0, 5.0);
    ch_speed_sb_->setSuffix(QStringLiteral(" rad/s"));
  } else {
    ch_speed_sb_->setSuffix(QStringLiteral(""));
  }

  // duration：平移 / 旋转有效；停车无效
  ch_dur_slider_->setEnabled(!stop);
  ch_dur_sb_->setEnabled(!stop);
  if (stop) {
    ch_dur_slider_->setValue(0);
    ch_dur_sb_->setValue(0);
  }

  // 说明文字（各模式语义，代替控件内联注释）
  if (tr) {
    ch_hint_lbl_->setText(QStringLiteral(
      "Translate: speed = chassis linear velocity (m/s), direction = heading angle (rad). "
      "Duration 0 = run until replaced / canceled."));
  } else if (rot) {
    ch_hint_lbl_->setText(QStringLiteral(
      "Rotate: speed = chassis angular velocity (rad/s); direction is ignored. "
      "Duration 0 = run until replaced / canceled."));
  } else {
    ch_hint_lbl_->setText(QStringLiteral(
      "Stop: fixed steering angle, zero wheel speed."));
  }

  updateDirectionWidget();
}

void TranslatePanel::updateDirectionWidget() {
  if (!ch_dir_widget_) return;
  ch_dir_widget_->setParams(ch_mode_cb_->currentData().toInt(),
                            ch_dir_sb_->value(), ch_speed_sb_->value());
}

// ============================================================
// Arm tab: part selector + per-joint rows (stacked by part)
// ============================================================
QWidget * TranslatePanel::buildArmTab() {
  auto w = new QWidget();
  auto lay = new QVBoxLayout(w);
  lay->setContentsMargins(0, 0, 0, 0);
  lay->setSpacing(4);

  auto part_row = new QHBoxLayout();
  part_row->addWidget(new QLabel(QStringLiteral("Part: ")));
  arm_part_cb_ = new QComboBox();
  arm_part_cb_->addItem(QStringLiteral("Torso (4)"), 0);
  arm_part_cb_->addItem(QStringLiteral("Left Arm (7)"), 1);
  arm_part_cb_->addItem(QStringLiteral("Right Arm (7)"), 2);
  part_row->addWidget(arm_part_cb_, 1);
  lay->addLayout(part_row);

  // ── Per-part joint rows ────────────────────────────────
  arm_stack_ = new QStackedWidget();
  arm_rows_.resize(armPartJoints().size());

  for (size_t p = 0; p < armPartJoints().size(); ++p) {
    auto page = new QWidget();
    auto page_lay = new QVBoxLayout(page);
    page_lay->setContentsMargins(0, 0, 0, 0);
    page_lay->setSpacing(2);

    const auto & defs = armPartJoints()[p];
    arm_rows_[p].resize(defs.size());
    for (size_t i = 0; i < defs.size(); ++i) {
      const auto & j = defs[i];
      auto row = new QHBoxLayout();
      auto lbl = new QLabel(j.label);
      lbl->setFixedWidth(70);
      auto slider = new QSlider(Qt::Horizontal);
      slider->setRange(static_cast<int>(j.lower * 100),
                       static_cast<int>(j.upper * 100));
      slider->setValue(0);
      auto spin = new QDoubleSpinBox();
      spin->setDecimals(3);
      spin->setRange(j.lower, j.upper);
      spin->setSingleStep(0.01);
      spin->setValue(0.0);
      row->addWidget(lbl);
      row->addWidget(slider, 1);
      row->addWidget(spin);

      JointRow & jr = arm_rows_[p][i];
      jr.slider = slider;
      jr.spin = spin;

      connect(slider, &QSlider::valueChanged, this, [this, p, i](int v) {
        if (!arm_rows_[p][i].spin->hasFocus())
          arm_rows_[p][i].spin->setValue(v / 100.0);
        arm_rows_[p][i].desired = v / 100.0;
      });
      connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
              [this, p, i](double v) {
                arm_rows_[p][i].desired = v;
                arm_rows_[p][i].slider->setValue(static_cast<int>(v * 100));
              });

      page_lay->addLayout(row);
    }
    page_lay->addStretch(1);

    auto scroll = new QScrollArea();
    scroll->setWidget(page);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    arm_stack_->addWidget(scroll);
  }
  lay->addWidget(arm_stack_, 1);
  connect(arm_part_cb_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          [this](int idx) { arm_stack_->setCurrentIndex(idx); });

  auto send_btn = new QPushButton(QStringLiteral("Send ArmMove"));
  lay->addWidget(send_btn);
  connect(send_btn, &QPushButton::clicked, this, &TranslatePanel::sendArm);

  arm_status_lbl_ = new QLabel(QStringLiteral("Ready"));
  arm_status_lbl_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  lay->addWidget(arm_status_lbl_);
  return w;
}

// ============================================================
// Gripper tab
// ============================================================
QWidget * TranslatePanel::buildGripperTab() {
  auto w = new QWidget();
  auto lay = new QVBoxLayout(w);
  lay->setContentsMargins(0, 0, 0, 0);
  lay->setSpacing(4);

  auto part_row = new QHBoxLayout();
  part_row->addWidget(new QLabel(QStringLiteral("Part: ")));
  g_part_cb_ = new QComboBox();
  g_part_cb_->addItem(QStringLiteral("Left"), 0);
  g_part_cb_->addItem(QStringLiteral("Right"), 1);
  part_row->addWidget(g_part_cb_, 1);
  lay->addLayout(part_row);

  auto btn_row = new QHBoxLayout();
  auto open_btn = new QPushButton(QStringLiteral("Open"));
  auto close_btn = new QPushButton(QStringLiteral("Close"));
  btn_row->addWidget(open_btn, 1);
  btn_row->addWidget(close_btn, 1);
  lay->addLayout(btn_row);
  connect(open_btn, &QPushButton::clicked, this, [this]() { sendGripper(true); });
  connect(close_btn, &QPushButton::clicked, this, [this]() { sendGripper(false); });

  g_status_lbl_ = new QLabel(QStringLiteral("Ready"));
  g_status_lbl_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  lay->addWidget(g_status_lbl_);

  lay->addStretch(1);
  return w;
}

// ============================================================
// Action senders
// ============================================================
void TranslatePanel::sendChassis() {
  if (!waitForActionServer<ChassisMove>(chassis_client_, QStringLiteral("chassis_move"),
                                        ch_status_lbl_, log_view_))
    return;
  auto goal = ChassisMove::Goal();
  goal.mode = ch_mode_cb_->currentData().toInt();
  goal.direction = ch_dir_sb_->value();
  goal.speed = ch_speed_sb_->value();
  goal.duration = ch_dur_sb_->value();

  last_ch_phase_.clear();
  auto opts = makeGoalOptions<ChassisMove>(QStringLiteral("chassis_move"),
                                           ch_status_lbl_, log_view_);
  opts.feedback_callback = [this](ChassisGoalHandle::SharedPtr,
                                   const std::shared_ptr<const ChassisMove::Feedback> fb) {
    const QString phase = QString::fromStdString(fb->phase);
    ch_status_lbl_->setText(QStringLiteral("phase: %1").arg(phase));
    if (phase != last_ch_phase_) {  // 阶段变化才记录（steering → moving → stopped）
      last_ch_phase_ = phase;
      appendLog(QStringLiteral("chassis_move: feedback phase=%1").arg(phase));
    }
  };
  chassis_client_->async_send_goal(goal, opts);
  ch_status_lbl_->setText(QStringLiteral("sent: %1 dir=%2 speed=%3 dur=%4")
                              .arg(QString::fromUtf8(chassisModeName(goal.mode)))
                              .arg(goal.direction, 0, 'f', 3)
                              .arg(goal.speed, 0, 'f', 2).arg(goal.duration, 0, 'f', 1));
  appendLog(QStringLiteral("chassis_move: sent goal mode=%1 dir=%2 rad speed=%3 dur=%4 s")
                .arg(QString::fromUtf8(chassisModeName(goal.mode)))
                .arg(goal.direction, 0, 'f', 3)
                .arg(goal.speed, 0, 'f', 2).arg(goal.duration, 0, 'f', 1));
}

void TranslatePanel::sendArm() {
  if (!waitForActionServer<ArmMove>(arm_client_, QStringLiteral("arm_move"),
                                    arm_status_lbl_, log_view_))
    return;
  auto goal = ArmMove::Goal();
  goal.part = arm_part_cb_->currentData().toInt();
  goal.positions.clear();
  for (const auto & jr : arm_rows_[static_cast<size_t>(goal.part)]) {
    goal.positions.push_back(jr.desired);
  }

  last_arm_prog_ = -1.0;
  auto opts = makeGoalOptions<ArmMove>(QStringLiteral("arm_move"),
                                       arm_status_lbl_, log_view_);
  opts.feedback_callback = [this](ArmGoalHandle::SharedPtr,
                                   const std::shared_ptr<const ArmMove::Feedback> fb) {
    const double pr = fb->progress;
    arm_status_lbl_->setText(QStringLiteral("progress: %1%").arg(pr * 100.0, 0, 'f', 0));
    if (pr - last_arm_prog_ >= 0.25 || (pr >= 1.0 && last_arm_prog_ < 1.0)) {  // 25% 步进节流
      last_arm_prog_ = pr;
      appendLog(QStringLiteral("arm_move: feedback progress %1%")
                    .arg(pr * 100.0, 0, 'f', 0));
    }
  };
  arm_client_->async_send_goal(goal, opts);
  arm_status_lbl_->setText(QStringLiteral("sent: %1 joints=%2")
                               .arg(QString::fromUtf8(armPartName(goal.part)))
                               .arg(static_cast<int>(goal.positions.size())));
  appendLog(QStringLiteral("arm_move: sent goal part=%1 joints=%2")
                .arg(QString::fromUtf8(armPartName(goal.part)))
                .arg(static_cast<int>(goal.positions.size())));
}

void TranslatePanel::sendGripper(bool open) {
  if (!waitForActionServer<GripperMove>(gripper_client_, QStringLiteral("gripper_move"),
                                        g_status_lbl_, log_view_))
    return;
  auto goal = GripperMove::Goal();
  goal.part = g_part_cb_->currentData().toInt();
  goal.open = open;

  last_gripper_prog_ = -1.0;
  auto opts = makeGoalOptions<GripperMove>(QStringLiteral("gripper_move"),
                                           g_status_lbl_, log_view_);
  opts.feedback_callback = [this](GripperGoalHandle::SharedPtr,
                                   const std::shared_ptr<const GripperMove::Feedback> fb) {
    const double pr = fb->progress;
    g_status_lbl_->setText(QStringLiteral("progress: %1%").arg(pr * 100.0, 0, 'f', 0));
    if (pr - last_gripper_prog_ >= 0.25 || (pr >= 1.0 && last_gripper_prog_ < 1.0)) {  // 25% 步进节流
      last_gripper_prog_ = pr;
      appendLog(QStringLiteral("gripper_move: feedback progress %1%")
                    .arg(pr * 100.0, 0, 'f', 0));
    }
  };
  gripper_client_->async_send_goal(goal, opts);
  g_status_lbl_->setText(QStringLiteral("sent: %1 %2")
                             .arg(QString::fromUtf8(gripperPartName(goal.part)))
                             .arg(open ? QStringLiteral("Open") : QStringLiteral("Close")));
  appendLog(QStringLiteral("gripper_move: sent goal part=%1 action=%2")
                .arg(QString::fromUtf8(gripperPartName(goal.part)))
                .arg(open ? QStringLiteral("Open") : QStringLiteral("Close")));
}

// ============================================================
// Helpers
// ============================================================
void TranslatePanel::appendLog(const QString & msg) {
  if (!log_view_) return;
  log_view_->appendPlainText(QTime::currentTime().toString(QStringLiteral("HH:mm:ss"))
                             + QStringLiteral("  ") + msg);
}

void TranslatePanel::refreshConnections() {
  if (!conn_lbl_) return;
  const bool c = chassis_client_ && chassis_client_->action_server_is_ready();
  const bool a = arm_client_ && arm_client_->action_server_is_ready();
  const bool g = gripper_client_ && gripper_client_->action_server_is_ready();
  auto cell = [](bool ok, const char * name) {
    return QStringLiteral("<span style='color:%1'>%2</span> <b>%3</b> %4")
        .arg(ok ? QStringLiteral("#2e7d32") : QStringLiteral("#c62828"))
        .arg(ok ? QStringLiteral("●") : QStringLiteral("○"), QString::fromUtf8(name),
             ok ? QStringLiteral("ready") : QStringLiteral("waiting"));
  };
  conn_lbl_->setText(QStringLiteral("Action servers&nbsp;&nbsp;") + cell(c, "chassis_move") +
                     QStringLiteral("&nbsp;&nbsp;") + cell(a, "arm_move") +
                     QStringLiteral("&nbsp;&nbsp;") + cell(g, "gripper_move"));
}

}  // namespace ylr1d_hmi
