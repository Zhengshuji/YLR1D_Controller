#include "ylr1d_hmi/panels/translate_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QTabWidget>
#include <QScrollArea>
#include <QToolButton>

namespace ylr1d_hmi {

namespace {

/// 机械臂部件关节定义（顺序与 ylr1d_translate 的 kPartJoints 一致）
struct ArmJoint {
  const char * name;
  const char * label;
  double lower;
  double upper;
};

const std::vector<std::vector<ArmJoint>> & armDefs() {
  static const std::vector<std::vector<ArmJoint>> defs = {
    {  // 0 躯干 (4)
      {"Joint_Base_to_Body1",    "Lift",    -0.30,  0.30},
      {"Joint_Body1_to_Body2",   "Yaw",     -3.14,  3.14},
      {"Joint_Body2_to_Body3",   "Pitch1",  -1.57,  1.57},
      {"Joint_Body3_to_Body4",   "Pitch2",  -1.57,  1.57},
    },
    {  // 1 左臂 (9)
      {"Joint_Body2_to_LeftArm1",     "Shoulder1", -2.62,  2.62},
      {"Joint_LeftArm1_to_LeftArm2",  "Shoulder2", -1.57,  1.83},
      {"Joint_LeftArm2_to_LeftArm3",  "Shoulder3", -2.62,  2.62},
      {"Joint_LeftArm3_to_LeftArm4",  "Elbow1",    -1.57,  1.57},
      {"Joint_LeftArm4_to_LeftArm5",  "Elbow2",    -2.62,  2.62},
      {"Joint_LeftArm5_to_LeftArm6",  "Wrist1",    -2.09,  2.09},
      {"Joint_LeftArm6_to_LeftArm7",  "Wrist2",    -6.28,  6.28},
      {"Joint_LeftArm7_to_LeftFinger1","Finger1",  -0.015, 0.0},
      {"Joint_LeftArm7_to_LeftFinger2","Finger2",  -0.015, 0.0},
    },
    {  // 2 右臂 (9)
      {"Joint_Body2_to_RightArm1",    "Shoulder1", -2.62,  2.62},
      {"Joint_RightArm1_to_RightArm2","Shoulder2", -1.57,  1.83},
      {"Joint_RightArm2_to_RightArm3","Shoulder3", -2.62,  2.62},
      {"Joint_RightArm3_to_RightArm4","Elbow1",    -1.57,  1.57},
      {"Joint_RightArm4_to_RightArm5","Elbow2",    -2.62,  2.62},
      {"Joint_RightArm5_to_RightArm6","Wrist1",    -2.09,  2.09},
      {"Joint_RightArm6_to_RightArm7","Wrist2",    -6.28,  6.28},
      {"Joint_RightArm7_to_RightFinger1","Finger1", 0.0,  0.015},
      {"Joint_RightArm7_to_RightFinger2","Finger2", 0.0,  0.015},
    },
  };
  return defs;
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
// UI skeleton: three tabs
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

  main_lay->addWidget(tabs, 1);
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
  ch_mode_cb_->addItem(QStringLiteral("Translate (平移)"), 0);
  ch_mode_cb_->addItem(QStringLiteral("Rotate (旋转)"), 1);
  ch_mode_cb_->addItem(QStringLiteral("Stop (停车)"), 2);
  mode_row->addWidget(ch_mode_cb_, 1);
  lay->addLayout(mode_row);

  // ── direction ───────────────────────────────────────────
  auto dir_row = new QHBoxLayout();
  dir_row->addWidget(new QLabel(QStringLiteral("Direction: ")));
  auto dir_slider = new QSlider(Qt::Horizontal);
  dir_slider->setRange(-314, 314);  // [-pi, pi] rad * 100
  dir_slider->setValue(0);
  dir_row->addWidget(dir_slider, 1);
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
  ch_speed_sb_ = new QDoubleSpinBox();
  ch_speed_sb_->setDecimals(3);
  ch_speed_sb_->setRange(-5.0, 5.0);
  ch_speed_sb_->setSingleStep(0.1);
  ch_speed_sb_->setValue(0.0);
  ch_speed_sb_->setSuffix(QStringLiteral(" rad/s"));
  speed_row->addWidget(ch_speed_sb_, 1);
  lay->addLayout(speed_row);

  // ── duration ────────────────────────────────────────────
  auto dur_row = new QHBoxLayout();
  dur_row->addWidget(new QLabel(QStringLiteral("Duration: ")));
  ch_dur_sb_ = new QDoubleSpinBox();
  ch_dur_sb_->setDecimals(1);
  ch_dur_sb_->setRange(0.0, 60.0);
  ch_dur_sb_->setSingleStep(0.5);
  ch_dur_sb_->setValue(0.0);
  ch_dur_sb_->setSuffix(QStringLiteral(" s (0=until replaced/cancel)"));
  dur_row->addWidget(ch_dur_sb_, 1);
  lay->addLayout(dur_row);

  // slider <-> spin
  connect(dir_slider, &QSlider::valueChanged, this, [this](int v) {
    if (!ch_dir_sb_->hasFocus()) ch_dir_sb_->setValue(v / 100.0);
  });
  connect(ch_dir_sb_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this, dir_slider](double v) {
            dir_slider->setValue(static_cast<int>(v * 100));
          });

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

  ch_status_lbl_ = new QLabel(QStringLiteral("idle"));
  ch_status_lbl_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  lay->addWidget(ch_status_lbl_);

  lay->addStretch(1);
  return w;
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
  arm_part_cb_->addItem(QStringLiteral("Torso (躯干, 4)"), 0);
  arm_part_cb_->addItem(QStringLiteral("Left Arm (左臂, 9)"), 1);
  arm_part_cb_->addItem(QStringLiteral("Right Arm (右臂, 9)"), 2);
  part_row->addWidget(arm_part_cb_, 1);
  lay->addLayout(part_row);

  // ── Per-part joint rows ────────────────────────────────
  arm_stack_ = new QStackedWidget();
  arm_rows_.resize(armDefs().size());

  for (size_t p = 0; p < armDefs().size(); ++p) {
    auto page = new QWidget();
    auto page_lay = new QVBoxLayout(page);
    page_lay->setContentsMargins(0, 0, 0, 0);
    page_lay->setSpacing(2);

    const auto & defs = armDefs()[p];
    arm_rows_[p].resize(defs.size());
    for (size_t i = 0; i < defs.size(); ++i) {
      const ArmJoint & j = defs[i];
      auto row = new QHBoxLayout();
      auto lbl = new QLabel(QString::fromUtf8(j.label));
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

  arm_status_lbl_ = new QLabel(QStringLiteral("idle"));
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
  g_part_cb_->addItem(QStringLiteral("Left (左)"), 0);
  g_part_cb_->addItem(QStringLiteral("Right (右)"), 1);
  part_row->addWidget(g_part_cb_, 1);
  lay->addLayout(part_row);

  auto btn_row = new QHBoxLayout();
  auto open_btn = new QPushButton(QStringLiteral("Open (开)"));
  auto close_btn = new QPushButton(QStringLiteral("Close (关)"));
  btn_row->addWidget(open_btn, 1);
  btn_row->addWidget(close_btn, 1);
  lay->addLayout(btn_row);
  connect(open_btn, &QPushButton::clicked, this, [this]() { sendGripper(true); });
  connect(close_btn, &QPushButton::clicked, this, [this]() { sendGripper(false); });

  g_status_lbl_ = new QLabel(QStringLiteral("idle"));
  g_status_lbl_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  lay->addWidget(g_status_lbl_);

  lay->addStretch(1);
  return w;
}

// ============================================================
// Action senders
// ============================================================
void TranslatePanel::sendChassis() {
  if (!chassis_client_->wait_for_action_server(std::chrono::seconds(1))) {
    setStatus(ch_status_lbl_, QStringLiteral("ERROR: /chassis_move server not found"), false);
    return;
  }
  auto goal = ChassisMove::Goal();
  goal.mode = ch_mode_cb_->currentData().toInt();
  goal.direction = ch_dir_sb_->value();
  goal.speed = ch_speed_sb_->value();
  goal.duration = ch_dur_sb_->value();

  auto opts = rclcpp_action::Client<ChassisMove>::SendGoalOptions();
  opts.feedback_callback = [this](ChassisGoalHandle::SharedPtr,
                                   const std::shared_ptr<const ChassisMove::Feedback> fb) {
    ch_status_lbl_->setText(QStringLiteral("phase: %1").arg(QString::fromStdString(fb->phase)));
  };
  opts.result_callback = [this](const typename ChassisGoalHandle::WrappedResult & res) {
    if (res.result)
      setStatus(ch_status_lbl_,
                QStringLiteral("result success=%1 msg=%2")
                    .arg(res.result->success ? QStringLiteral("true") : QStringLiteral("false"))
                    .arg(QString::fromStdString(res.result->message)),
                res.result->success);
    else
      setStatus(ch_status_lbl_, QStringLiteral("goal aborted (code=%1)").arg(static_cast<int>(res.code)), false);
  };
  chassis_client_->async_send_goal(goal, opts);
  ch_status_lbl_->setText(QStringLiteral("sent: mode=%1 dir=%2 speed=%3 dur=%4")
                              .arg(static_cast<int>(goal.mode))
                              .arg(goal.direction, 0, 'f', 3)
                              .arg(goal.speed, 0, 'f', 3).arg(goal.duration, 0, 'f', 1));
}

void TranslatePanel::sendArm() {
  if (!arm_client_->wait_for_action_server(std::chrono::seconds(1))) {
    setStatus(arm_status_lbl_, QStringLiteral("ERROR: /arm_move server not found"), false);
    return;
  }
  auto goal = ArmMove::Goal();
  goal.part = arm_part_cb_->currentData().toInt();
  goal.positions.clear();
  for (const auto & jr : arm_rows_[static_cast<size_t>(goal.part)]) {
    goal.positions.push_back(jr.desired);
  }

  auto opts = rclcpp_action::Client<ArmMove>::SendGoalOptions();
  opts.feedback_callback = [this](ArmGoalHandle::SharedPtr,
                                   const std::shared_ptr<const ArmMove::Feedback> fb) {
    arm_status_lbl_->setText(QStringLiteral("progress: %1").arg(fb->progress, 0, 'f', 3));
  };
  opts.result_callback = [this](const typename ArmGoalHandle::WrappedResult & res) {
    if (res.result)
      setStatus(arm_status_lbl_,
                QStringLiteral("result success=%1 msg=%2")
                    .arg(res.result->success ? QStringLiteral("true") : QStringLiteral("false"))
                    .arg(QString::fromStdString(res.result->message)),
                res.result->success);
    else
      setStatus(arm_status_lbl_, QStringLiteral("goal aborted (code=%1)").arg(static_cast<int>(res.code)), false);
  };
  arm_client_->async_send_goal(goal, opts);
  arm_status_lbl_->setText(QStringLiteral("sent: part=%1 joints=%2")
                               .arg(static_cast<int>(goal.part))
                               .arg(static_cast<int>(goal.positions.size())));
}

void TranslatePanel::sendGripper(bool open) {
  if (!gripper_client_->wait_for_action_server(std::chrono::seconds(1))) {
    setStatus(g_status_lbl_, QStringLiteral("ERROR: /gripper_move server not found"), false);
    return;
  }
  auto goal = GripperMove::Goal();
  goal.part = g_part_cb_->currentData().toInt();
  goal.open = open;

  auto opts = rclcpp_action::Client<GripperMove>::SendGoalOptions();
  opts.feedback_callback = [this](GripperGoalHandle::SharedPtr,
                                   const std::shared_ptr<const GripperMove::Feedback> fb) {
    g_status_lbl_->setText(QStringLiteral("progress: %1").arg(fb->progress, 0, 'f', 3));
  };
  opts.result_callback = [this](const typename GripperGoalHandle::WrappedResult & res) {
    if (res.result)
      setStatus(g_status_lbl_,
                QStringLiteral("result success=%1 msg=%2")
                    .arg(res.result->success ? QStringLiteral("true") : QStringLiteral("false"))
                    .arg(QString::fromStdString(res.result->message)),
                res.result->success);
    else
      setStatus(g_status_lbl_, QStringLiteral("goal aborted (code=%1)").arg(static_cast<int>(res.code)), false);
  };
  gripper_client_->async_send_goal(goal, opts);
  g_status_lbl_->setText(open ? QStringLiteral("sent: open") : QStringLiteral("sent: close"));
}

// ============================================================
// Helpers
// ============================================================
void TranslatePanel::setStatus(QLabel * lbl, const QString & text, bool ok) {
  lbl->setText(QStringLiteral("<font color='%1'>%2</font>")
                   .arg(ok ? QStringLiteral("#2e7d32") : QStringLiteral("#c62828"), text));
}

}  // namespace ylr1d_hmi
