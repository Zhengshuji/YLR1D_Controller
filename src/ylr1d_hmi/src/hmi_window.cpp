#include "ylr1d_hmi/hmi_window.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>
#include <QHeaderView>
#include <QStatusBar>
#include <QPalette>
#include <QAbstractItemView>
#include <QTreeWidgetItem>

namespace ylr1d_hmi {

// ============================================================
// Joint group definitions
// Limits/units are kept in sync with
//   src/ylr1d_mid_control/config/limits.yaml  (what Gazebo actually loads)
// ============================================================
static std::vector<JointDef> chassis_joints = {
  {"Joint_Base_to_RFWheelF",    "RF_Steer",  false, false, -3.14, 3.14},
  {"Joint_Base_to_LFWheelF",    "LF_Steer",  false, false, -3.14, 3.14},
  {"Joint_Base_to_RBWheelF",    "RB_Steer",  false, false, -3.14, 3.14},
  {"Joint_Base_to_LBWheelF",    "LB_Steer",  false, false, -3.14, 3.14},
  {"Joint_RFWheelF_to_RFWheel", "RF_Wheel",  true},
  {"Joint_LFWheelF_to_LFWheel", "LF_Wheel",  true},
  {"Joint_RBWheelF_to_RBWheel", "RB_Wheel",  true},
  {"Joint_LBWheelF_to_LBWheel", "LB_Wheel",  true},
};

static std::vector<JointDef> torso_joints = {
  {"Joint_Base_to_Body1",    "Lift",    false, true,  -0.30,  0.30},
  {"Joint_Body1_to_Body2",   "Yaw",     false, false, -3.14,  3.14},
  {"Joint_Body2_to_Body3",   "Pitch1",  false, false, -1.57,  1.57},
  {"Joint_Body3_to_Body4",   "Pitch2",  false, false, -1.57,  1.57},
};

static std::vector<JointDef> left_arm_joints = {
  {"Joint_Body2_to_LeftArm1",    "Shoulder1", false, false, -2.62,  2.62},
  {"Joint_LeftArm1_to_LeftArm2", "Shoulder2", false, false, -1.57,  1.83},
  {"Joint_LeftArm2_to_LeftArm3", "Shoulder3", false, false, -2.62,  2.62},
  {"Joint_LeftArm3_to_LeftArm4", "Elbow1",    false, false, -1.57,  1.57},
  {"Joint_LeftArm4_to_LeftArm5", "Elbow2",    false, false, -2.62,  2.62},
  {"Joint_LeftArm5_to_LeftArm6", "Wrist1",    false, false, -2.09,  2.09},
  {"Joint_LeftArm6_to_LeftArm7", "Wrist2",    false, false, -6.28,  6.28},
  {"Joint_LeftArm7_to_LeftFinger1","Finger1", false, true,  -0.015, 0.0},
  {"Joint_LeftArm7_to_LeftFinger2","Finger2", false, true,  -0.015, 0.0},
};

static std::vector<JointDef> right_arm_joints = {
  {"Joint_Body2_to_RightArm1",   "Shoulder1", false, false, -2.62,  2.62},
  {"Joint_RightArm1_to_RightArm2","Shoulder2", false, false, -1.57,  1.83},
  {"Joint_RightArm2_to_RightArm3","Shoulder3", false, false, -2.62,  2.62},
  {"Joint_RightArm3_to_RightArm4","Elbow1",    false, false, -1.57,  1.57},
  {"Joint_RightArm4_to_RightArm5","Elbow2",    false, false, -2.62,  2.62},
  {"Joint_RightArm5_to_RightArm6","Wrist1",    false, false, -2.09,  2.09},
  {"Joint_RightArm6_to_RightArm7","Wrist2",    false, false, -6.28,  6.28},
  {"Joint_RightArm7_to_RightFinger1","Finger1", false, true,  0.0, 0.015},
  {"Joint_RightArm7_to_RightFinger2","Finger2", false, true,  0.0, 0.015},
};

// ============================================================
// HmiWindow
// ============================================================

HmiWindow::HmiWindow(rclcpp::Node::SharedPtr node)
  : node_(node)
{
  // ROS2 subscribe / publish
  js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        auto it = name_to_idx_.find(msg->name[i]);
        if (it != name_to_idx_.end()) {
          auto & j = joints_[it->second];
          j.position = std::isnan(msg->position[i]) ? j.position : msg->position[i];
          if (i < msg->velocity.size())
            j.velocity = std::isnan(msg->velocity[i]) ? 0.0 : msg->velocity[i];
        }
      }
      ++js_count_;
    });

  desired_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/desired_joint_states", 10);

  // NOTE: buildUiLite() is NOT called here — the caller (main or subclass)
  // decides which layout to use.

  // ROS spin timer (50 Hz)
  ros_timer_ = new QTimer(this);
  connect(ros_timer_, &QTimer::timeout, this, &HmiWindow::onRosSpin);
  ros_timer_->start(20);

  // Low-frequency send timer: GUI ops only set desired_dirty_,
  // the actual publish happens here (default 5 Hz).
  send_timer_ = new QTimer(this);
  connect(send_timer_, &QTimer::timeout, this, &HmiWindow::onSendTimer);
  send_timer_->start(200);
}

HmiWindow::~HmiWindow() {
  ros_timer_->stop();
  if (send_timer_) send_timer_->stop();
}

// ============================================================
// Lite layout: four tabs (Observe on top, Control below)
// ============================================================
void HmiWindow::buildUiLite() {
  setWindowTitle("YLR1D HMI (lite)");
  resize(1200, 800);

  auto central = new QWidget(this);
  setCentralWidget(central);
  auto main_layout = new QVBoxLayout(central);
  main_layout->setContentsMargins(6, 6, 6, 2);
  main_layout->setSpacing(6);

  // Four tabs, one joint group per tab
  auto tabs = new QTabWidget();
  tabs->setDocumentMode(true);

  struct TabSpec {
    QString title;
    QString color;
    std::vector<JointDef> * defs;
  };
  TabSpec tab_specs[4] = {
    {QStringLiteral("Chassis"),   QStringLiteral("#2d6cdf"), &chassis_joints},
    {QStringLiteral("Torso"),     QStringLiteral("#1e9e4a"), &torso_joints},
    {QStringLiteral("Left Arm"),  QStringLiteral("#e07b00"), &left_arm_joints},
    {QStringLiteral("Right Arm"), QStringLiteral("#8e44ad"), &right_arm_joints},
  };

  for (int i = 0; i < 4; ++i) {
    QString card_title = tab_specs[i].title +
      QStringLiteral(" (%1)").arg(static_cast<int>(tab_specs[i].defs->size()));
    tabs->addTab(buildCard(i, card_title, tab_specs[i].color, *tab_specs[i].defs),
                 tab_specs[i].title);
  }
  main_layout->addWidget(tabs, 1);

  buildToolBar();
  buildStatusBar();
}

// ============================================================
// One group card: colored title + observe table + control rows
// ============================================================
QWidget * HmiWindow::buildCard(int group_idx, const QString & title,
                               const QString & title_color,
                               const std::vector<JointDef> & defs) {
  auto card = new QGroupBox();
  card->setStyleSheet(QString(
    "QGroupBox{border:1px solid %1; border-radius:6px; margin-top:10px;}"
    "QGroupBox::title{subcontrol-origin:margin; left:8px; padding:0 4px;"
    " color:%1; font-weight:bold;}").arg(title_color));
  card->setTitle(title);

  auto lay = new QVBoxLayout(card);
  lay->setContentsMargins(6, 6, 6, 6);
  lay->setSpacing(4);

  auto splitter = new QSplitter(Qt::Vertical);

  // ── Observe area (read-only table) ──────────────────────
  auto obs_wrap = new QWidget();
  auto obs_lay = new QVBoxLayout(obs_wrap);
  obs_lay->setContentsMargins(0, 0, 0, 0);
  obs_lay->setSpacing(2);
  // Units are shown per-row (translational joints: m / m/s, revolute: rad / rad/s)
  auto obs_label = new QLabel(QStringLiteral("<b>Observe</b>"));
  obs_label->setStyleSheet(QStringLiteral("color:%1;").arg(title_color));
  obs_lay->addWidget(obs_label);

  auto table = new QTableWidget(static_cast<int>(defs.size()), 3, obs_wrap);
  table->setHorizontalHeaderLabels({QStringLiteral("Joint"),
                                    QStringLiteral("Position"),
                                    QStringLiteral("Velocity")});
  table->verticalHeader()->setVisible(false);
  table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table->setSelectionMode(QAbstractItemView::NoSelection);
  table->setFocusPolicy(Qt::NoFocus);
  table->setAlternatingRowColors(true);
  table->setShowGrid(false);
  table->setBackgroundRole(QPalette::Window);  // distinguish from control area
  table->horizontalHeader()->setStretchLastSection(true);
  table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);

  for (int r = 0; r < table->rowCount(); ++r) {
    const JointDef & d = defs[static_cast<size_t>(r)];
    table->setItem(r, 0, new QTableWidgetItem(d.label));
    table->setItem(r, 1, new QTableWidgetItem(QStringLiteral("—")));
    table->setItem(r, 2, new QTableWidgetItem(QStringLiteral("—")));
    table->item(r, 0)->setData(Qt::UserRole, QString::fromStdString(d.name));
  }
  table->resizeRowsToContents();
  obs_lay->addWidget(table, 1);
  obs_tables_[static_cast<size_t>(group_idx)] = table;

  // ── Control area (interactive sliders / spin boxes) ─────
  auto ctrl_wrap = new QWidget();
  auto ctrl_lay = new QVBoxLayout(ctrl_wrap);
  ctrl_lay->setContentsMargins(0, 0, 0, 0);
  ctrl_lay->setSpacing(2);
  auto ctrl_label = new QLabel(QStringLiteral("<b>Control</b>"));
  ctrl_label->setStyleSheet(QStringLiteral("color:%1;").arg(title_color));
  ctrl_lay->addWidget(ctrl_label);

  auto ctrl_rows = new QWidget(ctrl_wrap);
  auto rows_lay = new QVBoxLayout(ctrl_rows);
  rows_lay->setContentsMargins(0, 0, 0, 0);
  rows_lay->setSpacing(2);
  for (size_t i = 0; i < defs.size(); ++i) {
    size_t idx = joints_.size();
    joints_.push_back(JointInfo{});
    name_to_idx_[defs[i].name] = idx;
    addControlRow(rows_lay, defs[i], idx);
  }
  rows_lay->addStretch(1);
  ctrl_lay->addWidget(ctrl_rows, 1);

  splitter->addWidget(obs_wrap);
  splitter->addWidget(ctrl_wrap);
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 2);
  splitter->setSizes({280, 420});

  lay->addWidget(splitter);
  return card;
}

// ============================================================
// Legacy builders — only used by the (unmaintained) RViz2 variant.
// Lite version uses buildUiLite() / buildCard() instead.
// ============================================================
QWidget * HmiWindow::buildObserver(QWidget * parent) {
  auto w = parent ? new QWidget(parent) : new QWidget();
  auto lay = new QVBoxLayout(w);
  lay->setContentsMargins(2, 2, 2, 2);

  auto title = new QLabel("<b>Joint Observer</b>");
  lay->addWidget(title);

  observer_tree_ = new QTreeWidget();
  observer_tree_->setColumnCount(3);
  observer_tree_->setHeaderLabels({"Joint", "Position", "Velocity"});
  observer_tree_->header()->setStretchLastSection(true);
  observer_tree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  observer_tree_->setRootIsDecorated(true);
  observer_tree_->setAlternatingRowColors(true);
  observer_tree_->setAnimated(true);

  auto add_group = [&](const std::string & group_name,
                        const std::vector<JointDef> & defs) {
    auto root = new QTreeWidgetItem(observer_tree_, {QString::fromStdString(group_name)});
    root->setExpanded(true);
    QFont bold = root->font(0);
    bold.setBold(true);
    root->setFont(0, bold);
    for (auto & d : defs) {
      auto item = new QTreeWidgetItem(root, {d.label, "—", "—"});
      item->setData(0, Qt::UserRole, QString::fromStdString(d.name));
    }
  };

  add_group("Chassis (8)", chassis_joints);
  add_group("Torso (4)", torso_joints);
  add_group("Left Arm (9)", left_arm_joints);
  add_group("Right Arm (9)", right_arm_joints);

  lay->addWidget(observer_tree_);
  return w;
}

QWidget * HmiWindow::buildController(QWidget * parent) {
  auto w = parent ? new QWidget(parent) : new QWidget();
  auto outer_lay = new QVBoxLayout(w);
  outer_lay->setContentsMargins(2, 2, 2, 2);

  auto title = new QLabel("<b>Joint Controller</b>");
  outer_lay->addWidget(title);

  auto hlay = new QHBoxLayout();
  outer_lay->addLayout(hlay);

  auto add_group = [&](const QString & group_name,
                        const std::vector<JointDef> & defs) {
    auto gb = new QGroupBox(group_name);
    auto glay = new QVBoxLayout(gb);
    glay->setSpacing(2);

    for (size_t i = 0; i < defs.size(); ++i) {
      size_t idx = joints_.size();
      joints_.push_back(JointInfo{});
      name_to_idx_[defs[i].name] = idx;
      addControlRow(glay, defs[i], idx);
    }
    hlay->addWidget(gb);
  };

  add_group("Chassis", chassis_joints);
  add_group("Torso", torso_joints);
  add_group("Left Arm", left_arm_joints);
  add_group("Right Arm", right_arm_joints);

  return w;
}

// ============================================================
// One control row: [label] [slider] [spinbox(unit)]
// ============================================================
void HmiWindow::addControlRow(QVBoxLayout * parent, const JointDef & d, size_t idx) {
  auto row = new QHBoxLayout();
  row->setSpacing(3);

  auto lbl = new QLabel(d.label);
  lbl->setFixedWidth(55);
  row->addWidget(lbl);

  auto slider = new QSlider(Qt::Horizontal);
  slider->setValue(0);
  row->addWidget(slider, 1);

  auto spin = new QDoubleSpinBox();
  spin->setDecimals(3);
  spin->setValue(0.0);
  spin->setFixedWidth(90);
  row->addWidget(spin);

  JointInfo & ji = joints_[idx];
  ji.name = d.name;
  ji.label = d.label;
  ji.is_velocity = d.is_velocity;
  ji.is_prismatic = d.is_prismatic;
  ji.lower = d.lower;
  ji.upper = d.upper;
  ji.slider = slider;
  ji.spin = spin;

  // Slider always works in SI (rad / m); spinbox shows converted units
  if (d.is_velocity) {
    slider->setRange(-500, 500);
  } else {
    slider->setRange(
      static_cast<int>(d.lower * 100),
      static_cast<int>(d.upper * 100));
  }
  applySpinRange(ji);  // range / suffix / value in current display units

  connect(slider, &QSlider::valueChanged, this, [this, idx](int v) {
    if (!joints_[idx].spin->hasFocus())
      joints_[idx].spin->setValue(toDisplay(v / 100.0, joints_[idx].is_prismatic));
    onSliderChanged(v);
  });
  connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this, idx](double v) { onSpinChanged(v); });

  parent->addLayout(row);
}

// ============================================================
// Status bar
// ============================================================
void HmiWindow::buildStatusBar() {
  auto bar = statusBar();

  js_status_lbl_ = new QLabel();
  js_status_lbl_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  bar->addWidget(js_status_lbl_);

  pub_status_lbl_ = new QLabel();
  pub_status_lbl_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  bar->addWidget(pub_status_lbl_);

  mode_lbl_ = new QLabel();
  mode_lbl_->setStyleSheet(QStringLiteral("color:#888;"));
  bar->addPermanentWidget(mode_lbl_);
  updateModeLabel();
}

// ============================================================
// Toolbar: units / rate / auto-send switch / send-now
// ============================================================
void HmiWindow::buildToolBar() {
  auto tb = addToolBar(QStringLiteral("Main"));
  tb->setMovable(false);
  tb->setToolButtonStyle(Qt::ToolButtonTextOnly);

  // ── Units ────────────────────────────────────────────────
  tb->addWidget(new QLabel(QStringLiteral(" Units: ")));
  angle_unit_cb_ = new QComboBox();
  angle_unit_cb_->addItem(QStringLiteral("rad"), QVariant::fromValue(static_cast<int>(AngleUnit::Rad)));
  angle_unit_cb_->addItem(QStringLiteral("deg"), QVariant::fromValue(static_cast<int>(AngleUnit::Deg)));
  angle_unit_cb_->setCurrentIndex(0);
  tb->addWidget(angle_unit_cb_);

  length_unit_cb_ = new QComboBox();
  length_unit_cb_->addItem(QStringLiteral("m"), QVariant::fromValue(static_cast<int>(LengthUnit::Meter)));
  length_unit_cb_->addItem(QStringLiteral("mm"), QVariant::fromValue(static_cast<int>(LengthUnit::Millimeter)));
  length_unit_cb_->setCurrentIndex(0);
  tb->addWidget(length_unit_cb_);

  tb->addSeparator();

  // ── Send rate ────────────────────────────────────────────
  tb->addWidget(new QLabel(QStringLiteral(" Rate: ")));
  rate_cb_ = new QComboBox();
  for (int hz : {1, 2, 5, 10, 20})
    rate_cb_->addItem(QString::number(hz) + QStringLiteral(" Hz"), hz);
  rate_cb_->setCurrentIndex(2);  // 5 Hz default
  tb->addWidget(rate_cb_);

  tb->addSeparator();

  // ── Auto-send master switch (with indicator) ─────────────
  auto_ind_lbl_ = new QLabel(QStringLiteral("●"));
  auto_ind_lbl_->setStyleSheet(QStringLiteral("color:#2e7d32; font-weight:bold;"));
  tb->addWidget(auto_ind_lbl_);
  auto_btn_ = new QToolButton();
  auto_btn_->setCheckable(true);
  auto_btn_->setChecked(true);
  auto_btn_->setText(QStringLiteral("Auto: ON"));
  tb->addWidget(auto_btn_);

  // ── Manual send now ──────────────────────────────────────
  send_now_btn_ = new QToolButton();
  send_now_btn_->setText(QStringLiteral("Send Now"));
  tb->addWidget(send_now_btn_);

  connect(angle_unit_cb_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &HmiWindow::onAngleUnitChanged);
  connect(length_unit_cb_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &HmiWindow::onLengthUnitChanged);
  connect(rate_cb_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &HmiWindow::onRateChanged);
  connect(auto_btn_, &QToolButton::toggled, this, &HmiWindow::onAutoToggled);
  connect(send_now_btn_, &QToolButton::clicked, this, &HmiWindow::onSendNow);
}

// ============================================================
// Unit conversion (internal storage is always SI: rad / m)
// ============================================================
double HmiWindow::angleFactor() const {
  return angle_unit_ == AngleUnit::Deg ? (180.0 / 3.14159265358979323846) : 1.0;
}
double HmiWindow::lengthFactor() const {
  return length_unit_ == LengthUnit::Millimeter ? 1000.0 : 1.0;
}
double HmiWindow::toDisplay(double si, bool is_prismatic) const {
  return si * (is_prismatic ? lengthFactor() : angleFactor());
}
double HmiWindow::toSI(double display, bool is_prismatic) const {
  return display / (is_prismatic ? lengthFactor() : angleFactor());
}
QString HmiWindow::unitStr(bool is_prismatic) const {
  if (is_prismatic)
    return length_unit_ == LengthUnit::Millimeter ? QStringLiteral("mm")
                                                  : QStringLiteral("m");
  return angle_unit_ == AngleUnit::Deg ? QStringLiteral("deg")
                                       : QStringLiteral("rad");
}

// Set spinbox range / suffix / current value from SI limits and desired
void HmiWindow::applySpinRange(JointInfo & j) {
  if (!j.spin) return;
  double sf = j.is_prismatic ? lengthFactor() : angleFactor();
  double lo = j.is_velocity ? -5.0 : j.lower;
  double hi = j.is_velocity ? 5.0 : j.upper;
  j.spin->setRange(lo * sf, hi * sf);
  j.spin->setSingleStep((j.is_velocity ? 0.1 : 0.05) * sf);
  if (j.is_velocity)
    j.spin->setSuffix(QStringLiteral(" %1/s").arg(unitStr(j.is_prismatic)));
  else
    j.spin->setSuffix(QStringLiteral(" %1").arg(unitStr(j.is_prismatic)));
  // Value change here is ignored by onSpinChanged (spin has no focus)
  j.spin->setValue(toDisplay(j.desired, j.is_prismatic));
}

void HmiWindow::refreshDisplays() {
  for (auto & j : joints_) applySpinRange(j);
  updateModeLabel();
}

void HmiWindow::updateModeLabel() {
  if (!mode_lbl_) return;
  bool on = auto_btn_ && auto_btn_->isChecked();
  int hz = rate_cb_ ? rate_cb_->currentData().toInt() : 5;
  mode_lbl_->setText(on ? QStringLiteral("Send: auto %1 Hz").arg(hz)
                        : QStringLiteral("Send: manual"));
}

// ============================================================
// Toolbar slots
// ============================================================
void HmiWindow::onAngleUnitChanged(int) {
  angle_unit_ = static_cast<AngleUnit>(angle_unit_cb_->currentData().toInt());
  refreshDisplays();
}
void HmiWindow::onLengthUnitChanged(int) {
  length_unit_ = static_cast<LengthUnit>(length_unit_cb_->currentData().toInt());
  refreshDisplays();
}
void HmiWindow::onRateChanged(int) {
  int hz = rate_cb_->currentData().toInt();
  if (auto_btn_ && auto_btn_->isChecked())
    send_timer_->start(1000 / hz);
  updateModeLabel();
}
void HmiWindow::onAutoToggled(bool on) {
  if (auto_ind_lbl_) {
    auto_ind_lbl_->setText(on ? QStringLiteral("●") : QStringLiteral("○"));
    auto_ind_lbl_->setStyleSheet(on ? QStringLiteral("color:#2e7d32; font-weight:bold;")
                                    : QStringLiteral("color:#888; font-weight:bold;"));
  }
  if (auto_btn_)
    auto_btn_->setText(on ? QStringLiteral("Auto: ON") : QStringLiteral("Auto: OFF"));
  if (on) {
    int hz = rate_cb_ ? rate_cb_->currentData().toInt() : 5;
    send_timer_->start(1000 / hz);
  } else {
    send_timer_->stop();
  }
  updateModeLabel();
}
void HmiWindow::onSendNow() {
  publishDesired();
  ++send_count_;
}

// ============================================================
// Slots
// ============================================================
void HmiWindow::onRosSpin() {
  rclcpp::spin_some(node_);

  // Update legacy tree observer (RViz2 variant only)
  if (observer_tree_) {
    for (int gi = 0; gi < observer_tree_->topLevelItemCount(); ++gi) {
      auto * root = observer_tree_->topLevelItem(gi);
      for (int ci = 0; ci < root->childCount(); ++ci) {
        auto * item = root->child(ci);
        QString name = item->data(0, Qt::UserRole).toString();
        auto it = name_to_idx_.find(name.toStdString());
        if (it == name_to_idx_.end()) continue;
        const auto & j = joints_[it->second];
        item->setText(1, QString::number(toDisplay(j.position, j.is_prismatic), 'f', 4)
                       + QStringLiteral(" ") + unitStr(j.is_prismatic));
        item->setText(2, QString::number(toDisplay(j.velocity, j.is_prismatic), 'f', 4)
                       + QStringLiteral(" ") + unitStr(j.is_prismatic) + QStringLiteral("/s"));
      }
    }
  }

  // Update observer tables
  for (size_t g = 0; g < obs_tables_.size(); ++g) {
    auto * table = obs_tables_[g];
    if (!table) continue;
    for (int r = 0; r < table->rowCount(); ++r) {
      QString name = table->item(r, 0)->data(Qt::UserRole).toString();
      auto it = name_to_idx_.find(name.toStdString());
      if (it == name_to_idx_.end()) continue;
      const auto & j = joints_[it->second];
      table->item(r, 1)->setText(
        QString::number(toDisplay(j.position, j.is_prismatic), 'f', 4)
        + QStringLiteral(" ") + unitStr(j.is_prismatic));
      table->item(r, 2)->setText(
        QString::number(toDisplay(j.velocity, j.is_prismatic), 'f', 4)
        + QStringLiteral(" ") + unitStr(j.is_prismatic) + QStringLiteral("/s"));
    }
  }

  // Status bar
  if (js_status_lbl_) {
    size_t pubs = js_sub_->get_publisher_count();
    bool ok = pubs > 0;
    js_status_lbl_->setText(QStringLiteral(
      "<font color='%1'>●</font> /joint_states  ·  %2 pub · %3 msg")
      .arg(ok ? QStringLiteral("#2e7d32") : QStringLiteral("#c62828"),
           QString::number(pubs),
           QString::number(js_count_)));
  }
  if (pub_status_lbl_) {
    size_t subs = desired_pub_->get_subscription_count();
    pub_status_lbl_->setText(QStringLiteral(
      "<font color='%1'>●</font> /desired_joint_states  ·  %2 sub · sent %3")
      .arg(subs > 0 ? QStringLiteral("#2e7d32") : QStringLiteral("#c62828"),
           QString::number(subs),
           QString::number(send_count_)));
  }
}

void HmiWindow::onSliderChanged(int value) {
  for (auto & j : joints_) {
    if (j.slider && j.slider->value() == value && j.slider->hasFocus()) {
      j.desired = static_cast<double>(value) / 100.0;
      desired_dirty_ = true;
      break;
    }
  }
}

void HmiWindow::onSpinChanged(double value) {
  for (auto & j : joints_) {
    if (j.spin && j.spin->hasFocus()) {
      j.desired = toSI(value, j.is_prismatic);
      int sv = static_cast<int>(j.desired * 100);
      if (j.slider && j.slider->value() != sv) j.slider->setValue(sv);
      desired_dirty_ = true;
      break;
    }
  }
}

void HmiWindow::onSendTimer() {
  if (!desired_dirty_) return;
  desired_dirty_ = false;
  publishDesired();
  ++send_count_;
}

void HmiWindow::publishDesired() {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->now();

  for (size_t i = 0; i < joints_.size(); ++i) {
    msg.name.push_back(joints_[i].name);
    if (joints_[i].is_velocity) {
      msg.position.push_back(0.0);
      msg.velocity.push_back(joints_[i].desired);
    } else {
      msg.position.push_back(joints_[i].desired);
      msg.velocity.push_back(0.0);
    }
  }
  desired_pub_->publish(msg);
}

}  // namespace ylr1d_hmi
