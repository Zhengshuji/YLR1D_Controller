#include "ylr1d_hmi/panels/monitor_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QFont>
#include <QTableWidgetItem>
#include <QAbstractItemView>
#include <QBrush>
#include <QColor>
#include <QSet>
#include <QStringList>

#include <cmath>
#include <chrono>
#include <cstring>
#include <set>
#include <string>
#include <utility>

namespace ylr1d_hmi {

namespace {

// ────────────────────────────────────────────────────────────
// Static configuration
// ────────────────────────────────────────────────────────────

/// Nodes the monitor expects for the standard position-sim setup.
const std::vector<ExpectedNode> kExpectedNodes = {
  {"robot_state_publisher", "URDF + TF"},
  {"gazebo_ros2_control",   "ros2_control plugin"},
  {"controller_manager",    "controller manager"},
  {"arm_simulate",          "arm soft-sim (position)"},
  {"chassis_simulate",      "chassis soft-sim (position)"},
  {"rviz2",                 "3D view (optional)"},
};

struct SensorSpec { const char * label; const char * topic; int kind; };
const SensorSpec kSensorSpecs[] = {
  {"Global Cam RGB",   "/global_camera/rgb/image_raw",         0},
  {"Global Cam Depth", "/global_camera/depth/image_raw",       0},
  {"Global Cam IR",    "/global_camera/infrared/image_raw",    0},
  {"Left Cam RGB",     "/left_camera/rgb/image_raw",           0},
  {"Left Cam Depth",   "/left_camera/depth/image_raw",         0},
  {"Left Cam IR",      "/left_camera/infrared/image_raw",      0},
  {"Right Cam RGB",    "/right_camera/rgb/image_raw",          0},
  {"Right Cam Depth",  "/right_camera/depth/image_raw",        0},
  {"Right Cam IR",     "/right_camera/infrared/image_raw",     0},
  {"Global Cloud",     "/global_camera/depth/points",          1},
  {"Left Cloud",       "/left_camera/depth/points",            1},
  {"Right Cloud",      "/right_camera/depth/points",           1},
  {"Radar",            "/radar/scan",                          2},
  {"Ultrasonic LF",    "/lf_ultrasonic/range",                 2},
  {"Ultrasonic RF",    "/rf_ultrasonic/range",                 2},
  {"Ultrasonic LB",    "/lb_ultrasonic/range",                 2},
  {"Ultrasonic RB",    "/rb_ultrasonic/range",                 2},
  {"IMU",              "/imu_data",                            3},
};

/// Joint groups for the Joints dropdown — same 5-group split as
/// ylr1d_control_sim (steering / wheels / torso / left / right), limits in
/// sync with src/ylr1d_hmi/src/panels/hmi_window.cpp and
/// src/ylr1d_control_sim/config/position_control_limits.yaml.
std::vector<JointGroup> makeJointGroups() {
  return {
    {"Chassis Steering", {
      {"RF_Steer", "Joint_Base_to_RFWheelF", false, false, -3.14, 3.14},
      {"LF_Steer", "Joint_Base_to_LFWheelF", false, false, -3.14, 3.14},
      {"RB_Steer", "Joint_Base_to_RBWheelF", false, false, -3.14, 3.14},
      {"LB_Steer", "Joint_Base_to_LBWheelF", false, false, -3.14, 3.14},
    }},
    {"Chassis Wheels", {
      {"RF_Wheel", "Joint_RFWheelF_to_RFWheel", true},
      {"LF_Wheel", "Joint_LFWheelF_to_LFWheel", true},
      {"RB_Wheel", "Joint_RBWheelF_to_RBWheel", true},
      {"LB_Wheel", "Joint_LBWheelF_to_LBWheel", true},
    }},
    {"Torso", {
      {"Lift",   "Joint_Base_to_Body1",  false, true,  -0.30, 0.30},
      {"Yaw",    "Joint_Body1_to_Body2", false, false, -3.14, 3.14},
      {"Pitch1", "Joint_Body2_to_Body3", false, false, -1.57, 1.57},
      {"Pitch2", "Joint_Body3_to_Body4", false, false, -1.57, 1.57},
    }},
    {"Left Arm", {
      {"Shoulder1", "Joint_Body2_to_LeftArm1",    false, false, -2.62, 2.62},
      {"Shoulder2", "Joint_LeftArm1_to_LeftArm2", false, false, -1.57, 1.83},
      {"Shoulder3", "Joint_LeftArm2_to_LeftArm3", false, false, -2.62, 2.62},
      {"Elbow1",    "Joint_LeftArm3_to_LeftArm4", false, false, -1.57, 1.57},
      {"Elbow2",    "Joint_LeftArm4_to_LeftArm5", false, false, -2.62, 2.62},
      {"Wrist1",    "Joint_LeftArm5_to_LeftArm6", false, false, -2.09, 2.09},
      {"Wrist2",    "Joint_LeftArm6_to_LeftArm7", false, false, -6.28, 6.28},
      {"Finger1",   "Joint_LeftArm7_to_LeftFinger1", false, true, -0.015, 0.0},
      {"Finger2",   "Joint_LeftArm7_to_LeftFinger2", false, true, -0.015, 0.0},
    }},
    {"Right Arm", {
      {"Shoulder1", "Joint_Body2_to_RightArm1",    false, false, -2.62, 2.62},
      {"Shoulder2", "Joint_RightArm1_to_RightArm2", false, false, -1.57, 1.83},
      {"Shoulder3", "Joint_RightArm2_to_RightArm3", false, false, -2.62, 2.62},
      {"Elbow1",    "Joint_RightArm3_to_RightArm4", false, false, -1.57, 1.57},
      {"Elbow2",    "Joint_RightArm4_to_RightArm5", false, false, -2.62, 2.62},
      {"Wrist1",    "Joint_RightArm5_to_RightArm6", false, false, -2.09, 2.09},
      {"Wrist2",    "Joint_RightArm6_to_RightArm7", false, false, -6.28, 6.28},
      {"Finger1",   "Joint_RightArm7_to_RightFinger1", false, true, 0.0, 0.015},
      {"Finger2",   "Joint_RightArm7_to_RightFinger2", false, true, 0.0, 0.015},
    }},
  };
}

QTableWidget * makeTable(const std::vector<QString> & headers) {
  auto t = new QTableWidget(0, static_cast<int>(headers.size()));
  QStringList hl;
  for (const auto & h : headers) hl << h;
  t->setHorizontalHeaderLabels(hl);
  t->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  t->horizontalHeader()->setStretchLastSection(true);
  t->verticalHeader()->setVisible(false);
  t->setEditTriggers(QAbstractItemView::NoEditTriggers);
  t->setSelectionMode(QAbstractItemView::NoSelection);
  t->setFocusPolicy(Qt::NoFocus);
  return t;
}

/// Compare a graph node name against an expected short name, tolerating
/// leading slashes and namespaces (take the last path segment).
bool nodeNameMatch(const QString & detected, const QString & expected) {
  QString d = detected;
  while (d.startsWith(QLatin1Char('/'))) d = d.mid(1);
  const int slash = d.lastIndexOf(QLatin1Char('/'));
  if (slash >= 0) d = d.mid(slash + 1);
  return d == expected;
}

}  // namespace

// ============================================================
// Construction
// ============================================================
MonitorWidget::MonitorWidget(rclcpp::Node::SharedPtr node, QWidget * parent)
  : QWidget(parent), node_(node)
{
  sensors_.clear();
  for (const auto & s : kSensorSpecs) {
    SensorRow r;
    r.label = QLatin1String(s.label);
    r.topic = QLatin1String(s.topic);
    r.kind = s.kind;
    sensors_.push_back(std::move(r));
  }
  joint_groups_ = makeJointGroups();

  buildUi();

  // ── Subscriptions ──
  subs_.push_back(node_->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", 200,
    [this](const rcl_interfaces::msg::Log::SharedPtr m) { onRosout(m); }));
  subs_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr m) { onJointState(m); }));
  // /clock is published BEST_EFFORT by gazebo; default RELIABLE would silently
  // receive nothing, so the sim would look "no /clock" even while running.
  subs_.push_back(node_->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::QoS(10).best_effort(),
    [this](const rosgraph_msgs::msg::Clock::SharedPtr m) { onClock(m); }));

  for (auto & s : sensors_) {
    switch (s.kind) {
      case 0:
        subs_.push_back(makeStatusSub<sensor_msgs::msg::Image>(s.topic.toStdString(), &s.st));
        break;
      case 1:
        subs_.push_back(makeStatusSub<sensor_msgs::msg::PointCloud2>(s.topic.toStdString(), &s.st));
        break;
      case 2:
        subs_.push_back(makeStatusSub<sensor_msgs::msg::LaserScan>(s.topic.toStdString(), &s.st));
        break;
      case 3:
        subs_.push_back(makeStatusSub<sensor_msgs::msg::Imu>(s.topic.toStdString(), &s.st));
        break;
    }
  }

  // ── Controller status service ──
  list_cli_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers");

  // ── Timers ──
  ros_timer_ = new QTimer(this);
  connect(ros_timer_, &QTimer::timeout, this, &MonitorWidget::onRosSpin);
  ros_timer_->start(20);

  poll_timer_ = new QTimer(this);
  connect(poll_timer_, &QTimer::timeout, this, &MonitorWidget::onPoll);
  poll_timer_->start(2000);

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &MonitorWidget::onRefresh);
  refresh_timer_->start(500);

  RCLCPP_INFO(node_->get_logger(),
              "monitor widget ready: %zu sensors, %zu joint groups",
              sensors_.size(), joint_groups_.size());
}

// ============================================================
// ROS callbacks
// ============================================================
void MonitorWidget::onRosSpin() {
  if (rclcpp::ok()) rclcpp::spin_some(node_);
}

void MonitorWidget::onRosout(const rcl_interfaces::msg::Log::SharedPtr m) {
  if (m->name == node_->get_name()) return;  // skip our own chatter
  LogEntry e;
  e.level = m->level;
  e.source = QString::fromStdString(m->name);
  e.text = QString::fromStdString(m->msg);
  log_buf_.push_back(std::move(e));
  if (log_buf_.size() > 800) log_buf_.pop_front();
  ++log_total_;
  classifyLog(log_buf_.back());
  log_dirty_ = true;
}

void MonitorWidget::classifyLog(const LogEntry & e) {
  if (e.level >= 40) ++hl_.error_total;
  else if (e.level >= 30) ++hl_.warn_total;

  const QString & t = e.text;
  if (t.contains(QLatin1String("Successfully spawned entity"))) hl_.robot_spawned = true;
  if (t.contains(QLatin1String("Loading controller_manager"))) hl_.cm_loaded = true;
  if (t.contains(QLatin1String("Successful initialization of hardware"))) hl_.hw_ok = true;
  if (t.contains(QLatin1String("activate successful"))) ++hl_.controllers_active;
  if (t.contains(QLatin1String("Publishing camera info"))) ++hl_.camera_pub;

  bool anomaly = e.level >= 30;
  if (!anomaly) {
    static const char * kBadKeywords[] = {
      "not valid", "failed", "slower than", "TF_NAN", "NaN", "timed out", "error"};
    for (const char * kw : kBadKeywords) {
      if (t.contains(QLatin1String(kw))) { anomaly = true; break; }
    }
  }
  if (anomaly) {
    // Dedup consecutive identical entries.
    if (notable_.empty() || !(notable_.back().source == e.source && notable_.back().text == e.text)) {
      notable_.push_back(e);
      if (notable_.size() > 80) notable_.pop_front();
    }
  }
}

void MonitorWidget::onJointState(const sensor_msgs::msg::JointState::SharedPtr m) {
  js_ = m;
  js_st_.touch();
}

void MonitorWidget::onClock(const rosgraph_msgs::msg::Clock::SharedPtr m) {
  sim_sec_ = rclcpp::Time(m->clock).seconds();
  clock_seen_ = true;
  clock_last_ = std::chrono::steady_clock::now();
}

void MonitorWidget::onPoll() {
  detected_nodes_ = node_->get_node_names();

  if (!poll_inflight_ && list_cli_ && list_cli_->service_is_ready()) {
    auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    poll_inflight_ = true;
    auto cb = [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture fut) {
      poll_inflight_ = false;
      try {
        auto resp = fut.get();
        controllers_ok_ = true;
        ctrl_names_.clear();
        ctrl_states_.clear();
        ctrl_types_.clear();
        for (const auto & c : resp->controller) {
          ctrl_names_.push_back(c.name);
          ctrl_states_.push_back(c.state);
          ctrl_types_.push_back(c.type);
        }
      } catch (const std::exception & e) {
        controllers_ok_ = false;
        RCLCPP_WARN(node_->get_logger(), "list_controllers failed: %s", e.what());
      }
    };
    list_cli_->async_send_request(req, cb);
  }
}

// ============================================================
// UI construction
// ============================================================
void MonitorWidget::buildUi() {
  auto tabs = new QTabWidget();
  tabs->setDocumentMode(true);
  tabs->addTab(buildOverview(), QStringLiteral("Overview"));
  tabs->addTab(buildLog(), QStringLiteral("Log"));
  tabs->addTab(buildSensors(), QStringLiteral("Sensors"));
  tabs->addTab(buildJoints(), QStringLiteral("Joints"));

  auto lay = new QVBoxLayout(this);
  lay->setContentsMargins(4, 4, 4, 4);
  lay->addWidget(tabs);
}

QWidget * MonitorWidget::buildOverview() {
  auto wrap = new QWidget();
  auto v = new QVBoxLayout(wrap);
  v->setSpacing(6);

  auto row = new QHBoxLayout();
  sim_status_lbl_ = new QLabel(QStringLiteral("Sim: --"));
  js_rate_lbl_ = new QLabel(QStringLiteral("/joint_states: --"));
  node_cnt_lbl_ = new QLabel(QStringLiteral("nodes: --"));
  ctrl_cnt_lbl_ = new QLabel(QStringLiteral("controllers: --"));
  for (auto * l : {sim_status_lbl_, js_rate_lbl_, node_cnt_lbl_, ctrl_cnt_lbl_}) {
    l->setStyleSheet(QStringLiteral("background:#222; padding:3px 8px; border-radius:3px;"));
    row->addWidget(l);
  }
  row->addStretch();
  v->addLayout(row);

  auto ctrl_box = new QGroupBox(QStringLiteral("Controllers"));
  auto cv = new QVBoxLayout(ctrl_box);
  ctrl_table_ = makeTable({"Name", "State", "Type"});
  cv->addWidget(ctrl_table_);
  v->addWidget(ctrl_box);

  auto node_box = new QGroupBox(QStringLiteral("Nodes"));
  auto nv = new QVBoxLayout(node_box);
  node_table_ = makeTable({"Name", "Role", "Status"});
  nv->addWidget(node_table_);
  v->addWidget(node_box);

  auto anom_box = new QGroupBox(QStringLiteral("Anomalies"));
  auto av = new QVBoxLayout(anom_box);
  anomaly_list_ = new QListWidget();
  anomaly_list_->setWordWrap(true);
  av->addWidget(anomaly_list_);
  v->addWidget(anom_box, 1);

  return wrap;
}

QWidget * MonitorWidget::buildLog() {
  auto w = new QWidget();
  auto v = new QVBoxLayout(w);

  auto row = new QHBoxLayout();
  row->addWidget(new QLabel(QStringLiteral("Level:")));
  log_level_cb_ = new QComboBox();
  log_level_cb_->addItems({QStringLiteral("All"), QStringLiteral("WARN+"), QStringLiteral("ERROR only")});
  row->addWidget(log_level_cb_);
  row->addWidget(new QLabel(QStringLiteral("Source:")));
  log_source_cb_ = new QComboBox();
  log_source_cb_->addItem(QStringLiteral("All"));
  row->addWidget(log_source_cb_);
  row->addStretch();
  log_count_lbl_ = new QLabel();
  row->addWidget(log_count_lbl_);
  v->addLayout(row);
  connect(log_level_cb_, qOverload<int>(&QComboBox::currentIndexChanged),
          this, &MonitorWidget::onLogFilterChanged);
  connect(log_source_cb_, qOverload<int>(&QComboBox::currentIndexChanged),
          this, &MonitorWidget::onLogFilterChanged);

  milestone_lbl_ = new QLabel();
  milestone_lbl_->setWordWrap(true);
  milestone_lbl_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  v->addWidget(milestone_lbl_);

  log_view_ = new QPlainTextEdit();
  log_view_->setReadOnly(true);
  log_view_->setMaximumBlockCount(2000);
  QFont mono(QStringLiteral("Monospace"));
  mono.setPointSize(9);
  log_view_->setFont(mono);
  v->addWidget(log_view_, 1);
  return w;
}

QWidget * MonitorWidget::buildSensors() {
  auto w = new QWidget();
  auto v = new QVBoxLayout(w);
  sensor_table_ = makeTable({"Sensor", "Topic", "Status"});
  sensor_table_->setRowCount(static_cast<int>(sensors_.size()));
  for (int i = 0; i < static_cast<int>(sensors_.size()); ++i) {
    auto & s = sensors_[i];
    sensor_table_->setItem(i, 0, new QTableWidgetItem(s.label));
    sensor_table_->setItem(i, 1, new QTableWidgetItem(s.topic));
    s.cell = new QLabel(QStringLiteral("--"));
    s.cell->setTextInteractionFlags(Qt::TextSelectableByMouse);
    sensor_table_->setCellWidget(i, 2, s.cell);
  }
  sensor_table_->setColumnWidth(0, 150);
  sensor_table_->setColumnWidth(1, 280);
  v->addWidget(sensor_table_);
  return w;
}

QWidget * MonitorWidget::buildJoints() {
  auto w = new QWidget();
  auto v = new QVBoxLayout(w);
  auto row = new QHBoxLayout();
  row->addWidget(new QLabel(QStringLiteral("Group:")));
  joint_group_cb_ = new QComboBox();
  for (const auto & g : joint_groups_) joint_group_cb_->addItem(g.name);
  row->addWidget(joint_group_cb_);
  row->addStretch();
  v->addLayout(row);
  connect(joint_group_cb_, qOverload<int>(&QComboBox::currentIndexChanged),
          this, &MonitorWidget::onJointGroupChanged);

  joint_table_ = makeTable({"Joint", "Position", "Velocity", "Limits", "Flags"});
  joint_table_->setColumnWidth(0, 110);
  v->addWidget(joint_table_);
  return w;
}

// ============================================================
// Refresh (called by refresh_timer_ at 2 Hz)
// ============================================================
void MonitorWidget::onRefresh() {
  clock_recent_ = clock_seen_ &&
    (std::chrono::steady_clock::now() - clock_last_) < std::chrono::seconds(2);

  refreshOverview();
  refreshLog();
  refreshSensors();
  refreshJoints();

  if (++summary_counter_ % 10 == 0) {
    int active = 0;
    for (const auto & s : ctrl_states_) if (s == "active") ++active;
    int alive = 0;
    for (const auto & s : sensors_) if (s.st.count > 0 && s.st.ageSeconds() < 3.0) ++alive;
    RCLCPP_INFO(node_->get_logger(),
                "monitor: sim=%s ctrl=%d/%zu active nodes=%zu sensors=%d/%zu alive js=%.1fHz",
                clock_recent_ ? "RUNNING" : (clock_seen_ ? "PAUSED" : "NO_CLOCK"),
                active, ctrl_names_.size(), detected_nodes_.size(),
                alive, sensors_.size(),
                js_st_.rateHz() >= 0.0 ? js_st_.rateHz() : 0.0);
  }
}

void MonitorWidget::refreshOverview() {
  QString sim;
  if (!clock_seen_) sim = QStringLiteral("<span style='color:#e77'>no /clock</span> (gzserver not publishing)");
  else if (clock_recent_) sim = QStringLiteral("<span style='color:#8d8'>RUNNING</span> t=%1 s").arg(sim_sec_, 0, 'f', 2);
  else sim = QStringLiteral("<span style='color:#ec6'>PAUSED</span> (clock stalled)");
  sim_status_lbl_->setText(QStringLiteral("Sim: ") + sim);

  const double r = js_st_.rateHz();
  js_rate_lbl_->setText(QStringLiteral("/joint_states: %1")
    .arg(r >= 0.0 ? QStringLiteral("%1 Hz").arg(r, 0, 'f', 1) : QStringLiteral("no data")));
  node_cnt_lbl_->setText(QStringLiteral("nodes: %1").arg(detected_nodes_.size()));

  int active = 0;
  if (controllers_ok_) {
    ctrl_table_->setRowCount(static_cast<int>(ctrl_names_.size()));
    for (int i = 0; i < static_cast<int>(ctrl_names_.size()); ++i) {
      const QString state = QString::fromStdString(ctrl_states_[i]);
      auto * n = new QTableWidgetItem(QString::fromStdString(ctrl_names_[i]));
      auto * st = new QTableWidgetItem(state);
      auto * ty = new QTableWidgetItem(QString::fromStdString(ctrl_types_[i]));
      const QColor c = state == QLatin1String("active") ? QColor(60, 180, 80)
        : (state == QLatin1String("configured") || state == QLatin1String("inactive"))
            ? QColor(230, 190, 60) : QColor(230, 80, 80);
      st->setForeground(QBrush(c));
      if (state == QLatin1String("active")) ++active;
      ctrl_table_->setItem(i, 0, n);
      ctrl_table_->setItem(i, 1, st);
      ctrl_table_->setItem(i, 2, ty);
    }
  } else {
    ctrl_table_->setRowCount(1);
    ctrl_table_->setItem(0, 0, new QTableWidgetItem(QStringLiteral("controller_manager")));
    auto * st = new QTableWidgetItem(QStringLiteral("not reachable"));
    st->setForeground(QBrush(QColor(230, 80, 80)));
    ctrl_table_->setItem(0, 1, st);
    ctrl_table_->setItem(0, 2, new QTableWidgetItem(QStringLiteral("—")));
  }
  ctrl_cnt_lbl_->setText(QStringLiteral("controllers: %1 active").arg(active));

  node_table_->setRowCount(static_cast<int>(kExpectedNodes.size()));
  for (int i = 0; i < static_cast<int>(kExpectedNodes.size()); ++i) {
    const auto & en = kExpectedNodes[i];
    bool found = false;
    for (const auto & dn : detected_nodes_) {
      if (nodeNameMatch(QString::fromStdString(dn), en.name)) { found = true; break; }
    }
    auto * s = new QTableWidgetItem(found ? QStringLiteral("online") : QStringLiteral("missing"));
    s->setForeground(QBrush(found ? QColor(60, 180, 80) : QColor(230, 80, 80)));
    node_table_->setItem(i, 0, new QTableWidgetItem(en.name));
    node_table_->setItem(i, 1, new QTableWidgetItem(en.role));
    node_table_->setItem(i, 2, s);
  }

  updateNotables();
}

void MonitorWidget::updateNotables() {
  anomaly_list_->clear();
  const bool sim_up = js_st_.count > 0 && js_st_.ageSeconds() < 3.0;

  for (int i = 0; i < static_cast<int>(ctrl_states_.size()); ++i) {
    if (ctrl_states_[i] != "active") {
      addAnomaly(QStringLiteral("controller %1: state=%2")
        .arg(QString::fromStdString(ctrl_names_[i]), QString::fromStdString(ctrl_states_[i])),
        QStringLiteral("#ec6"));
    }
  }
  for (const auto & en : kExpectedNodes) {
    bool found = false;
    for (const auto & dn : detected_nodes_) {
      if (nodeNameMatch(QString::fromStdString(dn), en.name)) { found = true; break; }
    }
    if (!found && en.role != QLatin1String("3D view (optional)")) {
      addAnomaly(QStringLiteral("node missing: %1 (%2)").arg(en.name, en.role),
                 QStringLiteral("#e77"));
    }
  }
  if (sim_up) {
    for (const auto & s : sensors_) {
      if (s.st.count == 0) {
        addAnomaly(QStringLiteral("no data: %1").arg(s.label), QStringLiteral("#e77"));
      } else if (s.st.ageSeconds() > 3.0) {
        addAnomaly(QStringLiteral("stale: %1 (%2s)").arg(s.label).arg(s.st.ageSeconds(), 0, 'f', 1),
                   QStringLiteral("#ec6"));
      }
    }
    if (js_) {
      for (size_t i = 0; i < js_->name.size() && i < js_->position.size(); ++i) {
        if (std::isnan(js_->position[i])) {
          addAnomaly(QStringLiteral("NaN joint: %1").arg(QString::fromStdString(js_->name[i])),
                     QStringLiteral("#ec6"));
        }
      }
    }
  }
  for (auto it = notable_.rbegin(); it != notable_.rend(); ++it) {
    addAnomaly(QStringLiteral("%1 %2: %3")
      .arg(levelName(it->level), it->source, it->text.left(180)),
      it->level >= 40 ? QStringLiteral("#e77") : QStringLiteral("#ec6"));
  }
}

void MonitorWidget::addAnomaly(const QString & text, const QString & color) {
  if (anomaly_list_->count() >= 60) return;
  auto * item = new QListWidgetItem(text);
  item->setForeground(QBrush(QColor(color)));
  anomaly_list_->addItem(item);
}

void MonitorWidget::refreshLog() {
  auto flag = [](bool v) {
    return v ? QStringLiteral("<span style='color:#8d8'>yes</span>")
             : QStringLiteral("<span style='color:#d77'>no</span>");
  };
  milestone_lbl_->setText(
    QStringLiteral("<b>Milestones</b>  robot:%1 · controller_manager:%2 · hardware:%3"
                   "  · controllers_activated:%4 · camera_pub:%5<br>"
                   "<b>Totals</b>  WARN <span style='color:#ec6'>%6</span>"
                   " · ERROR <span style='color:#e77'>%7</span> · log msgs %8")
      .arg(flag(hl_.robot_spawned), flag(hl_.cm_loaded), flag(hl_.hw_ok))
      .arg(hl_.controllers_active)
      .arg(hl_.camera_pub)
      .arg(hl_.warn_total).arg(hl_.error_total)
      .arg(log_total_));

  int ninfo = 0, nwarn = 0, nerr = 0;
  for (const auto & e : log_buf_) {
    if (e.level >= 40) ++nerr;
    else if (e.level >= 30) ++nwarn;
    else if (e.level >= 20) ++ninfo;
  }
  log_count_lbl_->setText(QStringLiteral("INFO %1 · WARN %2 · ERROR %3").arg(ninfo).arg(nwarn).arg(nerr));

  // Source filter combo (preserve selection).
  QString cur = log_source_cb_->currentText();
  QSet<QString> sources;
  for (const auto & e : log_buf_) sources.insert(e.source);
  QStringList sorted = sources.values();
  sorted.sort();
  log_source_cb_->blockSignals(true);
  log_source_cb_->clear();
  log_source_cb_->addItem(QStringLiteral("All"));
  log_source_cb_->addItems(sorted);
  int idx = log_source_cb_->findText(cur);
  log_source_cb_->setCurrentIndex(idx < 0 ? 0 : idx);
  log_source_cb_->blockSignals(false);

  if (!log_dirty_) return;
  log_dirty_ = false;

  int min_level = 0;
  switch (log_level_cb_->currentIndex()) {
    case 1: min_level = 30; break;
    case 2: min_level = 40; break;
    default: break;
  }
  const QString src = log_source_cb_->currentIndex() == 0
    ? QString() : log_source_cb_->currentText();

  log_view_->clear();
  for (const auto & e : log_buf_) {
    if (e.level < min_level) continue;
    if (!src.isEmpty() && e.source != src) continue;
    log_view_->appendHtml(logHtml(e));
  }
}

void MonitorWidget::refreshSensors() {
  for (auto & s : sensors_) {
    if (!s.cell) continue;
    QString color = QStringLiteral("#888");
    if (s.st.count > 0) {
      const double age = s.st.ageSeconds();
      if (age < 3.0) color = QStringLiteral("#8d8");
      else if (age < 10.0) color = QStringLiteral("#ec6");
      else color = QStringLiteral("#e77");
    }
    s.cell->setText(s.st.text());
    s.cell->setStyleSheet(QStringLiteral("color:%1;").arg(color));
  }
}

void MonitorWidget::refreshJoints() {
  const int gi = joint_group_cb_->currentIndex();
  if (gi < 0 || gi >= static_cast<int>(joint_groups_.size())) return;
  const auto & group = joint_groups_[gi];
  joint_table_->setRowCount(static_cast<int>(group.joints.size()));

  for (int r = 0; r < static_cast<int>(group.joints.size()); ++r) {
    const auto & d = group.joints[r];
    joint_table_->setItem(r, 0, new QTableWidgetItem(d.label));

    bool have = false;
    double pos = 0.0, vel = 0.0;
    if (js_) {
      for (size_t i = 0; i < js_->name.size(); ++i) {
        if (js_->name[i] == d.name) {
          if (i < js_->position.size()) pos = js_->position[i];
          if (i < js_->velocity.size()) vel = js_->velocity[i];
          have = true;
          break;
        }
      }
    }

    if (!have) {
      joint_table_->setItem(r, 1, new QTableWidgetItem(QStringLiteral("—")));
      joint_table_->setItem(r, 2, new QTableWidgetItem(QStringLiteral("—")));
    } else {
      const QString unit = d.is_prismatic ? QStringLiteral("m") : QStringLiteral("rad");
      const QString vunit = d.is_prismatic ? QStringLiteral("m/s") : QStringLiteral("rad/s");
      const bool nan_p = std::isnan(pos);
      const bool nan_v = std::isnan(vel);
      auto * pi = new QTableWidgetItem(
        nan_p ? QStringLiteral("NaN") : QStringLiteral("%1 %2").arg(pos, 0, 'f', 3).arg(unit));
      auto * vi = new QTableWidgetItem(
        nan_v ? QStringLiteral("NaN") : QStringLiteral("%1 %2").arg(vel, 0, 'f', 3).arg(vunit));
      if (nan_p) pi->setForeground(QBrush(QColor(230, 80, 80)));
      if (nan_v) vi->setForeground(QBrush(QColor(230, 80, 80)));
      joint_table_->setItem(r, 1, pi);
      joint_table_->setItem(r, 2, vi);
    }

    if (d.is_velocity) {
      joint_table_->setItem(r, 3, new QTableWidgetItem(QStringLiteral("velocity")));
    } else {
      joint_table_->setItem(r, 3, new QTableWidgetItem(
        QStringLiteral("[%1, %2]").arg(d.lower, 0, 'f', 2).arg(d.upper, 0, 'f', 2)));
    }

    QStringList flags;
    if (have && !d.is_velocity && !std::isnan(pos)) {
      const double range = d.upper - d.lower;
      if (range > 1e-6 &&
          (std::abs(pos - d.lower) < 0.02 * range || std::abs(pos - d.upper) < 0.02 * range)) {
        flags << QStringLiteral("near-limit");
      }
    }
    if (have && std::isnan(pos)) flags << QStringLiteral("NaN");
    auto * fi = new QTableWidgetItem(flags.join(QLatin1Char(' ')));
    if (!flags.isEmpty()) fi->setForeground(QBrush(QColor(230, 190, 60)));
    joint_table_->setItem(r, 4, fi);
  }
}

// ============================================================
// Log helpers
// ============================================================
QString MonitorWidget::logHtml(const LogEntry & e) {
  const QString color = e.level >= 40 ? QStringLiteral("#e77")
    : (e.level >= 30 ? QStringLiteral("#ec6") : QStringLiteral("#999"));
  return QStringLiteral("<span style='color:%1'>[%2]</span> <b>%3</b> %4")
    .arg(color, levelName(e.level), e.source.toHtmlEscaped(), e.text.toHtmlEscaped());
}

QString MonitorWidget::levelName(int level) {
  switch (level) {
    case 10: return QStringLiteral("DEBUG");
    case 20: return QStringLiteral("INFO");
    case 30: return QStringLiteral("WARN");
    case 40: return QStringLiteral("ERROR");
    case 50: return QStringLiteral("FATAL");
    default: return QStringLiteral("?");
  }
}

// ============================================================
// Slots
// ============================================================
void MonitorWidget::onLogFilterChanged() {
  log_dirty_ = true;
  refreshLog();
}

void MonitorWidget::onJointGroupChanged(int /*index*/) {
  refreshJoints();
}

}  // namespace ylr1d_hmi
