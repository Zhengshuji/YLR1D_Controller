#include "ylr1d_hmi/panels/monitor_panel.hpp"

#include <QFileDialog>
#include <QFile>
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
#include <QTextStream>

#include <cmath>
#include <chrono>
#include <cstring>
#include <iterator>
#include <set>
#include <string>
#include <utility>

#include "ylr1d_hmi/panels/monitor_nodes.hpp"
#include "ylr1d_hmi/config/sensor_topics.hpp"
#include "ylr1d_hmi/config/joint_defs.hpp"
#include "ylr1d_hmi/common/sim_control.hpp"

namespace ylr1d_hmi {

namespace {

// ────────────────────────────────────────────────────────────
// Static configuration lives in monitor_nodes.hpp / joint_defs.hpp /
// sensor_topics.hpp (inline, shared)
// ────────────────────────────────────────────────────────────

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

  sim_ctl_ = std::make_unique<SimControl>(node_);

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
  // Any clock message immediately (re)asserts the running state — a single
  // fresh tick resets the stall counter, so transient jitter never flips the
  // indicator to PAUSED. Real pauses stop /clock and stay PAUSED.
  sim_state_ = SimState::Running;
  stall_count_ = 0;
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

  // ── Sim control buttons (enabled once the gazebo services are up) ──
  auto btn_row = new QHBoxLayout();
  btn_pause_ = new QPushButton(QStringLiteral("Pause"));
  btn_continue_ = new QPushButton(QStringLiteral("Continue"));
  btn_reset_sim_ = new QPushButton(QStringLiteral("Reset Sim"));
  btn_reset_world_ = new QPushButton(QStringLiteral("Reset World"));
  for (auto * b : {btn_pause_, btn_continue_, btn_reset_sim_, btn_reset_world_}) {
    b->setEnabled(false);
    btn_row->addWidget(b);
  }
  btn_row->addStretch();
  v->addLayout(btn_row);
  connect(btn_pause_, &QPushButton::clicked, this, &MonitorWidget::onSimPause);
  connect(btn_continue_, &QPushButton::clicked, this, &MonitorWidget::onSimContinue);
  connect(btn_reset_sim_, &QPushButton::clicked, this, &MonitorWidget::onSimResetSim);
  connect(btn_reset_world_, &QPushButton::clicked, this, &MonitorWidget::onSimResetWorld);

  // ── Sim status bar — colored background reflects the state ──
  sim_status_lbl_ = new QLabel(QStringLiteral("● --"));
  sim_status_lbl_->setStyleSheet(
    QStringLiteral("color:#fff; background:#555; padding:4px 10px; border-radius:3px;"));
  v->addWidget(sim_status_lbl_);

  // ── One quiet stats line (no box) ──
  stat_line_lbl_ = new QLabel(QStringLiteral("js -- · nodes -- · controllers --"));
  stat_line_lbl_->setStyleSheet(QStringLiteral("color:#bbb;"));
  v->addWidget(stat_line_lbl_);

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
  log_save_btn_ = new QPushButton(QStringLiteral("Save"));
  log_save_btn_->setToolTip(QStringLiteral("Export the filtered log to a text file"));
  row->addWidget(log_save_btn_);
  v->addLayout(row);
  connect(log_save_btn_, &QPushButton::clicked, this, &MonitorWidget::onSaveLog);
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

  // Hysteresis: only flip RUNNING → PAUSED after 6 consecutive 500 ms refreshes
  // (~3 s) without a fresh /clock, so a momentarily stalling clock on a loaded
  // machine does not flicker the indicator. Any /clock tick resets both (onClock).
  if (clock_recent_) {
    sim_state_ = SimState::Running;
    stall_count_ = 0;
  } else if (sim_state_ == SimState::Running) {
    if (++stall_count_ >= 6) {
      sim_state_ = SimState::Paused;
    }
  }

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
  // ── Sim status bar — background color reflects state ──
  {
    const char * text = "NO CLOCK";
    const char * bg = "#a33";
    switch (sim_state_) {
      case SimState::Running: text = "RUNNING"; bg = "#2e7d32"; break;
      case SimState::Paused:  text = "PAUSED";  bg = "#b08a1e"; break;
      default: break;
    }
    QString label = QStringLiteral("\u25CF %1").arg(QLatin1String(text));
    if (sim_state_ == SimState::Running)
      label += QStringLiteral("  t=%1 s").arg(sim_sec_, 0, 'f', 2);
    sim_status_lbl_->setText(label);
    sim_status_lbl_->setStyleSheet(
      QStringLiteral("color:#fff; background:%1; padding:4px 10px; border-radius:3px;")
        .arg(QLatin1String(bg)));
  }

  // ── Stats line ──
  {
    const double r = js_st_.rateHz();
    QString js = r >= 0.0 ? QStringLiteral("%1 Hz").arg(r, 0, 'f', 1)
                          : QStringLiteral("no data");
    int active = 0;
    for (const auto & s : ctrl_states_) if (s == "active") ++active;
    stat_line_lbl_->setText(QStringLiteral("js %1 · nodes %2 · controllers %3/%4 active")
      .arg(js).arg(detected_nodes_.size()).arg(active).arg(ctrl_names_.size()));
  }

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
  node_table_->setRowCount(static_cast<int>(std::size(kExpectedNodes)));
  for (int i = 0; i < static_cast<int>(std::size(kExpectedNodes)); ++i) {
    const auto & en = kExpectedNodes[i];
    bool found = false;
    for (const auto & dn : detected_nodes_) {
      if (nodeNameMatch(QString::fromStdString(dn), en.name)) { found = true; break; }
    }
    QString status;
    QColor color;
    if (found) { status = QStringLiteral("online"); color = QColor(60, 180, 80); }
    else if (en.optional) { status = QStringLiteral("missing (optional)"); color = QColor(230, 190, 60); }
    else { status = QStringLiteral("missing"); color = QColor(230, 80, 80); }
    auto * s = new QTableWidgetItem(status);
    s->setForeground(QBrush(color));
    node_table_->setItem(i, 0, new QTableWidgetItem(en.name));
    node_table_->setItem(i, 1, new QTableWidgetItem(en.role));
    node_table_->setItem(i, 2, s);
  }

  // ── Sim control buttons — gated on service availability + sim state ──
  if (sim_ctl_) {
    const bool ready = sim_ctl_->servicesReady();
    btn_pause_->setEnabled(ready && sim_state_ == SimState::Running);
    btn_continue_->setEnabled(ready && sim_state_ == SimState::Paused);
    btn_reset_sim_->setEnabled(ready);
    btn_reset_world_->setEnabled(ready);
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
    if (!found && !en.optional) {
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

  log_view_->clear();
  for (const auto & e : visibleLogEntries()) {
    log_view_->appendHtml(logHtml(e));
  }
}

std::vector<LogEntry> MonitorWidget::visibleLogEntries() const {
  int min_level = 0;
  switch (log_level_cb_->currentIndex()) {
    case 1: min_level = 30; break;
    case 2: min_level = 40; break;
    default: break;
  }
  const QString src = log_source_cb_->currentIndex() == 0
    ? QString() : log_source_cb_->currentText();

  std::vector<LogEntry> out;
  for (const auto & e : log_buf_) {
    if (e.level < min_level) continue;
    if (!src.isEmpty() && e.source != src) continue;
    out.push_back(e);
  }
  return out;
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

void MonitorWidget::onSimPause() {
  if (sim_ctl_) sim_ctl_->request(SimControl::Action::Pause);
}

void MonitorWidget::onSimContinue() {
  if (sim_ctl_) sim_ctl_->request(SimControl::Action::Continue);
}

void MonitorWidget::onSimResetSim() {
  if (sim_ctl_) sim_ctl_->request(SimControl::Action::ResetSim);
}

void MonitorWidget::onSimResetWorld() {
  if (sim_ctl_) sim_ctl_->request(SimControl::Action::ResetWorld);
}

void MonitorWidget::onSaveLog() {
  const auto entries = visibleLogEntries();
  const QString path = QFileDialog::getSaveFileName(
    this, QStringLiteral("Save log"), QString(),
    QStringLiteral("Log files (*.log);;All files (*)"));
  if (path.isEmpty()) return;

  QFile f(path);
  if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
    RCLCPP_WARN(node_->get_logger(), "cannot open %s for writing",
                path.toStdString().c_str());
    return;
  }
  QTextStream out(&f);
  for (const auto & e : entries) {
    out << QStringLiteral("[%1] %2: %3\n")
            .arg(levelName(e.level), e.source, e.text);
  }
  RCLCPP_INFO(node_->get_logger(), "saved %zu log entries to %s",
              entries.size(), path.toStdString().c_str());
}

}  // namespace ylr1d_hmi
