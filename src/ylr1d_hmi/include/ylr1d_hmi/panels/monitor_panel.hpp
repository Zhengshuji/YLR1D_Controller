#ifndef YLR1D_HMI__PANELS__MONITOR_PANEL_HPP_
#define YLR1D_HMI__PANELS__MONITOR_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

#include <QWidget>
#include <QLabel>
#include <QTableWidget>
#include <QListWidget>
#include <QTabWidget>
#include <QComboBox>
#include <QPlainTextEdit>
#include <QTimer>
#include <QString>

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "ylr1d_hmi/panels/topic_status.hpp"

namespace ylr1d_hmi {

/// Joint definition for the monitor's joint table (limits in SI).
struct MonitorJointDef {
  QString label;
  std::string name;
  bool is_velocity{false};   // true = wheel (velocity control)
  bool is_prismatic{false};  // true = translational joint (unit m / m/s)
  double lower{-3.14};
  double upper{3.14};
};

/// One expected node the monitor tracks for liveness.
struct ExpectedNode {
  QString name;  // node name (namespace slash stripped)
  QString role;
};

/// One logical sensor row: subscribes a topic, tracks liveness.
struct SensorRow {
  QString label;
  QString topic;
  int kind{0};  // 0=Image, 1=PointCloud2, 2=LaserScan, 3=Imu
  TopicStatus st;
  QLabel * cell{nullptr};
};

/// One dropdown entry in the Joints tab.
struct JointGroup {
  QString name;
  std::vector<MonitorJointDef> joints;
};

/// A captured /rosout entry.
struct LogEntry {
  int level{20};       // rosgraph_msgs Log level
  QString source;      // node name
  QString text;        // message text
  double elapsed{-1};  // seconds since capture at display time (refreshed)
};

/// Rolling summary of startup milestones + anomaly counters.
struct Highlights {
  bool robot_spawned{false};
  bool cm_loaded{false};
  bool hw_ok{false};      // "Successful initialization of hardware 'GazeboSystem'"
  int controllers_active{0};
  int camera_pub{0};      // "Publishing camera info" lines
  int warn_total{0};
  int error_total{0};
};

/// Simulation monitoring panel — the shared core used by both the rviz2
/// plugin (MonitorRvizPanel) and the standalone ylr1d_hmi_monitor window.
///
/// Four tabs:
///   Overview — sim clock, controllers, expected nodes, anomaly summary
///   Log      — /rosout stream + filter + milestones/anomaly aggregation
///   Sensors  — liveness of every sensor topic on the robot
///   Joints   — dropdown per joint group (matches ylr1d_control_sim grouping)
///
/// Threading: mirrors the other panels — ROS callbacks run on the GUI thread
/// (a QTimer calls rclcpp::spin_some), so cached data needs no locking.
class MonitorWidget : public QWidget {
  Q_OBJECT

public:
  explicit MonitorWidget(rclcpp::Node::SharedPtr node, QWidget * parent = nullptr);

private:
  // ── ROS ──
  rclcpp::Node::SharedPtr node_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_cli_;

  // Cached data (GUI thread only)
  sensor_msgs::msg::JointState::SharedPtr js_;
  bool clock_seen_{false};
  bool clock_recent_{false};
  double sim_sec_{-1.0};
  std::vector<std::string> detected_nodes_;
  bool controllers_ok_{false};
  std::vector<std::string> ctrl_names_, ctrl_states_, ctrl_types_;

  std::deque<LogEntry> log_buf_;
  size_t log_total_{0};
  Highlights hl_;
  std::deque<LogEntry> notable_;  // milestones + anomalies (bounded)

  std::vector<SensorRow> sensors_;

  // ── Timers ──
  QTimer * ros_timer_{nullptr};
  QTimer * poll_timer_{nullptr};
  QTimer * refresh_timer_{nullptr};

  // ── UI (Overview) ──
  QLabel * sim_status_lbl_{nullptr};
  QLabel * js_rate_lbl_{nullptr};
  QLabel * node_cnt_lbl_{nullptr};
  QLabel * ctrl_cnt_lbl_{nullptr};
  QTableWidget * ctrl_table_{nullptr};
  QTableWidget * node_table_{nullptr};
  QListWidget * anomaly_list_{nullptr};

  // ── UI (Log) ──
  QComboBox * log_level_cb_{nullptr};
  QComboBox * log_source_cb_{nullptr};
  QLabel * log_count_lbl_{nullptr};
  QLabel * milestone_lbl_{nullptr};
  QPlainTextEdit * log_view_{nullptr};

  // ── UI (Sensors) ──
  QTableWidget * sensor_table_{nullptr};

  // ── UI (Joints) ──
  QComboBox * joint_group_cb_{nullptr};
  QTableWidget * joint_table_{nullptr};
  std::vector<JointGroup> joint_groups_;

  // ── Bookkeeping ──
  TopicStatus js_st_;                              // /joint_states liveness
  std::chrono::steady_clock::time_point clock_last_{};
  bool poll_inflight_{false};
  bool log_dirty_{true};
  int summary_counter_{0};

  // ── Helpers ──
  template<typename MsgT>
  rclcpp::SubscriptionBase::SharedPtr makeStatusSub(const std::string & topic,
                                                    TopicStatus * st);

  void onRosSpin();
  void onPoll();
  void onRefresh();
  void onLogFilterChanged();
  void onJointGroupChanged(int index);

  void onRosout(const rcl_interfaces::msg::Log::SharedPtr m);
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr m);
  void onClock(const rosgraph_msgs::msg::Clock::SharedPtr m);
  void classifyLog(const LogEntry & e);

  void buildUi();
  QWidget * buildOverview();
  QWidget * buildLog();
  QWidget * buildSensors();
  QWidget * buildJoints();

  void refreshOverview();
  void refreshLog();
  void refreshSensors();
  void refreshJoints();
  void updateNotables();
  void addAnomaly(const QString & text, const QString & color);

  /// HTML-color a log line for the plain-text stream.
  static QString logHtml(const LogEntry & e);
  static QString levelName(int level);
};

/// Liveness-only subscription: touches the row's TopicStatus on every message.
template<typename MsgT>
rclcpp::SubscriptionBase::SharedPtr MonitorWidget::makeStatusSub(
    const std::string & topic, TopicStatus * st) {
  return node_->create_subscription<MsgT>(
    topic, 1,
    [st](const MsgT &) { st->touch(); });
}

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__PANELS__MONITOR_PANEL_HPP_
