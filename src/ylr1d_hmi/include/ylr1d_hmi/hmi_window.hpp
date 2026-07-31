#ifndef YLR1D_HMI__HMI_WINDOW_HPP_
#define YLR1D_HMI__HMI_WINDOW_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <QMainWindow>
#include <QVBoxLayout>
#include <QTreeWidget>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QGroupBox>
#include <QScrollArea>
#include <QTableWidget>
#include <QTabWidget>
#include <QTimer>
#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace ylr1d_hmi {

/// Static joint definition (limits/unit from ylr1d_mid_control/config/limits.yaml)
struct JointDef {
  std::string name;
  QString label;
  bool is_velocity{false};     // true = wheel (velocity control)
  bool is_prismatic{false};    // true = translational joint (unit: m / m/s)
  double lower{-3.14};
  double upper{3.14};
};

/// Describe a joint in the UI
struct JointInfo {
  std::string name;       // ROS joint name
  QString label;          // display short name
  bool is_velocity{false};   // true = wheel (velocity control)
  bool is_prismatic{false};  // true = translational joint (unit: m / m/s)
  double position{0.0};
  double velocity{0.0};
  double desired{0.0};
  QSlider * slider{nullptr};
  QDoubleSpinBox * spin{nullptr};
};

/// HMI main window base — shared ROS + observer/controller builders.
/// Subclasses must call the desired layout builder (buildUiLite or custom).
class HmiWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit HmiWindow(rclcpp::Node::SharedPtr node);
  ~HmiWindow() override;

  /// Build the lite layout (Observer top + Controller bottom, no RViz).
  /// Call after construction (or not at all if a subclass provides its own layout).
  void buildUiLite();

protected:
  // Shared UI builders — usable by any subclass
  QWidget * buildObserver(QWidget * parent);
  QWidget * buildController(QWidget * parent);

  // Publish desired joint states
  void publishDesired();

  rclcpp::Node::SharedPtr node_;
  std::vector<JointInfo> joints_;
  std::map<std::string, size_t> name_to_idx_;

private slots:
  void onRosSpin();          // timer: spin + update observer
  void onSliderChanged(int value);   // slider drag
  void onSpinChanged(double value);  // spin box change

private:
  // ROS2
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_pub_;

  // One observer table per joint group (Chassis/Torso/Left/Right)
  std::array<QTableWidget *, 4> obs_tables_{nullptr, nullptr, nullptr, nullptr};
  // Legacy single-tree observer — only used by the (unmaintained) RViz2 variant
  QTreeWidget * observer_tree_{nullptr};
  // Low-frequency send timer (GUI 触发 + 固定频率发送)
  QTimer * send_timer_{nullptr};
  bool desired_dirty_{false};
  size_t send_count_{0};
  size_t js_count_{0};

  // Status bar labels
  QLabel * js_status_lbl_{nullptr};
  QLabel * pub_status_lbl_{nullptr};

  // UI build helpers
  void buildStatusBar();
  QWidget * buildCard(int group_idx, const QString & title,
                      const QString & title_color,
                      const std::vector<JointDef> & defs);
  void addControlRow(QVBoxLayout * parent, const JointDef & d, size_t idx);

  // Publish desired joint states (only if dirty, called by send_timer_)
  void onSendTimer();

  QTimer * ros_timer_{nullptr};
};

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__HMI_WINDOW_HPP_
