#ifndef YLR1D_HMI__PANELS__MONITOR_RVIZ_PANEL_HPP_
#define YLR1D_HMI__PANELS__MONITOR_RVIZ_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include <QTimer>
#include <memory>

namespace ylr1d_hmi {

class MonitorWidget;

/// rviz2 Panel plugin hosting the YLR1D simulation monitor.
/// Load it from the Panels menu (Add Panel → ylr1d_hmi/MonitorRvizPanel) or
/// via monitor.rviz / monitor.launch.py. The panel runs its own rclcpp node
/// and spins it on the GUI thread, exactly like the standalone monitor.
class MonitorRvizPanel : public rviz_common::Panel {
  Q_OBJECT

public:
  explicit MonitorRvizPanel(QWidget * parent = nullptr);
  ~MonitorRvizPanel() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  MonitorWidget * widget_{nullptr};
  QTimer * spin_timer_{nullptr};
};

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__PANELS__MONITOR_RVIZ_PANEL_HPP_
