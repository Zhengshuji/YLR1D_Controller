#include "ylr1d_hmi/panels/monitor_rviz_panel.hpp"
#include "ylr1d_hmi/panels/monitor_panel.hpp"

#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>

namespace ylr1d_hmi {

MonitorRvizPanel::MonitorRvizPanel(QWidget * parent)
  : rviz_common::Panel(parent)
{
  // rviz2 installs a global `__node` remap that would rename any freshly
  // created node to "rviz2" (collision + wrong logger name). Disable global
  // arguments so this panel keeps its own node name.
  rclcpp::NodeOptions opts;
  opts.use_global_arguments(false);
  node_ = std::make_shared<rclcpp::Node>("ylr1d_hmi_monitor", opts);

  auto lay = new QVBoxLayout(this);
  lay->setContentsMargins(2, 2, 2, 2);
  widget_ = new MonitorWidget(node_, this);
  lay->addWidget(widget_);

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, [this]() {
    if (rclcpp::ok()) rclcpp::spin_some(node_);
  });
  spin_timer_->start(20);
}

MonitorRvizPanel::~MonitorRvizPanel() {
  if (spin_timer_) spin_timer_->stop();
}

void MonitorRvizPanel::load(const rviz_common::Config &) {
  // No persistent panel state yet.
}

void MonitorRvizPanel::save(rviz_common::Config) const {
  // No persistent panel state yet.
}

}  // namespace ylr1d_hmi

PLUGINLIB_EXPORT_CLASS(ylr1d_hmi::MonitorRvizPanel, rviz_common::Panel)
