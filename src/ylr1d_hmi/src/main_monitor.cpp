#include "ylr1d_hmi/panels/monitor_panel.hpp"

#include <QApplication>
#include <QMainWindow>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

/// Standalone simulation monitor window (no rviz2 3D view).
///
/// Shares the same MonitorWidget as the rviz2 panel plugin; useful when you
/// only want the monitoring data without the 3D view, and for headless
/// verification (QT_QPA_PLATFORM=offscreen).
int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  QApplication::setApplicationName("YLR1D HMI (monitor)");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ylr1d_hmi_monitor");

  QMainWindow win;
  win.setWindowTitle("YLR1D simulation monitor");
  win.setCentralWidget(new ylr1d_hmi::MonitorWidget(node, &win));
  win.resize(920, 700);
  win.show();

  QTimer spin_timer;
  spin_timer.start(20);
  QObject::connect(&spin_timer, &QTimer::timeout, [&node]() {
    if (rclcpp::ok())
      rclcpp::spin_some(node);
    else
      QApplication::quit();  // SIGINT/SIGTERM 后 rcl 已 shutdown，干净退出 Qt
  });

  const int ret = app.exec();

  rclcpp::shutdown();
  return ret;
}
