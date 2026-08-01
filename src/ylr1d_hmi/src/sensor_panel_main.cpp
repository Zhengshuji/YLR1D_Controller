#include "ylr1d_hmi/sensor_panel.hpp"

#include <QApplication>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

/// Standalone sensor visualization window.
///
/// Runs its own rclcpp node and spins it on the GUI thread (a 20 ms QTimer
/// calls spin_some), so ROS callbacks and the SensorPanel's refresh timer
/// both live on the GUI thread — no locking needed.
int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  QApplication::setApplicationName("YLR1D Sensor Panel");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ylr1d_sensor_panel");

  ylr1d_hmi::SensorPanel panel(node);
  panel.resize(1000, 760);
  panel.show();

  QTimer spin_timer;
  spin_timer.start(20);
  QObject::connect(&spin_timer, &QTimer::timeout,
                   [&node]() { rclcpp::spin_some(node); });

  const int ret = app.exec();

  rclcpp::shutdown();
  return ret;
}
