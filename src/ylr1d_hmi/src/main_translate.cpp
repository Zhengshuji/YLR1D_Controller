#include "ylr1d_hmi/panels/translate_panel.hpp"

#include <QApplication>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

/// 传输层 HMI：向 ylr1d_translate 发送 action goal（/chassis_move /arm_move /gripper_move）。
int main(int argc, char ** argv) {
  QApplication app(argc, argv);
  QApplication::setApplicationName("YLR1D HMI (translate)");

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ylr1d_hmi_translate");

  ylr1d_hmi::TranslatePanel panel(node);
  panel.resize(720, 640);
  panel.show();

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
