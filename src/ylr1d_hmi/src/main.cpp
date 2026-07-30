#include "ylr1d_hmi/hmi_window.hpp"

#include <QApplication>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
  // Qt 应用
  QApplication app(argc, argv);
  QApplication::setApplicationName("YLR1D HMI");

  // ROS2 初始化
  rclcpp::init(argc, argv);

  // 创建节点
  auto node = std::make_shared<rclcpp::Node>("ylr1d_hmi_node");

  // 主窗口（lite 布局）
  ylr1d_hmi::HmiWindow window(node);
  window.buildUiLite();
  window.show();

  int ret = app.exec();

  rclcpp::shutdown();
  return ret;
}
