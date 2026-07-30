// 必须在任何 X11/Ogre 头之前包含全部的 Qt Widgets 头文件,
// 否则 X11 定义的 None / Status 等宏会破坏 Qt 的 enum 定义。
// 因此 ylr1d_hmi/hmi_window_rviz.hpp 放在最前面。
#include "ylr1d_hmi/hmi_window_rviz.hpp"

#include <QApplication>
#include <QSurfaceFormat>

#include <rclcpp/rclcpp.hpp>
#include <rviz_rendering/render_system.hpp>

int main(int argc, char ** argv) {
  // Qt 应用
  QApplication app(argc, argv);
  QApplication::setApplicationName("YLR1D HMI");

  // 设置 OpenGL 格式 (WSL 兼容)
  QSurfaceFormat fmt;
  fmt.setRenderableType(QSurfaceFormat::OpenGL);
  fmt.setAlphaBufferSize(8);
  fmt.setSwapInterval(0);  // 关闭垂直同步, 提升 WSL 响应
  QSurfaceFormat::setDefaultFormat(fmt);

  // ROS2 初始化
  rclcpp::init(argc, argv);

  // RViz2 Ogre 渲染初始化
  rviz_rendering::RenderSystem::get();

  // 创建节点
  auto node = std::make_shared<rclcpp::Node>("ylr1d_hmi_node");

  // RViz 配置路径由参数传入
  std::string rviz_config;
  node->declare_parameter("rviz_config", "");
  node->get_parameter("rviz_config", rviz_config);

  ylr1d_hmi::HmiWindowRviz window(rviz_config, node);
  window.show();

  int ret = app.exec();

  rclcpp::shutdown();
  return ret;
}
