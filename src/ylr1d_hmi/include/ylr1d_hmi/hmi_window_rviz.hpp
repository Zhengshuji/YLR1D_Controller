#ifndef YLR1D_HMI__HMI_WINDOW_RVIZ_HPP_
#define YLR1D_HMI__HMI_WINDOW_RVIZ_HPP_

#include "ylr1d_hmi/hmi_window.hpp"

// RViz2 forward declarations
namespace rviz_common {
class VisualizationManager;
class RenderPanel;
namespace ros_integration { class RosNodeAbstraction; }
}

namespace ylr1d_hmi {

/// 带 RViz2 渲染的 HMI 窗口
class HmiWindowRviz : public HmiWindow {
  Q_OBJECT

public:
  explicit HmiWindowRviz(const std::string & rviz_config, rclcpp::Node::SharedPtr node);
  ~HmiWindowRviz() override;

private:
  void buildRviz(QWidget * parent);

  // RViz2
  rviz_common::VisualizationManager * vis_manager_{nullptr};
  rviz_common::RenderPanel * render_panel_{nullptr};
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> rviz_ros_node_{nullptr};
};

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__HMI_WINDOW_RVIZ_HPP_
