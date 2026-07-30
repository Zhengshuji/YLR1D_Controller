// 必须在任何 X11/Ogre 头之前包含 Qt 的 qdatastream.h,
// 否则 X11 定义的 Status 宏会导致编译错误
#include <QDataStream>

#include "ylr1d_hmi/hmi_window_rviz.hpp"

#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <QVBoxLayout>
#include <QSplitter>

namespace ylr1d_hmi {

HmiWindowRviz::HmiWindowRviz(const std::string & rviz_config, rclcpp::Node::SharedPtr node)
  : HmiWindow(node)
{
  setWindowTitle("YLR1D HMI");

  // 重新布局: 水平分割 Rviz | Observer, 底部 Controller
  auto central = new QWidget(this);
  setCentralWidget(central);
  auto main_layout = new QVBoxLayout(central);
  main_layout->setContentsMargins(4, 4, 4, 4);
  main_layout->setSpacing(4);

  auto top_splitter = new QSplitter(Qt::Horizontal);

  // ── RViz ──────────────────────────────────────────
  auto rviz_container = new QWidget();
  buildRviz(rviz_container);
  top_splitter->addWidget(rviz_container);

  // ── Observer ──────────────────────────────────────
  auto obs = buildObserver(nullptr);
  top_splitter->addWidget(obs);
  top_splitter->setStretchFactor(0, 3);
  top_splitter->setStretchFactor(1, 2);

  // ── Controller ────────────────────────────────────
  auto ctrl_scroll = new QScrollArea();
  ctrl_scroll->setWidgetResizable(true);
  ctrl_scroll->setWidget(buildController(nullptr));

  // ── 整体布局 ──────────────────────────────────────
  auto top_splitter_v = new QSplitter(Qt::Vertical);
  top_splitter_v->addWidget(top_splitter);
  top_splitter_v->addWidget(ctrl_scroll);
  top_splitter_v->setStretchFactor(0, 2);
  top_splitter_v->setStretchFactor(1, 1);

  main_layout->addWidget(top_splitter_v);

  // 加载 RViz 配置
  if (!rviz_config.empty() && vis_manager_) {
    rviz_common::YamlConfigReader reader;
    rviz_common::Config config;
    reader.readFile(config, QString::fromStdString(rviz_config));
    if (!reader.error()) {
      vis_manager_->load(config);
    }
  }
}

HmiWindowRviz::~HmiWindowRviz() {
  if (vis_manager_) vis_manager_->stopUpdate();
}

void HmiWindowRviz::buildRviz(QWidget * parent) {
  auto layout = new QVBoxLayout(parent);
  layout->setContentsMargins(0, 0, 0, 0);

  render_panel_ = new rviz_common::RenderPanel();
  layout->addWidget(render_panel_);

  rviz_ros_node_ =
    std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("ylr1d_hmi_rviz");

  vis_manager_ = new rviz_common::VisualizationManager(
    render_panel_,
    rviz_ros_node_,
    nullptr,
    node_->get_clock()
  );
  render_panel_->initialize(vis_manager_);
  vis_manager_->initialize();
  vis_manager_->startUpdate();
  vis_manager_->setFixedFrame("map");
}

}  // namespace ylr1d_hmi
