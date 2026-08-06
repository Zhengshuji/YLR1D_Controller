#include "algorithm/ros/sim_controller.hpp"

#include <rclcpp_components/register_node_macro.hpp>

// 注册两个具体组件类（position / velocity），供 composition 容器加载。
RCLCPP_COMPONENTS_REGISTER_NODE(ylr1d_algorithm_sim::PositionSimController)
RCLCPP_COMPONENTS_REGISTER_NODE(ylr1d_algorithm_sim::VelocitySimController)
