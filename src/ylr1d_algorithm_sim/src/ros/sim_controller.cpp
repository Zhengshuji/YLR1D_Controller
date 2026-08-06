#include "ylr1d_algorithm_sim/ros/sim_controller.hpp"

#include <rclcpp_components/register_node_macro.hpp>

// 注册 5 个具名仿真控制器节点（与控制层分组一一对应），供 composition 容器加载。
RCLCPP_COMPONENTS_REGISTER_NODE(ylr1d_algorithm_sim::SteeringSimNode)
RCLCPP_COMPONENTS_REGISTER_NODE(ylr1d_algorithm_sim::WheelSimNode)
RCLCPP_COMPONENTS_REGISTER_NODE(ylr1d_algorithm_sim::TorsoSimNode)
RCLCPP_COMPONENTS_REGISTER_NODE(ylr1d_algorithm_sim::LeftArmSimNode)
RCLCPP_COMPONENTS_REGISTER_NODE(ylr1d_algorithm_sim::RightArmSimNode)
