#include "ylr1d_control/nodes/chassis_control_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ylr1d_control::ChassisControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
