#include "ylr1d_position_simulate/nodes/chassis_simulate_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ylr1d_position_simulate::ChassisSimulateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
