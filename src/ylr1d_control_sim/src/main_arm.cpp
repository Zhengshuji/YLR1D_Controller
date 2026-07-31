#include "ylr1d_control_sim/nodes/arm_simulate_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ylr1d_control_sim::ArmSimulateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
