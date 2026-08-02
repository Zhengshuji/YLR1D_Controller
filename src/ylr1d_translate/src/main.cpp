#include <rclcpp/rclcpp.hpp>

#include "ylr1d_translate/translate_node.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ylr1d_translate::TranslateNode>());
  rclcpp::shutdown();
  return 0;
}
