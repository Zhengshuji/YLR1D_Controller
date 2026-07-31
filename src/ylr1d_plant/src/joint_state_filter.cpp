#include "ylr1d_plant/joint_state_filter.hpp"

namespace ylr1d_plant {

JointStateFilter::JointStateFilter() : Node("joint_state_filter")
{
  sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { callback(msg); });

  pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states_filtered", 10);

  RCLCPP_INFO(get_logger(), "Joint state filter started (NaN -> 0.0)");
}

void JointStateFilter::callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  auto out = sensor_msgs::msg::JointState();
  out.header = msg->header;
  out.name = msg->name;
  out.position.resize(msg->position.size());
  out.velocity.resize(msg->velocity.size());
  out.effort.resize(msg->effort.size());

  for (std::size_t i = 0; i < msg->position.size(); ++i) {
    out.position[i] = std::isnan(msg->position[i]) ? 0.0 : msg->position[i];
  }
  for (std::size_t i = 0; i < msg->velocity.size(); ++i) {
    out.velocity[i] = std::isnan(msg->velocity[i]) ? 0.0 : msg->velocity[i];
  }
  for (std::size_t i = 0; i < msg->effort.size(); ++i) {
    out.effort[i] = std::isnan(msg->effort[i]) ? 0.0 : msg->effort[i];
  }

  pub_->publish(out);
}

}  // namespace ylr1d_plant

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ylr1d_plant::JointStateFilter>());
  rclcpp::shutdown();
  return 0;
}
