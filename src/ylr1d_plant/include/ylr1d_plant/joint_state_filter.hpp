#ifndef YLR1D_PLANT__JOINT_STATE_FILTER_HPP_
#define YLR1D_PLANT__JOINT_STATE_FILTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cmath>

namespace ylr1d_plant {

/// Filters NaN values from /joint_states and republishes to /joint_states_filtered.
/// This prevents TF_NAN errors from prismatic joints in Gazebo Classic.
class JointStateFilter : public rclcpp::Node
{
public:
  JointStateFilter();

private:
  void callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

}  // namespace ylr1d_plant

#endif  // YLR1D_PLANT__JOINT_STATE_FILTER_HPP_
