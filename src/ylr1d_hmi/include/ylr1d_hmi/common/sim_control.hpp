#ifndef YLR1D_HMI__COMMON__SIM_CONTROL_HPP_
#define YLR1D_HMI__COMMON__SIM_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <array>
#include <memory>

namespace ylr1d_hmi {

/// Client for the four gazebo_ros sim-control services
/// (/pause_physics /unpause_physics /reset_simulation /reset_world), all
/// std_srvs/srv/Empty. These are provided by libgazebo_ros_init.so which
/// gzserver already loads in gazebo.launch.py — no launch change needed.
///
/// Single-threaded: invoked from the GUI thread (rclcpp::spin_some), so no
/// locking is required; the per-action inflight flag prevents stacking
/// duplicate requests while a previous one is still outstanding.
class SimControl {
public:
  enum class Action { Pause, Continue, ResetSim, ResetWorld };

  explicit SimControl(rclcpp::Node::SharedPtr node);

  /// Fire-and-forget: no-op if the service is not discovered yet or a request
  /// is already in flight. Outcome is logged via the node logger.
  void request(Action a);

  /// True when all four services have been discovered.
  bool servicesReady() const;

  bool inflight(Action a) const;

private:
  void doRequest(Action a, const std::string & service_name);

  rclcpp::Logger logger_;
  std::array<rclcpp::Client<std_srvs::srv::Empty>::SharedPtr, 4> clis_{};
  std::array<bool, 4> inflight_{};
};

}  // namespace ylr1d_hmi

#endif  // YLR1D_HMI__COMMON__SIM_CONTROL_HPP_
