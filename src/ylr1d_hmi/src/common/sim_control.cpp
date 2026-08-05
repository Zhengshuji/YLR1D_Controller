#include "ylr1d_hmi/common/sim_control.hpp"

#include <string>

namespace ylr1d_hmi {

namespace {

constexpr const char * kServiceNames[] = {
  "/pause_physics",
  "/unpause_physics",
  "/reset_simulation",
  "/reset_world",
};

}  // namespace

SimControl::SimControl(rclcpp::Node::SharedPtr node)
  : logger_(node->get_logger())
{
  for (size_t i = 0; i < clis_.size(); ++i) {
    clis_[i] = node->create_client<std_srvs::srv::Empty>(kServiceNames[i]);
  }
}

void SimControl::request(Action a) {
  const size_t i = static_cast<size_t>(a);
  if (i >= clis_.size()) return;
  if (!clis_[i] || !clis_[i]->service_is_ready() || inflight_[i]) return;
  doRequest(a, kServiceNames[i]);
}

void SimControl::doRequest(Action a, const std::string & service_name) {
  const size_t i = static_cast<size_t>(a);
  inflight_[i] = true;
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  auto cb = [this, i, service_name](
      rclcpp::Client<std_srvs::srv::Empty>::SharedFuture fut) {
    inflight_[i] = false;
    try {
      fut.get();
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "sim control %s failed: %s",
                  service_name.c_str(), e.what());
    }
  };
  clis_[i]->async_send_request(req, cb);
}

bool SimControl::servicesReady() const {
  for (const auto & c : clis_) {
    if (!c || !c->service_is_ready()) return false;
  }
  return true;
}

bool SimControl::inflight(Action a) const {
  return inflight_[static_cast<size_t>(a)];
}

}  // namespace ylr1d_hmi
